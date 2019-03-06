import math
from pid import PID

class MotorInfo:
    def __init__(self, name, kp, ki, kd,
                 speed_smooth_k,
                 wheel_radius_meters, ticks_per_rotation,
                 min_speed_meters_per_s, max_speed_meters_per_s,
                 min_command, max_command, output_deadzone):
        self.name = name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.speed_smooth_k = speed_smooth_k
        self.wheel_radius = wheel_radius_meters
        self.ticks_per_rotation = ticks_per_rotation
        self.min_speed_meters_per_s = min_speed_meters_per_s
        self.max_speed_meters_per_s = max_speed_meters_per_s
        self.min_command = min_command
        self.max_command = max_command
        self.output_deadzone = output_deadzone

class MotorController:
    def __init__(self, motor_info):
        self.info = motor_info

        self.ticks_to_meters = 2 * math.pi * self.info.wheel_radius / self.info.ticks_per_rotation
        self.setpoint_mps = 0.0
        self.open_loop_setpoint = 0.0

        self.enc_tick = 0
        self.current_distance = 0.0
        self.current_speed = 0.0
        self.smooth_speed = 0.0
        self.prev_enc_dist = 0.0
        self.prev_enc_dist_for_odom = 0.0
        self.delta_dist = 0.0

        self.prev_time = None

        self.pid = PID.init_with_constants(self.info.kp, self.info.ki, self.info.kd)

    def convert_speed_to_command(self, speed_mps):
        slope = (self.info.max_command - self.info.min_command) / (self.info.max_speed_meters_per_s - self.info.min_speed_meters_per_s)
        command = slope * (speed_mps - self.info.min_speed_meters_per_s) + self.info.min_command
        return command

    def set_target(self, setpoint_mps):
        self.setpoint_mps = setpoint_mps

        # apply the open loop constant k (the convert_speed_to_command method)
        self.open_loop_setpoint = self.convert_speed_to_command(self.setpoint_mps)

    def compute_speed(self, dt):
        self.current_distance = self.enc_tick * self.ticks_to_meters
        self.delta_dist = self.current_distance - self.prev_enc_dist

        self.prev_enc_dist = self.current_distance
        self.current_speed = self.delta_dist / dt

        # return self.current_speed

        error = self.current_speed - self.smooth_speed
        self.smooth_speed += self.info.speed_smooth_k * error

        # print self.info.name, dt, self.delta_dist / self.ticks_to_meters, self.smooth_speed, self.current_speed
        return self.smooth_speed

    def get_speed(self):
        # return self.smooth_speed
        return self.current_speed

    def get_dist(self):
        return self.current_distance

    def get_delta_dist(self):
        current_distance = self.enc_tick * self.ticks_to_meters
        delta_dist = current_distance - self.prev_enc_dist_for_odom
        self.prev_enc_dist_for_odom = current_distance
        return delta_dist

    def tune_pid(self, kp, ki, kd):
        self.pid.reset()
        self.pid.kp = kp
        self.pid.ki = ki
        self.pid.kd = kd

    def get_dt(self, current_time):
        if self.prev_time is None:
            self.prev_time = current_time
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time
        return dt

    def update(self, dt):
        if dt == 0.0:
            return 0.0

        speed = self.compute_speed(dt)
        error = self.setpoint_mps - speed
        output = self.pid.compute(dt, error, self.open_loop_setpoint)

        # self.compute_speed(dt)
        # output = self.open_loop_setpoint

        if abs(output) < self.info.output_deadzone:
            output = 0.0

        return output
