from .pid import PID

class MotorInfo:
    def __init__(self, kp, ki, kd,
                 wheel_radius_meters, ticks_per_rotation,
                 min_speed_meters_per_s, max_speed_meters_per_s,
                 min_command, max_command):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.wheel_radius = wheel_radius_meters
        self.ticks_per_rotation = ticks_per_rotation
        self.min_speed_meters_per_s = min_speed_meters_per_s
        self.max_speed_meters_per_s = max_speed_meters_per_s
        self.min_command = min_command
        self.max_command = max_command

class MotorController:
    def __init__(self, motor_info):
        self.info = motor_info

        self.ticks_to_meters = 2 * math.pi * self.info.wheel_radius / self.info.ticks_per_rotation
        self.setpoint_mps = 0.0
        self.open_loop_setpoint = 0.0
        
        self.enc_tick = 0
        self.current_distance = 0.0
        self.current_speed = 0.0
        self.prev_enc_dist = 0.0
        self.delta_dist = 0.0

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
        return self.delta_dist * self.ticks_to_meters / dt

    def tune_pid(self, kp, ki, kd):
        self.pid.reset()
        self.pid.kp = kp
        self.pid.ki = ki
        self.pid.kd = kd

    def update(self, dt):
        self.current_speed = self.compute_speed(dt)

        error = self.setpoint_mps - self.current_speed
        output = self.pid.compute(dt, error, self.open_loop_setpoint)

        return output
