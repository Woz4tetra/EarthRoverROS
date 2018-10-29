class PID:
    def __init__(self):
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0

        self.min_output = -1.0
        self.max_output = 1.0

        self.reset()

    @classmethod
    def init_with_constants(cls, kp, ki, kd):
        obj = cls()
        obj.kp = kp
        obj.ki = ki
        obj.kd = kd

        return obj

    def set_limits(self, min_output, max_output):
        self.min_output = min_output
        self.max_output = max_output

    def reset(self):
        self.integral_total = 0.0
        self.prev_state = 0.0
        self.prev_error = 0.0

    def compute(self, dt, error, open_loop_setpoint):
        if self.kd != 0.0:
            d_value = (error - self.prev_error) / dt
        else:
            d_value = 0.0

        if self.ki != 0.0:
            self.integral_total += error * dt

        output = open_loop_setpoint + self.kp * error + self.ki * self.integral_total + self.kd * d_value
        output = min(output, self.max_output)
        output = max(output, self.min_output)

        self.prev_error = error

        return output
