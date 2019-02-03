
class Direction:
    FRONT = 0
    BACK = 1
    LEFT = 2
    RIGHT = 3

class UltrasonicTracker:
    def __init__(self, stop_dist, ease_dist):
        self.ease_dist = ease_dist
        self.stop_dist = stop_dist
        self.velocity_scale = 1.0

        self.smooth_dist = 0.0
        self.smooth_k = 1.0

    def update(self, distance):
        self.smooth_dist += self.smooth_k * (distance - self.smooth_dist)

        if self.smooth_dist >= self.ease_dist:
            self.velocity_scale = 1.0
        elif self.smooth_dist <= self.stop_dist:
            self.velocity_scale = 0.0
        else:
            self.velocity_scale = (self.smooth_dist - self.stop_dist) / (self.ease_dist - self.stop_dist)

    def scale_velocity(self, velocity):
        return self.velocity_scale * velocity


class TrackerCollection:
    def __init__(self):
        self.trackers = []

    def append(self, tracker):
        self.trackers.append(tracker)

    def update(self, velocity):
        return min([tracker.scale_velocity(velocity) for tracker in self.trackers])
