
class Direction:
    FRONT = 0
    BACK = 1
    LEFT = 2
    RIGHT = 3

    directions = [
        "FRONT",
        "BACK",
        "LEFT",
        "RIGHT",
    ]

    @staticmethod
    def name(direction):
        return Direction.directions[direction]

class UltrasonicTracker:
    def __init__(self, stop_dist, ease_dist):
        self.ease_dist = ease_dist
        self.stop_dist = stop_dist
        self.velocity_scale = 1.0

        self.smooth_dist = 0.0
        self.smooth_k = 0.8

    def update(self, distance):
        self.smooth_dist += self.smooth_k * (distance - self.smooth_dist)

        if self.smooth_dist >= self.ease_dist:
            self.velocity_scale = 1.0
        elif self.smooth_dist <= self.stop_dist:
            self.velocity_scale = 0.0
        else:
            self.velocity_scale = (self.smooth_dist - self.stop_dist) / (self.ease_dist - self.stop_dist)

        self.velocity_scale *= self.velocity_scale  # create a quadratic relation


class TrackerCollection:
    def __init__(self):
        self.trackers = []
        self.scale = 0.0

    def append(self, tracker):
        self.trackers.append(tracker)

    def scale_v(self, velocity):
        if len(self.trackers) == 1:
            self.scale = self.trackers[0].velocity_scale
        else:
            self.scale = min([tracker.velocity_scale for tracker in self.trackers])
        return velocity * self.scale

    def get_dists(self):
        return [tracker.smooth_dist for tracker in self.trackers]

    def get_scale(self):
        return self.scale
