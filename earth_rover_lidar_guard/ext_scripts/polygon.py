# /usr/bin/python3
# https://stackoverflow.com/questions/10846431/ordering-shuffled-points-that-can-be-joined-to-form-a-polygon-in-python
import math


class Polygon:
    def __init__(self, *points):
        self.points = []
        self.hashed_points = set()
        for point in points:
            self.add_point(point)

        self._centeroid = (0.0, 0.0)
        self._computed_centeroid_len = 0

    def add_point(self, point: tuple):
        hashed_point = hash(tuple(point))
        if hashed_point not in self.hashed_points:
            self.hashed_points.add(hashed_point)
            self.points.append(point)

    @property
    def centeroid(self):
        if self._computed_centeroid_len != len(self.points):
            self._computed_centeroid_len = len(self.points)
            x_sum = 0
            y_sum = 0
            for x, y in self.points:
                x_sum += x
                y_sum += y
            self._centeroid = x_sum / len(self.points), y_sum / (len(self.points))
        return self._centeroid

    def order_points(self):
        self.points.sort(key=lambda p: math.atan2(p[1] - self.centeroid[1], p[0] - self.centeroid[0]))


    def __str__(self):
        return self.points
