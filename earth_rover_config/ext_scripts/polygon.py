# /usr/bin/python3
# https://stackoverflow.com/questions/10846431/ordering-shuffled-points-that-can-be-joined-to-form-a-polygon-in-python
import math
import pyclipper


class Polygon:
    def __init__(self, *points):
        self.points = []
        self.hashed_points = set()
        for point in points:
            self.add_point(point)

        self._centeroid = (0.0, 0.0)
        self._computed_centeroid_len = 0

    @classmethod
    def with_polygon(cls, polygon):
        return Polygon(*polygon.points)

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

    def scale(self, x_scale, y_scale, center=None):
        if center is None:
            center = self.centeroid

        x_center, y_center = center

        for index in range(len(self.points)):
            x, y = self.points[index]
            new_x = (x - x_center) * x_scale + x_center
            new_y = (y - y_center) * y_scale + y_center
            self.points[index] = new_x, new_y

    def offset(self, distance, offset_style=pyclipper.JT_MITER):
        # JT_MITER = 2
        # JT_ROUND = 1
        # JT_SQUARE = 0
        clipper_offset = pyclipper.PyclipperOffset()
        coordinates_scaled = pyclipper.scale_to_clipper(self.points)
        clipper_offset.AddPath(coordinates_scaled, offset_style, pyclipper.ET_CLOSEDPOLYGON)
        new_coordinates = clipper_offset.Execute(pyclipper.scale_to_clipper(distance))
        new_coordinates_scaled = pyclipper.scale_from_clipper(new_coordinates)
        self.points = new_coordinates_scaled[0]

    def __str__(self):
        return self.points
