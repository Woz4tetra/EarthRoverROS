import re
import json
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches

from polygon import Polygon

from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon


def reference_algorithm(polygon, point):
    point = ShapelyPoint(point)
    return polygon.contains(point)


def my_algorithm(polygon, point):
    x, y = point
    n = len(polygon)
    inside = False

    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]

        # if point is in bounds of line segment
        if min(p1y, p2y) < y <= max(p1y, p2y) and x <= max(p1x, p2x):
            x_intersect = None
            if p1y != p2y:  # if not a horizontal line segment
                x_intersect = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x

            # if a vertical line segment or point has a intersection along sloped line
            if p1x == p2x or (x_intersect is not None and x <= x_intersect):
                # toggle inside boolean. Odd number of intersections means point is inside. Even number means outside.
                inside = not inside
        p1x, p1y = p2x, p2y

    return inside


def read_test_points(path):
    test_points = []
    with open(path) as file:
        contents = file.read()
        matches = re.findall(r"\[ INFO\] \[\d*.\d*\]: x: ([-\d.]*), y: ([-\d.]*)", contents)
        for match in matches:
            test_points.append([float(match[0]), float(match[1])])
    return test_points


def test_algorithm(shapely_polygon, my_polygon, test_points):
    reference_in_points = []
    reference_out_points = []
    my_in_points = []
    my_out_points = []
    for test_point in test_points:
        reference_result = reference_algorithm(shapely_polygon, test_point)
        my_result = my_algorithm(my_polygon, test_point)

        if reference_result != my_result:
            print("Failed on test point: %s" % test_point)

        if reference_result:
            reference_in_points.append(test_point)
        else:
            reference_out_points.append(test_point)

        if my_result:
            my_in_points.append(test_point)
        else:
            my_out_points.append(test_point)
    reference_in_points = np.array(reference_in_points)
    reference_out_points = np.array(reference_out_points)
    my_in_points = np.array(my_in_points)
    my_out_points = np.array(my_out_points)

    if len(reference_in_points) > 0:
        plt.plot(reference_in_points[:, 0], reference_in_points[:, 1], 'x', color='g')
    if len(reference_out_points) > 0:
        plt.plot(reference_out_points[:, 0], reference_out_points[:, 1], 'x', color='r')

    if len(my_in_points) > 0:
        plt.plot(my_in_points[:, 0], my_in_points[:, 1], 'o', color='g', markersize=7, markeredgewidth=1,
                 markerfacecolor='None')
    if len(my_out_points) > 0:
        plt.plot(my_out_points[:, 0], my_out_points[:, 1], 'o', color='r', markersize=7, markeredgewidth=1,
                 markerfacecolor='None')


def write_to_file(path, polygon):
    buffer = "[\n"
    for point in polygon.tolist():
        buffer += "    [%s, %s],\n" % (point[0], point[1])
    buffer = buffer[:-2]
    buffer += "\n]"
    with open(path, 'w+') as file:
        file.write(buffer)


def load_points():
    with open("points_of_interest.json") as file:
        points_of_interest = json.load(file)

    bounding_box = Polygon(*points_of_interest["bounding_box"])

    # push front points forward
    for index in range(4, 10):
        bounding_box.points[index][0] += 0.05

    bounding_box.order_points()

    bounding_box_max = bounding_box.with_polygon(bounding_box)
    bounding_box_min = bounding_box.with_polygon(bounding_box)

    # bounding_box_max.scale(1.2, 1.2)
    bounding_box_max.offset(0.02, 0)

    bounding_box_max_points = np.array(bounding_box_max.points)
    bounding_box_min_points = np.array(bounding_box_min.points)

    theta = np.radians(180)
    c, s = np.cos(theta), np.sin(theta)
    rotation_mat = np.array([[c, -s], [s, c]])

    bounding_box_max_points = bounding_box_max_points.dot(rotation_mat)
    bounding_box_min_points = bounding_box_min_points.dot(rotation_mat)

    shapely_bounding_box_max = ShapelyPolygon(bounding_box_max_points)
    # shapely_bounding_box_min = ShapelyPolygon(bounding_box_max_points)

    plt.plot(0, 0, 'ko')
    plt.plot(bounding_box_max_points[:, 0], bounding_box_max_points[:, 1], 'g.')
    plt.plot(bounding_box_min_points[:, 0], bounding_box_min_points[:, 1], '.')

    plt.gca().add_patch(patches.Polygon(bounding_box_max_points, closed=True, fill=False))
    plt.gca().add_patch(patches.Polygon(bounding_box_min_points, closed=True, fill=False))

    # test_points = np.random.uniform(-0.5, 0.5, (100, 2))
    test_points = read_test_points("test_points.txt")
    test_algorithm(shapely_bounding_box_max, bounding_box_max_points, test_points)
    # test_algorithm(shapely_bounding_box_min, bounding_box_min_points)

    write_to_file("bounding_box_max.txt", bounding_box_max_points)
    write_to_file("bounding_box_min.txt", bounding_box_min_points)

    plt.show()


load_points()
