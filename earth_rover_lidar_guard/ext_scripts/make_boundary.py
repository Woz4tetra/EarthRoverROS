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


def test_algorithm(shapely_polygon, my_polygon):
    reference_in_points = []
    reference_out_points = []
    my_in_points = []
    my_out_points = []
    for test_point in np.random.uniform(-0.5, 0.5, (100, 2)):
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
    buffer = ""
    for point in polygon.tolist():
        buffer += "%s, %s,\n" % (point[0], point[1])
    buffer = buffer[:-2]
    with open(path, 'w+') as file:
        file.write(buffer)



def load_points():
    with open("points_of_interest.json") as file:
        points_of_interest = json.load(file)

    bounding_box_max = Polygon(*points_of_interest["bounding_box_max"])
    bounding_box_min = Polygon(*points_of_interest["bounding_box_min"])

    bounding_box_max.order_points()
    bounding_box_min.order_points()

    bounding_box_max_points = np.array(bounding_box_max.points)
    bounding_box_min_points = np.array(bounding_box_min.points)

    # bounding_box_max_points = np.rot90(bounding_box_max_points.T, 3)
    # bounding_box_min_points = np.rot90(bounding_box_min_points.T, 3)

    shapely_bounding_box_max = ShapelyPolygon(bounding_box_max_points)
    shapely_bounding_box_min = ShapelyPolygon(bounding_box_max_points)

    plt.plot(0, 0, 'ko')
    plt.plot(bounding_box_max_points[:, 0], bounding_box_max_points[:, 1], '.')
    plt.plot(bounding_box_min_points[:, 0], bounding_box_min_points[:, 1], '.')

    plt.gca().add_patch(patches.Polygon(bounding_box_max_points, closed=True, fill=False))
    plt.gca().add_patch(patches.Polygon(bounding_box_min_points, closed=True, fill=False))

    test_algorithm(shapely_bounding_box_max, bounding_box_max_points)
    # test_algorithm(shapely_bounding_box_min, bounding_box_min_points)

    write_to_file("bounding_box_max.txt", bounding_box_max_points)

    plt.show()


load_points()
