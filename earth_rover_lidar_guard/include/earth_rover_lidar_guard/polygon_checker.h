// A C++ program to check if a given point lies inside a given polygon
// Refer https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
// for explanation of functions onSegment(), orientation() and doIntersect()
#include <iostream>
using namespace std;

// Returns true if the point p (x, y) lies inside the polygon[] with n vertices
bool isInside(vector<PolygonPoint>* polygon, double x, double y)
{
    int n = polygon->size();
    bool inside = false;

    // There must be at least 3 vertices in polygon[]
	if (polygon->size() < 3) {
        return false;
    }

    double p1x = polygon[0];
    double p1y = polygon[1];

    double p2x = 0.0;
    double p2y = 0.0;

	for (size_t index = 0; index < n + 1; index += 2) {
	    p2x = polygon[mod(index, n)];
	    p2y = polygon[mod(index + 1, n)];

	    double min_y = p1y < p2y ? p1y : p2y;
	    double max_y = p1y < p2y ? p2y : p1y;
	    double max_x = p1x < p2x ? p1x : p2x;

        // if point is in bounds of line segment
        if (min_y < y && y < max_y && x <= max_x) {
	        double x_intersect = 0.0;
	        bool does_intersect = false;
	        if (p1y != p2y) {  // if not a horizontal line segment
                x_intersect = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x;
	        }

	        // if a vertical line segment or point has a intersection along sloped line
            if (p1x == p2x || (does_intersect && x <= x_intersect)) {
                // toggle inside boolean. Odd number of intersections means point is inside. Even number means outside.
                inside = !inside;
            }
	    }
	    p1x = p2x;
	    p1y = p2y;
	}

	return inside
}
