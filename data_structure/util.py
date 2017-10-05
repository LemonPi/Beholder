import math

import numpy as np


def cross(v, w):
    """
    Returns the "two dimensional vector cross product." Essentially the same as the three dimensional case, assuming z=0.
    :param v: The first vector in (x, y) form.
    :param w: The second vector in (x, y) form.
    :return:
    """
    return v[0] * w[1] - v[1] * w[0]


# Implemented based on https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/565282#565282.
def get_distance(p, h, l2):
    """
    Gets the distance along the ray starting from p, with heading h to the intersection with line segment l2.
    :param p: starting point
    :param h: heading
    :param l2: target line segment
    :return: distance from p, at heading h, to intersection with l2
    """
    p = np.array(p)
    h_rad = math.radians(h)
    r = np.array([math.sin(h_rad), -math.cos(h_rad)])

    q = np.array(l2[0])
    s = np.array(l2[1]) - q

    r_cross_s = cross(r, s)
    t = cross(q - p, s)
    u = cross(q - p, r)

    if r_cross_s == 0 and u == 0:
        # Lines are collinear.
        r_dot_r = np.dot(r, r)
        t0 = np.dot(q - p, r) / r_dot_r
        t0 = t0 if t0 >= 0 else math.inf
        t1 = t0 + np.dot(s, r) / r_dot_r
        t1 = t1 if t1 >= 0 else math.inf
        return min(t0, t1)
    if r_cross_s == 0 and u != 0:
        # Lines are parallel and non-intersecting.
        return math.inf
    t /= r_cross_s
    u /= r_cross_s
    if r_cross_s != 0 and t >= 0 and 0 <= u <= 1:
        # The ray intersects the line segment.
        assert np.linalg.norm((p + t * r) - (q + u * s)) < 1E-10
        intersection_point = p + t * r
        return np.linalg.norm(intersection_point - p)

    # No intersection
    return math.inf


if __name__ == '__main__':
    # Test cases
    # Should find an intersection.
    assert (get_distance([0, 0], 0, [[-1, -3], [10, -3]]) - 3) < 1E-10
    assert (get_distance([0, 0], 180, [[-1, 10], [10, 10]]) - 10) < 1E-10
    assert (get_distance([0, 0], 90, [[5, -1], [5, 10]]) - 5) < 1E-10

    # Make sure directionality works.
    assert get_distance([0, 0], 0, [[-1, 10], [10, 10]]) == math.inf
    assert get_distance([0, 0], 90, [[-1, 10], [10, 10]]) == math.inf

    # No intersection.
    assert get_distance([0, 0], 45, [[-1, 10], [10, 10]]) == math.inf
    assert get_distance([0, 0], 90, [[-1, 10], [10, 10]]) == math.inf

    # Parallel.
    assert get_distance([0, 0], 0, [[-1, 10], [-1, 20]]) == math.inf
    assert get_distance([0, 0], 90, [[10, 10], [-1, 10]]) == math.inf

    # Collinear.
    # Intersecting.
    assert (get_distance([0, 0], 0, [[0, -1], [0, -2]]) - 1) < 1E-10
    assert (get_distance([0, 0], 0, [[0, -3], [0, -2]]) - 2) < 1E-10
    # Not intersecting.
    assert get_distance([0, 0], 180, [[0, -2], [0, -1]]) == math.inf
    assert get_distance([0, 0], 180, [[0, -1], [0, -2]]) == math.inf
