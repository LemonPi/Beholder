import math

import numpy as np
np.seterr(divide='ignore', invalid='ignore')

def round_to_base(x, base):
    return base * round(float(x)/base)

def cross(v, w):
    """
    Returns the "two dimensional vector cross product." Essentially the same as the three dimensional case, assuming z=0.
    :param v: The first vector in (x, y) form.
    :param w: The second vector in (x, y) form.
    :return:
    """
    return v[0] * w[1] - v[1] * w[0]

def dot2d(v, w):
    """
    Returns the "two dimensional vector dot product."
    :param v: The first vector in (x, y) form.
    :param w: The second vector in (x, y) form.
    :return:
    """
    return v[0] * w[0] + v[1] * w[1]


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
    
    # TODO: Figure out how to vectorize this
    r = np.array([math.sin(h), -math.cos(h)])

    q = np.array(l2[0])
    s = np.array(l2[1]) - q

    r_cross_s = cross(r, s)
    t = cross(q - p, s)
    u = cross(q - p, r)

    if r_cross_s == 0 and u == 0:
        # Lines are collinear.
        r_dot_r = dot2d(r, r)
        t0 = dot2d(q - p, r) / r_dot_r
        t0 = t0 if t0 >= 0 else math.inf
        t1 = t0 + dot2d(s, r) / r_dot_r
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

def get_distance_rectilinear(p, h, l2):
    """
    Gets the distance along the ray starting from p, with heading h to the intersection with line segment l2.
    :param p: starting points
    :param h: headings
    :param p1: target line segments
    :return: distance from p, at heading h, to intersection with l2
    """

    n = p.shape[1]
    m = l2.shape[2]

    #print('p', p.shape, 'w', l2.shape)

    # p elem R(2xn)
    # l2 elem R(2x2xm), m is the number of line segments
    delta = l2[:, 1] - l2[:, 0]
    dir_wall = delta / np.sum(delta, axis=0) # (2xm)
    dir_perp = 1 - dir_wall    # (2xm)

    w0 = l2[:, 0]
    
    p_l = np.expand_dims(w0, axis=2) - np.expand_dims(p, axis=1)    # (2xmxn)
    p_l_perp = np.sum(np.transpose(np.multiply(np.transpose(dir_perp), np.transpose(p_l))), axis=0, keepdims=False)  # (mxn)

    r = np.array([np.sin(np.pi/2 - h), np.cos(np.pi/2 - h)])    # (2xn)
    
    dist_mat = np.divide(p_l_perp, np.transpose(dir_wall) @ r)   # (mxn)
    
    # Determine whether the crossing is actually at the right spot
    p_l_parallel = np.sum(np.transpose(np.multiply(np.transpose(dir_wall), np.transpose(p_l))), axis=0)  # (mxn)
    delta_parallel = np.transpose(np.sum(delta, axis=0, keepdims=True))  # (m)

    zero_dist = np.zeros_like(p_l_parallel)
    intersects_pos = np.logical_and(np.greater_equal(p_l_parallel, zero_dist), np.less_equal(p_l_parallel, delta_parallel))
    intersects_neg = np.logical_and(np.less_equal(p_l_parallel, zero_dist), np.greater_equal(p_l_parallel, -delta_parallel))
    intersects = np.logical_or(intersects_pos, intersects_neg)
    
    # Make all non-intersecting distances inf
    dist_mat[intersects == 0] = np.inf
    dist_mat[dist_mat < 0] = np.inf
    dist_mat[dist_mat > 10] = np.inf

    # Min distances accross wall axis to find the smallest distance
    min_dists = np.min(dist_mat, axis = 0)

    return min_dists

def vectorized_raycast(p, h, segment_p1, segment_p2):
    q = segment_p1
    s = segment_p2 - segment_p1

    r_cross_s = np.cross(r, s)
    t = np.cross(q - p, s)
    u = np.cross(q - p, r)


if __name__ == '__main__':
    # Test cases
    # Should find an intersection.
    #assert (get_distance([0, 0], 0, [[-1, -3], [10, -3]]) - 3) < 1E-10
    #assert (get_distance([0, 0], 180, [[-1, 10], [10, 10]]) - 10) < 1E-10
    #assert (get_distance([0, 0], 90, [[5, -1], [5, 10]]) - 5) < 1E-10

    # # Make sure directionality works.
    # assert get_distance([0, 0], 0, [[-1, 10], [10, 10]]) == math.inf
    # assert get_distance([0, 0], 90, [[-1, 10], [10, 10]]) == math.inf

    # # No intersection.
    # assert get_distance([0, 0], 45, [[-1, 10], [10, 10]]) == math.inf
    # assert get_distance([0, 0], 90, [[-1, 10], [10, 10]]) == math.inf

    # # Parallel.
    # assert get_distance([0, 0], 0, [[-1, 10], [-1, 20]]) == math.inf
    # assert get_distance([0, 0], 90, [[10, 10], [-1, 10]]) == math.inf

    # # Collinear.
    # # Intersecting.
    # assert (get_distance([0, 0], 0, [[0, -1], [0, -2]]) - 1) < 1E-10
    # assert (get_distance([0, 0], 0, [[0, -3], [0, -2]]) - 2) < 1E-10
    # # Not intersecting.
    # assert get_distance([0, 0], 180, [[0, -2], [0, -1]]) == math.inf
    # assert get_distance([0, 0], 180, [[0, -1], [0, -2]]) == math.inf

    # TEST RECTALINEAR CASE
    N = 1
    M = 10
    wall_list = [[[i+1,-5], [i+1, 5]] for i in range(M)]
    particle_list = [[0, 0]]*N
    
    particles = np.transpose(np.array(particle_list))
    headings = np.array([0,]*N)
    walls = np.transpose(np.array(wall_list))
    print(get_distance_rectilinear(particles, headings, walls))

    # NO INTERSECTION
    particle_list = [[0, 0]]
    
    particles = np.transpose(np.array(particle_list))
    headings = np.array([np.pi,])
    walls = np.expand_dims(np.transpose(np.array([[-1, -5], [-1, 5]])), axis=2)
    print(get_distance_rectilinear(particles, headings, walls))
