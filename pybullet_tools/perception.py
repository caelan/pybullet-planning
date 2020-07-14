from itertools import combinations

import numpy as np

from pybullet_tools.utils import clip, unit_pose, tform_point, invert


def get_camera_matrix(width, height, fx, fy):
    # cx, cy = 320.5, 240.5
    cx, cy = width / 2., height / 2.
    return np.array([[fx, 0, cx],
                     [0, fy, cy],
                     [0, 0, 1]])


def clip_pixel(pixel, width, height):
    x, y = pixel
    return clip(x, 0, width-1), clip(y, 0, height-1)


def ray_from_pixel(camera_matrix, pixel):
    return np.linalg.inv(camera_matrix).dot(np.append(pixel, 1))


def pixel_from_ray(camera_matrix, ray):
    return camera_matrix.dot(np.array(ray) / ray[2])[:2]


def dimensions_from_camera_matrix(camera_matrix):
    width, height = 2 * np.array(camera_matrix)[:2, 2]
    return width, height


def is_visible_point(camera_matrix, depth, point_world, camera_pose=unit_pose()):
    point_camera = tform_point(invert(camera_pose), point_world)
    if not (0 <= point_camera[2] < depth):
        return False
    px, py = pixel_from_ray(camera_matrix, point_camera)
    width, height = dimensions_from_camera_matrix(camera_matrix)
    return (0 <= px < width) and (0 <= py < height)


def support_from_aabb(aabb):
    lower, upper = aabb
    min_x, min_y, z = lower
    max_x, max_y, _ = upper
    return [(min_x, min_y, z), (min_x, max_y, z),
            (max_x, max_y, z), (max_x, min_y, z)]

#####################################

def cone_vertices_from_base(base):
    return [np.zeros(3)] + base


def cone_wires_from_support(support):
    #vertices = cone_vertices_from_base(support)
    # TODO: could obtain from cone_mesh_from_support
    # TODO: could also just return vertices and indices
    apex = np.zeros(3)
    lines = []
    for vertex in support:
        lines.append((apex, vertex))
    #for i, v2 in enumerate(support):
    #    v1 = support[i-1]
    #    lines.append((v1, v2))
    for v1, v2 in combinations(support, 2):
        lines.append((v1, v2))
    center = np.average(support, axis=0)
    lines.append((apex, center))
    return lines


def cone_mesh_from_support(support):
    assert(len(support) == 4)
    vertices = cone_vertices_from_base(support)
    faces = [(1, 4, 3), (1, 3, 2)]
    for i in range(len(support)):
        index1 = 1+i
        index2 = 1+(i+1)%len(support)
        faces.append((0, index1, index2))
    return vertices, faces