from utility import *
import numpy as np


def Clamp(value, min_value, max_value):
    if value > max_value:
        value = max_value
    if value < min_value:
        value = min_value
    return value


def closest_point_on_cube(point, center, cube_extents):
    delta = point - center
    delta[0] = Clamp(delta[0], -cube_extents[0], cube_extents[0])
    delta[1] = Clamp(delta[1], -cube_extents[1], cube_extents[1])
    delta[2] = Clamp(delta[2], -cube_extents[2], cube_extents[2])
    return center + delta


def closest_point_on_sphere(point, center, radius):
    delta = point - center
    magnitude = np.linalg.norm(delta)
    normalized = delta / magnitude
    return center + radius * normalized


# vertical cylinder
def closest_point_on_cylinder(point, center, half_height, radius):
    delta = point - center

    delta_on_plane = delta.copy()
    delta_on_plane[2] = 0.0
    magnitude = np.linalg.norm(delta_on_plane)
    normalized = delta_on_plane / magnitude
    if magnitude > radius:
        delta_on_plane = normalized * radius

    closest_point_height_delta = Clamp(delta[2], -half_height, half_height)

    closest_point = center + closest_point_height_delta * vec(0,0,1) + delta_on_plane
    return closest_point



print(closest_point_on_cylinder(vec(), vec(10,10,6), 5, 5))

