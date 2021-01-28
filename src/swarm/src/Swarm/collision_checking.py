from utility import *
import numpy as np


class Obstacle:
    def __init__(self):
        pass

    def find_closest_point(self, point):
        pass


class Cube(Obstacle):
    def __init__(self, center, cube_extends):
        super(Obstacle, self).__init__()
        self.center = center
        self.cube_extents = cube_extends

    def find_closest_point(self, point):
        return closest_point_on_cube(point, self.center, self.cube_extents)


class Sphere(Obstacle):
    def __init__(self, center, radius):
        super(Obstacle, self).__init__()
        self.center = center
        self.radius = radius

    def find_closest_point(self, point):
        return closest_point_on_sphere(point, self.center, self.radius)


class Cylinder(Obstacle):
    def __init__(self, center, half_height, radius):
        super(Obstacle, self).__init__()
        self.center = center
        self.half_height = half_height
        self.radius = radius

    def find_closest_point(self, point):
        return closest_point_on_cylinder(point, self.center, self.half_height, self.radius)


class CollisionChecking:
    instance = None

    @staticmethod
    def get_or_create():
        if CollisionChecking.instance is None:
            CollisionChecking.instance = CollisionChecking()
        return CollisionChecking.instance

    def __init__(self):
        self.obstacles = []
        pass

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def check_closest_point(self, point):
        global_closest_point = vec()
        global_closest_distance = 99999.9
        for obstacle in self.obstacles:
            closest_point = obstacle.find_closest_point(point)
            delta = closest_point - point
            distance = np.linalg.norm(delta)

            if distance < global_closest_distance:
                global_closest_point = closest_point
                global_closest_distance = distance
        return global_closest_point
