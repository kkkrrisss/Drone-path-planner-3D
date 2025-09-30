# utils/geometry_utils.py

from modules.data_structures.point import Point
from modules.data_structures.obstacle import BaseObstacle, SphereObstacle, BoxObstacle
from typing import List

def calculate_distance(point1: Point, point2: Point) -> float:
    #Евклидово расстояние между двумя 3D точками
    return point1.distance_to(point2)

def check_collision_point_obstacle(point: Point, obstacle: BaseObstacle, point_radius: float = 0.0) -> bool:
    #Проверка столкновения между точкой (с радиусом) и сферическим препятствием.
    return obstacle.is_collision(point, point_radius)

def get_vector_direction(start_point: Point, end_point: Point) -> Point:
    #возвращает нормализованный вектор направления от start_point к end_point
    direction = end_point - start_point
    return direction.normalize()