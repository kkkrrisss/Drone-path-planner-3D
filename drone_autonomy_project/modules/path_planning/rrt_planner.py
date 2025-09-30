# modules/path_planning/rrt_planner.py

import random
import math
from typing import List, Optional

from modules.data_structures.point import Point
from modules.data_structures.path import Path
from modules.data_structures.obstacle import BaseObstacle, SphereObstacle, BoxObstacle
from config import settings
from utils.geometry_utils import calculate_distance, get_vector_direction

class RRTNode:
    # Узел дерева RRT
    def __init__(self, position: Point, parent: Optional['RRTNode'] = None):
        self.position = position
        self.parent = parent # ссылка на родительский узел (чтобы потом восстановить путь)
        self.cost = 0.0      # суммарная стоимость пути до этого узла

        if parent:
            # если есть родитель, добавляем длину сегмента к его стоимости
            self.cost = parent.cost + calculate_distance(parent.position, self.position)

    def __repr__(self):
        return f"RRTNode({self.position.x:.2f}, {self.position.y:.2f}, {self.position.z:.2f})"

class RRTPlanner:
    """
    Класс для планирования пути с помощью Rapidly-exploring Random Tree (RRT) в 3D.
    По сути, строим случайное дерево, которое "разрастается" в сторону цели,
    пока не доберёмся до неё или не исчерпаем итерации.
    """
    def __init__(self,
                 max_iterations: int = 2000, # сколько максимум итераций даём алгоритму
                 step_size: float = 1.0,     # длина одного шага дерева
                 goal_sample_rate: float = 0.05, #вероятность "подсмотреть" напрямую цель
                 goal_tolerance: float = 0.8,    #радиус, в котором считаем цель достигнутой
                 drone_radius: float = settings.DRONE_RADIUS):
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.goal_tolerance = goal_tolerance
        self.drone_radius = drone_radius
        self.world_bounds = settings.WORLD_BOUNDS

    def _get_random_point(self) -> Point:
        """Берём случайную точку в пределах мира"""
        x = random.uniform(self.world_bounds['x'][0], self.world_bounds['x'][1])
        y = random.uniform(self.world_bounds['y'][0], self.world_bounds['y'][1])
        z = random.uniform(self.world_bounds['z'][0], self.world_bounds['z'][1])
        return Point(x, y, z)

    def _get_nearest_node(self, nodes: List[RRTNode], sample_point: Point) -> RRTNode:
        """Находим ближайший к случайной точке узел дерева"""
        min_dist = float('inf')
        nearest_node = None
        for node in nodes:
            dist = calculate_distance(node.position, sample_point)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def _steer(self, from_point: Point, to_point: Point) -> Point:
        """
        Двигаемся от from_point к to_point, но не больше чем на step_size
        То есть "обрезаем" направление на фиксированный шаг.
        """
        direction = to_point - from_point
        distance = direction.magnitude()

        if distance < self.step_size:
            return to_point #если точка рядом, берём её напрямую
        else:
            return from_point + direction.normalize() * self.step_size

    def _check_collision(self, p1: Point, p2: Point, obstacles: List[BaseObstacle]) -> bool:
        """
        Проверка, пересекает ли отрезок (p1 -> p2) какие-либо препятствия
        Для упрощения проверяем несколько точек на линии
        """
        num_checks = 10
        for i in range(num_checks + 1):
            t = i / num_checks
            interp_point = Point(
                p1.x + t * (p2.x - p1.x),
                p1.y + t * (p2.y - p1.y),
                p1.z + t * (p2.z - p1.z)
            )
            for obs in obstacles:
                if obs.is_collision(interp_point, self.drone_radius):
                    return True #нашли пересечение
        return False # свободный сегмент

    def plan_path(self, start_point: Point, target_point: Point,
                  known_obstacles: List[BaseObstacle]) -> Optional[Path]:
        """
        Главная функция — строим путь от start_point до target_point.
        Если путь найден, возвращаем Path; иначе None.
        """
        start_node = RRTNode(start_point)
        nodes = [start_node]
        goal_node = None

        print(f"RRT: Планирование от {start_point} к {target_point}, препятствий: {len(known_obstacles)}")

        for i in range(self.max_iterations):
            #  1. выбираем случайную точку (иногда цель напрямую)
            if random.random() < self.goal_sample_rate:
                sample_point = target_point
            else:
                sample_point = self._get_random_point()

            #  2. ищем ближайший узел в дереве
            nearest_node = self._get_nearest_node(nodes, sample_point)

            #  3. делаем шаг от него к sample_point
            new_position = self._steer(nearest_node.position, sample_point)

            #  4. проверяем коллизию
            if not self._check_collision(nearest_node.position, new_position, known_obstacles):
                new_node = RRTNode(new_position, nearest_node)
                nodes.append(new_node)

                #  5. проверяем, достигли ли цели
                if calculate_distance(new_node.position, target_point) < self.goal_tolerance:
                    # если почти у цели, пробуем напрямую дотянуться до неё
                    if not self._check_collision(new_node.position, target_point, known_obstacles):
                        goal_node = RRTNode(target_point, new_node) # финальный узел
                        nodes.append(goal_node)
                        break # путь найден

            if (i + 1) % 100 == 0:
                print(f"RRT: Итерация {i+1}/{self.max_iterations}, узлов в дереве: {len(nodes)}")

        #  Восстановление пути
        if goal_node:
            path_waypoints = []
            current = goal_node
            while current:
                path_waypoints.append(current.position)
                current = current.parent
            path_waypoints.reverse() # идём от цели к старту, поэтому разворачиваем
            print(f"RRT: Путь найден! Вейпоинтов: {len(path_waypoints)}")
            return Path(path_waypoints)
        else:
            print("RRT: Путь не найден (итерации закончились).")
            return None
