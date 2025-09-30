# modules/environment_sim/dynamic_obstacle_map.py

from modules.data_structures.point import Point
from modules.data_structures.obstacle import BaseObstacle, SphereObstacle, BoxObstacle
from typing import List, Set

#Представляет карту препятствий, которую дрон построил на основе обнаруженных данных
#сначала пустая и пополняется по мере обнаружения
class DynamicObstacleMap:
    def __init__(self):
        self._known_obstacles: Set[BaseObstacle] = set()

    #добавляем препятсвие
    def add_obstacle(self, obstacle: BaseObstacle):
        self._known_obstacles.add(obstacle)
    #обновление на оснве радара
    def update_from_radar(self, detected_obstacles: List[BaseObstacle]):
        for obs in detected_obstacles:
            self.add_obstacle(obs)

    #Возвращает список всех известных дрону препятствий
    def get_known_obstacles(self) -> List[BaseObstacle]:
        return list(self._known_obstacles)

    #
    def check_collision(self, point: Point, object_radius: float) -> bool:
        for obs in self._known_obstacles:
            if obs.is_collision(point, object_radius):
                return True
        return False

    def check_collision_along_line(self, p1, p2, num_samples=20):
        for i in range(num_samples + 1):
            alpha = i / num_samples
            x = p1.x * (1 - alpha) + p2.x * alpha
            y = p1.y * (1 - alpha) + p2.y * alpha
            z = p1.z * (1 - alpha) + p2.z * alpha
            point = type(p1)(x, y, z)
            if self.check_collision(point, object_radius=0):
                return True
        return False