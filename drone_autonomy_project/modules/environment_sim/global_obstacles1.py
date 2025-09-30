# modules/environment_sim/global_obstacles1.py

from modules.data_structures.point import Point
from modules.data_structures.obstacle import BaseObstacle, SphereObstacle, BoxObstacle
from typing import List

#КАРТА 1
#возвращает список всех препятствий в мире
#это "истинные" препятствия, о которых дрон изначально не знает
def get_all_global_obstacles() -> List[SphereObstacle]:
    obstacles = [
        SphereObstacle(Point(2, 3, 2), 0.8),    # Препятствие 1
        SphereObstacle(Point(-4, -4, 1), 1.0),  # Препятствие 2
        SphereObstacle(Point(0, 0, 3.5), 0.7),  # Препятствие 3 (над стартом)
        SphereObstacle(Point(7, 7, 2), 0.9),    # Препятствие 4 (ближе к цели)
        SphereObstacle(Point(-8, 5, 1.5), 0.6),  # Препятствие 5
        BoxObstacle(Point(-6,-6,1), Point(-3,-3,2))
    ]
    return obstacles