# modules/environment_sim/global_obstacles1.py

from modules.data_structures.point import Point
from modules.data_structures.obstacle import BaseObstacle, SphereObstacle, BoxObstacle
from typing import List

#КАРТА 2
def get_all_global_obstacles() -> List[SphereObstacle]:
    obstacles = [
        BoxObstacle(Point(1,5,0), Point(7,12,4)),
        BoxObstacle(Point(10, 9, 0), Point(17, 17, 7)),
        BoxObstacle(Point(20, 15, 0), Point(30, 20, 8)),
        SphereObstacle(Point(20, 24, 2), 2.0),  # Препятствие 1
        SphereObstacle(Point(24, 24, 2), 2.0),  # Препятствие 2
        SphereObstacle(Point(26, 22, 2), 2.0),  # Препятствие 3
        SphereObstacle(Point(10, 19, 2), 2.0),  # Препятствие 4
        SphereObstacle(Point(17, 7, 2), 2.0),  # Препятствие 5
    ]
    return obstacles