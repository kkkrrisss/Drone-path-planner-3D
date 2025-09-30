# modules/environment_sim/radar_sensor.py

from modules.data_structures.point import Point
from modules.data_structures.obstacle import BaseObstacle, SphereObstacle, BoxObstacle
from typing import List

class RadarSensor:
    def __init__(self, radar_range: float):
        self.radar_range = radar_range

    def scan_for_obstacles(self, current_drone_position: Point,
                          all_global_obstacles: List[BaseObstacle]) -> List[BaseObstacle]:
        detected_obstacles = []
        for obs in all_global_obstacles:
            if obs.distance_to_point(current_drone_position) <= self.radar_range:
                detected_obstacles.append(obs)
        return detected_obstacles