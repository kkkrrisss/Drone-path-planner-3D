# modules/data_structures/path.py

from typing import List
from .point import Point

#последовательность 3D-точек (вейпоинтов), образующих путь
class Path:
    def __init__(self, waypoints: List[Point] = None):
        if waypoints is None:
            self.waypoints = []
        else:
            if not all(isinstance(wp, Point) for wp in waypoints):
                raise TypeError("All waypoints in the path must be Point objects.")
            self.waypoints = list(waypoints)

        self._current_waypoint_idx = 0

    def add_waypoint(self, waypoint: Point):
        if not isinstance(waypoint, Point):
            raise TypeError("Waypoint must be a Point object.")
        self.waypoints.append(waypoint)

    #Возвращает текущий вейпоинт, который дрон должен достичь
    def get_current_waypoint(self) -> Point | None:
        if not self.waypoints:
            return None
        if self._current_waypoint_idx < len(self.waypoints):
            return self.waypoints[self._current_waypoint_idx]
        return self.waypoints[-1] # Если достигнут конец пути, остаемся на последней точке

    #Перемещает указатель на следующий вейпоинт, если текущий достигнут
    def advance_to_next_waypoint(self):
        if self._current_waypoint_idx < len(self.waypoints) - 1:
            self._current_waypoint_idx += 1
            return True
        return False # Достигнут последний вейпоинт

    def reset_progress(self):
        self._current_waypoint_idx = 0

    def __len__(self):
        return len(self.waypoints)

    def __getitem__(self, index):
        return self.waypoints[index]

    def __str__(self):
        if not self.waypoints:
            return "Empty Path"
        return f"Path with {len(self.waypoints)} waypoints. Current: {self.get_current_waypoint()}"

    def __iter__(self):
        return iter(self.waypoints)