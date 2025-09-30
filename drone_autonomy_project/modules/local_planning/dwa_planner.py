# modules/local_planning/dwa_planner.py

import math
import numpy as np
from typing import List, Tuple, Optional

from modules.data_structures.point import Point
import time # добавил импорт, если вдруг захочу профилировать или замерять время
from modules.data_structures.obstacle import BaseObstacle, SphereObstacle, BoxObstacle
from modules.drone_state.state import DroneState
from modules.drone_state.motion_model import DroneMotionModel
from config import settings
from utils.geometry_utils import calculate_distance


class DWAPlanner:
    """
    Реализация алгоритма Dynamic Window Approach (DWA),
    но адаптированного для 3D (дрон может летать в xyz, а не только в 2D).
    """
    def __init__(self):
        # базовые параметры, которые берём из конфигов
        self.dt = settings.SIMULATION_DT  # шаг симуляции
        self.predict_time = settings.DWA_PREDICT_TIME  # насколько далеко вперёд предсказываем траектории
        self.heading_weight = settings.DWA_HEADING_WEIGHT  # вес "направленности" к цели
        self.distance_weight = settings.DWA_DISTANCE_WEIGHT  # вес "дистанции до препятствий"
        self.velocity_weight = settings.DWA_VELOCITY_WEIGHT  # вес "скорости"
        self.drone_radius = settings.DRONE_RADIUS
        self.radar_range = settings.RADAR_RANGE
        self.world_bounds = settings.WORLD_BOUNDS

        self.motion_model = DroneMotionModel()

        # разрешение сетки по линейным и угловым скоростям
        self.lin_vel_res = settings.DWA_VELOCITY_RESOLUTION_LINEAR
        self.ang_vel_res = settings.DWA_VELOCITY_RESOLUTION_ANGULAR

    def _generate_dynamic_window(self, current_vel: Point) -> List[Point]:
        """
        Генерация "динамического окна" — набора возможных линейных скоростей (vx, vy, vz)
        В классическом DWA это обычно (v, ω) для робота на плоскости,
        но здесь у нас дрон, так что упрощённая версия для трёх осей
        """
        possible_velocities = []

        #ограничение на изменение скорости за один шаг по акселерации
        accel_limit = settings.MAX_LINEAR_ACCELERATION * self.dt

        # считаем диапазоны для каждой оси (мин/макс скорости)
        min_vx = max(-self.motion_model.max_lin_vel, current_vel.x - accel_limit)
        max_vx = min(self.motion_model.max_lin_vel, current_vel.x + accel_limit)
        min_vy = max(-self.motion_model.max_lin_vel, current_vel.y - accel_limit)
        max_vy = min(self.motion_model.max_lin_vel, current_vel.y + accel_limit)
        min_vz = max(-self.motion_model.max_lin_vel, current_vel.z - accel_limit)
        max_vz = min(self.motion_model.max_lin_vel, current_vel.z + accel_limit)

        # дискретизируем диапазоны скоростей
        vx_values = np.linspace(min_vx, max_vx, self.lin_vel_res) if self.lin_vel_res > 1 else [current_vel.x]
        vy_values = np.linspace(min_vy, max_vy, self.lin_vel_res) if self.lin_vel_res > 1 else [current_vel.y]
        vz_values = np.linspace(min_vz, max_vz, self.lin_vel_res) if self.lin_vel_res > 1 else [current_vel.z]

        # собираем все возможные комбинации
        for vx in vx_values:
            for vy in vy_values:
                for vz in vz_values:
                    potential_vel = Point(vx, vy, vz)
                    # ограничиваем скорость по модулю
                    if potential_vel.magnitude() <= self.motion_model.max_lin_vel:
                        possible_velocities.append(potential_vel)

        return possible_velocities

    def _predict_trajectory(self, current_state: DroneState, desired_velocity: Point) -> List[Point]:
        """
        Строим предсказанную траекторию для выбранной скорости
        Просто симулируем движение по шагам dt до predict_time
        """
        trajectory = [current_state.get_position()]
        temp_state = current_state  # копия текущего состояния (без deepcopy, но тут ок)

        steps = int(self.predict_time / self.dt)
        for _ in range(steps):
            temp_state = self.motion_model.predict_next_state(temp_state, desired_velocity)
            trajectory.append(temp_state.get_position())
        return trajectory

    def _evaluate_trajectory(self, trajectory: List[Point], current_state: DroneState,
                             target_point: Point, known_obstacles: List[BaseObstacle]) -> float:
        """
        Оценка траектории по трём критериям:
        - направленность к цели (heading)
        - безопасность от препятствий (distance)
        - скорость (velocity)
        """
        if not trajectory:
            return -float('inf')  # траектория пустая → сразу в мусор

        # --- 1. Heading cost ---
        final_pos = trajectory[-1]
        vec_to_target = target_point - final_pos

        if len(trajectory) > 1:
            trajectory_direction = final_pos - trajectory[-2]
        else:
            trajectory_direction = current_state.get_velocity()
            if trajectory_direction.magnitude() == 0:
                trajectory_direction = (target_point - current_state.get_position()).normalize()
                if trajectory_direction.magnitude() == 0:
                    return 0.0  # дрон уже в цели

        if vec_to_target.magnitude() == 0 or trajectory_direction.magnitude() == 0:
            heading_cost = 0.0
        else:
            # считаем косинус угла между направлением траектории и вектором к цели
            dot_product = vec_to_target.dot(trajectory_direction)
            magnitude_product = vec_to_target.magnitude() * trajectory_direction.magnitude()
            cosine_angle = dot_product / magnitude_product
            heading_cost = 0.5 * (cosine_angle + 1.0)  # нормируем в [0, 1]

        # --- 2. Distance cost ---
        min_dist_to_obstacle = float('inf')
        collision_detected = False
        for pos_on_traj in trajectory:
            for obs in known_obstacles:
                if obs.is_collision(pos_on_traj, point_radius=self.drone_radius):
                    collision_detected = True
                    break
                dist = obs.distance_to_point(pos_on_traj) - self.drone_radius
                min_dist_to_obstacle = min(min_dist_to_obstacle, dist)
            if collision_detected:
                break

        if collision_detected:
            distance_cost = -float('inf')
        elif min_dist_to_obstacle == float('inf'):
            distance_cost = 1.0
        else:
            distance_cost = min(max(min_dist_to_obstacle, 0.0) / self.radar_range, 1.0)

        # --- 3. Velocity cost ---
        velocity_magnitude = current_state.get_velocity().magnitude()
        velocity_cost = velocity_magnitude / settings.MAX_LINEAR_VELOCITY

        # --- Итог ---
        if collision_detected:
            return -float('inf')

        score = (self.heading_weight * heading_cost +
                 self.distance_weight * distance_cost +
                 self.velocity_weight * velocity_cost)

        return score

    def find_best_velocity(self, current_state: DroneState, target_point: Point,
                           known_obstacles: List[BaseObstacle]) -> Optional[Point]:
        """
        Основная функция: находим оптимальную скорость для дрона.
        """
        # генерим кандидатов
        possible_velocities = self._generate_dynamic_window(current_state.get_velocity())

        best_score = -float('inf')
        best_velocity = None

        if not possible_velocities:
            # если совсем нет вариантов, то останавливаемся
            return Point(0, 0, 0)

        # прогоняем все кандидаты через предсказание и оценку
        for desired_vel in possible_velocities:
            trajectory = self._predict_trajectory(current_state, desired_vel)
            score = self._evaluate_trajectory(trajectory, current_state, target_point, known_obstacles)

            if score > best_score:
                best_score = score
                best_velocity = desired_vel

        return best_velocity
