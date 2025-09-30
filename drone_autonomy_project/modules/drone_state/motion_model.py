# modules/drone_state/motion_model.py

from modules.data_structures.point import Point
from modules.drone_state.state import DroneState
from config import settings
import math


class DroneMotionModel:
    """Кинематическая модель движения дрона в 3D.
    Обновляет состояние дрона на основе текущей скорости и времени.
    Учитывает максимальные скорости и ускорения.
    """
    def __init__(self):
        self.max_lin_vel = settings.MAX_LINEAR_VELOCITY
        self.max_ang_vel = settings.MAX_ANGULAR_VELOCITY  # Пока не используется напрямую для 3D
        self.max_lin_accel = settings.MAX_LINEAR_ACCELERATION
        self.max_ang_accel = settings.MAX_ANGULAR_ACCELERATION  # Пока не используется напрямую для 3D
        self.dt = settings.SIMULATION_DT
        self.velocity_decay = settings.VELOCITY_DECAY

    def predict_next_state(self, current_state: DroneState, desired_velocity: Point) -> DroneState:
        """
        Предсказывает следующее состояние дрона на основе желаемой скорости и текущего состояния.
        Учитывает кинематические ограничения и шаг симуляции.
        """
        current_pos = current_state.get_position()
        current_vel = current_state.get_velocity()  # Это теперь будет "линейная скорость"

        # Ограничение желаемой скорости по максимальной
        desired_velocity_magnitude = desired_velocity.magnitude()
        if desired_velocity_magnitude > self.max_lin_vel:
            desired_velocity = desired_velocity.normalize() * self.max_lin_vel

        # Расчет нового ускорения (для простоты - это не полная динамическая модель)
        # Просто стремимся к желаемой скорости, ограниченной ускорением.
        velocity_change = desired_velocity - current_vel
        acceleration_vector = Point(
            min(abs(velocity_change.x / self.dt), self.max_lin_accel) * math.copysign(1,
                                                                                      velocity_change.x) if self.dt > 0 else 0,
            min(abs(velocity_change.y / self.dt), self.max_lin_accel) * math.copysign(1,
                                                                                      velocity_change.y) if self.dt > 0 else 0,
            min(abs(velocity_change.z / self.dt), self.max_lin_accel) * math.copysign(1,
                                                                                      velocity_change.z) if self.dt > 0 else 0
        )

        # Обновление скорости с учетом ускорения
        new_vel_x = current_vel.x + acceleration_vector.x * self.dt
        new_vel_y = current_vel.y + acceleration_vector.y * self.dt
        new_vel_z = current_vel.z + acceleration_vector.z * self.dt

        # Применение затухания скорости
        new_vel_x *= self.velocity_decay
        new_vel_y *= self.velocity_decay
        new_vel_z *= self.velocity_decay

        # Ограничение новой скорости по максимальной
        new_vel = Point(new_vel_x, new_vel_y, new_vel_z)
        new_vel_magnitude = new_vel.magnitude()
        if new_vel_magnitude > self.max_lin_vel:
            new_vel = new_vel.normalize() * self.max_lin_vel

        # Обновление позиции
        new_pos_x = current_pos.x + new_vel.x * self.dt
        new_pos_y = current_pos.y + new_vel.y * self.dt
        new_pos_z = current_pos.z + new_vel.z * self.dt

        new_position = Point(new_pos_x, new_pos_y, new_pos_z)

        # Создаем новое состояние для предсказания
        predicted_state = DroneState(new_position, new_vel)
        return predicted_state