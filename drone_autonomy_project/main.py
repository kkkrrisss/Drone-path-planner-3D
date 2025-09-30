# main.py

import time
import math
import matplotlib.pyplot as plt

from typing import Optional

from modules.data_structures.point import Point
from modules.data_structures.obstacle import BaseObstacle, SphereObstacle, BoxObstacle
from modules.data_structures.path import Path
from modules.drone_state.state import DroneState
from modules.path_planning.path_smoothing import smooth_path
from modules.path_planning.path_utils import add_intermediate_points

# Новые импорты
from modules.drone_state.motion_model import DroneMotionModel  # <--- НОВЫЙ ИМПОРТ
from modules.local_planning.dwa_planner import DWAPlanner  # <--- НОВЫЙ ИМПОРТ

from modules.environment_sim.global_obstacles2 import get_all_global_obstacles
from modules.environment_sim.dynamic_obstacle_map import DynamicObstacleMap
from modules.environment_sim.radar_sensor import RadarSensor
from modules.visualization.plotter_3d import Plotter3D
from modules.path_planning.rrt_planner import RRTPlanner, RRTNode

from utils.geometry_utils import calculate_distance, get_vector_direction

from config import settings

#cтарт и финиш
START_POINT = Point(0, 0, 0.1)
TARGET_POINT = Point(30, 30, 5)

drone_state = DroneState(initial_position=START_POINT)

#инициализация глобальных препятствий
all_global_obstacles = get_all_global_obstacles()

#инициализация динамической карты препятствий(изначально пустая для дрона)
dynamic_obstacle_map = DynamicObstacleMap()

#симулятор радара
radar_sensor = RadarSensor(settings.RADAR_RANGE)

#планировщик RRT
rrt_planner = RRTPlanner(
    max_iterations=2000,
    step_size=1.0,
    goal_sample_rate=0.05,
    goal_tolerance=0.8,
    drone_radius=settings.DRONE_RADIUS
)

#модель движения дрона
drone_motion_model = DroneMotionModel()  # <--- ИНИЦИАЛИЗАЦИЯ

#DWA планировщика
dwa_planner = DWAPlanner()  # <--- ИНИЦИАЛИЗАЦИЯ

#визуализатор
plotter = None
if settings.ENABLE_VISUALIZATION:
    plotter = Plotter3D(START_POINT, TARGET_POINT, all_global_obstacles)

#глобальный путь
global_path: Optional[Path] = None
last_replan_position: Point = drone_state.get_position()
REPLAN_THRESHOLD = 2.0
REPLAN_INTERVAL_SEC = 5.0
last_replan_time = -float('inf')

#Основной цикл симуляции
current_time = 0.0
print(f"Симуляция начата. Старт: {START_POINT}, Цель: {TARGET_POINT}")
print(f"Дрон находится в: {drone_state.get_position()}")

while current_time < settings.SIMULATION_DURATION:
    #1) обнаружение препятствий с помощью симулятора радара
    nearby_obstacles = radar_sensor.scan_for_obstacles(
        drone_state.get_position(),
        all_global_obstacles
    )

    #2) обновление динамической карты препятствий дрона
    dynamic_obstacle_map.update_from_radar(nearby_obstacles)

    #3)проверка на столкновение с УЖЕ ИЗВЕСТНЫМИ препятствиями
    if dynamic_obstacle_map.check_collision(drone_state.get_position(), settings.DRONE_RADIUS):
        print(f"!!! СТОЛКНОВЕНИЕ НА {current_time:.2f}с !!! Дрон в: {drone_state.get_position()}")
        break

    #4) Глобальное планирование/Перепланирование
    current_drone_pos = drone_state.get_position()
    distance_from_last_replan = calculate_distance(current_drone_pos, last_replan_position)

    replan_needed = False
    if global_path is None:
        replan_needed = True
        print("RRT: Путь отсутствует, начинаю первичное планирование.")
    elif distance_from_last_replan > REPLAN_THRESHOLD:
        replan_needed = True
        print(
            f"RRT: Дрон отошел на {distance_from_last_replan:.2f}м от точки последнего планирования, перепланирование.")
    # elif current_time - last_replan_time > REPLAN_INTERVAL_SEC and len(dynamic_obstacle_map.get_known_obstacles()) > 0:
    #     replan_needed = True
    #     print(f"RRT: Прошло {REPLAN_INTERVAL_SEC}с, перепланирование для обновления карты препятствий.")

    if replan_needed:
        #RRT планирует до цели, используя известные препятствия
        new_path = rrt_planner.plan_path(current_drone_pos, TARGET_POINT, dynamic_obstacle_map.get_known_obstacles())
        if new_path:
            #ДОБАВЛЯЕМ СГЛАЖИВАНИЕ
            path_points = new_path.waypoints
            smoothed_points = smooth_path(path_points, dynamic_obstacle_map)
            smoothed_points = add_intermediate_points(smoothed_points, max_segment_length=1.0)
            from modules.data_structures.path import Path

            smoothed_path = Path(smoothed_points)
            #используем сглаженный путь как глобальный
            global_path = smoothed_path
            global_path.reset_progress()
            last_replan_position = current_drone_pos
            last_replan_time = current_time
            print(f"RRT: Путь сглажен. Количество точек до: {len(path_points)}, после: {len(smoothed_points)}")
        else:
            print("RRT: Не удалось найти путь. Дрон стоит на месте.")
            drone_state.update_state(drone_state.get_position(), Point(0, 0, 0))
            current_time += settings.SIMULATION_DT
            if plotter:
                plotter.update_plot(drone_state.get_position(), global_path, dynamic_obstacle_map.get_known_obstacles())
            continue

    #если глобального пути все еще нет после попытки планирования, прекращаем
    if global_path is None:
        print("RRT: Нет глобального пути, симуляция остановлена.")
        break

    #5) локальное избегание препятствий и получение желаемой скорости (DWA)
    # Целевой вейпоинт для DWA - это следующий вейпоинт из глобального пути
    current_target_waypoint = global_path.get_current_waypoint()
    if current_target_waypoint is None:
        current_target_waypoint = TARGET_POINT  #Если RRT вдруг не дал вейпоинт

    #DWA выбирает оптимальную скорость, основываясь на текущем состоянии,
    #следующем вейпоинте глобального пути и обнаруженных препятствиях
    desired_velocity_vector = dwa_planner.find_best_velocity(
        drone_state,
        current_target_waypoint,  #DWA ведет к следующему вейпоинту глобального пути
        dynamic_obstacle_map.get_known_obstacles()
    )

    if desired_velocity_vector is None:
        print("DWA: Не удалось найти оптимальную скорость. Дрон стоит на месте.")
        desired_velocity_vector = Point(0, 0, 0)  #если DWA не нашел решения, остановиться

    #6) Обновление состояния дрона с использованием MOTION_MODEL
    new_drone_state = drone_motion_model.predict_next_state(drone_state, desired_velocity_vector)

    drone_state.update_state(new_drone_state.get_position(), new_drone_state.get_velocity())

    #проверка границ мира после обновления
    current_pos_clamped = Point(
        max(min(drone_state.position.x, settings.WORLD_BOUNDS['x'][1]), settings.WORLD_BOUNDS['x'][0]),
        max(min(drone_state.position.y, settings.WORLD_BOUNDS['y'][1]), settings.WORLD_BOUNDS['y'][0]),
        max(min(drone_state.position.z, settings.WORLD_BOUNDS['z'][1]), settings.WORLD_BOUNDS['z'][0])
    )
    drone_state.update_state(current_pos_clamped, drone_state.get_velocity())

    #7) проверка, достиг ли дрон текущего вейпоинта глобального пути
    #Если DWA успешно приведет дрон к текущему вейпоинту, переходим к следующему
    if calculate_distance(drone_state.get_position(), current_target_waypoint) < rrt_planner.goal_tolerance:
        if global_path.advance_to_next_waypoint():
            print(f"Достигнут вейпоинт, следующий: {global_path.get_current_waypoint()}")
        #если это был последний вейпоинт, но не цель, просто продолжаем двигаться к цели
        elif drone_state.is_at_target(TARGET_POINT):  # если последний вейпоинт это цель
            print(f"Дрон достиг финальной цели за {current_time:.2f} секунд.")
            break

    # 8) Визуализация
    if plotter:
        plotter.update_plot(drone_state.get_position(), global_path, dynamic_obstacle_map.get_known_obstacles())

    # 9) Проверка достижения финальной цели
    if drone_state.is_at_target(TARGET_POINT):
        print(f"Дрон достиг финальной цели за {current_time:.2f} секунд.")
        break

    if int(current_time * 10) % 10 == 0:
        print(
            f"Время: {current_time:.2f}с, Позиция дрона: {drone_state.get_position()}, Скорость: {drone_state.get_velocity().magnitude():.2f}м/с, Целевой вейпоинт: {current_target_waypoint}, Обнаружено препятствий: {len(dynamic_obstacle_map.get_known_obstacles())}")

    current_time += settings.SIMULATION_DT

else:
    print(f"Симуляция завершена. Дрон не достиг цели за {settings.SIMULATION_DURATION} секунд.")

print("Симуляция завершена.")
if plotter:
    plt.ioff()
    plt.show()