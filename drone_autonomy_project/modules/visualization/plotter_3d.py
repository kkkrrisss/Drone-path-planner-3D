# modules/visualization/plotter_3d.py

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import to_rgba
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from modules.data_structures.point import Point
from modules.data_structures.obstacle import BaseObstacle, SphereObstacle, BoxObstacle
from modules.data_structures.path import Path
from config import settings
from typing import List, Tuple

# Вспомогательная функция для конвертации цвета в RGBA
def _to_rgba_helper(color, alpha=1.0):
    """Конвертирует строку цвета или RGB-кортеж в RGBA"""
    return to_rgba(color, alpha=alpha)


class Plotter3D:
    """
    Класс для визуализации 3D-среды с дроном, препятствиями и путями
    """
    def __init__(self, start_point: Point, target_point: Point,
                 global_obstacles: List[BaseObstacle]):
        # создаём фигуру и оси
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title("Симуляция Автономного Дрона")
        self.ax.set_xlabel("X (м)")
        self.ax.set_ylabel("Y (м)")
        self.ax.set_zlabel("Z (м)")

        # стили линий
        self.path_line_style = dict(color='k', linestyle='--', linewidth=2, alpha=0.7, label='Запланированный путь RRT')
        self.trajectory_line_style = dict(color='b', linestyle='-', linewidth=2, label='Пройденная траектория')

        # границы мира
        self.ax.set_xlim(settings.WORLD_BOUNDS['x'])
        self.ax.set_ylim(settings.WORLD_BOUNDS['y'])
        self.ax.set_zlim(settings.WORLD_BOUNDS['z'])

        # рисуем все глобальные препятствия (скрытые/необнаруженные)
        self.global_obstacle_surfaces = []
        for obs in global_obstacles:
            if isinstance(obs, SphereObstacle):
                surface = self._create_sphere_surface(obs.position, obs.radius, color='gray', alpha=0.2, wireframe=True)
                self.global_obstacle_surfaces.append(surface)
            elif isinstance(obs, BoxObstacle):
                surface = self._create_box_surface(obs.min_point, obs.max_point, color='gray', alpha=0.2)
                self.global_obstacle_surfaces.append(surface)

        # старт и цель
        self.ax.scatter(start_point.x, start_point.y, start_point.z,
                        color='green', marker='o', s=100, label='Старт')
        self.ax.scatter(target_point.x, target_point.y, target_point.z,
                        color='red', marker='X', s=100, label='Цель')

        # маркер дрона и линии пути/траектории
        self.drone_marker, = self.ax.plot([], [], [], 'o', color='blue', markersize=8, label='Дрон')
        self.path_line, = self.ax.plot([], [], [], **self.path_line_style)
        self.trajectory_line, = self.ax.plot([], [], [], **self.trajectory_line_style)

        self.detected_obstacles_surfaces = []  # динамически добавляемые препятствия
        self.drone_trajectory_points: List[Point] = [start_point]  # запоминаем пройденный путь

        self.ax.legend()
        plt.ion()  # включаем интерактивный режим
        plt.show()

    def _create_sphere_surface(self, center: Point, radius: float, color, alpha: float, wireframe: bool = False):
        """
        Создаёт поверхность сферы для визуализации.
        wireframe=True — рисуем каркас, иначе залитая сфера.
        """
        u, v = np.mgrid[0:2*np.pi:30j, 0:np.pi:15j]
        x = center.x + radius * np.cos(u) * np.sin(v)
        y = center.y + radius * np.sin(u) * np.sin(v)
        z = center.z + radius * np.cos(v)

        face_color_rgba = _to_rgba_helper(color, alpha)

        kwargs = {
            'color': face_color_rgba,
            'linewidth': 0.5 if wireframe else 0,
            'rstride': 1,
            'cstride': 1
        }
        if wireframe:
            kwargs['edgecolors'] = face_color_rgba

        return self.ax.plot_surface(x, y, z, **kwargs)

    def _create_box_wireframe(self, min_point: Point, max_point: Point, color, alpha):
        """Рисуем каркас параллелепипеда по двум противоположным вершинам."""
        x = [min_point.x, max_point.x]
        y = [min_point.y, max_point.y]
        z = [min_point.z, max_point.z]
        verts = [
            [x[0], y[0], z[0]], [x[1], y[0], z[0]],
            [x[1], y[1], z[0]], [x[0], y[1], z[0]],
            [x[0], y[0], z[1]], [x[1], y[0], z[1]],
            [x[1], y[1], z[1]], [x[0], y[1], z[1]],
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # низ
            (4, 5), (5, 6), (6, 7), (7, 4),  # верх
            (0, 4), (1, 5), (2, 6), (3, 7)   # вертикали
        ]
        for e in edges:
            xs = [verts[e[0]][0], verts[e[1]][0]]
            ys = [verts[e[0]][1], verts[e[1]][1]]
            zs = [verts[e[0]][2], verts[e[1]][2]]
            self.ax.plot(xs, ys, zs, color=color, alpha=alpha, linewidth=1.5)

    def _create_box_surface(self, min_point, max_point, color='gray', alpha=0.2):
        """Создаём залитый параллелепипед для визуализации."""
        x = [min_point.x, max_point.x]
        y = [min_point.y, max_point.y]
        z = [min_point.z, max_point.z]
        vertices = [
            [x[0], y[0], z[0]], [x[1], y[0], z[0]],
            [x[1], y[1], z[0]], [x[0], y[1], z[0]],
            [x[0], y[0], z[1]], [x[1], y[0], z[1]],
            [x[1], y[1], z[1]], [x[0], y[1], z[1]],
        ]
        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # низ
            [vertices[4], vertices[5], vertices[6], vertices[7]],  # верх
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # бок1
            [vertices[1], vertices[2], vertices[6], vertices[5]],  # бок2
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # бок3
            [vertices[3], vertices[0], vertices[4], vertices[7]],  # бок4
        ]
        box = Poly3DCollection(faces, facecolors=color, edgecolors='k', alpha=alpha)
        self.ax.add_collection3d(box)
        return box

    def update_plot(self, current_drone_pos: Point,
                    global_path: Path,
                    known_obstacles: List[BaseObstacle]):
        """
        Обновление визуализации:
        - положение дрона
        - пройденная траектория
        - обнаруженные препятствия
        - глобальный путь
        """
        # удаляем старые поверхности динамических препятствий
        for surf in self.detected_obstacles_surfaces:
            surf.remove()
        self.detected_obstacles_surfaces.clear()

        # добавляем новые препятствия
        for obs in known_obstacles:
            if isinstance(obs, SphereObstacle):
                surface = self._create_sphere_surface(obs.position, obs.radius, color='orange', alpha=0.4, wireframe=False)
                self.detected_obstacles_surfaces.append(surface)
            elif isinstance(obs, BoxObstacle):
                surface = self._create_box_surface(obs.min_point, obs.max_point, color='orange', alpha=0.4)
                self.detected_obstacles_surfaces.append(surface)

        # обновляем позицию дрона
        self.drone_marker.set_data([current_drone_pos.x], [current_drone_pos.y])
        self.drone_marker.set_3d_properties(current_drone_pos.z)

        # обновляем траекторию дрона
        self.drone_trajectory_points.append(current_drone_pos)
        traj_xs = [p.x for p in self.drone_trajectory_points]
        traj_ys = [p.y for p in self.drone_trajectory_points]
        traj_zs = [p.z for p in self.drone_trajectory_points]
        self.trajectory_line.set_data(traj_xs, traj_ys)
        self.trajectory_line.set_3d_properties(traj_zs)

        # обновляем глобальный путь
        if global_path and len(global_path) > 0:
            path_xs = [p.x for p in global_path.waypoints]
            path_ys = [p.y for p in global_path.waypoints]
            path_zs = [p.z for p in global_path.waypoints]
            self.path_line.set_data(path_xs, path_ys)
            self.path_line.set_3d_properties(path_zs)
        else:
            self.path_line.set_data([], [])
            self.path_line.set_3d_properties([])

        # обновляем путь, чтобы он был поверх остальных объектов
        if hasattr(self, 'path_line'):
            try:
                self.path_line.remove()
            except Exception:
                pass
        if global_path and len(global_path) > 0:
            path_xs = [p.x for p in global_path.waypoints]
            path_ys = [p.y for p in global_path.waypoints]
            path_zs = [p.z for p in global_path.waypoints]
            self.path_line, = self.ax.plot(path_xs, path_ys, path_zs, **self.path_line_style)
        else:
            self.path_line, = self.ax.plot([], [], [], **self.path_line_style)

        plt.pause(0.001)  # короткая пауза для обновления графика
