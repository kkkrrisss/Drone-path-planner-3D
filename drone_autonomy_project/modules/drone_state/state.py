# modules/drone_state/state.py

from modules.data_structures.point import Point

class DroneState:
    """
    Представляет текущее состояние дрона: позиция, скорость.
    Для простоты ориентация (крен, тангаж, курс) пока не учитывается явно,
    дрон моделируется как точка, движущаяся в 3D.
    """
    def __init__(self, initial_position: Point = Point(0, 0, 0),
                 initial_velocity: Point = Point(0, 0, 0)):
        if not isinstance(initial_position, Point):
            raise TypeError("Initial position must be a Point object.")
        if not isinstance(initial_velocity, Point):
            raise TypeError("Initial velocity must be a Point object.")

        self.position = initial_position
        self.velocity = initial_velocity

    def get_position(self) -> Point:
        return self.position

    def get_velocity(self) -> Point:
        return self.velocity

    def update_state(self, new_position: Point, new_velocity: Point):
        """Обновляет позицию и скорость дрона."""
        if not isinstance(new_position, Point):
            raise TypeError("New position must be a Point object.")
        if not isinstance(new_velocity, Point):
            raise TypeError("New velocity must be a Point object.")
        self.position = new_position
        self.velocity = new_velocity

    def is_at_target(self, target_position: Point, tolerance: float = 0.5) -> bool:
        """Проверяет, находится ли дрон в пределах заданного допуска от целевой позиции."""
        return self.position.distance_to(target_position) <= tolerance

    def __str__(self):
        return f"DroneState(Pos: {self.position}, Vel: {self.velocity})"