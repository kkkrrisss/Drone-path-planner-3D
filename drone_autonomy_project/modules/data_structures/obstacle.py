from .point import Point

#Базовый класс препятствия
class BaseObstacle:
    def is_collision(self, point: Point, point_radius: float = 0.0):
        raise NotImplementedError("Override in child classes.")

#Сферическое препятствие в 3D
class SphereObstacle(BaseObstacle):
    def __init__(self, position: Point, radius: float):
        if not isinstance(position, Point):
            raise TypeError("Position must be a Point object.")
        if not isinstance(radius, (int, float)) or radius < 0:
            raise ValueError("Radius must be a non-negative number.")
        self.position = position
        self.radius = radius

    def __str__(self):
        return f"SphereObstacle at {self.position} with radius {self.radius:.2f}"

    def __repr__(self):
        return f"SphereObstacle(position={self.position!r}, radius={self.radius})"

    def is_collision(self, point: Point, point_radius: float = 0.0):
        distance = self.position.distance_to(point)
        return distance <= (self.radius + point_radius)
    def distance_to_point(self, point: Point) -> float:
        return max(0.0, self.position.distance_to(point) - self.radius)

#Параллелепипед
class BoxObstacle(BaseObstacle):
    def __init__(self, min_point: Point, max_point: Point):
        if not (isinstance(min_point, Point) and isinstance(max_point, Point)):
            raise TypeError("min_point and max_point must be Point objects.")
        # Проверка, что min <= max по всем координатам
        if not (min_point.x <= max_point.x and min_point.y <= max_point.y and min_point.z <= max_point.z):
            raise ValueError("min_point must be <= max_point by each coordinate.")
        self.min_point = min_point
        self.max_point = max_point

    def __str__(self):
        return f"BoxObstacle from {self.min_point} to {self.max_point}"

    def __repr__(self):
        return f"BoxObstacle(min_point={self.min_point!r}, max_point={self.max_point!r})"

    def distance_to_point(self, point: Point) -> float:
        # Классическая формула расстояния от точки до AABB
        dx = max(self.min_point.x - point.x, 0, point.x - self.max_point.x)
        dy = max(self.min_point.y - point.y, 0, point.y - self.max_point.y)
        dz = max(self.min_point.z - point.z, 0, point.z - self.max_point.z)
        return (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

#Проверяет, пересекается ли сфера точки радиусом point_radius с коробкой
    def is_collision(self, point: Point, point_radius: float = 0.0):
        #Находим ближайшую точку на коробке к центру сферы
        closest_x = min(max(point.x, self.min_point.x), self.max_point.x)
        closest_y = min(max(point.y, self.min_point.y), self.max_point.y)
        closest_z = min(max(point.z, self.min_point.z), self.max_point.z)
        #Считаем расстояние
        dist_sq = ((point.x - closest_x) ** 2 +
                   (point.y - closest_y) ** 2 +
                   (point.z - closest_z) ** 2)
        #Столкновение, если расстояние <= радиуса сферы
        return dist_sq <= point_radius ** 2