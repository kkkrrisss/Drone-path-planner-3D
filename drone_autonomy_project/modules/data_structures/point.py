# modules/data_structures/point.py

import math

class Point:
    def __init__(self, x, y, z):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __str__(self):
        return f"({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"

    def __repr__(self):
        return f"Point({self.x}, {self.y}, {self.z})"

    def __add__(self, other):
        if isinstance(other, Point):
            return Point(self.x + other.x, self.y + other.y, self.z + other.z)
        raise TypeError("Можно добавлять только другой объект Point.")

    def __sub__(self, other):
        if isinstance(other, Point):
            return Point(self.x - other.x, self.y - other.y, self.z - other.z)
        raise TypeError("Можно вычитать только другой объект Point.")

    def __mul__(self, scalar):
        if isinstance(scalar, (int, float)):
            return Point(self.x * scalar, self.y * scalar, self.z * scalar)
        raise TypeError("Можно умножать только на числовой скаляр.")

    def dot(self, other: 'Point') -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def __rmul__(self, scalar):
        return self.__mul__(scalar) # Поддержка scalar * Point

    def magnitude(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalize(self):
        mag = self.magnitude()
        if mag == 0:
            return Point(0, 0, 0)
        return Point(self.x / mag, self.y / mag, self.z / mag)

    def distance_to(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)