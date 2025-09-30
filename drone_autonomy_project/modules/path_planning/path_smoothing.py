import random

def smooth_path(path_points, obstacle_map, max_attempts=100, num_samples=20):
    """
    Shortcut smoothing для пути из точек
    path_points: List[Point]
    obstacle_map: объект с методом check_collision_along_line(p1, p2, num_samples)
    max_attempts: сколько попыток сделать сокращение пути
    num_samples: сколько точек проверять между двумя точками

    Возвращает сглаженный путь (List[Point])
    """
    if len(path_points) < 3:
        return path_points

    new_path = path_points.copy()
    for _ in range(max_attempts):
        if len(new_path) < 3:
            break
        i = random.randint(0, len(new_path) - 3)
        j = random.randint(i + 2, len(new_path) - 1)
        p1, p2 = new_path[i], new_path[j]
        if not obstacle_map.check_collision_along_line(p1, p2, num_samples=num_samples):
            new_path = new_path[:i+1] + new_path[j:]
    return new_path