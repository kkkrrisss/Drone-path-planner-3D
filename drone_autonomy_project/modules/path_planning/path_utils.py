from utils.geometry_utils import calculate_distance

def add_intermediate_points(path_points, max_segment_length=2.0):
    """
    Добавляет промежуточные точки на каждом сегменте пути, если он длиннее max_segment_length
    path_points: List[Point]
    Возвращает: List[Point] с равномерными промежуточными точками.
    """
    if len(path_points) < 2:
        return path_points

    new_points = [path_points[0]]
    for i in range(1, len(path_points)):
        prev, curr = path_points[i-1], path_points[i]
        seg_len = calculate_distance(prev, curr)
        if seg_len > max_segment_length:
            num_extra = int(seg_len // max_segment_length)
            for j in range(1, num_extra + 1):
                alpha = j / (num_extra + 1)
                x = prev.x * (1 - alpha) + curr.x * alpha
                y = prev.y * (1 - alpha) + curr.y * alpha
                z = prev.z * (1 - alpha) + curr.z * alpha
                new_points.append(type(prev)(x, y, z))
        new_points.append(curr)
    return new_points