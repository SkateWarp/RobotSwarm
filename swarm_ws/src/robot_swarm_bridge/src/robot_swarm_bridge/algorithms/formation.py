"""ROS-independent helpers for scalable swarm formations."""

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


Point2 = Tuple[float, float]


def hungarian_assignment(cost_matrix: Sequence[Sequence[float]]) -> List[int]:
    """Return the minimum-cost column for every row using the Hungarian method.

    The implementation is dependency-free and supports rectangular matrices
    with at least as many columns as rows. Formation control uses the common
    square case (one robot per target slot).
    """
    if not cost_matrix:
        return []

    row_count = len(cost_matrix)
    column_count = len(cost_matrix[0])
    if column_count < row_count:
        raise ValueError("assignment requires at least as many columns as rows")
    if any(len(row) != column_count for row in cost_matrix):
        raise ValueError("assignment cost matrix must be rectangular")
    if any(not math.isfinite(float(value)) for row in cost_matrix for value in row):
        raise ValueError("assignment costs must be finite")

    # Classic O(n^3) shortest augmenting path form, using one-based arrays.
    row_potential = [0.0] * (row_count + 1)
    column_potential = [0.0] * (column_count + 1)
    matched_row = [0] * (column_count + 1)
    predecessor = [0] * (column_count + 1)

    for row_index in range(1, row_count + 1):
        matched_row[0] = row_index
        current_column = 0
        minimum_slack = [float("inf")] * (column_count + 1)
        used = [False] * (column_count + 1)

        while True:
            used[current_column] = True
            current_row = matched_row[current_column]
            delta = float("inf")
            next_column = 0

            for column_index in range(1, column_count + 1):
                if used[column_index]:
                    continue
                reduced_cost = (
                    float(cost_matrix[current_row - 1][column_index - 1])
                    - row_potential[current_row]
                    - column_potential[column_index]
                )
                if reduced_cost < minimum_slack[column_index]:
                    minimum_slack[column_index] = reduced_cost
                    predecessor[column_index] = current_column
                if minimum_slack[column_index] < delta:
                    delta = minimum_slack[column_index]
                    next_column = column_index

            for column_index in range(column_count + 1):
                if used[column_index]:
                    row_potential[matched_row[column_index]] += delta
                    column_potential[column_index] -= delta
                else:
                    minimum_slack[column_index] -= delta

            current_column = next_column
            if matched_row[current_column] == 0:
                break

        while True:
            previous_column = predecessor[current_column]
            matched_row[current_column] = matched_row[previous_column]
            current_column = previous_column
            if current_column == 0:
                break

    assignment = [-1] * row_count
    for column_index in range(1, column_count + 1):
        if matched_row[column_index] != 0:
            assignment[matched_row[column_index] - 1] = column_index - 1
    return assignment


def minimum_distance_assignment(
    robot_positions: Sequence[Point2],
    target_positions: Sequence[Point2],
    previous_slots: Optional[Sequence[Optional[int]]] = None,
    switch_penalty: float = 0.0,
) -> List[int]:
    """Assign robots to targets with minimum total squared travel distance.

    A small optional slot-switch penalty prevents periodic recomputation from
    making robots exchange otherwise equivalent positions and crossing paths.
    """
    if len(robot_positions) != len(target_positions):
        raise ValueError("robot and target counts must match")
    if not robot_positions:
        return []

    costs: List[List[float]] = []
    for robot_index, (robot_x, robot_y) in enumerate(robot_positions):
        previous = (
            previous_slots[robot_index]
            if previous_slots is not None and robot_index < len(previous_slots)
            else None
        )
        row: List[float] = []
        for slot_index, (target_x, target_y) in enumerate(target_positions):
            distance_cost = (robot_x - target_x) ** 2 + (robot_y - target_y) ** 2
            if previous is not None and previous != slot_index:
                distance_cost += max(0.0, switch_penalty)
            row.append(distance_cost)
        costs.append(row)
    return hungarian_assignment(costs)


def _split_strokes(points: Sequence[Point2]) -> List[List[Point2]]:
    """Split an ordered grid glyph whenever consecutive points are disconnected."""
    if not points:
        return []

    maximum_connected_step = math.sqrt(2.0) + 1e-9
    strokes: List[List[Point2]] = [[points[0]]]
    for point in points[1:]:
        previous = strokes[-1][-1]
        if math.hypot(point[0] - previous[0], point[1] - previous[1]) <= maximum_connected_step:
            strokes[-1].append(point)
        else:
            strokes.append([point])
    return strokes


def _stroke_length(stroke: Sequence[Point2]) -> float:
    return sum(
        math.hypot(end[0] - start[0], end[1] - start[1])
        for start, end in zip(stroke, stroke[1:])
    )


def _dense_stroke_candidates(
    strokes: Sequence[Sequence[Point2]],
    desired_count: int,
) -> List[Point2]:
    total_length = sum(_stroke_length(stroke) for stroke in strokes)
    density = max(4.0, (desired_count * 3.0) / max(total_length, 1.0))
    candidates: List[Point2] = []

    for stroke in strokes:
        if len(stroke) == 1:
            candidates.append(stroke[0])
            continue
        for start, end in zip(stroke, stroke[1:]):
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            subdivisions = max(1, int(math.ceil(length * density)))
            for index in range(subdivisions):
                fraction = index / subdivisions
                candidates.append((
                    start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]),
                ))
        candidates.append(stroke[-1])

    # Preserve order but remove exact junction duplicates.
    return list(dict.fromkeys(candidates))


def _farthest_point_sample(points: Sequence[Point2], count: int) -> List[Point2]:
    """Choose spatially well-distributed points deterministically."""
    if count <= 0 or not points:
        return []
    if count >= len(points):
        return list(points)

    center_x = sum(point[0] for point in points) / len(points)
    center_y = sum(point[1] for point in points) / len(points)
    first_index = max(
        range(len(points)),
        key=lambda index: (
            (points[index][0] - center_x) ** 2 + (points[index][1] - center_y) ** 2,
            -index,
        ),
    )

    selected_indices = [first_index]
    minimum_distances = [
        (point[0] - points[first_index][0]) ** 2
        + (point[1] - points[first_index][1]) ** 2
        for point in points
    ]

    while len(selected_indices) < count:
        next_index = max(
            (index for index in range(len(points)) if index not in selected_indices),
            key=lambda index: (minimum_distances[index], -index),
        )
        selected_indices.append(next_index)
        selected_point = points[next_index]
        for index, point in enumerate(points):
            distance = (
                (point[0] - selected_point[0]) ** 2
                + (point[1] - selected_point[1]) ** 2
            )
            minimum_distances[index] = min(minimum_distances[index], distance)

    return [points[index] for index in selected_indices]


def sample_letter_formation(
    grid_points: Iterable[Tuple[int, int]],
    robot_count: int,
    spacing: float,
) -> List[Point2]:
    """Sample any number of collision-conscious slots along an ordered glyph.

    The glyph expands when necessary so adding robots increases available path
    length instead of stacking several robots onto jittered duplicates.
    """
    if robot_count <= 0:
        return []

    raw_points = [(float(column), float(row)) for column, row in grid_points]
    if not raw_points:
        return []

    minimum_x = min(point[0] for point in raw_points)
    maximum_x = max(point[0] for point in raw_points)
    minimum_y = min(point[1] for point in raw_points)
    maximum_y = max(point[1] for point in raw_points)
    center_x = (minimum_x + maximum_x) / 2.0
    center_y = (minimum_y + maximum_y) / 2.0
    centered = [(point[0] - center_x, point[1] - center_y) for point in raw_points]

    strokes = _split_strokes(centered)
    grid_path_length = sum(_stroke_length(stroke) for stroke in strokes)
    safe_spacing = max(float(spacing), 0.05)
    base_scale = safe_spacing * 0.6
    scale_for_capacity = (
        ((robot_count - 1) * safe_spacing * 0.75) / grid_path_length
        if grid_path_length > 0.0
        else base_scale
    )
    scale = max(base_scale, scale_for_capacity)

    candidates = _dense_stroke_candidates(strokes, robot_count)
    sampled = _farthest_point_sample(candidates, robot_count)
    return [(point[0] * scale, point[1] * scale) for point in sampled]

