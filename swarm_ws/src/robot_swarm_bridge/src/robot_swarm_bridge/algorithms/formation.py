"""ROS-independent helpers for scalable swarm formations."""

import heapq
import math
from collections.abc import Mapping as MappingABC
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


Point2 = Tuple[float, float]
ModelPoseMap = Mapping[str, Tuple[float, float, float]]


def center_formation(points: Sequence[Point2]) -> List[Point2]:
    """Return the same shape translated so its centroid is at the origin."""
    if not points:
        return []

    center_x = sum(point[0] for point in points) / len(points)
    center_y = sum(point[1] for point in points) / len(points)
    return [
        (point[0] - center_x, point[1] - center_y)
        for point in points
    ]


def ensure_minimum_spacing(
    points: Sequence[Point2], minimum_spacing: float
) -> List[Point2]:
    """Uniformly expand a shape when sampled slots end up too close.

    Partial polygon edges and sparsely sampled letters do not always retain
    the requested spacing. Scaling around the centroid preserves the shape
    while giving every pair of TurtleBots the same minimum center clearance.
    """
    centered = center_formation(points)
    if len(centered) < 2:
        return centered

    requested = max(0.0, float(minimum_spacing))
    if requested == 0.0:
        return centered

    closest = min(
        math.hypot(first[0] - second[0], first[1] - second[1])
        for index, first in enumerate(centered)
        for second in centered[index + 1:]
    )
    if closest <= 1e-9:
        raise ValueError("formation slots must be unique")
    if closest >= requested:
        return centered

    scale = requested / closest
    return [(x * scale, y * scale) for x, y in centered]


def _zone_is_active(zone: Mapping, arena_profile: str) -> bool:
    worlds = zone.get('worlds')
    if isinstance(worlds, str):
        worlds = [worlds]
    return (
        (not worlds or arena_profile in worlds)
        and zone.get('shape') in ('box', 'circle')
    )


def _active_zones(
    exclusion_zones: Sequence[Mapping], arena_profile: str
) -> List[Mapping]:
    return [
        zone for zone in exclusion_zones
        if isinstance(zone, MappingABC) and _zone_is_active(zone, arena_profile)
    ]


def _zone_clearance(zone: Mapping, default_clearance: float) -> float:
    """Return a zone-specific clearance plus its optional padding."""

    clearance = max(
        0.0, float(zone.get('clearance', default_clearance))
    )
    padding = max(0.0, float(zone.get('padding', 0.0)))
    return clearance + padding


def signed_distance_to_zone(
    point: Point2,
    zone: Mapping,
    model_poses: Optional[ModelPoseMap] = None,
) -> float:
    """Return signed planar distance from a point to one configured zone."""

    center_x = float(zone.get('x', 0.0))
    center_y = float(zone.get('y', 0.0))
    yaw = float(zone.get('yaw', 0.0))

    model_name = zone.get('model')
    if model_poses is not None and model_name in model_poses:
        center_x, center_y, yaw = model_poses[model_name]
        yaw += float(zone.get('yaw_offset', 0.0))

    x, y = point
    if zone.get('shape') == 'circle':
        radius = max(0.0, float(zone.get('radius', 0.0)))
        return math.hypot(x - center_x, y - center_y) - radius

    half_width = max(0.0, float(zone.get('width', 0.0))) / 2.0
    half_height = max(0.0, float(zone.get('height', 0.0))) / 2.0
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    relative_x = x - center_x
    relative_y = y - center_y
    local_x = cosine * relative_x + sine * relative_y
    local_y = -sine * relative_x + cosine * relative_y

    outside_x = abs(local_x) - half_width
    outside_y = abs(local_y) - half_height
    if outside_x > 0.0 or outside_y > 0.0:
        return math.hypot(max(outside_x, 0.0), max(outside_y, 0.0))
    return -min(half_width - abs(local_x), half_height - abs(local_y))


def formation_targets_are_safe(
    targets: Sequence[Point2],
    arena_size: float,
    arena_margin: float,
    obstacle_clearance: float,
    exclusion_zones: Sequence[Mapping],
    arena_profile: str,
    model_poses: Optional[ModelPoseMap] = None,
) -> bool:
    """Check bounds and configured obstacle clearance for every target."""

    usable_half = arena_size / 2.0 - arena_margin
    if usable_half <= 0.0:
        return False

    active_zones = _active_zones(exclusion_zones, arena_profile)
    for x, y in targets:
        if not math.isfinite(x) or not math.isfinite(y):
            return False
        if abs(x) > usable_half + 1e-9 or abs(y) > usable_half + 1e-9:
            return False
        for zone in active_zones:
            if signed_distance_to_zone(
                (x, y), zone, model_poses
            ) < _zone_clearance(zone, obstacle_clearance) - 1e-9:
                return False
    return True


def _point_to_segment_distance(
    point: Point2, start: Point2, end: Point2
) -> float:
    segment_x = end[0] - start[0]
    segment_y = end[1] - start[1]
    length_squared = segment_x * segment_x + segment_y * segment_y
    if length_squared <= 1e-18:
        return math.hypot(point[0] - start[0], point[1] - start[1])

    fraction = (
        (point[0] - start[0]) * segment_x
        + (point[1] - start[1]) * segment_y
    ) / length_squared
    fraction = max(0.0, min(1.0, fraction))
    closest = (
        start[0] + fraction * segment_x,
        start[1] + fraction * segment_y,
    )
    return math.hypot(point[0] - closest[0], point[1] - closest[1])


def point_to_route_distance(
    point: Point2, route: Sequence[Point2]
) -> float:
    """Return the closest distance from a point to a polyline route."""

    if not route:
        return float('inf')
    if len(route) == 1:
        return math.hypot(
            point[0] - route[0][0], point[1] - route[0][1]
        )
    return min(
        _point_to_segment_distance(point, start, end)
        for start, end in zip(route, route[1:])
    )


def _segments_intersect(
    first_start: Point2,
    first_end: Point2,
    second_start: Point2,
    second_end: Point2,
) -> bool:
    """Return whether two closed line segments intersect."""

    def orientation(start: Point2, middle: Point2, end: Point2) -> float:
        return (
            (middle[0] - start[0]) * (end[1] - start[1])
            - (middle[1] - start[1]) * (end[0] - start[0])
        )

    first_side = orientation(first_start, first_end, second_start)
    second_side = orientation(first_start, first_end, second_end)
    third_side = orientation(second_start, second_end, first_start)
    fourth_side = orientation(second_start, second_end, first_end)
    if (
        first_side * second_side < -1e-12
        and third_side * fourth_side < -1e-12
    ):
        return True

    return min(
        _point_to_segment_distance(first_start, second_start, second_end),
        _point_to_segment_distance(first_end, second_start, second_end),
        _point_to_segment_distance(second_start, first_start, first_end),
        _point_to_segment_distance(second_end, first_start, first_end),
    ) <= 1e-9


def routes_conflict(
    first_route: Sequence[Point2],
    second_route: Sequence[Point2],
    minimum_clearance: float,
) -> bool:
    """Return whether two polylines come too close to run together."""

    if minimum_clearance < 0.0 or not math.isfinite(minimum_clearance):
        raise ValueError('route clearance must be finite and non-negative')
    if not first_route or not second_route:
        return False

    first_segments = list(zip(first_route, first_route[1:]))
    second_segments = list(zip(second_route, second_route[1:]))
    if not first_segments:
        return point_to_route_distance(
            first_route[0], second_route
        ) < minimum_clearance - 1e-9
    if not second_segments:
        return point_to_route_distance(
            second_route[0], first_route
        ) < minimum_clearance - 1e-9

    for first_start, first_end in first_segments:
        for second_start, second_end in second_segments:
            if _segments_intersect(
                first_start, first_end, second_start, second_end
            ):
                return minimum_clearance > 0.0
            distance = min(
                _point_to_segment_distance(
                    first_start, second_start, second_end
                ),
                _point_to_segment_distance(
                    first_end, second_start, second_end
                ),
                _point_to_segment_distance(
                    second_start, first_start, first_end
                ),
                _point_to_segment_distance(
                    second_end, first_start, first_end
                ),
            )
            if distance < minimum_clearance - 1e-9:
                return True
    return False


def _segment_intersects_box(
    start: Point2, end: Point2, half_width: float, half_height: float
) -> bool:
    """Clip a segment against an axis-aligned box using two slabs."""

    minimum_fraction = 0.0
    maximum_fraction = 1.0
    for origin, delta, lower, upper in (
        (start[0], end[0] - start[0], -half_width, half_width),
        (start[1], end[1] - start[1], -half_height, half_height),
    ):
        if abs(delta) <= 1e-12:
            if origin < lower - 1e-9 or origin > upper + 1e-9:
                return False
            continue

        first = (lower - origin) / delta
        second = (upper - origin) / delta
        entry = min(first, second)
        exit_ = max(first, second)
        minimum_fraction = max(minimum_fraction, entry)
        maximum_fraction = min(maximum_fraction, exit_)
        if minimum_fraction > maximum_fraction + 1e-9:
            return False
    return True


def _point_to_box_distance(
    point: Point2, half_width: float, half_height: float
) -> float:
    outside_x = max(abs(point[0]) - half_width, 0.0)
    outside_y = max(abs(point[1]) - half_height, 0.0)
    return math.hypot(outside_x, outside_y)


def _route_clearance_to_zone(
    start: Point2,
    end: Point2,
    zone: Mapping,
    model_poses: Optional[ModelPoseMap],
) -> float:
    """Return the closest route-to-surface distance for one obstacle."""

    center_x = float(zone.get('x', 0.0))
    center_y = float(zone.get('y', 0.0))
    yaw = float(zone.get('yaw', 0.0))
    model_name = zone.get('model')
    if model_poses is not None and model_name in model_poses:
        center_x, center_y, yaw = model_poses[model_name]
        yaw += float(zone.get('yaw_offset', 0.0))

    if zone.get('shape') == 'circle':
        radius = max(0.0, float(zone.get('radius', 0.0)))
        return max(
            0.0,
            _point_to_segment_distance(
                (center_x, center_y), start, end
            ) - radius,
        )

    cosine = math.cos(yaw)
    sine = math.sin(yaw)

    def local(point: Point2) -> Point2:
        relative_x = point[0] - center_x
        relative_y = point[1] - center_y
        return (
            cosine * relative_x + sine * relative_y,
            -sine * relative_x + cosine * relative_y,
        )

    local_start = local(start)
    local_end = local(end)
    half_width = max(0.0, float(zone.get('width', 0.0))) / 2.0
    half_height = max(0.0, float(zone.get('height', 0.0))) / 2.0

    if _segment_intersects_box(
        local_start, local_end, half_width, half_height
    ):
        return 0.0

    corners = (
        (-half_width, -half_height),
        (half_width, -half_height),
        (half_width, half_height),
        (-half_width, half_height),
    )
    return min(
        _point_to_box_distance(local_start, half_width, half_height),
        _point_to_box_distance(local_end, half_width, half_height),
        *(
            _point_to_segment_distance(corner, local_start, local_end)
            for corner in corners
        ),
    )


def _straight_route_is_safe_for_zones(
    start: Point2,
    end: Point2,
    obstacle_clearance: float,
    active_zones: Sequence[Mapping],
    model_poses: Optional[ModelPoseMap] = None,
) -> bool:
    if not all(math.isfinite(value) for point in (start, end) for value in point):
        return False

    for zone in active_zones:
        required_clearance = _zone_clearance(zone, obstacle_clearance)
        route_clearance = _route_clearance_to_zone(
            start, end, zone, model_poses
        )
        start_clearance = signed_distance_to_zone(
            start, zone, model_poses
        )

        if start_clearance >= required_clearance - 1e-9:
            if route_clearance < required_clearance - 1e-9:
                return False
            continue

        # Gazebo can settle a robot a few centimetres inside the planning
        # buffer without touching the obstacle itself. Let that robot leave
        # only when the straight route never gets closer and its target is
        # back outside the full buffer. Require a useful early clearance gain
        # as well; a nearly tangent route is unsafe for a differential-drive
        # robot while it is still turning toward the target.
        end_clearance = signed_distance_to_zone(end, zone, model_poses)
        route_length = math.hypot(end[0] - start[0], end[1] - start[1])
        probe_fraction = min(1.0, 0.20 / max(route_length, 1e-12))
        probe = (
            start[0] + probe_fraction * (end[0] - start[0]),
            start[1] + probe_fraction * (end[1] - start[1]),
        )
        probe_clearance = signed_distance_to_zone(
            probe, zone, model_poses
        )
        required_gain = min(
            0.03, max(0.0, required_clearance - start_clearance)
        )
        if (
            start_clearance < -1e-9
            or end_clearance < required_clearance - 1e-9
            or route_clearance < max(0.0, start_clearance) - 1e-9
            or probe_clearance < start_clearance + required_gain - 1e-9
        ):
            return False
    return True


def straight_route_is_safe(
    start: Point2,
    end: Point2,
    obstacle_clearance: float,
    exclusion_zones: Sequence[Mapping],
    arena_profile: str,
    model_poses: Optional[ModelPoseMap] = None,
) -> bool:
    """Check a straight robot route against configured obstacle geometry."""

    if obstacle_clearance < 0.0 or not math.isfinite(obstacle_clearance):
        raise ValueError('route clearance must be finite and non-negative')
    return _straight_route_is_safe_for_zones(
        start,
        end,
        obstacle_clearance,
        _active_zones(exclusion_zones, arena_profile),
        model_poses,
    )


def _zone_pose(
    zone: Mapping,
    model_poses: Optional[ModelPoseMap],
) -> Tuple[float, float, float]:
    """Resolve a configured obstacle pose, including a live model pose."""

    center_x = float(zone.get('x', 0.0))
    center_y = float(zone.get('y', 0.0))
    yaw = float(zone.get('yaw', 0.0))
    model_name = zone.get('model')
    if model_poses is not None and model_name in model_poses:
        center_x, center_y, yaw = model_poses[model_name]
        yaw += float(zone.get('yaw_offset', 0.0))
    return center_x, center_y, yaw


def _route_candidate_points(
    active_zones: Sequence[Mapping],
    obstacle_clearance: float,
    model_poses: Optional[ModelPoseMap],
    circle_samples: int,
) -> List[Point2]:
    """Build visibility points just outside inflated obstacle boundaries."""

    candidates: List[Point2] = []
    for zone in active_zones:
        center_x, center_y, yaw = _zone_pose(zone, model_poses)
        required_clearance = _zone_clearance(zone, obstacle_clearance)
        guard = max(1e-4, required_clearance * 0.01)

        if zone.get('shape') == 'circle':
            radius = max(0.0, float(zone.get('radius', 0.0)))
            # The sampled ring circumscribes the inflated circle. This keeps
            # the chord between neighboring samples outside the clearance
            # boundary instead of cutting through it.
            ring_radius = (
                radius + required_clearance + guard
            ) / math.cos(math.pi / circle_samples)
            for sample_index in range(circle_samples):
                angle = 2.0 * math.pi * sample_index / circle_samples
                candidates.append((
                    center_x + ring_radius * math.cos(angle),
                    center_y + ring_radius * math.sin(angle),
                ))
            continue

        half_width = max(0.0, float(zone.get('width', 0.0))) / 2.0
        half_height = max(0.0, float(zone.get('height', 0.0))) / 2.0
        expanded_width = half_width + required_clearance + guard
        expanded_height = half_height + required_clearance + guard
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        for local_x, local_y in (
            (-expanded_width, -expanded_height),
            (expanded_width, -expanded_height),
            (expanded_width, expanded_height),
            (-expanded_width, expanded_height),
        ):
            candidates.append((
                center_x + cosine * local_x - sine * local_y,
                center_y + sine * local_x + cosine * local_y,
            ))
    return candidates


def plan_obstacle_aware_route(
    start: Point2,
    end: Point2,
    arena_size: float,
    arena_margin: float,
    obstacle_clearance: float,
    exclusion_zones: Sequence[Mapping],
    arena_profile: str,
    model_poses: Optional[ModelPoseMap] = None,
    circle_samples: int = 16,
) -> Optional[List[Point2]]:
    """Return a shortest piecewise-linear route around configured obstacles.

    The returned list includes both ``start`` and ``end``. Box corners and a
    circumscribed sample ring around circles form a small visibility graph;
    Dijkstra's algorithm then selects a deterministic shortest route. All
    segments preserve the requested obstacle clearance and remain inside the
    arena margin.
    """

    if arena_size <= 0.0 or not math.isfinite(arena_size):
        raise ValueError('arena size must be finite and positive')
    if arena_margin < 0.0 or not math.isfinite(arena_margin):
        raise ValueError('arena margin must be finite and non-negative')
    if obstacle_clearance < 0.0 or not math.isfinite(obstacle_clearance):
        raise ValueError('route clearance must be finite and non-negative')
    if circle_samples < 8:
        raise ValueError('circle route sampling requires at least 8 points')
    if not all(math.isfinite(value) for point in (start, end) for value in point):
        return None

    usable_half = arena_size / 2.0 - arena_margin
    if usable_half <= 0.0:
        return None

    def inside_arena(point: Point2) -> bool:
        return (
            abs(point[0]) <= usable_half + 1e-9
            and abs(point[1]) <= usable_half + 1e-9
        )

    if not inside_arena(start) or not inside_arena(end):
        return None

    active_zones = _active_zones(exclusion_zones, arena_profile)
    if not formation_targets_are_safe(
        [end],
        arena_size,
        arena_margin,
        obstacle_clearance,
        exclusion_zones,
        arena_profile,
        model_poses,
    ):
        return None

    # A robot may begin just inside a conservative clearance buffer, but it
    # cannot plan its way out from inside the physical obstacle itself.
    if any(
        signed_distance_to_zone(start, zone, model_poses) < -1e-9
        for zone in active_zones
    ):
        return None

    if math.hypot(end[0] - start[0], end[1] - start[1]) <= 1e-12:
        return [start]
    if _straight_route_is_safe_for_zones(
        start, end, obstacle_clearance, active_zones, model_poses
    ):
        return [start, end]

    raw_candidates = _route_candidate_points(
        active_zones,
        obstacle_clearance,
        model_poses,
        circle_samples,
    )
    candidates: List[Point2] = []
    seen = set()
    for point in raw_candidates:
        key = (round(point[0], 10), round(point[1], 10))
        if key in seen or not inside_arena(point):
            continue
        if not formation_targets_are_safe(
            [point],
            arena_size,
            arena_margin,
            obstacle_clearance,
            exclusion_zones,
            arena_profile,
            model_poses,
        ):
            continue
        seen.add(key)
        candidates.append(point)

    nodes = [start, end] + candidates
    neighbors: List[List[Tuple[int, float]]] = [
        [] for _ in nodes
    ]
    for first_index, first in enumerate(nodes):
        for second_index in range(first_index + 1, len(nodes)):
            second = nodes[second_index]
            if not _straight_route_is_safe_for_zones(
                first,
                second,
                obstacle_clearance,
                active_zones,
                model_poses,
            ):
                continue
            distance = math.hypot(
                second[0] - first[0], second[1] - first[1]
            )
            neighbors[first_index].append((second_index, distance))
            neighbors[second_index].append((first_index, distance))

    distances = [float('inf')] * len(nodes)
    previous = [-1] * len(nodes)
    distances[0] = 0.0
    queue = [(0.0, 0)]
    while queue:
        distance_so_far, node_index = heapq.heappop(queue)
        if distance_so_far > distances[node_index] + 1e-12:
            continue
        if node_index == 1:
            break
        for next_index, edge_length in neighbors[node_index]:
            new_distance = distance_so_far + edge_length
            if new_distance >= distances[next_index] - 1e-12:
                continue
            distances[next_index] = new_distance
            previous[next_index] = node_index
            heapq.heappush(queue, (new_distance, next_index))

    if not math.isfinite(distances[1]):
        return None

    path_indices = []
    node_index = 1
    while node_index >= 0:
        path_indices.append(node_index)
        if node_index == 0:
            break
        node_index = previous[node_index]
    if not path_indices or path_indices[-1] != 0:
        return None
    path = [nodes[index] for index in reversed(path_indices)]

    # Remove redundant graph vertices whenever the same safe line of sight is
    # available. This also collapses collinear samples along obstacle edges.
    simplified = [path[0]]
    current_index = 0
    while current_index < len(path) - 1:
        next_index = len(path) - 1
        while next_index > current_index + 1:
            if _straight_route_is_safe_for_zones(
                path[current_index],
                path[next_index],
                obstacle_clearance,
                active_zones,
                model_poses,
            ):
                break
            next_index -= 1
        simplified.append(path[next_index])
        current_index = next_index
    return simplified


def _search_axis(minimum: float, maximum: float, step: float) -> List[float]:
    """Return grid offsets plus exact endpoints for narrow feasible ranges."""

    first = int(math.ceil((minimum - 1e-9) / step))
    last = int(math.floor((maximum + 1e-9) / step))
    values = {
        round(minimum, 10),
        round(maximum, 10),
    }
    values.update(round(index * step, 10) for index in range(first, last + 1))
    if minimum <= 0.0 <= maximum:
        values.add(0.0)
    return sorted(value for value in values if minimum - 1e-9 <= value <= maximum + 1e-9)


def _formation_center_candidates(
    offsets: Sequence[Point2],
    preferred_center: Point2,
    arena_size: float,
    arena_margin: float,
    search_step: float,
) -> List[Point2]:
    """Return feasible centers ordered by distance from the fleet center."""

    if not offsets:
        return [preferred_center]
    if search_step <= 0.0 or not math.isfinite(search_step):
        raise ValueError('formation search step must be finite and positive')
    if arena_size <= 0.0 or not math.isfinite(arena_size):
        raise ValueError('arena size must be finite and positive')
    if arena_margin < 0.0 or not math.isfinite(arena_margin):
        raise ValueError('arena margin must be finite and non-negative')
    if not all(math.isfinite(value) for point in offsets for value in point):
        return []
    if not all(math.isfinite(value) for value in preferred_center):
        return []

    usable_half = arena_size / 2.0 - arena_margin
    if usable_half <= 0.0:
        return []

    center_min_x = -usable_half - min(x for x, _ in offsets)
    center_max_x = usable_half - max(x for x, _ in offsets)
    center_min_y = -usable_half - min(y for _, y in offsets)
    center_max_y = usable_half - max(y for _, y in offsets)
    if center_min_x > center_max_x or center_min_y > center_max_y:
        return []

    preferred_x, preferred_y = preferred_center
    x_offsets = _search_axis(
        center_min_x - preferred_x,
        center_max_x - preferred_x,
        search_step,
    )
    y_offsets = _search_axis(
        center_min_y - preferred_y,
        center_max_y - preferred_y,
        search_step,
    )
    center_offsets = [
        (delta_x, delta_y)
        for delta_x in x_offsets
        for delta_y in y_offsets
    ]
    center_offsets.sort(key=lambda offset: (
        round(offset[0] * offset[0] + offset[1] * offset[1], 12),
        abs(offset[0]) + abs(offset[1]),
        abs(offset[0]),
        0 if offset[1] >= 0.0 else 1,
        0 if offset[0] >= 0.0 else 1,
    ))
    return [
        (preferred_x + delta_x, preferred_y + delta_y)
        for delta_x, delta_y in center_offsets
    ]


def find_safe_formation_center(
    offsets: Sequence[Point2],
    preferred_center: Point2,
    arena_size: float,
    arena_margin: float,
    obstacle_clearance: float,
    search_step: float,
    exclusion_zones: Sequence[Mapping],
    arena_profile: str,
    model_poses: Optional[ModelPoseMap] = None,
) -> Optional[Point2]:
    """Find the nearest whole-pattern placement that is safe for every slot.

    The requested shape is translated as one piece, so its scale, spacing and
    robot-to-slot assignment remain unchanged. The preferred center is tested
    first and is retained whenever it is already safe.
    """

    if obstacle_clearance < 0.0 or not math.isfinite(obstacle_clearance):
        raise ValueError('obstacle clearance must be finite and non-negative')

    preferred_targets = [
        (preferred_center[0] + offset_x, preferred_center[1] + offset_y)
        for offset_x, offset_y in offsets
    ]
    if formation_targets_are_safe(
        preferred_targets,
        arena_size,
        arena_margin,
        obstacle_clearance,
        exclusion_zones,
        arena_profile,
        model_poses,
    ):
        return preferred_center

    for center in _formation_center_candidates(
        offsets,
        preferred_center,
        arena_size,
        arena_margin,
        search_step,
    ):
        if center == preferred_center:
            continue
        targets = [
            (center[0] + offset_x, center[1] + offset_y)
            for offset_x, offset_y in offsets
        ]
        if formation_targets_are_safe(
            targets,
            arena_size,
            arena_margin,
            obstacle_clearance,
            exclusion_zones,
            arena_profile,
            model_poses,
        ):
            return center
    return None


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


def collision_free_assignment(
    robot_positions: Sequence[Point2],
    target_positions: Sequence[Point2],
    obstacle_clearance: float,
    exclusion_zones: Sequence[Mapping],
    arena_profile: str,
    model_poses: Optional[ModelPoseMap] = None,
    previous_slots: Optional[Sequence[Optional[int]]] = None,
    switch_penalty: float = 0.0,
) -> Optional[List[int]]:
    """Find the shortest perfect assignment whose straight routes are clear."""

    if len(robot_positions) != len(target_positions):
        raise ValueError("robot and target counts must match")
    if obstacle_clearance < 0.0 or not math.isfinite(obstacle_clearance):
        raise ValueError('route clearance must be finite and non-negative')
    if not robot_positions:
        return []

    allowed: List[List[bool]] = []
    costs: List[List[float]] = []
    largest_cost = 0.0
    active_zones = _active_zones(exclusion_zones, arena_profile)
    for robot_index, robot_position in enumerate(robot_positions):
        previous = (
            previous_slots[robot_index]
            if previous_slots is not None and robot_index < len(previous_slots)
            else None
        )
        allowed_row: List[bool] = []
        cost_row: List[float] = []
        for slot_index, target_position in enumerate(target_positions):
            route_allowed = _straight_route_is_safe_for_zones(
                robot_position,
                target_position,
                obstacle_clearance,
                active_zones,
                model_poses,
            )
            route_cost = (
                (robot_position[0] - target_position[0]) ** 2
                + (robot_position[1] - target_position[1]) ** 2
            )
            if previous is not None and previous != slot_index:
                route_cost += max(0.0, switch_penalty)
            allowed_row.append(route_allowed)
            cost_row.append(route_cost)
            largest_cost = max(largest_cost, route_cost)
        allowed.append(allowed_row)
        costs.append(cost_row)

    if any(not any(row) for row in allowed):
        return None
    if any(not any(row[column] for row in allowed)
           for column in range(len(target_positions))):
        return None

    # One blocked edge must cost more than the worst complete clear solution.
    # The assignment is verified below, so this remains safe even when the
    # clear-edge graph has no perfect matching.
    blocked_cost = (largest_cost + 1.0) * (len(robot_positions) + 1)
    penalized_costs = [
        [
            route_cost if route_allowed else blocked_cost
            for route_cost, route_allowed in zip(cost_row, allowed_row)
        ]
        for cost_row, allowed_row in zip(costs, allowed)
    ]
    assignment = hungarian_assignment(penalized_costs)
    if any(
        slot_index < 0 or not allowed[robot_index][slot_index]
        for robot_index, slot_index in enumerate(assignment)
    ):
        return None
    return assignment


def find_safe_formation_plan(
    offsets: Sequence[Point2],
    preferred_center: Point2,
    robot_positions: Sequence[Point2],
    arena_size: float,
    arena_margin: float,
    obstacle_clearance: float,
    search_step: float,
    exclusion_zones: Sequence[Mapping],
    arena_profile: str,
    model_poses: Optional[ModelPoseMap] = None,
    previous_slots: Optional[Sequence[Optional[int]]] = None,
    switch_penalty: float = 0.0,
    maximum_center_candidates: int = 512,
) -> Optional[Tuple[Point2, List[Point2], List[int]]]:
    """Place and assign a static formation without routing through obstacles.

    The preferred whole-pattern center wins whenever it has a collision-free
    perfect assignment. Otherwise centers are tried by increasing translation
    distance, keeping the formation's orientation, scale and spacing intact.
    """

    if len(offsets) != len(robot_positions):
        raise ValueError("robot and formation slot counts must match")
    if obstacle_clearance < 0.0 or not math.isfinite(obstacle_clearance):
        raise ValueError('obstacle clearance must be finite and non-negative')
    if maximum_center_candidates <= 0:
        raise ValueError('formation center candidate limit must be positive')
    if not offsets:
        return preferred_center, [], []

    preferred_targets = [
        (preferred_center[0] + offset_x, preferred_center[1] + offset_y)
        for offset_x, offset_y in offsets
    ]
    if formation_targets_are_safe(
        preferred_targets,
        arena_size,
        arena_margin,
        obstacle_clearance,
        exclusion_zones,
        arena_profile,
        model_poses,
    ):
        preferred_assignment = collision_free_assignment(
            robot_positions,
            preferred_targets,
            obstacle_clearance,
            exclusion_zones,
            arena_profile,
            model_poses,
            previous_slots,
            switch_penalty,
        )
        if preferred_assignment is not None:
            return preferred_center, preferred_targets, preferred_assignment

    alternate_candidates_tested = 0
    for center in _formation_center_candidates(
        offsets,
        preferred_center,
        arena_size,
        arena_margin,
        search_step,
    ):
        if center == preferred_center:
            continue
        targets = [
            (center[0] + offset_x, center[1] + offset_y)
            for offset_x, offset_y in offsets
        ]
        if not formation_targets_are_safe(
            targets,
            arena_size,
            arena_margin,
            obstacle_clearance,
            exclusion_zones,
            arena_profile,
            model_poses,
        ):
            continue

        # The limit bounds the comparatively expensive assignment attempts,
        # not cheap centers that cannot hold the formation in the first place.
        if alternate_candidates_tested >= maximum_center_candidates:
            break
        alternate_candidates_tested += 1

        assignment = collision_free_assignment(
            robot_positions,
            targets,
            obstacle_clearance,
            exclusion_zones,
            arena_profile,
            model_poses,
            previous_slots,
            switch_penalty,
        )
        if assignment is not None:
            return center, targets, assignment
    return None


def _glyph_edges(points: Sequence[Point2]) -> List[Tuple[Point2, Point2]]:
    """Connect neighboring grid cells without relying on glyph list order."""

    maximum_step = math.sqrt(2.0) + 1e-9
    edges: List[Tuple[Point2, Point2]] = []
    for index, start in enumerate(points):
        for end in points[index + 1:]:
            distance = math.hypot(end[0] - start[0], end[1] - start[1])
            if 1e-9 < distance <= maximum_step:
                edges.append((start, end))
    return edges


def _dense_glyph_candidates(
    points: Sequence[Point2], desired_count: int
) -> List[Point2]:
    """Densify the grid skeleton while merging shared stroke crossings."""

    edges = _glyph_edges(points)
    if not edges:
        return list(points)

    total_length = sum(
        math.hypot(end[0] - start[0], end[1] - start[1])
        for start, end in edges
    )
    density = max(4.0, (desired_count * 3.0) / max(total_length, 1.0))

    while True:
        candidates = list(points)
        for start, end in edges:
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            subdivisions = max(1, int(math.ceil(length * density)))
            for index in range(1, subdivisions):
                fraction = index / subdivisions
                candidates.append((
                    start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]),
                ))

        # Rounding joins the same crossing reached from two different edges.
        deduplicated = list({
            (round(point[0], 10), round(point[1], 10)): point
            for point in candidates
        }.values())
        if len(deduplicated) >= desired_count:
            return deduplicated
        density *= 2.0


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

    safe_spacing = max(float(spacing), 0.05)
    unique_points = list(dict.fromkeys(centered))
    if robot_count <= len(unique_points):
        # The 5x7 glyph cells are already one grid unit apart. Sampling those
        # cells directly keeps small fleets compact and avoids inventing two
        # almost-identical slots where strokes cross.
        candidates = unique_points
    else:
        candidates = _dense_glyph_candidates(unique_points, robot_count)
    sampled = _farthest_point_sample(candidates, robot_count)
    scaled = [
        (point[0] * safe_spacing, point[1] * safe_spacing)
        for point in sampled
    ]
    return ensure_minimum_spacing(scaled, safe_spacing)
