"""Small geometry and state adapters used by collaborative transport."""

import math
from typing import Optional, Sequence, Tuple

from .grf_transport import (
    ObjectContour,
    RobotSnapshot,
    TransportSnapshot,
    Vec2,
)


def centered_object_contour(
    center: Vec2,
    radius: float,
    sample_count: int = 16,
) -> ObjectContour:
    if not center.is_finite():
        raise ValueError('object center must be finite')
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError('object radius must be finite and positive')
    if sample_count < 3:
        raise ValueError('object contour needs at least three samples')

    points = tuple(
        center
        + Vec2(
            radius * math.cos(2.0 * math.pi * index / sample_count),
            radius * math.sin(2.0 * math.pi * index / sample_count),
        )
        for index in range(sample_count)
    )
    return ObjectContour(points=points)


def oriented_box_contour(
    center: Vec2,
    half_width: float,
    half_height: float,
    yaw: float = 0.0,
    sample_count: int = 16,
) -> ObjectContour:
    """Sample an oriented box perimeter while retaining all four corners."""

    if not center.is_finite() or not math.isfinite(yaw):
        raise ValueError('box pose must be finite')
    if not math.isfinite(half_width) or half_width <= 0.0:
        raise ValueError('box half width must be finite and positive')
    if not math.isfinite(half_height) or half_height <= 0.0:
        raise ValueError('box half height must be finite and positive')
    if sample_count < 4:
        raise ValueError('box contour needs at least four samples')

    corners = (
        Vec2(-half_width, -half_height),
        Vec2(half_width, -half_height),
        Vec2(half_width, half_height),
        Vec2(-half_width, half_height),
    )
    edge_lengths = (
        2.0 * half_width,
        2.0 * half_height,
        2.0 * half_width,
        2.0 * half_height,
    )

    extra_samples = sample_count - len(corners)
    perimeter = sum(edge_lengths)
    shares = tuple(
        extra_samples * edge_length / perimeter
        for edge_length in edge_lengths
    )
    samples_per_edge = [1 + int(math.floor(share)) for share in shares]
    unassigned = sample_count - sum(samples_per_edge)
    ranked_edges = sorted(
        range(len(corners)),
        key=lambda index: (
            -(shares[index] - math.floor(shares[index])),
            index,
        ),
    )
    for edge_index in ranked_edges[:unassigned]:
        samples_per_edge[edge_index] += 1

    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    points = []
    for edge_index, edge_sample_count in enumerate(samples_per_edge):
        start = corners[edge_index]
        end = corners[(edge_index + 1) % len(corners)]
        edge = end - start
        for sample_index in range(edge_sample_count):
            local = start + edge * (sample_index / edge_sample_count)
            points.append(Vec2(
                center.x + local.x * cosine - local.y * sine,
                center.y + local.x * sine + local.y * cosine,
            ))

    return ObjectContour(points=tuple(points))


def build_transport_snapshot(
    robots: Sequence[RobotSnapshot],
    object_center: Vec2,
    target: Vec2,
    obstacle_points: Sequence[Vec2] = (),
    object_radius: float = 0.2,
    contour_samples: int = 16,
    object_half_width: Optional[float] = None,
    object_half_height: Optional[float] = None,
    object_yaw: float = 0.0,
) -> TransportSnapshot:
    if (object_half_width is None) != (object_half_height is None):
        raise ValueError('object box requires both half extents')

    if object_half_width is None:
        contour = centered_object_contour(
            object_center,
            object_radius,
            contour_samples,
        )
    else:
        contour = oriented_box_contour(
            object_center,
            object_half_width,
            object_half_height,
            object_yaw,
            contour_samples,
        )
    return TransportSnapshot(
        robots=tuple(robots),
        object_contour=contour,
        target=target,
        obstacle_points=tuple(
            point for point in obstacle_points if point.is_finite()
        ),
    )


def mcmc_iterations_for_fleet(
    fleet_size: int,
    normal_iterations: int,
    large_fleet_threshold: int,
    large_fleet_iterations: int,
) -> int:
    if fleet_size < 0:
        raise ValueError('fleet size must be non-negative')
    if normal_iterations <= 0 or large_fleet_iterations <= 0:
        raise ValueError('MCMC iteration counts must be positive')
    if large_fleet_threshold <= 0:
        raise ValueError('large fleet threshold must be positive')
    if fleet_size >= large_fleet_threshold:
        return min(normal_iterations, large_fleet_iterations)
    return normal_iterations


def contact_center_distance(
    object_radius: float,
    robot_radius: float,
    contact_margin: float,
) -> float:
    """Return the centre-to-centre distance used to declare contact ready.

    The state machine tracks model centres, while useful pushing starts when
    the two footprints are almost touching.  Keeping that conversion here
    prevents the much larger sensor range from accidentally becoming a
    contact test.
    """

    values = (object_radius, robot_radius, contact_margin)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('contact geometry must be finite')
    if object_radius <= 0.0 or robot_radius <= 0.0:
        raise ValueError('object and robot radii must be positive')
    if contact_margin < 0.0:
        raise ValueError('contact margin must be non-negative')
    return object_radius + robot_radius + contact_margin


def occupied_lidar_sectors(
    robot_position: Vec2,
    robot_heading: float,
    object_center: Vec2,
    object_radius: float,
    maximum_distance: float,
    sector_count: int = 8,
) -> Tuple[int, ...]:
    """Find LiDAR sectors occupied by a nearby, known transport object.

    The result is intentionally empty outside the short contact corridor.
    It is used only to stop the generic obstacle filter from rejecting the
    object that a robot is meant to touch; robot-to-robot avoidance remains
    active in the safety layer.
    """

    if not robot_position.is_finite() or not object_center.is_finite():
        return ()
    if not math.isfinite(robot_heading):
        return ()
    if not math.isfinite(object_radius) or object_radius <= 0.0:
        raise ValueError('object radius must be finite and positive')
    if not math.isfinite(maximum_distance) or maximum_distance <= 0.0:
        raise ValueError('maximum distance must be finite and positive')
    if sector_count <= 0:
        raise ValueError('sector count must be positive')

    offset = object_center - robot_position
    distance = offset.norm()
    if distance > maximum_distance:
        return ()
    if distance <= object_radius:
        return tuple(range(sector_count))

    width = 2.0 * math.pi / sector_count
    bearing = math.atan2(offset.y, offset.x) - robot_heading
    bearing = (bearing + math.pi) % (2.0 * math.pi) - math.pi
    half_object_angle = math.asin(min(1.0, object_radius / distance))
    half_sector_angle = 0.5 * width

    occupied = []
    for index in range(sector_count):
        centre = index * width
        difference = (centre - bearing + math.pi) % (2.0 * math.pi) - math.pi
        if abs(difference) <= half_object_angle + half_sector_angle:
            occupied.append(index)
    return tuple(occupied)
