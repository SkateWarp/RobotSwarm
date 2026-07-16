"""Small state adapter used by the ROS collaborative transport node."""

import math
from typing import Sequence

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


def build_transport_snapshot(
    robots: Sequence[RobotSnapshot],
    object_center: Vec2,
    target: Vec2,
    obstacle_points: Sequence[Vec2] = (),
    object_radius: float = 0.2,
    contour_samples: int = 16,
) -> TransportSnapshot:
    contour = centered_object_contour(
        object_center,
        object_radius,
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
