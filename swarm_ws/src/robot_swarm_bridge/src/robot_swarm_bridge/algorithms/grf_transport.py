"""Deterministic Gibbs Random Field kernel for cooperative transport.

This module contains no ROS or Gazebo dependencies. It implements the
mathematical ideas described by Rezeck, Assuncao, and Chaimowicz in
"Cooperative Object Transportation using Gibbs Random Fields" (IROS 2021):

* paper-correct Coulomb-Buckingham potentials;
* local robot neighborhoods;
* obstacle repulsion;
* object-contour orbiting and goal-directed pushing preferences;
* velocity consensus; and
* bounded Metropolis-Hastings sampling.

The task state machine, sensing adapters, collision safety layer, and
differential-drive projection intentionally remain outside this kernel.
"""

from __future__ import annotations

import hashlib
import math
import random
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple


_GEOMETRY_EPSILON = 1.0e-9
_EXPONENT_LIMIT = 700.0


@dataclass(frozen=True)
class Vec2:
    """A lightweight immutable two-dimensional vector."""

    x: float = 0.0
    y: float = 0.0

    def __add__(self, other: 'Vec2') -> 'Vec2':
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: 'Vec2') -> 'Vec2':
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> 'Vec2':
        return Vec2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> 'Vec2':
        return self * scalar

    def __truediv__(self, scalar: float) -> 'Vec2':
        if abs(scalar) <= _GEOMETRY_EPSILON:
            return Vec2()
        return Vec2(self.x / scalar, self.y / scalar)

    def dot(self, other: 'Vec2') -> float:
        return self.x * other.x + self.y * other.y

    def cross(self, other: 'Vec2') -> float:
        return self.x * other.y - self.y * other.x

    def norm_squared(self) -> float:
        return self.x * self.x + self.y * self.y

    def norm(self) -> float:
        return math.hypot(self.x, self.y)

    def normalized(self) -> 'Vec2':
        magnitude = self.norm()
        if not math.isfinite(magnitude) or magnitude <= _GEOMETRY_EPSILON:
            return Vec2()
        return self / magnitude

    def is_finite(self) -> bool:
        return math.isfinite(self.x) and math.isfinite(self.y)

    def bounded(self, maximum_norm: float) -> 'Vec2':
        """Return a finite vector with magnitude no greater than the bound."""

        if not self.is_finite() or maximum_norm <= 0.0:
            return Vec2()
        magnitude = self.norm()
        if magnitude <= maximum_norm:
            return self
        return self * (maximum_norm / magnitude)


@dataclass(frozen=True)
class Segment2:
    """A closed two-dimensional line segment."""

    start: Vec2
    end: Vec2

    def is_finite(self) -> bool:
        return self.start.is_finite() and self.end.is_finite()


@dataclass(frozen=True)
class RobotSnapshot:
    """World-frame state required by one GRF control step."""

    robot_id: str
    position: Vec2
    velocity: Vec2 = field(default_factory=Vec2)
    heading: float = 0.0

    def has_valid_position(self) -> bool:
        return bool(self.robot_id) and self.position.is_finite()

    def finite_velocity(self) -> Vec2:
        return self.velocity if self.velocity.is_finite() else Vec2()


@dataclass(frozen=True)
class ObjectContour:
    """Ordered object-boundary samples in the world frame."""

    points: Tuple[Vec2, ...]
    closed: bool = True

    def finite_points(self) -> Tuple[Vec2, ...]:
        return tuple(point for point in self.points if point.is_finite())

    def center(self) -> Vec2:
        points = self.finite_points()
        if not points:
            return Vec2()
        return Vec2(
            sum(point.x for point in points) / len(points),
            sum(point.y for point in points) / len(points),
        )


@dataclass(frozen=True)
class TransportSnapshot:
    """Complete immutable input for one cooperative-transport control step."""

    robots: Tuple[RobotSnapshot, ...]
    object_contour: ObjectContour
    target: Vec2
    obstacle_points: Tuple[Vec2, ...] = field(default_factory=tuple)
    occluding_segments: Tuple[Segment2, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class PotentialParameters:
    """Coulomb-Buckingham potential parameters."""

    epsilon: float
    r0: float
    alpha: float
    q1: float
    q2: float
    epsilon0: float

    def validate(self, name: str) -> None:
        values = (
            self.epsilon,
            self.r0,
            self.alpha,
            self.q1,
            self.q2,
            self.epsilon0,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('{} parameters must be finite'.format(name))
        if self.epsilon < 0.0:
            raise ValueError('{} epsilon must be non-negative'.format(name))
        if self.r0 <= 0.0:
            raise ValueError('{} r0 must be positive'.format(name))
        if self.epsilon0 <= 0.0:
            raise ValueError('{} epsilon0 must be positive'.format(name))
        if abs(self.alpha - 6.0) <= 1.0e-8:
            raise ValueError('{} alpha must not equal 6'.format(name))


def _obstacle_defaults() -> PotentialParameters:
    return PotentialParameters(
        epsilon=0.04,
        r0=0.18,
        alpha=0.05,
        q1=1.0,
        q2=1.0,
        epsilon0=0.04,
    )


def _object_orbit_defaults() -> PotentialParameters:
    return PotentialParameters(
        epsilon=0.04,
        r0=0.30,
        alpha=0.20,
        q1=100.0,
        q2=-1.0,
        epsilon0=0.04,
    )


def _object_push_defaults() -> PotentialParameters:
    return PotentialParameters(
        epsilon=0.04,
        r0=0.12,
        alpha=0.20,
        q1=100.0,
        q2=-1.0,
        epsilon0=0.04,
    )


def _robot_defaults() -> PotentialParameters:
    return PotentialParameters(
        epsilon=0.04,
        r0=0.65,
        alpha=0.65,
        q1=50.0,
        q2=-1.0,
        epsilon0=0.04,
    )


@dataclass(frozen=True)
class GRFConfig:
    """Validated tuning and sampling configuration for the GRF kernel."""

    time_step: float = 0.1
    max_speed: float = 0.15
    sensing_radius: float = 2.0
    obstacle_influence_radius: float = 0.5
    robot_mass: float = 1.0
    object_mass: float = 1.0
    mcmc_iterations: int = 60
    proposal_sigma: float = 0.05
    burn_in_fraction: float = 0.60
    temperature: float = 1.0
    random_seed: int = 0
    minimum_distance: float = 1.0e-4
    energy_limit: float = 1.0e6
    object_potential_weight: float = 0.02
    neighbor_potential_weight: float = 0.02
    orbit_alignment_weight: float = 60.0
    push_alignment_weight: float = 80.0
    velocity_consensus_weight: float = 12.0
    cruise_speed_weight: float = 0.2
    obstacle_potential: PotentialParameters = field(
        default_factory=_obstacle_defaults
    )
    object_orbit_potential: PotentialParameters = field(
        default_factory=_object_orbit_defaults
    )
    object_push_potential: PotentialParameters = field(
        default_factory=_object_push_defaults
    )
    robot_potential: PotentialParameters = field(
        default_factory=_robot_defaults
    )

    def __post_init__(self) -> None:
        finite_positive = {
            'time_step': self.time_step,
            'max_speed': self.max_speed,
            'sensing_radius': self.sensing_radius,
            'obstacle_influence_radius': self.obstacle_influence_radius,
            'robot_mass': self.robot_mass,
            'object_mass': self.object_mass,
            'proposal_sigma': self.proposal_sigma,
            'temperature': self.temperature,
            'minimum_distance': self.minimum_distance,
            'energy_limit': self.energy_limit,
        }
        for name, value in finite_positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError('{} must be finite and positive'.format(name))
        if self.mcmc_iterations <= 0:
            raise ValueError('mcmc_iterations must be positive')
        if not 0.0 <= self.burn_in_fraction < 1.0:
            raise ValueError('burn_in_fraction must be in [0, 1)')
        for name, value in {
            'object_potential_weight': self.object_potential_weight,
            'neighbor_potential_weight': self.neighbor_potential_weight,
            'orbit_alignment_weight': self.orbit_alignment_weight,
            'push_alignment_weight': self.push_alignment_weight,
            'velocity_consensus_weight': self.velocity_consensus_weight,
            'cruise_speed_weight': self.cruise_speed_weight,
        }.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError('{} must be finite and non-negative'.format(name))
        self.obstacle_potential.validate('obstacle_potential')
        self.object_orbit_potential.validate('object_orbit_potential')
        self.object_push_potential.validate('object_push_potential')
        self.robot_potential.validate('robot_potential')


class InteractionMode(str, Enum):
    """Local object interaction selected for an individual robot."""

    INVALID = 'INVALID'
    NO_OBJECT = 'NO_OBJECT'
    ORBIT = 'ORBIT'
    PUSH = 'PUSH'


@dataclass(frozen=True)
class VelocityEvaluation:
    """Energy breakdown for a proposed world-frame robot velocity."""

    total_energy: float
    obstacle_energy: float
    object_energy: float
    neighbor_energy: float
    motion_energy: float
    interaction_mode: InteractionMode
    preferred_direction: Vec2


@dataclass(frozen=True)
class RobotVelocityResult:
    """Sampled world-frame command and diagnostics for one robot."""

    robot_id: str
    velocity: Vec2
    interaction_mode: InteractionMode
    total_energy: float
    accepted_proposals: int
    proposal_count: int
    neighbor_ids: Tuple[str, ...]
    preferred_direction: Vec2


@dataclass(frozen=True)
class GRFStepResult:
    """Deterministically ordered results for a complete swarm step."""

    robots: Tuple[RobotVelocityResult, ...]

    def command_for(self, robot_id: str) -> RobotVelocityResult:
        for result in self.robots:
            if result.robot_id == robot_id:
                return result
        raise KeyError(robot_id)


NeighborGraph = Mapping[str, Tuple[RobotSnapshot, ...]]


@dataclass(frozen=True)
class _NeighborState:
    """Proposal-independent state of one locally visible neighbor."""

    robot_id: str
    predicted_position: Vec2
    velocity: Vec2


@dataclass(frozen=True)
class _RobotEvaluationContext:
    """Static per-robot values reused by every MCMC proposal."""

    robot: RobotSnapshot
    valid: bool
    obstacle_points: Tuple[Vec2, ...]
    visible_object_points: Tuple[Vec2, ...]
    interaction_mode: InteractionMode
    preferred_direction: Vec2
    object_potential: Optional[PotentialParameters]
    object_alignment_weight: float
    neighbors: Tuple[_NeighborState, ...]

    @property
    def neighbor_ids(self) -> Tuple[str, ...]:
        return tuple(neighbor.robot_id for neighbor in self.neighbors)


def _clip_energy(value: float, limit: float) -> float:
    if math.isnan(value):
        return limit
    if value == math.inf:
        return limit
    if value == -math.inf:
        return -limit
    return max(-limit, min(limit, value))


def _energy_sum(values: Iterable[float], limit: float) -> float:
    total = 0.0
    for value in values:
        total = _clip_energy(total + _clip_energy(value, limit), limit)
    return total


def _energy_mean(values: Iterable[float], limit: float) -> float:
    """Average an interaction family without depending on sample count."""

    total = 0.0
    count = 0
    for value in values:
        total += _clip_energy(value, limit)
        count += 1
    if count == 0:
        return 0.0
    return _clip_energy(total / count, limit)


def coulomb_buckingham(
    distance: float,
    parameters: PotentialParameters,
    minimum_distance: float = 1.0e-4,
    energy_limit: float = 1.0e6,
) -> float:
    """Evaluate the paper's Coulomb-Buckingham equation safely.

    The exponential is ``exp(alpha * (1 - r / r0))``. This placement of the
    parentheses follows Equation 4 in the paper.
    """

    parameters.validate('potential')
    if not math.isfinite(minimum_distance) or minimum_distance <= 0.0:
        raise ValueError('minimum_distance must be finite and positive')
    if not math.isfinite(energy_limit) or energy_limit <= 0.0:
        raise ValueError('energy_limit must be finite and positive')

    return _coulomb_buckingham_unchecked(
        distance,
        parameters,
        minimum_distance,
        energy_limit,
    )


def _coulomb_buckingham_unchecked(
    distance: float,
    parameters: PotentialParameters,
    minimum_distance: float,
    energy_limit: float,
) -> float:
    """Fast path for parameters already validated by ``GRFConfig``."""

    if not math.isfinite(distance):
        return energy_limit
    radius = max(abs(distance), minimum_distance)
    denominator = parameters.alpha - 6.0
    exponent = parameters.alpha * (1.0 - radius / parameters.r0)
    exponent = max(-_EXPONENT_LIMIT, min(_EXPONENT_LIMIT, exponent))

    try:
        buckingham = parameters.epsilon * (
            (6.0 / denominator) * math.exp(exponent)
            - (parameters.alpha / denominator)
            * math.pow(parameters.r0 / radius, 6)
        )
        coulomb = (
            parameters.q1
            * parameters.q2
            / (4.0 * math.pi * parameters.epsilon0 * radius)
        )
        return _clip_energy(buckingham + coulomb, energy_limit)
    except (OverflowError, ValueError, ZeroDivisionError):
        return energy_limit


def _orientation(a: Vec2, b: Vec2, c: Vec2) -> float:
    return (b - a).cross(c - a)


def _point_on_segment(point: Vec2, segment: Segment2) -> bool:
    if abs(_orientation(segment.start, segment.end, point)) > 1.0e-8:
        return False
    return (
        min(segment.start.x, segment.end.x) - _GEOMETRY_EPSILON
        <= point.x
        <= max(segment.start.x, segment.end.x) + _GEOMETRY_EPSILON
        and min(segment.start.y, segment.end.y) - _GEOMETRY_EPSILON
        <= point.y
        <= max(segment.start.y, segment.end.y) + _GEOMETRY_EPSILON
    )


def _segments_intersect(first: Segment2, second: Segment2) -> bool:
    if not first.is_finite() or not second.is_finite():
        return False
    o1 = _orientation(first.start, first.end, second.start)
    o2 = _orientation(first.start, first.end, second.end)
    o3 = _orientation(second.start, second.end, first.start)
    o4 = _orientation(second.start, second.end, first.end)

    if (
        ((o1 > 0.0 and o2 < 0.0) or (o1 < 0.0 and o2 > 0.0))
        and ((o3 > 0.0 and o4 < 0.0) or (o3 < 0.0 and o4 > 0.0))
    ):
        return True
    return (
        (abs(o1) <= 1.0e-8 and _point_on_segment(second.start, first))
        or (abs(o2) <= 1.0e-8 and _point_on_segment(second.end, first))
        or (abs(o3) <= 1.0e-8 and _point_on_segment(first.start, second))
        or (abs(o4) <= 1.0e-8 and _point_on_segment(first.end, second))
    )


def _line_of_sight_clear(
    start: Vec2,
    end: Vec2,
    occluders: Sequence[Segment2],
) -> bool:
    sight_line = Segment2(start, end)
    return not any(
        segment.is_finite() and _segments_intersect(sight_line, segment)
        for segment in occluders
    )


def build_local_neighbor_graph(
    robots: Sequence[RobotSnapshot],
    sensing_radius: float,
    occluding_segments: Sequence[Segment2] = (),
) -> Dict[str, Tuple[RobotSnapshot, ...]]:
    """Build a deterministic, range- and visibility-limited neighbor graph.

    A uniform spatial grid avoids comparing every pair when the swarm occupies
    an area larger than one sensing neighborhood.
    """

    if not math.isfinite(sensing_radius) or sensing_radius <= 0.0:
        raise ValueError('sensing_radius must be finite and positive')

    ids = [robot.robot_id for robot in robots]
    if len(ids) != len(set(ids)):
        raise ValueError('robot IDs must be unique')

    valid = sorted(
        (robot for robot in robots if robot.has_valid_position()),
        key=lambda robot: robot.robot_id,
    )
    graph_lists: Dict[str, list] = {robot.robot_id: [] for robot in robots}
    cells: Dict[Tuple[int, int], list] = {}

    def cell_for(position: Vec2) -> Tuple[int, int]:
        return (
            math.floor(position.x / sensing_radius),
            math.floor(position.y / sensing_radius),
        )

    for robot in valid:
        cells.setdefault(cell_for(robot.position), []).append(robot)

    radius_squared = sensing_radius * sensing_radius
    for robot in valid:
        cell_x, cell_y = cell_for(robot.position)
        for offset_x in (-1, 0, 1):
            for offset_y in (-1, 0, 1):
                for other in cells.get(
                    (cell_x + offset_x, cell_y + offset_y), ()
                ):
                    if other.robot_id <= robot.robot_id:
                        continue
                    if (
                        other.position - robot.position
                    ).norm_squared() > radius_squared:
                        continue
                    if not _line_of_sight_clear(
                        robot.position,
                        other.position,
                        occluding_segments,
                    ):
                        continue
                    graph_lists[robot.robot_id].append(other)
                    graph_lists[other.robot_id].append(robot)

    return {
        robot_id: tuple(sorted(neighbors, key=lambda robot: robot.robot_id))
        for robot_id, neighbors in graph_lists.items()
    }


def _contour_segments(contour: ObjectContour) -> Tuple[Segment2, ...]:
    points = contour.finite_points()
    if len(points) < 2:
        return ()
    segments = [
        Segment2(points[index], points[index + 1])
        for index in range(len(points) - 1)
    ]
    if contour.closed and len(points) >= 3:
        segments.append(Segment2(points[-1], points[0]))
    return tuple(segments)


def target_is_occluded(
    robot_position: Vec2,
    target: Vec2,
    contour: ObjectContour,
) -> bool:
    """Return whether the object lies between a robot and the target."""

    if not robot_position.is_finite() or not target.is_finite():
        return False
    target_line = Segment2(robot_position, target)
    return any(
        _segments_intersect(target_line, edge)
        for edge in _contour_segments(contour)
    )


def _nearest_contour_tangent(
    robot_position: Vec2,
    contour: ObjectContour,
    target: Vec2,
) -> Vec2:
    segments = _contour_segments(contour)
    if not segments:
        return Vec2()

    def squared_distance_to_segment(segment: Segment2) -> float:
        edge = segment.end - segment.start
        edge_length_squared = edge.norm_squared()
        if edge_length_squared <= _GEOMETRY_EPSILON:
            return (robot_position - segment.start).norm_squared()
        projection = (
            (robot_position - segment.start).dot(edge) / edge_length_squared
        )
        projection = max(0.0, min(1.0, projection))
        closest = segment.start + edge * projection
        return (robot_position - closest).norm_squared()

    nearest_segment = min(segments, key=squared_distance_to_segment)
    contour_tangent = (
        nearest_segment.end - nearest_segment.start
    ).normalized()
    if contour_tangent.norm_squared() <= _GEOMETRY_EPSILON:
        return Vec2()

    center = contour.center()
    radial = (robot_position - center).normalized()
    desired_behind = (center - target).normalized()
    if (
        radial.norm_squared() <= _GEOMETRY_EPSILON
        or desired_behind.norm_squared() <= _GEOMETRY_EPSILON
    ):
        return contour_tangent

    turn_sign = radial.cross(desired_behind)
    if abs(turn_sign) <= 1.0e-8:
        return contour_tangent
    desired_turn_tangent = Vec2(-radial.y, radial.x)
    if turn_sign < 0.0:
        desired_turn_tangent = desired_turn_tangent * -1.0
    if contour_tangent.dot(desired_turn_tangent) < 0.0:
        contour_tangent = contour_tangent * -1.0
    return contour_tangent


def _seed_for(base_seed: int, step_index: int, robot_id: str) -> int:
    payload = '{}:{}:{}'.format(base_seed, step_index, robot_id).encode('utf-8')
    digest = hashlib.blake2b(payload, digest_size=8).digest()
    return int.from_bytes(digest, byteorder='big', signed=False)


class GibbsRandomFieldTransport:
    """ROS-independent cooperative-transport velocity sampler."""

    def __init__(self, config: Optional[GRFConfig] = None):
        self.config = config or GRFConfig()

    def evaluate_velocity(
        self,
        snapshot: TransportSnapshot,
        robot_id: str,
        velocity: Vec2,
        neighbor_graph: Optional[NeighborGraph] = None,
    ) -> VelocityEvaluation:
        """Evaluate one bounded proposed velocity without sampling."""

        robots_by_id = self._robots_by_id(snapshot.robots)
        try:
            robot = robots_by_id[robot_id]
        except KeyError:
            raise KeyError('unknown robot ID: {}'.format(robot_id))
        graph = (
            neighbor_graph
            if neighbor_graph is not None
            else build_local_neighbor_graph(
                snapshot.robots,
                self.config.sensing_radius,
                snapshot.occluding_segments,
            )
        )
        context = self._build_context(
            snapshot,
            robot,
            graph.get(robot_id, ()),
        )
        return self._evaluate(
            context,
            velocity.bounded(self.config.max_speed),
        )

    def compute(
        self,
        snapshot: TransportSnapshot,
        step_index: int = 0,
    ) -> GRFStepResult:
        """Sample a bounded preferred velocity for every robot."""

        robots_by_id = self._robots_by_id(snapshot.robots)
        graph = build_local_neighbor_graph(
            tuple(robots_by_id.values()),
            self.config.sensing_radius,
            snapshot.occluding_segments,
        )
        results = [
            self._sample_robot(
                self._build_context(
                    snapshot,
                    robot,
                    graph.get(robot.robot_id, ()),
                ),
                step_index,
            )
            for robot in sorted(
                robots_by_id.values(), key=lambda item: item.robot_id
            )
        ]
        return GRFStepResult(robots=tuple(results))

    @staticmethod
    def _robots_by_id(
        robots: Sequence[RobotSnapshot],
    ) -> Dict[str, RobotSnapshot]:
        result: Dict[str, RobotSnapshot] = {}
        for robot in robots:
            if not robot.robot_id:
                raise ValueError('robot IDs must not be empty')
            if robot.robot_id in result:
                raise ValueError('duplicate robot ID: {}'.format(robot.robot_id))
            result[robot.robot_id] = robot
        return result

    def _build_context(
        self,
        snapshot: TransportSnapshot,
        robot: RobotSnapshot,
        neighbors: Sequence[RobotSnapshot],
    ) -> _RobotEvaluationContext:
        valid = robot.has_valid_position() and snapshot.target.is_finite()
        if not valid:
            return _RobotEvaluationContext(
                robot=robot,
                valid=False,
                obstacle_points=(),
                visible_object_points=(),
                interaction_mode=InteractionMode.INVALID,
                preferred_direction=Vec2(),
                object_potential=None,
                object_alignment_weight=0.0,
                neighbors=(),
            )

        obstacle_radius = min(
            self.config.sensing_radius,
            self.config.obstacle_influence_radius,
        )
        obstacle_radius_squared = obstacle_radius * obstacle_radius
        obstacle_points = tuple(
            point
            for point in snapshot.obstacle_points
            if point.is_finite()
            and (
                point - robot.position
            ).norm_squared() <= obstacle_radius_squared
        )

        sensing_squared = (
            self.config.sensing_radius * self.config.sensing_radius
        )
        visible_object_points = tuple(
            point
            for point in snapshot.object_contour.finite_points()
            if (
                point - robot.position
            ).norm_squared() <= sensing_squared
        )
        if visible_object_points:
            pushing = target_is_occluded(
                robot.position,
                snapshot.target,
                snapshot.object_contour,
            )
            if pushing:
                interaction_mode = InteractionMode.PUSH
                preferred_direction = (
                    snapshot.target - snapshot.object_contour.center()
                ).normalized()
                object_potential = self.config.object_push_potential
                object_alignment_weight = self.config.push_alignment_weight
            else:
                interaction_mode = InteractionMode.ORBIT
                preferred_direction = _nearest_contour_tangent(
                    robot.position,
                    snapshot.object_contour,
                    snapshot.target,
                )
                object_potential = self.config.object_orbit_potential
                object_alignment_weight = self.config.orbit_alignment_weight
        else:
            interaction_mode = InteractionMode.NO_OBJECT
            preferred_direction = Vec2()
            object_potential = None
            object_alignment_weight = 0.0

        neighbor_states = []
        for neighbor in neighbors:
            if not neighbor.has_valid_position():
                continue
            neighbor_velocity = (
                neighbor.finite_velocity().bounded(self.config.max_speed)
            )
            neighbor_states.append(
                _NeighborState(
                    robot_id=neighbor.robot_id,
                    predicted_position=(
                        neighbor.position
                        + neighbor_velocity * self.config.time_step
                    ),
                    velocity=neighbor_velocity,
                )
            )

        return _RobotEvaluationContext(
            robot=robot,
            valid=True,
            obstacle_points=obstacle_points,
            visible_object_points=visible_object_points,
            interaction_mode=interaction_mode,
            preferred_direction=preferred_direction,
            object_potential=object_potential,
            object_alignment_weight=object_alignment_weight,
            neighbors=tuple(neighbor_states),
        )

    def _sample_robot(
        self,
        context: _RobotEvaluationContext,
        step_index: int,
    ) -> RobotVelocityResult:
        robot = context.robot
        if not context.valid:
            return RobotVelocityResult(
                robot_id=robot.robot_id,
                velocity=Vec2(),
                interaction_mode=InteractionMode.INVALID,
                total_energy=self.config.energy_limit,
                accepted_proposals=0,
                proposal_count=0,
                neighbor_ids=context.neighbor_ids,
                preferred_direction=Vec2(),
            )

        rng = random.Random(
            _seed_for(self.config.random_seed, step_index, robot.robot_id)
        )
        current = robot.finite_velocity().bounded(self.config.max_speed)
        current_evaluation = self._evaluate(context, current)
        samples = []
        accepted = 0

        for _ in range(self.config.mcmc_iterations):
            proposed = Vec2(
                current.x + rng.gauss(0.0, self.config.proposal_sigma),
                current.y + rng.gauss(0.0, self.config.proposal_sigma),
            ).bounded(self.config.max_speed)
            proposed_evaluation = self._evaluate(context, proposed)
            energy_delta = (
                proposed_evaluation.total_energy
                - current_evaluation.total_energy
            )
            accept = energy_delta <= 0.0
            if not accept:
                exponent = max(
                    -_EXPONENT_LIMIT,
                    min(
                        0.0,
                        -energy_delta / self.config.temperature,
                    ),
                )
                accept = rng.random() < math.exp(exponent)
            if accept:
                current = proposed
                current_evaluation = proposed_evaluation
                accepted += 1
            samples.append(current)

        burn_in = int(self.config.burn_in_fraction * len(samples))
        kept = samples[burn_in:] or samples[-1:]
        average = Vec2(
            sum(sample.x for sample in kept) / len(kept),
            sum(sample.y for sample in kept) / len(kept),
        ).bounded(self.config.max_speed)
        final_evaluation = self._evaluate(context, average)
        return RobotVelocityResult(
            robot_id=robot.robot_id,
            velocity=average,
            interaction_mode=final_evaluation.interaction_mode,
            total_energy=final_evaluation.total_energy,
            accepted_proposals=accepted,
            proposal_count=self.config.mcmc_iterations,
            neighbor_ids=context.neighbor_ids,
            preferred_direction=final_evaluation.preferred_direction,
        )

    def _evaluate(
        self,
        context: _RobotEvaluationContext,
        velocity: Vec2,
    ) -> VelocityEvaluation:
        if not context.valid:
            return VelocityEvaluation(
                total_energy=self.config.energy_limit,
                obstacle_energy=0.0,
                object_energy=0.0,
                neighbor_energy=0.0,
                motion_energy=self.config.energy_limit,
                interaction_mode=InteractionMode.INVALID,
                preferred_direction=Vec2(),
            )

        velocity = velocity.bounded(self.config.max_speed)
        predicted_position = (
            context.robot.position + velocity * self.config.time_step
        )
        obstacle_energy = self._obstacle_energy(
            predicted_position,
            context.obstacle_points,
        )
        (
            object_energy,
            interaction_mode,
            preferred_direction,
        ) = self._object_energy(context, predicted_position, velocity)
        neighbor_energy = self._neighbor_energy(
            predicted_position,
            velocity,
            context.neighbors,
        )
        speed_error = self.config.max_speed - velocity.norm()
        motion_energy = _clip_energy(
            self.config.cruise_speed_weight
            * 0.5
            * self.config.robot_mass
            * speed_error
            * speed_error,
            self.config.energy_limit,
        )
        total = _energy_sum(
            (
                obstacle_energy,
                object_energy,
                neighbor_energy,
                motion_energy,
            ),
            self.config.energy_limit,
        )
        return VelocityEvaluation(
            total_energy=total,
            obstacle_energy=obstacle_energy,
            object_energy=object_energy,
            neighbor_energy=neighbor_energy,
            motion_energy=motion_energy,
            interaction_mode=interaction_mode,
            preferred_direction=preferred_direction,
        )

    def _obstacle_energy(
        self,
        predicted_position: Vec2,
        obstacle_points: Sequence[Vec2],
    ) -> float:
        return _energy_sum(
            (
                _coulomb_buckingham_unchecked(
                    (predicted_position - obstacle).norm(),
                    self.config.obstacle_potential,
                    self.config.minimum_distance,
                    self.config.energy_limit,
                )
                for obstacle in obstacle_points
            ),
            self.config.energy_limit,
        )

    def _object_energy(
        self,
        context: _RobotEvaluationContext,
        predicted_position: Vec2,
        velocity: Vec2,
    ) -> Tuple[float, InteractionMode, Vec2]:
        if (
            not context.visible_object_points
            or context.object_potential is None
        ):
            return 0.0, InteractionMode.NO_OBJECT, Vec2()

        # A contour is one geometric body, not one body per sample.  Averaging
        # keeps the energy stable when its resolution changes.
        potential_energy = self.config.object_potential_weight * _energy_mean(
            (
                _coulomb_buckingham_unchecked(
                    (predicted_position - point).norm(),
                    context.object_potential,
                    self.config.minimum_distance,
                    self.config.energy_limit,
                )
                for point in context.visible_object_points
            ),
            self.config.energy_limit,
        )

        desired_velocity = (
            context.preferred_direction * self.config.max_speed
        )
        mismatch = velocity - desired_velocity
        alignment_energy = (
            context.object_alignment_weight
            * 0.5
            * self.config.object_mass
            * mismatch.norm_squared()
        )
        return (
            _energy_sum(
                (potential_energy, alignment_energy),
                self.config.energy_limit,
            ),
            context.interaction_mode,
            context.preferred_direction,
        )

    def _neighbor_energy(
        self,
        predicted_position: Vec2,
        proposed_velocity: Vec2,
        neighbors: Sequence[_NeighborState],
    ) -> float:
        potential_values = []
        relative_velocity_squares = []
        valid_neighbor_count = 0

        for neighbor in neighbors:
            potential_values.append(
                _coulomb_buckingham_unchecked(
                    (
                        predicted_position - neighbor.predicted_position
                    ).norm(),
                    self.config.robot_potential,
                    self.config.minimum_distance,
                    self.config.energy_limit,
                )
            )
            relative_velocity_squares.append(
                (neighbor.velocity - proposed_velocity).norm_squared()
            )
            valid_neighbor_count += 1

        potential_energy = (
            self.config.neighbor_potential_weight
            * _energy_mean(potential_values, self.config.energy_limit)
        )
        if valid_neighbor_count == 0:
            return potential_energy
        mean_squared_mismatch = (
            sum(relative_velocity_squares) / valid_neighbor_count
        )
        consensus_energy = (
            self.config.velocity_consensus_weight
            * 0.5
            * self.config.robot_mass
            * mean_squared_mismatch
        )
        return _energy_sum(
            (potential_energy, consensus_energy),
            self.config.energy_limit,
        )
