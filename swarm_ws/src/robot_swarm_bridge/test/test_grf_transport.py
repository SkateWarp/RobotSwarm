"""Unit tests for the ROS-independent cooperative-transport GRF kernel."""

import math
import sys
import unittest
from pathlib import Path


PACKAGE_SOURCE = Path(__file__).resolve().parents[1] / 'src'
sys.path.insert(0, str(PACKAGE_SOURCE))

from robot_swarm_bridge.algorithms import (  # noqa: E402
    GRFConfig,
    GibbsRandomFieldTransport,
    InteractionMode,
    ObjectContour,
    PotentialParameters,
    RobotSnapshot,
    Segment2,
    TransportSnapshot,
    Vec2,
    build_local_neighbor_graph,
    coulomb_buckingham,
    target_is_occluded,
)
from robot_swarm_bridge.algorithms.grf_adapter import (  # noqa: E402
    build_transport_snapshot,
    centered_object_contour,
    mcmc_iterations_for_fleet,
)


def square_contour(half_extent=0.2):
    return ObjectContour(
        points=(
            Vec2(-half_extent, -half_extent),
            Vec2(half_extent, -half_extent),
            Vec2(half_extent, half_extent),
            Vec2(-half_extent, half_extent),
        )
    )


def disabled_potential():
    return PotentialParameters(
        epsilon=0.0,
        r0=1.0,
        alpha=1.0,
        q1=0.0,
        q2=0.0,
        epsilon0=1.0,
    )


class CoulombBuckinghamTests(unittest.TestCase):
    def test_equation_matches_paper_at_equilibrium_distance(self):
        parameters = PotentialParameters(
            epsilon=0.4,
            r0=0.8,
            alpha=0.2,
            q1=2.0,
            q2=-3.0,
            epsilon0=0.7,
        )
        expected = (
            -parameters.epsilon
            + parameters.q1
            * parameters.q2
            / (4.0 * math.pi * parameters.epsilon0 * parameters.r0)
        )
        actual = coulomb_buckingham(parameters.r0, parameters)
        self.assertAlmostEqual(expected, actual, places=12)

    def test_zero_and_nonfinite_distances_return_finite_bounded_energy(self):
        parameters = PotentialParameters(
            epsilon=0.04,
            r0=0.8,
            alpha=0.2,
            q1=1.0,
            q2=1.0,
            epsilon0=0.04,
        )
        for distance in (0.0, float('nan'), float('inf')):
            energy = coulomb_buckingham(
                distance, parameters, energy_limit=1234.0
            )
            self.assertTrue(math.isfinite(energy))
            self.assertLessEqual(abs(energy), 1234.0)


class NeighborGraphTests(unittest.TestCase):
    def test_filters_by_range_and_static_line_of_sight(self):
        robots = (
            RobotSnapshot('a', Vec2(0.0, 0.0)),
            RobotSnapshot('b', Vec2(0.8, 0.0)),
            RobotSnapshot('c', Vec2(0.0, 0.8)),
            RobotSnapshot('far', Vec2(3.0, 0.0)),
        )
        wall = Segment2(Vec2(0.4, -0.2), Vec2(0.4, 0.2))
        graph = build_local_neighbor_graph(
            robots, sensing_radius=1.0, occluding_segments=(wall,)
        )
        self.assertEqual(('c',), tuple(item.robot_id for item in graph['a']))
        self.assertNotIn('a', tuple(item.robot_id for item in graph['b']))
        self.assertEqual((), graph['far'])

    def test_duplicate_robot_ids_are_rejected(self):
        with self.assertRaises(ValueError):
            build_local_neighbor_graph(
                (
                    RobotSnapshot('same', Vec2()),
                    RobotSnapshot('same', Vec2(1.0, 0.0)),
                ),
                sensing_radius=2.0,
            )


class AdapterTests(unittest.TestCase):
    def test_centered_contour_and_snapshot_use_finite_gazebo_points(self):
        center = Vec2(1.0, -2.0)
        contour = centered_object_contour(center, 0.2, sample_count=8)
        self.assertEqual(8, len(contour.points))
        for point in contour.points:
            self.assertAlmostEqual(0.2, (point - center).norm(), places=12)

        robot = RobotSnapshot('tb3_0', Vec2())
        snapshot = build_transport_snapshot(
            robots=(robot,),
            object_center=center,
            target=Vec2(3.0, 3.0),
            obstacle_points=(Vec2(0.5, 0.5), Vec2(float('nan'), 1.0)),
            object_radius=0.2,
            contour_samples=8,
        )
        self.assertEqual((robot,), snapshot.robots)
        self.assertEqual((Vec2(0.5, 0.5),), snapshot.obstacle_points)
        self.assertEqual(contour, snapshot.object_contour)

    def test_large_fleet_uses_the_lower_proposal_count_at_threshold(self):
        self.assertEqual(60, mcmc_iterations_for_fleet(19, 60, 20, 24))
        self.assertEqual(24, mcmc_iterations_for_fleet(20, 60, 20, 24))
        self.assertEqual(24, mcmc_iterations_for_fleet(100, 60, 20, 24))
        self.assertEqual(60, mcmc_iterations_for_fleet(20, 60, 20, 80))


class ObjectInteractionTests(unittest.TestCase):
    def setUp(self):
        self.contour = square_contour()
        self.target = Vec2(2.0, 0.0)

    def test_object_occlusion_selects_pushing_side(self):
        self.assertTrue(
            target_is_occluded(Vec2(-1.0, 0.0), self.target, self.contour)
        )
        self.assertFalse(
            target_is_occluded(Vec2(1.0, 0.0), self.target, self.contour)
        )

    def test_orbiting_prefers_local_contour_tangent(self):
        zero = disabled_potential()
        config = GRFConfig(
            max_speed=0.2,
            sensing_radius=3.0,
            obstacle_potential=zero,
            object_orbit_potential=zero,
            object_push_potential=zero,
            robot_potential=zero,
            orbit_alignment_weight=20.0,
            push_alignment_weight=20.0,
            velocity_consensus_weight=0.0,
            cruise_speed_weight=0.0,
            mcmc_iterations=4,
        )
        robot = RobotSnapshot('front', Vec2(1.0, 0.3))
        snapshot = TransportSnapshot(
            robots=(robot,),
            object_contour=self.contour,
            target=self.target,
        )
        kernel = GibbsRandomFieldTransport(config)
        initial = kernel.evaluate_velocity(snapshot, 'front', Vec2())
        tangent_velocity = initial.preferred_direction * config.max_speed
        tangent = kernel.evaluate_velocity(
            snapshot, 'front', tangent_velocity
        )
        radial = kernel.evaluate_velocity(
            snapshot, 'front', Vec2(config.max_speed, 0.0)
        )
        self.assertEqual(InteractionMode.ORBIT, tangent.interaction_mode)
        self.assertGreater(abs(initial.preferred_direction.y), 0.9)
        self.assertLess(tangent.object_energy, radial.object_energy)

    def test_goal_directed_velocity_is_preferred_when_pushing(self):
        zero = disabled_potential()
        config = GRFConfig(
            max_speed=0.2,
            sensing_radius=3.0,
            obstacle_potential=zero,
            object_orbit_potential=zero,
            object_push_potential=zero,
            robot_potential=zero,
            orbit_alignment_weight=20.0,
            push_alignment_weight=20.0,
            velocity_consensus_weight=0.0,
            cruise_speed_weight=0.0,
            mcmc_iterations=4,
        )
        robot = RobotSnapshot('behind', Vec2(-1.0, 0.0))
        snapshot = TransportSnapshot(
            robots=(robot,),
            object_contour=self.contour,
            target=self.target,
        )
        kernel = GibbsRandomFieldTransport(config)
        toward_goal = kernel.evaluate_velocity(
            snapshot, 'behind', Vec2(config.max_speed, 0.0)
        )
        away_from_goal = kernel.evaluate_velocity(
            snapshot, 'behind', Vec2(-config.max_speed, 0.0)
        )
        self.assertEqual(InteractionMode.PUSH, toward_goal.interaction_mode)
        self.assertLess(
            toward_goal.object_energy, away_from_goal.object_energy
        )


class SamplerTests(unittest.TestCase):
    def setUp(self):
        self.snapshot = TransportSnapshot(
            robots=(
                RobotSnapshot('tb3_10', Vec2(-0.8, 0.0), Vec2()),
                RobotSnapshot('tb3_2', Vec2(-0.8, 0.5), Vec2()),
                RobotSnapshot('tb3_far', Vec2(4.0, 4.0), Vec2()),
            ),
            object_contour=square_contour(),
            target=Vec2(2.0, 0.0),
            obstacle_points=(Vec2(-0.5, -0.5),),
        )
        self.config = GRFConfig(
            random_seed=73,
            mcmc_iterations=24,
            sensing_radius=2.0,
            max_speed=0.15,
        )

    def test_sampling_is_repeatable_bounded_and_locally_scoped(self):
        first = GibbsRandomFieldTransport(self.config).compute(
            self.snapshot, step_index=9
        )
        second = GibbsRandomFieldTransport(self.config).compute(
            self.snapshot, step_index=9
        )
        self.assertEqual(first, second)
        self.assertEqual(
            ('tb3_10', 'tb3_2', 'tb3_far'),
            tuple(result.robot_id for result in first.robots),
        )
        for result in first.robots:
            self.assertTrue(result.velocity.is_finite())
            self.assertLessEqual(
                result.velocity.norm(), self.config.max_speed + 1.0e-12
            )
            self.assertTrue(math.isfinite(result.total_energy))
            self.assertLessEqual(
                result.accepted_proposals, result.proposal_count
            )
        self.assertEqual(('tb3_2',), first.command_for('tb3_10').neighbor_ids)
        self.assertEqual((), first.command_for('tb3_far').neighbor_ids)

    def test_robot_input_order_does_not_change_results(self):
        reversed_snapshot = TransportSnapshot(
            robots=tuple(reversed(self.snapshot.robots)),
            object_contour=self.snapshot.object_contour,
            target=self.snapshot.target,
            obstacle_points=self.snapshot.obstacle_points,
        )
        kernel = GibbsRandomFieldTransport(self.config)
        self.assertEqual(
            kernel.compute(self.snapshot, step_index=4),
            kernel.compute(reversed_snapshot, step_index=4),
        )

    def test_invalid_position_fails_safe_without_poisoning_other_results(self):
        snapshot = TransportSnapshot(
            robots=(
                RobotSnapshot('invalid', Vec2(float('nan'), 0.0)),
                RobotSnapshot('valid', Vec2(-1.0, 0.0)),
            ),
            object_contour=square_contour(),
            target=Vec2(2.0, 0.0),
        )
        result = GibbsRandomFieldTransport(self.config).compute(snapshot)
        invalid = result.command_for('invalid')
        valid = result.command_for('valid')
        self.assertEqual(InteractionMode.INVALID, invalid.interaction_mode)
        self.assertEqual(Vec2(), invalid.velocity)
        self.assertTrue(valid.velocity.is_finite())


if __name__ == '__main__':
    unittest.main()
