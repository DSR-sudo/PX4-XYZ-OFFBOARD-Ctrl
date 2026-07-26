import unittest

from core_types import SafetyConfig
from node_flight import FlightStateMixin
from planner import SmoothPositionPlanner
from safety_core import PositionSafetyCore


class LcpGateTests(unittest.TestCase):
    def make_core(self, samples=3):
        return PositionSafetyCore(
            SafetyConfig(lcp_ready_samples=samples, lcp_status_timeout_s=0.75,
                         lcp_odometry_timeout_s=0.75)
        )

    def test_old_status_cannot_satisfy_new_initialization(self):
        core = self.make_core()
        core.update_lcp_status(2, 1.0)
        core.update_lcp_odometry(1.0, 2.0, 0.0, 1.0)
        core.begin_lcp_initialization(2.0)
        core.update_lcp_init_state("accepted", 2.0, message="started")
        self.assertFalse(core.lcp_ready(2.1))

    def test_three_fresh_status_samples_and_odometry_are_ready(self):
        core = self.make_core()
        core.begin_lcp_initialization(1.0)
        core.update_lcp_init_state("accepted", 1.0, message="started")
        for timestamp in (1.1, 1.2, 1.3):
            core.update_lcp_status(2, timestamp)
            core.update_lcp_odometry(1.0, 2.0, 0.0, timestamp)
        self.assertTrue(core.lcp_ready(1.3))

    def test_status_three_blocks_runtime_health(self):
        core = self.make_core()
        core.begin_lcp_initialization(1.0)
        core.update_lcp_init_state("accepted", 1.0, message="started")
        core.update_lcp_status(2, 1.1)
        core.update_lcp_odometry(1.0, 2.0, 0.0, 1.1)
        self.assertTrue(core.lcp_runtime_healthy(1.2))
        core.update_lcp_status(3, 1.3)
        self.assertFalse(core.lcp_runtime_healthy(1.3))


class PlannerLcpHoldTests(unittest.TestCase):
    def test_freeze_and_yaw_zero_do_not_use_lcp_coordinates(self):
        planner = SmoothPositionPlanner(SafetyConfig())
        planner.latch(10.0, 20.0, 0.0, 0.0, 0.0, 0.7071, 0.7071)
        planner.set_relative_target(1.0)
        planner.update(0.2)
        planner.freeze_xy_at(10.25, 20.5)
        planner.freeze_z()
        self.assertEqual((planner.x_m, planner.y_m), (10.25, 20.5))
        self.assertAlmostEqual(planner.target_z_m, planner.command_z_m)
        planner.set_yaw_rad(0.0)
        self.assertAlmostEqual(planner.yaw_rad(), 0.0)


class FlightLcpPolicyTests(unittest.TestCase):
    class Harness(FlightStateMixin):
        def __init__(self):
            self.config = SafetyConfig()
            self.core = PositionSafetyCore(self.config)
            self.core.flight_errors = lambda *args, **kwargs: []
            self.planner = SmoothPositionPlanner(self.config)
            self.planner.latch(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            self.phase = "climb"
            self.phase_started_at = 0.0
            self.flight_started_at = 0.0
            self.last_errors = []
            self.sensor_loss_started_at = None
            self.result = "UNCONFIRMED"
            self.abort_reason = None
            self.waypoints = []
            self.waypoint_index = -1
            self.published = 0

        def _publish_setpoint(self, dt_s):
            del dt_s
            self.published += 1

        def _request_mode(self, now, mode):
            del now, mode

    def test_status_three_is_ignored_during_climb(self):
        harness = self.Harness()
        harness.core.telemetry.mode = "OFFBOARD"
        harness.core.telemetry.armed = True
        harness.core.telemetry.local_z_m = 1.0
        harness.planner.set_relative_target(1.0)
        harness.planner.command_z_m = harness.planner.target_z_m
        harness.core.update_lcp_status(3, 0.1)
        harness._flight_tick(0.2, 0.05)
        self.assertNotEqual(harness.phase, "lcp_hold")
        self.assertEqual(harness.phase, "hold")

    def test_hold_waits_for_lcp_before_yaw_zero_and_waypoint(self):
        harness = self.Harness()
        harness.phase = "hold"
        harness.phase_started_at = 0.0
        harness.core.telemetry.mode = "OFFBOARD"
        harness.core.telemetry.armed = True
        harness.core.telemetry.local_z_m = 1.0
        harness.planner.set_relative_target(1.0)
        harness.planner.command_z_m = harness.planner.target_z_m
        harness.core.update_lcp_status(3, 9.9)
        harness._flight_tick(10.0, 0.05)
        self.assertEqual(harness.phase, "hold")
        harness.core.update_lcp_status(2, 10.1)
        harness.core.update_lcp_odometry(50.0, 60.0, 1.2, 10.1)
        harness._flight_tick(10.2, 0.05)
        self.assertEqual(harness.phase, "waypoint")
        self.assertAlmostEqual(harness.planner.yaw_rad(), 0.0)
        self.assertEqual(harness.planner.origin_x_m, 0.0)
        self.assertEqual(harness.planner.origin_y_m, 0.0)


if __name__ == "__main__":
    unittest.main()
