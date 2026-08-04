#!/usr/bin/env python3
"""Regression tests for the Plan1 B-point simulated GCS."""

import importlib.util
from pathlib import Path
import time
import unittest


SOURCE = Path(__file__).with_name("simulated_ground_station_udp.py")
SPEC = importlib.util.spec_from_file_location("simulated_ground_station_udp", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeSocket:
    def __init__(self):
        self.sent = []

    def sendto(self, payload, destination):
        self.sent.append((payload, destination))


class Plan1ScenarioTest(unittest.TestCase):
    def setUp(self):
        self.socket = FakeSocket()
        self.scenario = MODULE.Scenario(
            self.socket, ("127.0.0.1", 5005), 4.0, 60.0)

    def sent_headers(self):
        return [MODULE.json.loads(payload.decode("utf-8"))["header"]
                for payload, _ in self.socket.sent]

    def test_ok_b_starts_continuous_car_status_without_a_go_ahead_command(self):
        self.scenario.latest_xyzstatus = (2.375, -0.375, 0.0)
        self.scenario.phase = "waiting_ok_b"
        self.scenario.process_packet(MODULE.Scenario.payload("ok_b"))
        self.assertEqual(self.scenario.phase, "sending_car_status")
        self.assertEqual(self.sent_headers(), ["ack"])

        self.scenario.advance()
        self.assertEqual(self.sent_headers(), ["ack", "car_status"])
        sent = MODULE.json.loads(self.socket.sent[-1][0].decode("utf-8"))
        self.assertAlmostEqual(sent["data"]["distance_m"], MODULE.THROW_DISTANCE_M)
        self.assertAlmostEqual(sent["data"]["bearing_rad"], 0.0)

    def test_all_ok_events_are_acked_and_the_throw_path_needs_no_return_command(self):
        self.scenario.phase = "sending_car_status"
        for header, expected_phase in (
                ("ok_throw", "waiting_ok_return"),
                ("ok_return", "waiting_ok_downing"),
                ("ok_downing", "waiting_ok_down"),
                ("ok_down", "complete")):
            self.scenario.process_packet(MODULE.Scenario.payload(header))
            self.assertEqual(self.scenario.phase, expected_phase)
        self.assertEqual(self.sent_headers(), ["ack", "ack", "ack", "ack"])


if __name__ == "__main__":
    unittest.main()
