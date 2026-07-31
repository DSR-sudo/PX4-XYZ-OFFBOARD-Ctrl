#!/usr/bin/env python3
"""Regression tests for the simulated GCS go-ahead handshake."""

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


class GoAheadHandshakeTest(unittest.TestCase):
    def setUp(self):
        self.socket = FakeSocket()
        self.scenario = MODULE.Scenario(
            self.socket, ("127.0.0.1", 5005), 4.0, 60.0, True, 2.3)

    def sent_headers(self):
        return [MODULE.json.loads(payload.decode("utf-8"))["header"]
                for payload, _ in self.socket.sent]

    def start_handshake(self):
        self.scenario.alignment_start = (0.0, 0.0, 0.0)
        self.scenario.latest_xyzstatus = (0.0, -0.375, 0.0)
        self.scenario.begin_phase("waiting_alignment_check")
        self.scenario.advance()

    def test_go_ahead_ok_retries_until_forward_motion_confirms_acceptance(self):
        self.start_handshake()
        self.assertEqual(self.scenario.phase, "waiting_pursuit_start")
        self.assertEqual(self.sent_headers(), ["go_ahead_ok"])

        self.scenario.go_ahead_last_sent_at = time.monotonic() - MODULE.GO_AHEAD_RETRY_S
        self.scenario.advance()
        self.assertEqual(self.scenario.phase, "waiting_pursuit_start")
        self.assertEqual(self.sent_headers(), ["go_ahead_ok", "go_ahead_ok"])

        self.scenario.latest_xyzstatus = (0.11, -0.375, 0.0)
        self.scenario.advance()
        self.assertEqual(self.scenario.phase, "tracking_to_match")
        self.assertEqual(self.sent_headers(), ["go_ahead_ok", "go_ahead_ok", "car_status"])


if __name__ == "__main__":
    unittest.main()
