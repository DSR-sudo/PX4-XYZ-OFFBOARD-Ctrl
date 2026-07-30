#!/usr/bin/env python3
"""Run the bounded UDP scenario against a live UAV application."""

import argparse
import json
import math
import socket
import sys
import time


OK_HEADERS = {
    "ok_wait",
    "ok_height",
    "ok_throw",
    "ok_return",
    "ok_downing",
    "ok_down",
}

GO_AHEAD_WAIT_S = 10.0
GO_AHEAD_RETRY_S = 0.5
PURSUIT_CONFIRM_DISTANCE_M = 0.10


class Scenario:
    """Implements the GCS side of the ordered UDP scenario and records every event."""

    def __init__(self, sock, destination, command_wait_s, max_duration_s, line_centered, car_forward_m):
        self.sock = sock
        self.destination = destination
        self.command_wait_s = command_wait_s
        self.deadline = time.monotonic() + max_duration_s
        self.phase = "waiting_ok_wait"
        self.phase_started_at = time.monotonic()
        self.command_due_at = None
        self.received_events = []
        self.latest_xyzstatus = None
        self.line_centered = line_centered
        self.car_forward_m = car_forward_m
        self.alignment_start = None
        self.car_position = None
        self.go_ahead_reference = None
        self.go_ahead_last_sent_at = None

    @staticmethod
    def payload(header, data=None):
        return json.dumps({"header": header, "data": data or {}}, separators=(",", ":")).encode()

    def send(self, header, data=None):
        self.sock.sendto(self.payload(header, data), self.destination)
        print("TX", header, json.dumps(data or {}, separators=(",", ":")), flush=True)

    def acknowledge(self):
        # ACK is the immediate transport response required to drain the ordered ok_* queue.
        self.send("ack")

    def begin_phase(self, name):
        self.phase = name
        self.phase_started_at = time.monotonic()

    def process_packet(self, packet):
        try:
            message = json.loads(packet.decode("utf-8"))
            header = message["header"]
            data = message["data"]
        except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError):
            print("RX invalid datagram", file=sys.stderr, flush=True)
            return

        if header == "xyzstatus":
            self.latest_xyzstatus = self.pose_from_xyzstatus(data)
            return
        if header not in OK_HEADERS or data != {}:
            print("RX unexpected", header, json.dumps(data, separators=(",", ":")), flush=True)
            return

        print("RX", header, flush=True)
        self.acknowledge()
        if header not in self.received_events:
            self.received_events.append(header)

        now = time.monotonic()
        if header == "ok_wait" and self.phase == "waiting_ok_wait":
            self.command_due_at = now + self.command_wait_s
            self.begin_phase("waiting_run_plan1")
        elif header == "ok_height" and self.phase == "waiting_ok_height":
            if self.line_centered:
                self.begin_phase("waiting_alignment_start")
            else:
                self.command_due_at = now + GO_AHEAD_WAIT_S
                self.begin_phase("waiting_go_ahead")
        elif header == "ok_throw" and self.phase == "waiting_ok_throw":
            self.command_due_at = now + self.command_wait_s
            self.begin_phase("waiting_b_ok")
        elif header == "ok_return" and self.phase == "waiting_ok_return":
            self.begin_phase("waiting_ok_downing")
        elif header == "ok_downing" and self.phase == "waiting_ok_downing":
            self.begin_phase("waiting_ok_down")
        elif header == "ok_down" and self.phase == "waiting_ok_down":
            self.begin_phase("complete")

    @staticmethod
    def pose_from_xyzstatus(data):
        try:
            x = float(data["position_x_m"])
            y = float(data["position_y_m"])
            yaw = float(data["yaw_rad"])
        except (KeyError, TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return None
        return x, y, yaw

    def alignment_verified(self):
        if not self.alignment_start or not self.latest_xyzstatus or not self.line_centered:
            return False
        start_x, start_y, start_yaw = self.alignment_start
        current_x, current_y, _ = self.latest_xyzstatus
        dx = current_x - start_x
        dy = current_y - start_y
        forward_m = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
        right_m = dx * math.sin(start_yaw) - dy * math.cos(start_yaw)
        return 0.35 <= right_m <= 0.40 and abs(forward_m) <= 0.10

    def set_simulated_car_position(self):
        current_x, current_y, yaw = self.latest_xyzstatus
        self.car_position = (
            current_x + self.car_forward_m * math.cos(yaw),
            current_y + self.car_forward_m * math.sin(yaw),
        )

    def car_distance_below_match_threshold(self):
        if not self.latest_xyzstatus or not self.car_position:
            return False
        current_x, current_y, _ = self.latest_xyzstatus
        car_x, car_y = self.car_position
        return math.hypot(current_x - car_x, current_y - car_y) < 0.10

    def pursuit_started(self):
        if not self.go_ahead_reference or not self.latest_xyzstatus:
            return False
        start_x, start_y, start_yaw = self.go_ahead_reference
        current_x, current_y, _ = self.latest_xyzstatus
        dx = current_x - start_x
        dy = current_y - start_y
        forward_m = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
        return forward_m >= PURSUIT_CONFIRM_DISTANCE_M

    def begin_go_ahead_handshake(self, now):
        if not self.latest_xyzstatus:
            return False
        self.set_simulated_car_position()
        self.go_ahead_reference = self.latest_xyzstatus
        self.go_ahead_last_sent_at = now
        self.send("go_ahead_ok")
        self.begin_phase("waiting_pursuit_start")
        return True

    def advance(self):
        now = time.monotonic()
        if now >= self.deadline:
            raise TimeoutError("scenario deadline reached before ok_down")

        if self.phase == "waiting_run_plan1" and now >= self.command_due_at:
            self.send("run_plan1")
            self.begin_phase("waiting_ok_height")
        elif self.phase == "waiting_alignment_start" and self.latest_xyzstatus:
            self.alignment_start = self.latest_xyzstatus
            self.begin_phase("waiting_alignment_check")
        elif self.phase == "waiting_alignment_check" and self.alignment_verified():
            self.begin_go_ahead_handshake(now)
        elif self.phase == "waiting_go_ahead" and now >= self.command_due_at:
            self.begin_go_ahead_handshake(now)
        elif self.phase == "waiting_pursuit_start":
            if self.pursuit_started():
                self.begin_phase("pursuing_car")
            elif now - self.go_ahead_last_sent_at >= GO_AHEAD_RETRY_S:
                self.send("go_ahead_ok")
                self.go_ahead_last_sent_at = now
        elif self.phase == "pursuing_car" and self.car_distance_below_match_threshold():
            self.send("match_car_ok")
            self.begin_phase("waiting_ok_throw")
        elif self.phase == "waiting_b_ok" and now >= self.command_due_at:
            # Map and business criteria stay on the real GCS; the UAV receives only b_ok.
            self.send("b_ok")
            self.begin_phase("waiting_ok_return")



def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-ip", required=True)
    parser.add_argument("--bind-port", type=int, required=True)
    parser.add_argument("--uav-ip", required=True)
    parser.add_argument("--uav-port", type=int, required=True)
    parser.add_argument("--command-wait-s", type=float, default=4.0)
    parser.add_argument("--max-duration-s", type=float, default=180.0)
    parser.add_argument(
        "--line-centered", action="store_true",
        help="Use the original simulated visual-alignment gate instead of the 10 s go-ahead delay.")
    parser.add_argument(
        "--simulated-car-forward-m", type=float, default=2.3,
        help="Place the simulated vision-detected car this far ahead after line alignment.")
    args = parser.parse_args()
    if not 4.0 <= args.command_wait_s <= 9.0:
        parser.error("--command-wait-s must be within 4..9 seconds")
    for name in ("bind_port", "uav_port"):
        if not 1 <= getattr(args, name) <= 65535:
            parser.error("ports must be within 1..65535")
    if args.max_duration_s <= 0.0:
        parser.error("--max-duration-s must be positive")
    if args.simulated_car_forward_m <= 0.10:
        parser.error("--simulated-car-forward-m must exceed the 0.1 m match threshold")
    return args


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind_ip, args.bind_port))
    sock.settimeout(0.05)
    scenario = Scenario(
        sock, (args.uav_ip, args.uav_port), args.command_wait_s, args.max_duration_s,
        args.line_centered, args.simulated_car_forward_m)
    print("GCS listening on {}:{}".format(args.bind_ip, args.bind_port), flush=True)

    try:
        while scenario.phase != "complete":
            try:
                packet, source = sock.recvfrom(65535)
                if source != (args.uav_ip, args.uav_port):
                    print("RX ignored source {}:{}".format(*source), flush=True)
                    continue
                scenario.process_packet(packet)
            except socket.timeout:
                pass
            scenario.advance()
    except TimeoutError as error:
        print("FAIL", error, file=sys.stderr, flush=True)
        return 1
    finally:
        sock.close()

    expected = ["ok_wait", "ok_height", "ok_throw", "ok_return", "ok_downing", "ok_down"]
    if scenario.received_events != expected:
        print("FAIL unexpected event order: {}".format(scenario.received_events), file=sys.stderr, flush=True)
        return 1
    print("PASS UDP scenario completed", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
