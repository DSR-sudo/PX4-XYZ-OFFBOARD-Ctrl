#!/usr/bin/env python3
"""Run the Plan1 B-point interception scenario against a live UAV application."""

import argparse
import json
import math
import socket
import sys
import time


OK_HEADERS = {
    "ok_wait",
    "ok_b",
    "ok_throw",
    "ok_return",
    "ok_downing",
    "ok_down",
}
CAR_STATUS_PERIOD_S = 0.10
THROW_DISTANCE_M = 0.15


class Scenario:
    """GCS side of the ordered ACK protocol and continuous target observation stream."""

    def __init__(self, sock, destination, command_wait_s, max_duration_s):
        self.sock = sock
        self.destination = destination
        self.command_wait_s = command_wait_s
        self.deadline = time.monotonic() + max_duration_s
        self.phase = "waiting_ok_wait"
        self.phase_started_at = time.monotonic()
        self.command_due_at = None
        self.received_events = []
        self.latest_xyzstatus = None
        self.car_position = None
        self.car_status_last_sent_at = None

    @staticmethod
    def payload(header, data=None):
        return json.dumps({"header": header, "data": data or {}}, separators=(",", ":")).encode()

    def send(self, header, data=None):
        self.sock.sendto(self.payload(header, data), self.destination)
        print("TX", header, json.dumps(data or {}, separators=(",", ":")), flush=True)

    def acknowledge(self):
        self.send("ack")

    def begin_phase(self, name):
        self.phase = name
        self.phase_started_at = time.monotonic()

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

    def set_simulated_car_position(self):
        x, y, yaw = self.latest_xyzstatus
        self.car_position = (
            x + THROW_DISTANCE_M * math.cos(yaw),
            y + THROW_DISTANCE_M * math.sin(yaw),
        )

    def send_car_status(self, now):
        if not self.latest_xyzstatus or not self.car_position:
            return False
        x, y, yaw = self.latest_xyzstatus
        car_x, car_y = self.car_position
        distance_m = math.hypot(car_x - x, car_y - y)
        bearing_rad = math.atan2(car_y - y, car_x - x) - yaw
        bearing_rad = math.atan2(math.sin(bearing_rad), math.cos(bearing_rad))
        self.send("car_status", {"distance_m": distance_m, "bearing_rad": bearing_rad})
        self.car_status_last_sent_at = now
        return True

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
        elif header == "ok_b" and self.phase == "waiting_ok_b":
            if self.latest_xyzstatus:
                self.set_simulated_car_position()
            self.begin_phase("sending_car_status")
        elif header == "ok_throw" and self.phase == "sending_car_status":
            self.begin_phase("waiting_ok_return")
        elif header == "ok_return" and self.phase == "waiting_ok_return":
            self.begin_phase("waiting_ok_downing")
        elif header == "ok_downing" and self.phase == "waiting_ok_downing":
            self.begin_phase("waiting_ok_down")
        elif header == "ok_down" and self.phase == "waiting_ok_down":
            self.begin_phase("complete")

    def advance(self):
        now = time.monotonic()
        if now >= self.deadline:
            raise TimeoutError("scenario deadline reached before ok_down")
        if self.phase == "waiting_run_plan1" and now >= self.command_due_at:
            self.send("run_plan1")
            self.begin_phase("waiting_ok_b")
        elif self.phase == "sending_car_status":
            if self.car_status_last_sent_at is None or now - self.car_status_last_sent_at >= CAR_STATUS_PERIOD_S:
                self.send_car_status(now)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-ip", required=True)
    parser.add_argument("--bind-port", type=int, required=True)
    parser.add_argument("--uav-ip", required=True)
    parser.add_argument("--uav-port", type=int, required=True)
    parser.add_argument("--command-wait-s", type=float, default=4.0)
    parser.add_argument("--max-duration-s", type=float, default=180.0)
    args = parser.parse_args()
    if not 4.0 <= args.command_wait_s <= 9.0:
        parser.error("--command-wait-s must be within 4..9 seconds")
    for name in ("bind_port", "uav_port"):
        if not 1 <= getattr(args, name) <= 65535:
            parser.error("ports must be within 1..65535")
    if args.max_duration_s <= 0.0:
        parser.error("--max-duration-s must be positive")
    return args


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind_ip, args.bind_port))
    sock.settimeout(0.05)
    scenario = Scenario(
        sock, (args.uav_ip, args.uav_port), args.command_wait_s, args.max_duration_s)
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

    expected = ["ok_wait", "ok_b", "ok_throw", "ok_return", "ok_downing", "ok_down"]
    if scenario.received_events != expected:
        print("FAIL unexpected event order: {}".format(scenario.received_events), file=sys.stderr, flush=True)
        return 1
    print("PASS UDP scenario completed", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
