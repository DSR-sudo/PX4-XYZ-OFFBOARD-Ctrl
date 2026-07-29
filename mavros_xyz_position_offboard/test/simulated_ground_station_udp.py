#!/usr/bin/env python3
"""Run the bounded no-gripper UDP scenario against a live UAV application."""

import argparse
import json
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


class Scenario:
    """Implements the GCS side of the ordered UDP scenario and records every event."""

    def __init__(self, sock, destination, command_wait_s, max_duration_s):
        self.sock = sock
        self.destination = destination
        self.command_wait_s = command_wait_s
        self.deadline = time.monotonic() + max_duration_s
        self.phase = "waiting_ok_wait"
        self.phase_started_at = time.monotonic()
        self.next_car_status_at = 0.0
        self.command_due_at = None
        self.received_events = []
        self.latest_xyzstatus = None

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
            self.latest_xyzstatus = data
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
            self.next_car_status_at = now
            self.begin_phase("right_front_60deg")
        elif header == "ok_throw" and self.phase == "waiting_ok_throw":
            self.command_due_at = now + self.command_wait_s
            self.begin_phase("waiting_b_ok")
        elif header == "ok_return" and self.phase == "waiting_ok_return":
            self.begin_phase("waiting_ok_downing")
        elif header == "ok_downing" and self.phase == "waiting_ok_downing":
            self.begin_phase("waiting_ok_down")
        elif header == "ok_down" and self.phase == "waiting_ok_down":
            self.begin_phase("complete")

    def send_car_status_if_due(self, now):
        if now < self.next_car_status_at:
            return
        if self.phase == "right_front_60deg":
            # 1 m target motion: 0.10 m standoff plus 1.00 m travel at body-right-front -60 deg.
            self.send("car_status", {"distance": 1.10, "angle": -60.0})
        elif self.phase == "forward_0_5m":
            # The UAV has been commanded to turn toward the first target; angle 0 is its current body front.
            self.send("car_status", {"distance": 0.60, "angle": 0.0})
        elif self.phase == "standoff_before_match":
            self.send("car_status", {"distance": 0.10, "angle": 0.0})
        else:
            return
        self.next_car_status_at = now + 0.10

    def advance(self):
        now = time.monotonic()
        if now >= self.deadline:
            raise TimeoutError("scenario deadline reached before ok_down")

        if self.phase == "waiting_run_plan1" and now >= self.command_due_at:
            self.send("run_plan1")
            self.begin_phase("waiting_ok_height")
        elif self.phase == "right_front_60deg" and now - self.phase_started_at >= 1.0:
            self.next_car_status_at = now
            self.begin_phase("forward_0_5m")
        elif self.phase == "forward_0_5m" and now - self.phase_started_at >= 1.0:
            self.command_due_at = now + self.command_wait_s
            self.next_car_status_at = now
            self.begin_phase("standoff_before_match")
        elif self.phase == "standoff_before_match" and now >= self.command_due_at:
            self.send("match_car_ok")
            self.begin_phase("waiting_ok_throw")
        elif self.phase == "waiting_b_ok" and now >= self.command_due_at:
            # Map and business criteria stay on the real GCS; the UAV receives only b_ok.
            self.send("b_ok")
            self.begin_phase("waiting_ok_return")

        self.send_car_status_if_due(now)


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
    scenario = Scenario(sock, (args.uav_ip, args.uav_port), args.command_wait_s, args.max_duration_s)
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
    print("PASS no-gripper UDP scenario completed", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
