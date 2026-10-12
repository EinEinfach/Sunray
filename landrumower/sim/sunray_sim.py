#!/usr/bin/env python3
"""Plays the role of Sunray for the Pico firmware 4.x over USB.

Sends AT+M every 20 ms and AT+S every 500 ms like the SerialRobotDriver, follows a
profile of demands and evaluates the answers and the telemetry lines of the firmware.
While a demand is not zero it reports the state "mow" to the Pico, otherwise "idle".

Examples:
    sunray_sim.py                                  speed profile 300, -300, 0 mm/s
    sunray_sim.py --profile 150:3,400:3,0:2        own profile, demand:seconds
    sunray_sim.py --pwm --profile 128:3,0:2        PWM pass-through instead of speed
    sunray_sim.py --csv run.csv                    store the telemetry
"""

import argparse
import csv
import glob
import sys
import time

import serial

MOTOR_PERIOD = 0.02
SUMMARY_PERIOD = 0.5
TELEMETRY_FIELDS = ["ms",
                    "right_demand", "right_speed", "right_drive", "right_brake",
                    "left_demand", "left_speed", "left_drive", "left_brake",
                    "mow_demand", "mow_speed", "mow_drive", "mow_brake",
                    "max_period_us"]


def with_crc(payload):
    return "%s,0x%02x\r\n" % (payload, sum(payload.encode()) % 256)


def crc_valid(line):
    body, _, crc = line.rpartition(",")
    try:
        return int(crc, 16) == sum(body.encode()) % 256
    except ValueError:
        return False


def parse_profile(text):
    phases = []
    for part in text.split(","):
        demand, _, seconds = part.partition(":")
        phases.append((int(demand), float(seconds)))
    return phases


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("no Pico found, use --port")
    return ports[0]


class Link:
    def __init__(self, port):
        self.serial = serial.Serial(port, 115200, timeout=0)
        self.buffer = b""
        self.sent = {"M": 0, "S": 0}
        self.answered = {"M": 0, "S": 0}
        self.crc_errors = 0
        self.unknown = []
        self.ticks = None       # right, left, mow from the latest M answer
        self.telemetry = []     # rows of all D lines
        self.info = []          # fields of all I lines as dictionaries

    def send(self, payload):
        self.serial.write(with_crc(payload).encode())
        self.sent[payload[3]] += 1

    def receive(self):
        self.buffer += self.serial.read(4096)
        while b"\n" in self.buffer:
            raw, _, self.buffer = self.buffer.partition(b"\n")
            line = raw.decode(errors="replace").strip()
            if line:
                self.handle(line)

    def handle(self, line):
        fields = line.split(",")
        if fields[0] == "D" and len(fields) == len(TELEMETRY_FIELDS) + 1:
            try:
                self.telemetry.append([float(value) for value in fields[1:]])
            except ValueError:
                self.unknown.append(line)
        elif fields[0] == "I":
            self.info.append(dict(field.split("=", 1) for field in fields[1:] if "=" in field))
        elif fields[0] in ("M", "S", "V"):
            if not crc_valid(line):
                self.crc_errors += 1
                return
            if fields[0] in self.answered:
                self.answered[fields[0]] += 1
            if fields[0] == "M":
                self.ticks = [int(value) for value in fields[1:4]]
            elif fields[0] == "V":
                print("firmware: %s %s" % (fields[1], fields[2]))
        else:
            self.unknown.append(line)


def run_phase(link, demand, seconds, pwm_mode, mow_pwm):
    # Sunray state shown on the LCD and signalled by the buzzer: 1 = mow, 0 = idle
    summary = "AT+S,%d" % (1 if demand != 0 else 0)
    if pwm_mode:
        command = "AT+M,%d,%d,%d" % (demand, demand, mow_pwm)
    else:
        command = "AT+M,0,0,%d,%d,%d" % (mow_pwm, demand, demand)
    first_row = len(link.telemetry)
    ticks_before = link.ticks
    start = time.monotonic()
    next_motor = start
    next_summary = start
    while True:
        now = time.monotonic()
        if now - start >= seconds:
            break
        if now >= next_motor:
            link.send(command)
            next_motor += MOTOR_PERIOD
        if now >= next_summary:
            link.send(summary)
            next_summary += SUMMARY_PERIOD
        link.receive()
        time.sleep(0.001)
    return link.telemetry[first_row:], ticks_before


def settle_time(rows, demand, tolerance):
    """Seconds until the right wheel stays within tolerance of the demand."""
    target = abs(demand)
    settled_at = None
    for row in rows:
        if abs(row[2] - target) <= tolerance:
            if settled_at is None:
                settled_at = row[0]
        else:
            settled_at = None
    if settled_at is None:
        return None
    return (settled_at - rows[0][0]) / 1000.0


def report_phase(link, demand, rows, ticks_before, pwm_mode):
    if not rows:
        print("  demand %5d: no telemetry received" % demand)
        return
    last = rows[-1]
    line = "  demand %5d: right %6.1f  left %6.1f  drive %+.2f  brake %.2f" % (
        demand, last[2], last[6], last[3], last[4])
    if not pwm_mode:
        settled = settle_time(rows, demand, max(10.0, abs(demand) * 0.05))
        line += "  settled " + ("after %.2f s" % settled if settled is not None else "never")
    if ticks_before and link.ticks:
        line += "  ticks +%d/+%d" % (link.ticks[0] - ticks_before[0], link.ticks[1] - ticks_before[1])
    print(line)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port, default: first Pico found")
    parser.add_argument("--profile", default="300:3,-300:4,0:2",
                        help="demand:seconds,... in mm/s, or PWM values with --pwm")
    parser.add_argument("--pwm", action="store_true", help="send PWM values without speed fields")
    parser.add_argument("--mow", type=int, default=0, help="PWM of the mow motor")
    parser.add_argument("--csv", help="write all telemetry lines to this file")
    args = parser.parse_args()

    link = Link(args.port or find_port())
    link.serial.write(with_crc("AT+V").encode())
    time.sleep(0.2)
    link.receive()

    print("%s profile %s" % ("pwm" if args.pwm else "speed", args.profile))
    for demand, seconds in parse_profile(args.profile):
        rows, ticks_before = run_phase(link, demand, seconds, args.pwm, args.mow)
        report_phase(link, demand, rows, ticks_before, args.pwm)
    time.sleep(0.1)
    link.receive()

    print("answers: AT+M %d of %d, AT+S %d of %d, crc errors %d, unreadable lines %d" % (
        link.answered["M"], link.sent["M"], link.answered["S"], link.sent["S"],
        link.crc_errors, len(link.unknown)))
    if link.telemetry:
        periods = [row[-1] for row in link.telemetry[1:] if row[-1] > 0]
        if periods:
            print("control period: longest %d us, typical longest per telemetry line %d us" % (
                max(periods), sorted(periods)[len(periods) // 2]))
    if link.info:
        states = []
        for entry in link.info:
            if not states or states[-1] != entry.get("state"):
                states.append(entry.get("state"))
        print("lcd %s, buzzer switched on in %d of %d info lines, states: %s" % (
            "found" if link.info[-1].get("lcd") == "1" else "NOT found",
            sum(1 for entry in link.info if entry.get("buzzer") == "1"), len(link.info),
            " > ".join(str(state) for state in states)))
    for line in link.unknown[:5]:
        print("  unreadable: %r" % line)

    if args.csv:
        with open(args.csv, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(TELEMETRY_FIELDS)
            writer.writerows(link.telemetry)
        print("telemetry written to %s" % args.csv)


if __name__ == "__main__":
    main()
