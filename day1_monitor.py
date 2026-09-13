#!/usr/bin/env python3
"""
Day 1 deliverable: read live telemetry from SITL, print VehicleState at 1 Hz.

    python3 scripts/day1_monitor.py
    python3 scripts/day1_monitor.py --endpoint udpin:0.0.0.0:14551

Kill SITL while this is running. It should log the link going dead,
keep retrying, and pick straight back up when SITL returns. If it exits,
Day 8 will not work.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time

sys.path.insert(0, ".")  # so `python3 scripts/day1_monitor.py` works from repo root

from guardian.link import MavLink
from guardian.telemetry import TelemetryTracker

LOOP_HZ = 10.0
PRINT_HZ = 1.0

_running = True


def _stop(signum, frame):
    global _running
    _running = False


def fmt(value, spec="6.2f", unit=""):
    """Never print a bare 0.0 for a field you have not received yet."""
    if value is None:
        return "  --  " + unit
    return format(value, spec) + unit


def render(s) -> str:
    lines = [
        "=" * 62,
        f" LINK   {'UP' if s.connected else 'DOWN':<5} "
        f"hb {s.seconds_since_heartbeat:5.1f}s ago   "
        f"msgs {sum(s.message_counts.values()):>7}",
        f" MODE   {s.flight_mode:<10} {'ARMED' if s.armed else 'disarmed'}",
        f" POS    lat {fmt(s.lat_deg, '11.7f')}  lon {fmt(s.lon_deg, '11.7f')}",
        f" ALT    rel {fmt(s.alt_rel_m, '7.2f', ' m')}   amsl {fmt(s.alt_amsl_m, '7.2f', ' m')}",
        f" VEL    gs {fmt(s.groundspeed_mps, '5.2f', ' m/s')}   "
        f"climb {fmt(s.climb_mps, '5.2f', ' m/s')}   hdg {fmt(s.heading_deg, '5.1f', ' deg')}",
        f" ATT    roll {fmt(s.roll_deg, '6.1f')}  pitch {fmt(s.pitch_deg, '6.1f')}  "
        f"yaw {fmt(s.yaw_deg, '6.1f')}",
        f" BATT   {fmt(s.voltage_v, '5.2f', ' V')}  {fmt(s.current_a, '6.2f', ' A')}  "
        f"{fmt(s.battery_remaining_pct, '5.1f', ' %')}  used {fmt(s.consumed_mah, '7.1f', ' mAh')}",
        f" GPS    {s.gps_fix_name:<9} sats {fmt(s.satellites, '3.0f')}  hdop {fmt(s.hdop, '5.2f')}",
        f" EKF    ok={s.ekf_ok}  vel {fmt(s.ekf_velocity_variance, '5.3f')}  "
        f"posh {fmt(s.ekf_pos_horiz_variance, '5.3f')}  posv {fmt(s.ekf_pos_vert_variance, '5.3f')}",
    ]
    if s.last_statustext:
        lines.append(f" TEXT   [{s.last_statustext_severity}] {s.last_statustext[:48]}")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoint", default="udpin:0.0.0.0:14551")
    ap.add_argument("--rate", type=int, default=10, help="requested stream rate, Hz")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    link = MavLink(endpoint=args.endpoint)
    tracker = TelemetryTracker()

    while _running and not link.connect(timeout=10.0):
        logging.warning("waiting for SITL...")
    if link.connected:
        link.request_data_streams(args.rate)

    loop_period = 1.0 / LOOP_HZ
    next_loop = time.monotonic()
    next_print = time.monotonic()
    was_connected = True

    while _running:
        # 1. drain everything waiting on the socket this tick
        deadline = time.monotonic() + loop_period * 0.8
        while time.monotonic() < deadline:
            msg = link.recv(timeout=0.01)
            if msg is None:
                break
            tracker.update(msg)

        # 2. link health is authoritative, not inferred from messages
        tracker.apply_link_status(link.connected, link.seconds_since_heartbeat)

        # 3. self-heal
        if not link.connected:
            if was_connected:
                logging.error("LINK LOST")
                was_connected = False
            if link.ensure_connected(timeout=3.0):
                logging.info("LINK RESTORED")
                link.request_data_streams(args.rate)
                was_connected = True
        else:
            was_connected = True

        # 4. print at 1 Hz
        now = time.monotonic()
        if now >= next_print:
            print(render(tracker.state), flush=True)
            next_print += 1.0 / PRINT_HZ

        # 5. fixed-rate loop (absolute deadline, not sleep(period) —
        #    sleep(period) drifts, and on Day 10 you are measuring jitter)
        next_loop += loop_period
        sleep_for = next_loop - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_loop = time.monotonic()

    link.close()
    print("\nmessage counts:")
    for name, count in tracker.state.message_counts.most_common():
        print(f"  {name:<26} {count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())