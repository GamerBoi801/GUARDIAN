#!/usr/bin/env python3
"""
Learning tool. Listen for N seconds and report what is actually on the wire
and how fast. Run this BEFORE you write telemetry.py.

    python3 scripts/sniff.py --seconds 15
    python3 scripts/sniff.py --seconds 15 --show GPS_RAW_INT

Two things you should learn from it:
  1. There are far more message types than the eight you care about.
  2. The rates are not what you assumed. HEARTBEAT is 1 Hz. ATTITUDE is
     fast. EKF_STATUS_REPORT is slow. That asymmetry is why VehicleState
     is a struct you patch incrementally rather than something you
     rebuild from one message.
"""

from __future__ import annotations

import argparse
import time
from collections import Counter

from pymavlink import mavutil


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoint", default="udpin:0.0.0.0:14551")
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--show", default=None, help="dump raw fields of this message type")
    args = ap.parse_args()

    print(f"connecting to {args.endpoint} ...")
    conn = mavutil.mavlink_connection(args.endpoint, source_system=251)
    hb = conn.wait_heartbeat(timeout=30)
    if hb is None:
        print("no heartbeat — is SITL running and forwarding to this port?")
        return 1
    print(f"heartbeat from system {conn.target_system} component {conn.target_component}")
    print(f"listening for {args.seconds:.0f}s ...\n")

    counts: Counter = Counter()
    shown = 0
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        msg = conn.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        mtype = msg.get_type()
        counts[mtype] += 1
        if args.show and mtype == args.show and shown < 3:
            print(f"--- raw {mtype} ---")
            for fname in msg.get_fieldnames():
                print(f"    {fname:<28} {getattr(msg, fname)!r}")
            print()
            shown += 1

    elapsed = time.monotonic() - start
    print(f"{'MESSAGE':<30} {'COUNT':>7} {'Hz':>7}")
    print("-" * 46)
    for name, count in counts.most_common():
        print(f"{name:<30} {count:>7} {count / elapsed:>7.1f}")
    print(f"\n{len(counts)} distinct message types in {elapsed:.1f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())