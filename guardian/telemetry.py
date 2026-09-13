"""
guardian.telemetry — turn a stream of MAVLink messages into one struct.

Everything downstream (health monitors, the safety state machine, the
energy solver) reads VehicleState and nothing else. That is deliberate:
it means those modules can be unit-tested with a hand-built VehicleState
and no SITL running at all, which is what makes Days 4-6 fast.

ALL UNITS IN VehicleState ARE SI AND HUMAN-READABLE.
Raw MAVLink units (mm, cm/s, mV, cA, radians, HDOP*100) are converted
exactly once, here. If a conversion is wrong, it is wrong in one place.
"""

from __future__ import annotations

import math
import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Optional

from pymavlink import mavutil

# Sentinel used by MAVLink for "this field is unknown".
_UNKNOWN_I16 = -1
_UNKNOWN_U16 = 65535

_GPS_FIX_NAMES = {
    0: "NO_GPS",
    1: "NO_FIX",
    2: "2D_FIX",
    3: "3D_FIX",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
    7: "STATIC",
    8: "PPP",
}

# EKF_STATUS_REPORT.flags bits we actually care about.
_EKF_HEALTHY_BITS = (
    mavutil.mavlink.EKF_ATTITUDE
    | mavutil.mavlink.EKF_VELOCITY_HORIZ
    | mavutil.mavlink.EKF_POS_HORIZ_REL
    | mavutil.mavlink.EKF_POS_HORIZ_ABS
    | mavutil.mavlink.EKF_POS_VERT_ABS
)


@dataclass
class VehicleState:
    """One snapshot of everything GUARDIAN needs to make a decision."""

    # --- link -----------------------------------------------------
    connected: bool = False
    seconds_since_heartbeat: float = float("inf")

    # --- mode and arming ------------------------------------------
    armed: bool = False
    flight_mode: str = "UNKNOWN"
    system_status: int = 0

    # --- position (degrees, metres, m/s) --------------------------
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    alt_amsl_m: Optional[float] = None
    alt_rel_m: Optional[float] = None      # AGL relative to home — use this
    groundspeed_mps: Optional[float] = None
    climb_mps: Optional[float] = None
    heading_deg: Optional[float] = None

    # --- attitude (degrees) ---------------------------------------
    roll_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    yaw_deg: Optional[float] = None

    # --- battery (volts, amps, percent, mAh) ----------------------
    voltage_v: Optional[float] = None
    current_a: Optional[float] = None
    battery_remaining_pct: Optional[float] = None
    consumed_mah: Optional[float] = None
    cell_count: Optional[int] = None

    # --- GPS ------------------------------------------------------
    gps_fix_type: Optional[int] = None
    gps_fix_name: str = "UNKNOWN"
    satellites: Optional[int] = None
    hdop: Optional[float] = None

    # --- EKF ------------------------------------------------------
    ekf_flags: Optional[int] = None
    ekf_ok: Optional[bool] = None
    ekf_velocity_variance: Optional[float] = None
    ekf_pos_horiz_variance: Optional[float] = None
    ekf_pos_vert_variance: Optional[float] = None
    ekf_compass_variance: Optional[float] = None

    # --- last words from the autopilot ----------------------------
    last_statustext: str = ""
    last_statustext_severity: Optional[int] = None

    # --- bookkeeping ----------------------------------------------
    updated_at: float = 0.0
    message_counts: Counter = field(default_factory=Counter)

    def is_stale(self, max_age_s: float = 2.0) -> bool:
        return (time.monotonic() - self.updated_at) > max_age_s


class TelemetryTracker:
    """Feed it messages, read `.state`."""

    def __init__(self) -> None:
        self.state = VehicleState()

    def update(self, msg) -> None:
        if msg is None:
            return
        mtype = msg.get_type()
        s = self.state
        s.message_counts[mtype] += 1
        s.updated_at = time.monotonic()

        handler = getattr(self, "_on_" + mtype.lower(), None)
        if handler is not None:
            handler(msg)

    # ------------------------------------------------------------------
    # per-message handlers — one conversion site each
    # ------------------------------------------------------------------

    def _on_heartbeat(self, msg) -> None:
        # MAVProxy and QGC also send HEARTBEAT. Ignore anything that is
        # not the autopilot, or your "armed" flag will flap.
        if msg.type in (
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        ):
            return
        s = self.state
        s.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        s.system_status = msg.system_status
        try:
            s.flight_mode = mavutil.mode_string_v10(msg)
        except Exception:
            s.flight_mode = "MODE_%d" % msg.custom_mode

    def _on_global_position_int(self, msg) -> None:
        s = self.state
        s.lat_deg = msg.lat / 1e7          # degrees * 1e7
        s.lon_deg = msg.lon / 1e7
        s.alt_amsl_m = msg.alt / 1000.0    # millimetres
        s.alt_rel_m = msg.relative_alt / 1000.0
        vx = msg.vx / 100.0                # cm/s -> m/s
        vy = msg.vy / 100.0
        s.groundspeed_mps = math.hypot(vx, vy)
        s.climb_mps = -msg.vz / 100.0      # vz is NED: positive = descending
        s.heading_deg = None if msg.hdg == _UNKNOWN_U16 else msg.hdg / 100.0

    def _on_attitude(self, msg) -> None:
        s = self.state
        s.roll_deg = math.degrees(msg.roll)     # radians in the wire format
        s.pitch_deg = math.degrees(msg.pitch)
        s.yaw_deg = math.degrees(msg.yaw) % 360.0

    def _on_sys_status(self, msg) -> None:
        s = self.state
        if msg.voltage_battery != _UNKNOWN_U16:
            s.voltage_v = msg.voltage_battery / 1000.0     # millivolts
        if msg.current_battery != _UNKNOWN_I16:
            s.current_a = msg.current_battery / 100.0      # CENTIAMPS
        if msg.battery_remaining != _UNKNOWN_I16:
            s.battery_remaining_pct = float(msg.battery_remaining)

    def _on_battery_status(self, msg) -> None:
        s = self.state
        cells = [v for v in msg.voltages if v != _UNKNOWN_U16]
        if cells:
            s.cell_count = len(cells)
            s.voltage_v = sum(cells) / 1000.0              # per-cell mV
        if msg.current_battery != _UNKNOWN_I16:
            s.current_a = msg.current_battery / 100.0
        if msg.current_consumed != -1:
            s.consumed_mah = float(msg.current_consumed)
        if msg.battery_remaining != _UNKNOWN_I16:
            s.battery_remaining_pct = float(msg.battery_remaining)

    def _on_gps_raw_int(self, msg) -> None:
        s = self.state
        s.gps_fix_type = msg.fix_type
        s.gps_fix_name = _GPS_FIX_NAMES.get(msg.fix_type, "UNKNOWN")
        s.satellites = msg.satellites_visible
        s.hdop = None if msg.eph == _UNKNOWN_U16 else msg.eph / 100.0  # HDOP*100

    def _on_ekf_status_report(self, msg) -> None:
        s = self.state
        s.ekf_flags = msg.flags
        s.ekf_ok = (msg.flags & _EKF_HEALTHY_BITS) == _EKF_HEALTHY_BITS
        s.ekf_velocity_variance = msg.velocity_variance
        s.ekf_pos_horiz_variance = msg.pos_horiz_variance
        s.ekf_pos_vert_variance = msg.pos_vert_variance
        s.ekf_compass_variance = msg.compass_variance

    def _on_statustext(self, msg) -> None:
        s = self.state
        text = msg.text
        if isinstance(text, (bytes, bytearray)):
            text = text.decode("utf-8", errors="replace")
        s.last_statustext = text.rstrip("\x00").strip()
        s.last_statustext_severity = msg.severity

    # ------------------------------------------------------------------

    def apply_link_status(self, connected: bool, seconds_since_heartbeat: float) -> None:
        """Link health is owned by link.py, not inferred from messages."""
        self.state.connected = connected
        self.state.seconds_since_heartbeat = seconds_since_heartbeat