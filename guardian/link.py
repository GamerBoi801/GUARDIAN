"""
guardian.link — the MAVLink transport layer.

Responsibility: own the socket, know whether the autopilot is alive, and
recover on its own when the far end disappears. Nothing in this module
understands what any message *means* — that is telemetry.py's job.

Why the split matters: on Day 8 the test harness restarts SITL between
every scenario. If reconnect logic is tangled into parsing code, one lost
link kills the whole suite run.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

from pymavlink import mavutil

log = logging.getLogger("guardian.link")

# Component IDs we care about. MAVProxy and ground stations also emit
# HEARTBEAT on the same wire; if you count those as "the autopilot is
# alive" you will never detect a flight controller failure.
_GCS_TYPES = {
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
}


class MavLink:
    """A self-healing MAVLink connection.

    Parameters
    ----------
    endpoint:
        pymavlink connection string. Note the direction:
          'udpin:0.0.0.0:14551'  -> we BIND and wait to be sent to.
          'udpout:127.0.0.1:14550' -> we SEND to that address.
        SITL's `--out=udp:127.0.0.1:14551` pushes to us, so we use udpin.
    heartbeat_timeout:
        Seconds without an autopilot HEARTBEAT before we declare the link
        dead. ArduPilot emits HEARTBEAT at 1 Hz, so 3-5 s is sane.
    source_system:
        Our own MAVLink system ID. Use something that is not 1 (the
        autopilot) and not 255 (MAVProxy's default) so you can tell who
        sent what when you are staring at a packet capture.
    """

    def __init__(
        self,
        endpoint: str = "udpin:0.0.0.0:14551",
        heartbeat_timeout: float = 5.0,
        source_system: int = 250,
    ) -> None:
        self.endpoint = endpoint
        self.heartbeat_timeout = heartbeat_timeout
        self.source_system = source_system

        self._conn: Optional[mavutil.mavfile] = None
        self._last_heartbeat: float = 0.0
        self._connect_attempts: int = 0

    # ------------------------------------------------------------------
    # connection lifecycle
    # ------------------------------------------------------------------

    def connect(self, timeout: float = 30.0) -> bool:
        """Open the socket and block until the first autopilot HEARTBEAT.

        Returns True on success. Never raises on a normal failure — the
        caller decides whether to retry, because a supervisor that dies
        at startup is worse than one that keeps trying.
        """
        self.close()
        self._connect_attempts += 1
        log.info("connecting to %s (attempt %d)", self.endpoint, self._connect_attempts)

        try:
            self._conn = mavutil.mavlink_connection(
                self.endpoint,
                source_system=self.source_system,
                autoreconnect=True,
            )
        except Exception:
            log.exception("could not open %s", self.endpoint)
            self._conn = None
            return False

        hb = self._conn.wait_heartbeat(timeout=timeout)
        if hb is None:
            log.warning("no heartbeat within %.0fs", timeout)
            self.close()
            return False

        self._last_heartbeat = time.monotonic()
        log.info(
            "connected: system %d component %d (%s)",
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.enums["MAV_AUTOPILOT"][hb.autopilot].name,
        )
        return True

    def close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:
                pass
        self._conn = None

    # ------------------------------------------------------------------
    # state
    # ------------------------------------------------------------------

    @property
    def seconds_since_heartbeat(self) -> float:
        if self._last_heartbeat == 0.0:
            return float("inf")
        return time.monotonic() - self._last_heartbeat

    @property
    def connected(self) -> bool:
        return (
            self._conn is not None
            and self.seconds_since_heartbeat < self.heartbeat_timeout
        )

    @property
    def target_system(self) -> int:
        return self._conn.target_system if self._conn else 0

    @property
    def target_component(self) -> int:
        return self._conn.target_component if self._conn else 0

    # ------------------------------------------------------------------
    # receive
    # ------------------------------------------------------------------

    def recv(self, timeout: float = 0.1):
        """Return one message, or None if nothing arrived in `timeout`.

        Heartbeat bookkeeping happens here so that no caller can forget
        to do it.
        """
        if self._conn is None:
            return None
        try:
            msg = self._conn.recv_match(blocking=True, timeout=timeout)
        except Exception:
            log.exception("recv failed; dropping connection")
            self.close()
            return None

        if msg is None:
            return None
        if msg.get_type() == "BAD_DATA":
            return None

        if msg.get_type() == "HEARTBEAT" and msg.type not in _GCS_TYPES:
            self._last_heartbeat = time.monotonic()

        return msg

    def ensure_connected(self, timeout: float = 5.0) -> bool:
        """Reconnect if the link has gone quiet. Call this every loop.

        This is the whole reason Day 1 bothers with a class instead of a
        bare mavutil connection.
        """
        if self.connected:
            return True
        log.warning(
            "link dead (%.1fs since heartbeat) — reconnecting",
            self.seconds_since_heartbeat,
        )
        return self.connect(timeout=timeout)

    # ------------------------------------------------------------------
    # send
    # ------------------------------------------------------------------

    def request_data_streams(self, rate_hz: int = 10) -> None:
        """Ask the autopilot to push telemetry at `rate_hz`.

        This is the legacy REQUEST_DATA_STREAM path. It is deprecated in
        the MAVLink spec but ArduPilot still honours it and it is one
        call instead of twelve. `set_message_interval` below is the
        modern per-message equivalent — use it when you need one message
        faster than the rest.
        """
        if self._conn is None:
            return
        self._conn.mav.request_data_stream_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            rate_hz,
            1,  # 1 = start sending, 0 = stop
        )
        log.info("requested all data streams at %d Hz", rate_hz)

    def set_message_interval(self, message_id: int, rate_hz: float) -> None:
        """Modern per-message rate control via MAV_CMD_SET_MESSAGE_INTERVAL."""
        if self._conn is None:
            return
        interval_us = 0 if rate_hz <= 0 else int(1_000_000 / rate_hz)
        self._conn.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,                # confirmation
            message_id,       # param1: message id
            interval_us,      # param2: interval in microseconds
            0, 0, 0, 0, 0,
        )