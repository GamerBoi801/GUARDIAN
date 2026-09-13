# GUARDIAN — architecture

## Day 1: transport and telemetry

### Deployment
[paste the block diagram from the plan]

SITL runs on the laptop (Fedora, inside a distrobox Ubuntu container)
and forwards MAVLink over UDP. On Day 2 the same forwarding mechanism
feeds the Raspberry Pi.

### Module responsibilities

**link.py** — <<< one sentence: what job it owns, including how it
detects liveness >>>

It deliberately understands no message semantics. It filters out
HEARTBEAT from ground stations, because <<< why? >>>

**telemetry.py** — <<< one sentence >>>

This is the ONLY place raw MAVLink units are converted.

### Why the split

<<< two or three sentences: what the separation buys you >>>

### VehicleState

<<< one sentence: what it is >>>

Fields not yet received are None, never 0.0 — a zero-volt battery
reading would trip a false failsafe.

### MAVLink unit conversions
[paste the unit table from the Day 1 walkthrough]

### Observed SITL limitations
- GPS: RTK_FIXED reported alongside HDOP 1.21. Fix type alone is not a
  trustworthy health signal; correlate with HDOP and satellite count.
- Battery: reaches 0% with the aircraft still flying normally. SITL does
  not model the consequence of exhausting the pack.