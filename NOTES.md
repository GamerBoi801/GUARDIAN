# GUARDIAN — build notes

## Day 1 — MAVLink, SITL, telemetry

**Time sink:** getting ArduPilot SITL running under distrobox, then
reading the console/map output. Also spent real time working through
what sniff.py reported — which messages exist, at what rate, and
using those to catch unit-conversion mistakes.

**Learned:**
- HEARTBEAT is 1 Hz by design. A few missed beats is enough to declare
  a dead link, and the saved bandwidth goes to telemetry — which matters
  on a real 915 MHz radio.
- How to run the ArduPilot dev environment in distrobox on Fedora.
  Shared host networking means SITL and my Python talk over 127.0.0.1
  with no port forwarding.

**Anomalies found in my own output (→ limitations.md):**
- GPS reported RTK_FIXED with HDOP 1.21 — contradictory on real
  hardware. SITL's GPS model gives an optimistic fix type with no
  matching error model. GPS monitor must not trust fix_type alone.
- Battery hit 0% / 11199 mAh consumed and the aircraft kept hovering.
  SITL models the electrical battery, not the consequence of draining it.

**Gotcha:** ARMING_CHECK doesn't exist under that name in my build.
Parameter names drift between ArduPilot versions — grep the saved
parameter list rather than trusting docs. Matters on Day 7.

**Tomorrow:** Day 2 — DietPi companion computer, guardian.service under
systemd, SITL forwarding telemetry to the Pi over the LAN.