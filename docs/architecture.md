# GUARDIAN — architecture

## Day 1: transport and telemetry

### Deployment
![Schema of the whole project](../assets/schema.png)

SITL runs on the laptop (Fedora, inside a distrobox Ubuntu container)
and forwards MAVLink over UDP. On Day 2 the same forwarding mechanism
feeds the Raspberry Pi.

### Module responsibilities

**link.py** — Owns the socket and tracks liveliness of the of the via autopilot HEARTBEATS
It deliberately understands no message semantics. It filters out
HEARTBEAT from ground stations, because 

**telemetry.py** — Turns the MavLink stream into a single Vehicle State(which is like a bref of allthe figues and variables that will be concernd from this project). Patches the struct field by field as the messages arrive.
This is the ONLY place raw MAVLink units are converted to their appropiate units.

### Why the split

2 Files one own teh transport while the other owns the meanigns. Units get converted in exactly in  one placeto look. and since everythignis downstream reads a plain data classs. the health monitors, and saftey lgoic acn be unit tested agaisnt fabricatedstates with no sim running 
### VehicleState
Vehicle state is like a liek a struct of all the figures and varialbes that are of concern for u in this project


Fields not yet received are None, never 0.0 — a zero-volt battery
reading would trip a false failsafe.

### MAVLink unit conversions
![unit table which is the output of the]

### Observed SITL limitations
- GPS: RTK_FIXED reported alongside HDOP 1.21. Fix type alone is not a
  trustworthy health signal; correlate with HDOP and satellite count.
- Battery: reaches 0% with the aircraft still flying normally. SITL does
  not model the consequence of exhausting the pack.