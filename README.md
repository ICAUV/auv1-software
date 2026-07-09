# AUV1 Software

Control software for AUV1 (Project Mer) — Imperial College AUV Society.

## Vehicle

Torpedo-style AUV: stern T200 (surge), lateral bow tunnel T200 (low-speed yaw),
four stern fins in "+" configuration (pitch/roll/yaw at speed), two buoyancy
engine actuators (depth/pitch trim, TBC). See
`02_System Design/SYS00 System Integration/SYS00 Vehicle Configuration v1.0.docx`
in the society Teams folder.

## Architecture (summer 2026)

```
Pixhawk 6C (ArduSub 4.5.7) --USB--> RPi5 (BlueOS) --Ethernet--> Topside PC
                                                        |-- QGroundControl (UDP 14550)
                                                        `-- this code      (UDP 14551)
```

ArduSub handles arming, sensors, telemetry, and failsafes. Custom control
(fin mixing, depth/heading hold) lives here and talks MAVLink via pymavlink.

**Design rule:** control logic (`fin_mixer.py`, `controllers.py`) is plain
Python with no MAVLink imports. Only `mavlink_io.py` touches pymavlink. This
keeps the maths portable for the planned ROS2 migration (autumn 2026): wrap
the modules in nodes, swap the I/O layer for MAVROS, done.

## Setup

1. `pip install pymavlink`
2. In BlueOS -> MAVLink Endpoints, ensure a **UDP Client -> <your PC IP>:14551**
   endpoint exists (PC is `192.168.2.1` on the tether).
3. Test the read path:  `python scripts/read_attitude.py`
4. Test the write path: `python scripts/test_servo.py` (safe with nothing
   connected; moves fin servo on MAIN 3 if one is attached and BEC-powered)

## Layout

| Path | Purpose |
|---|---|
| `auv1/mavlink_io.py` | Thin MAVLink I/O layer (only file importing pymavlink) |
| `auv1/fin_mixer.py` | "+" fin mixing maths (pure Python, unit-testable) |
| `auv1/controllers.py` | PID controllers for depth/heading hold |
| `scripts/` | Bench-test and utility scripts |
| `docs/` | Software docs |

## Safety

Props off for all bench work. Hardware kill switch before any powered
in-water test. Never trust "Disarmed" alone.
