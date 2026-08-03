# AUV1 Software

![CI](https://github.com/ICAUV/auv1-software/actions/workflows/ci.yml/badge.svg)

Control software for AUV1 (Project Mer) — Imperial College AUV Society.

**Status (Aug 2026):** simulation stack complete and validated — teleop,
depth/heading hold loops, supervisory (fly-by-wire) control, autonomous
missions, telemetry logging + plotting, unit tests + CI. Real-vehicle
bench work (fin stabilisation via the cascade controller) is next.

## Vehicle

Torpedo-style AUV: stern T200 (surge), lateral bow tunnel T200
(low-speed yaw), four stern fins in "+" configuration (pitch/roll/yaw
at speed). Depth on the real vehicle is **pitch-and-drive** (no
vertical thruster); a variable-buoyancy system is deferred (foam +
weights trim for now). Full spec: `SYS00 Vehicle Configuration
v1.0.docx` in the society Teams folder.

## Architecture

```
REAL:  Pixhawk 6C (ArduSub) --USB--> RPi5 (BlueOS) --Ethernet--> Topside PC
SIM:   ArduSub SITL in WSL2 ------------------------> same PC, same scripts
                                     |-- QGroundControl   (UDP 14550)
                                     |-- scripts, Ubuntu  (UDP 14551)
                                     `-- scripts, Windows (UDP 14552)
```

ArduSub provides sensors, telemetry, arming, failsafes — **no mixing**.
Our Python owns every actuator via DO_SET_SERVO (real vehicle) or
drives ArduSub's mixer via MANUAL_CONTROL (simulation).

**Design rule:** control maths (`fin_mixer`, `controllers`,
`vehicle_control`) is plain Python with no MAVLink imports; only
`mavlink_io.py` speaks the protocol. That keeps the maths
unit-testable without hardware and makes the planned ROS2 migration
(autumn 2026) cheap.

## Quick start

Full from-zero setup (WSL, SITL, BlueOS, controller): see
**`AUV1_Software_Manual_v1.0.docx`** in Teams → 07_Documentation, or
`docs/sitl-notes.md` for the short version. Daily sim session:

```bash
# Ubuntu terminal 1 — the simulator (RATBeach is mandatory):
cd ~/ardupilot/ArduSub
../Tools/autotest/sim_vehicle.py -L RATBeach --console \
    --out udp:127.0.0.1:14551 \
    --out udp:$(ip route show default | awk '{print $3}'):14552

# Ubuntu terminal 2 — fly something:
cd ~/auv1-software && git pull
python3 scripts/mission_square.py
```

## Layout

| Path | Purpose |
|---|---|
| `auv1/mavlink_io.py` | MAVLink I/O layer (the only pymavlink importer); output-map table |
| `auv1/fin_mixer.py` | "+" fin mixing; `SIGNS` = bench-verified direction facts |
| `auv1/controllers.py` | PID (anti-windup, injectable time), angle wrap, SIM gain sets |
| `auv1/vehicle_control.py` | Real-airframe cascade: depth→pitch, attitude loops, blended yaw |
| `auv1/telemetry_log.py` | CSV black box (`AUV1_LOG_DIR` overrides destination) |
| `scripts/read_attitude.py`, `test_servo.py` | Bench read/write path tests |
| `scripts/teleop.py` | Raw teleop + `--debug` controller identification |
| `scripts/depth_hold.py`, `heading_hold.py` | Single hold loops (SITL-tuned) |
| `scripts/teleop_assisted.py` | Supervisory teleop — sticks move setpoints, PIDs fly |
| `scripts/mission_square.py` | Autonomous mission runner (step list + completion conditions) |
| `scripts/plot_log.py` | Telemetry CSV → PNG |
| `tests/` | 21 unit tests (run `py -m pytest -q`); CI runs them on every push |
| `docs/devlog.md` | **The engineering log — update after every session** |

## Conventions

- Depth positive down; pitch positive nose-up; heading 0–360 (0 = N).
- Servo language: PWM µs, 1500 neutral, ±400 full deflection.
- Windows commits (GitHub Desktop), Ubuntu pulls and runs.
- Commit style: `feat:` / `fix:` / `docs:` / `test:` / `chore:`, imperative.

## Safety

Props off for all bench work. Thrusters are software-disabled by
default in teleop. Every script neutrals all outputs on any exit.
Arm deliberately, disarm when you walk away. Never trust "Disarmed"
alone — hardware kill before real water.
