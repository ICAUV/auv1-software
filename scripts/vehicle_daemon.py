"""Vehicle daemon — runs ON THE PI. The actuator half of teleop.

    Xbox pad -> ground_station.py (laptop) --UDP setpoints--> THIS SCRIPT
    THIS SCRIPT --MAVLink DO_SET_SERVO--> BlueOS endpoint --> Pixhawk 6C

Same maths as scripts/teleop.py (fin_mixer.mix + the thruster lines), but
the Xbox reading has moved to the laptop and a link watchdog sits in
between. Run it from the repo root on the Pi:

    python scripts/vehicle_daemon.py                # defaults below
    python scripts/vehicle_daemon.py --thrusters    # enable thruster outputs

Failsafes (all local — they work even if the laptop vanishes):
  - no setpoint packet for STALE_S       -> every output to neutral
  - no setpoint packet for DISARM_AFTER_S -> also disarm
  - estop flag in the packet (B button)   -> every output to neutral
  - any exit (Back on the pad, Ctrl+C, crash) -> neutral + disarm
  - thrusters are OFF unless --thrusters is given (fins always live)

It also sends a 1 Hz GCS heartbeat to ArduSub so the GCS failsafe stays
quiet when QGroundControl isn't running, and streams a small telemetry
dict back to the laptop (armed flag, link age, attitude, depth).
"""

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from auv1.mavlink_io import MavlinkIO
from auv1.fin_mixer import mix, deflection_to_pwm
from auv1.setpoint_link import SetpointReceiver, NEUTRAL, DEFAULT_PORT
from auv1.telemetry_log import CsvLogger

# ── Configuration (mirrors teleop.py) ────────────────────────────────
FIN_OUTPUTS = {"top": 4, "bottom": 5, "left": 6, "right": 7}
STERN_THRUSTER_OUTPUT = 8   # MAIN 8  (T500)
BOW_THRUSTER_OUTPUT = 9     # AUX 1   (T200)

THRUSTER_STOP = 1500
THRUSTER_RANGE = 100        # gentle: +/-100 us for first tests (max 400)

LOOP_HZ = 20
TELEMETRY_HZ = 5
HEARTBEAT_HZ = 1

STALE_S = 0.7               # ~14 missed packets at 20 Hz -> neutral
DISARM_AFTER_S = 5.0        # still nothing -> disarm

# BlueOS: MAVLink Endpoints -> add "UDP Client" 127.0.0.1:14551.
# BlueOS then *sends* to this port; we listen on it.
DEFAULT_CONNECTION = "udpin:0.0.0.0:14551"

# MAVLink constants, written as numbers so this file needs no pymavlink
# import (mavlink_io.py stays the only importer).
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8


# ── Actuation helpers ────────────────────────────────────────────────

def neutral_all(io) -> None:
    for out in FIN_OUTPUTS.values():
        io.set_servo_pwm(out, deflection_to_pwm(0.0))
    io.set_servo_pwm(STERN_THRUSTER_OUTPUT, THRUSTER_STOP)
    io.set_servo_pwm(BOW_THRUSTER_OUTPUT, THRUSTER_STOP)


def apply_setpoint(io, sp: dict, thrusters_enabled: bool) -> dict:
    """Turn one setpoint packet into servo commands. Returns the PWMs sent
    (handy for logging and for tests with a fake io)."""
    sent = {}
    fins = mix(pitch_cmd=sp["pitch"], roll_cmd=sp["roll"], yaw_cmd=sp["yaw"])
    for name, deflection in fins.items():
        pwm = deflection_to_pwm(deflection)
        io.set_servo_pwm(FIN_OUTPUTS[name], pwm)
        sent[name] = pwm

    if thrusters_enabled:
        stern = int(THRUSTER_STOP + sp["surge"] * THRUSTER_RANGE)
        bow = int(THRUSTER_STOP + sp["yaw"] * THRUSTER_RANGE)   # bow tunnel assists yaw
    else:
        stern = bow = THRUSTER_STOP
    io.set_servo_pwm(STERN_THRUSTER_OUTPUT, stern)
    io.set_servo_pwm(BOW_THRUSTER_OUTPUT, bow)
    sent["stern"] = stern
    sent["bow"] = bow
    return sent


def drain_telemetry(io, state: dict) -> None:
    """Read whatever ATTITUDE / GLOBAL_POSITION_INT messages are waiting
    without blocking the control loop. Display + log only — the raw
    teleop loop does not use these for control."""
    while True:
        msg = io.conn.recv_match(type=["ATTITUDE", "GLOBAL_POSITION_INT"],
                                 blocking=False)
        if msg is None:
            return
        if msg.get_type() == "ATTITUDE":
            state["roll_deg"] = math.degrees(msg.roll)
            state["pitch_deg"] = math.degrees(msg.pitch)
            state["yaw_deg"] = math.degrees(msg.yaw) % 360.0
        else:
            state["depth_m"] = -msg.relative_alt / 1000.0


# ── Main loop ────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--connect", default=DEFAULT_CONNECTION,
                    help=f"pymavlink connection string (default {DEFAULT_CONNECTION})")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT,
                    help=f"UDP port to listen on for setpoints (default {DEFAULT_PORT})")
    ap.add_argument("--thrusters", action="store_true",
                    help="enable thruster outputs (default: forced to stop)")
    args = ap.parse_args()

    io = MavlinkIO(args.connect)
    print(f"Waiting for heartbeat on {args.connect} ...")
    if not io.wait_heartbeat():
        print("No heartbeat. Check BlueOS endpoint (UDP Client -> 127.0.0.1:14551).")
        return
    print("Autopilot connected.")
    if not args.thrusters:
        print("NOTE: thrusters DISABLED (forced to stop). Run with --thrusters when ready.")

    rx = SetpointReceiver(args.port)
    print(f"Listening for setpoints on UDP {args.port}. Waiting for ground station ...")

    log = CsvLogger("vehicle_daemon", [
        "t", "link_age", "armed", "estop",
        "surge", "yaw", "pitch", "roll",
        "top", "bottom", "left", "right", "stern", "bow",
        "roll_deg", "pitch_deg", "yaw_deg", "depth_m",
    ])

    period = 1.0 / LOOP_HZ
    telem_every = 1.0 / TELEMETRY_HZ
    hb_every = 1.0 / HEARTBEAT_HZ
    next_telem = next_hb = 0.0

    armed = False               # what we believe the autopilot state is
    prev_arm_cmd = False        # last packet's arm flag (edge detection)
    link_up = False
    stale_disarmed = False
    state = {"roll_deg": float("nan"), "pitch_deg": float("nan"),
             "yaw_deg": float("nan"), "depth_m": float("nan")}

    neutral_all(io)
    try:
        while True:
            tick = time.monotonic()
            sp = rx.poll() or NEUTRAL
            age = rx.age()
            drain_telemetry(io, state)

            # ── link watchdog ────────────────────────────────────────
            if age > STALE_S:
                if link_up:
                    print(f"\nLINK STALE ({age:.1f} s) — outputs neutral.")
                    link_up = False
                sent = apply_setpoint(io, NEUTRAL, thrusters_enabled=False)
                if age > DISARM_AFTER_S and armed and not stale_disarmed:
                    print("Link dead — disarming.")
                    io.disarm()
                    armed = False
                    stale_disarmed = True
            else:
                if not link_up:
                    print(f"\nLink up from {rx.peer[0]}.")
                    link_up = True
                    stale_disarmed = False

                # arm/disarm only on a CHANGE of the requested flag, so a
                # failed arm doesn't re-block the loop every tick
                if sp["arm"] != prev_arm_cmd:
                    if sp["arm"]:
                        print("Arming ...", end=" ", flush=True)
                        armed = io.arm()
                        print("OK" if armed else "FAILED (check QGC / pre-arm)")
                    else:
                        io.disarm()
                        armed = False
                        print("Disarmed.")
                prev_arm_cmd = sp["arm"]

                if sp["estop"]:
                    sent = apply_setpoint(io, NEUTRAL, thrusters_enabled=False)
                else:
                    sent = apply_setpoint(io, sp, thrusters_enabled=args.thrusters)

            # ── housekeeping: heartbeat, telemetry back, log ─────────
            if tick >= next_hb:
                io.conn.mav.heartbeat_send(MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0)
                next_hb = tick + hb_every
            if tick >= next_telem:
                rx.send_telemetry(armed=armed, link_age=round(age, 2),
                                  thrusters=args.thrusters, **{
                                      k: (None if math.isnan(v) else round(v, 2))
                                      for k, v in state.items()})
                next_telem = tick + telem_every
            log.row(link_age=round(min(age, 99.0), 3), armed=int(armed),
                    estop=int(bool(sp["estop"])),
                    surge=sp["surge"], yaw=sp["yaw"], pitch=sp["pitch"], roll=sp["roll"],
                    **sent, **state)

            time.sleep(max(0.0, period - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        print("\nCtrl+C — stopping.")
    finally:
        try:
            neutral_all(io)
            io.disarm()
            print("All outputs neutralled, disarmed.")
        except Exception:
            print("WARNING: could not neutral/disarm — check vehicle!")
        rx.close()
        log.close()


if __name__ == "__main__":
    main()
