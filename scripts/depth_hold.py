"""Depth-hold controller — our first closed loop. (Roadmap week 6.)

Run against SITL:  python3 scripts/depth_hold.py 2.0
(2.0 = target depth in metres; default 2.0 if omitted)

What it does, 10 times a second:
  1. read current depth from the vehicle
  2. error = target - current
  3. feed the error to a PID controller
  4. send the PID's output as a vertical thrust demand (MANUAL_CONTROL)

In SITL the vertical demand drives the simulated ROV's thrusters via
ArduSub's own mixer. On the real vehicle the same PID output will
instead feed the VBS and/or pitch-and-drive via the fin mixer — the
loop's logic is identical, only the actuation stage changes.

Controls: Ctrl+C stops it; the vehicle is neutralled and disarmed on
the way out, whatever happens.

Tuning: start with the P gain only (KP). If the vehicle oscillates
around the target, lower KP or raise KD. If it settles short of the
target, raise KI slightly. Tune one knob at a time.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from auv1.mavlink_io import MavlinkIO
from auv1.controllers import PID

# ── Tuning ───────────────────────────────────────────────────────────
KP = 0.5    # thrust per metre of error (0.5 -> full thrust at 2 m off)
KI = 0.2   # creeps out steady-state error (buoyancy imbalance)
KD = 0.7   # damping — resists fast depth changes, kills overshoot

LOOP_HZ = 10
MAX_EFFORT = 0.6   # cap vertical demand at 60% while developing

# If the vehicle runs AWAY from the target depth, flip this to -1.
# (Sign depends on the convention of the vertical axis vs our depth.)
DIRECTION = -1     # -1: positive "go deeper" error -> push down


def main() -> None:
    target_m = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    print(f"Depth hold: target {target_m:.2f} m")

    io = MavlinkIO()
    print("Waiting for heartbeat...")
    if not io.wait_heartbeat():
        print("No heartbeat. Is the sim (or vehicle link) up?")
        return

    if not io.set_mode("MANUAL"):
        print("Could not set MANUAL mode.")
        return
    print("Mode MANUAL. Arming...")
    if not io.arm():
        print("Arming failed/refused (check prearm messages in MAVProxy).")
        return
    print("Armed. Holding depth — Ctrl+C to stop.")

    pid = PID(kp=KP, ki=KI, kd=KD, output_limit=MAX_EFFORT, integral_limit=1.0)
    period = 1.0 / LOOP_HZ

    try:
        while True:
            depth = io.get_depth()
            if depth is None:
                # No reading this tick — command neutral rather than
                # acting on stale information.
                io.send_manual_control(vertical=0.0)
                continue

            error = target_m - depth          # +ve = need to go deeper
            effort = pid.update(error)        # -MAX_EFFORT..+MAX_EFFORT
            io.send_manual_control(vertical=DIRECTION * effort)

            print(f"depth={depth:6.2f} m  target={target_m:5.2f}  "
                  f"error={error:+6.2f}  effort={DIRECTION * effort:+5.2f}",
                  end="\r")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nCtrl+C — stopping.")
    finally:
        try:
            io.send_manual_control()   # all neutral
            io.disarm()
            print("Neutralled and disarmed.")
        except Exception:
            print("WARNING: could not neutral/disarm — check vehicle!")


if __name__ == "__main__":
    main()
