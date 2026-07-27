"""Heading-hold controller — second closed loop. Same skeleton as
depth_hold.py, but the controlled variable is yaw (compass heading).

Run against SITL:  python3 scripts/heading_hold.py 90
(90 = target heading in degrees, 0 = North, 90 = East; default 0)

The one genuinely new problem here is ANGLE WRAP-AROUND: heading lives
on a circle. If you're pointing at 350° and want 010°, the error is
+20° (turn right), not -340° (turn almost all the way round the wrong
way). A naive `target - current` gets this wrong; wrap_error() fixes it
by mapping every error into -180..+180. This bug class bites every
robotics team once — we choose to be bitten in simulation.
"""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from auv1.mavlink_io import MavlinkIO
from auv1.controllers import PID

# ── Tuning (sim values — real vehicle will differ) ───────────────────
KP = 0.02   # effort per degree of error (0.02 -> full effort at 30 deg)
KI = 0.005    # usually unnecessary for heading; add if it settles offset
KD = 0.02   # damping against fast swings

LOOP_HZ = 10
MAX_EFFORT = 0.5

# Flip if the vehicle turns away from the target instead of toward it.
DIRECTION = +1


def wrap_error(target_deg: float, current_deg: float) -> float:
    """Shortest signed angular difference, in -180..+180 degrees."""
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def heading_deg(io: MavlinkIO):
    """Current heading in 0..360 degrees, or None on timeout."""
    att = io.get_attitude()
    if att is None:
        return None
    _roll, _pitch, yaw = att          # yaw is radians, -pi..+pi
    return math.degrees(yaw) % 360.0


def main() -> None:
    target = float(sys.argv[1]) % 360.0 if len(sys.argv) > 1 else 0.0
    print(f"Heading hold: target {target:.1f} deg")

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
    print("Armed. Holding heading — Ctrl+C to stop.")

    pid = PID(kp=KP, ki=KI, kd=KD, output_limit=MAX_EFFORT, integral_limit=60.0)
    period = 1.0 / LOOP_HZ

    try:
        while True:
            hdg = heading_deg(io)
            if hdg is None:
                io.send_manual_control()   # neutral, don't act on stale data
                continue

            error = wrap_error(target, hdg)   # -180..+180, sign = turn dir
            effort = pid.update(error)
            io.send_manual_control(yaw=DIRECTION * effort)

            print(f"heading={hdg:6.1f}  target={target:6.1f}  "
                  f"error={error:+7.1f}  effort={DIRECTION * effort:+5.2f}",
                  end="\r")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nCtrl+C — stopping.")
    finally:
        try:
            io.send_manual_control()
            io.disarm()
            print("Neutralled and disarmed.")
        except Exception:
            print("WARNING: could not neutral/disarm — check vehicle!")


if __name__ == "__main__":
    main()
