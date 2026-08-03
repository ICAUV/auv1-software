"""Supervisory ("fly-by-wire") teleoperation — the way real pilots fly.

Instead of driving thrust directly, the sticks adjust SETPOINTS and the
depth/heading PID loops do the actual flying underneath. This is the
same interface the mission runner uses; a mission is just this script
with the human replaced by a list.

Run on Windows (controller plugged in), against SITL:
    py scripts\\teleop_assisted.py

Needs the sim launched with an extra output to Windows (see
docs/sitl-notes.md), e.g. in WSL:
    ../Tools/autotest/sim_vehicle.py -L RATBeach --console \
        --out udp:127.0.0.1:14551 \
        --out udp:$(ip route show default | awk '{print $3}'):14552

Controls:
  Left stick  up/down    forward speed (direct effort, not a loop)
  Left stick  left/right heading setpoint slew (deg/s while deflected)
  RB / LB                depth setpoint deeper / shallower
  B button               freeze: zero forward, hold current depth+heading
  Back button            quit (neutral + disarm)

Everything is logged to logs/teleop_assisted_*.csv for post-run plots.
"""

import sys
import time
from pathlib import Path

import pygame

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from auv1.mavlink_io import MavlinkIO
from auv1.controllers import (PID, wrap_error_deg,
                              SIM_DEPTH_GAINS, SIM_HEADING_GAINS,
                              SIM_DEPTH_DIRECTION, SIM_HEADING_DIRECTION)
from auv1.telemetry_log import CsvLogger

# ── Configuration ────────────────────────────────────────────────────
CONNECTION = "udpin:0.0.0.0:14552"   # Windows-side port from the sim

LOOP_HZ = 10
DEADZONE = 0.12
MAX_FORWARD = 0.5          # cap forward effort
DEPTH_RATE_M_S = 0.3       # setpoint slew while RB/LB held
DEPTH_MIN_M, DEPTH_MAX_M = 0.0, 10.0
HEADING_RATE_DEG_S = 30.0  # setpoint slew at full stick deflection

# pygame ids — verify with teleop.py --debug if controls misbehave
AXIS_LX, AXIS_LY = 0, 1
BTN_B, BTN_BACK = 1, 6
BTN_LB, BTN_RB = 4, 5

import math


def deadzone(x: float) -> float:
    return 0.0 if abs(x) < DEADZONE else x


def main() -> None:
    conn = sys.argv[1] if len(sys.argv) > 1 else CONNECTION

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No controller found. Plug in / pair the Xbox controller.")
        return
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller: {js.get_name()}")

    io = MavlinkIO(conn)
    print(f"Listening on {conn} — waiting for heartbeat...")
    if not io.wait_heartbeat():
        print("No heartbeat. Is the sim outputting to this port? "
              "(see the launch command in this file's docstring)")
        return
    if not io.set_mode("MANUAL"):
        print("Could not set MANUAL mode.")
        return
    print("Arming...")
    if not io.arm():
        print("Arming refused — check MAVProxy console.")
        return

    # Setpoints start where the vehicle is: no surprise moves on start.
    depth = io.get_depth() or 0.0
    att = io.get_attitude()
    hdg = (math.degrees(att[2]) % 360.0) if att else 0.0
    depth_sp, heading_sp = max(0.0, depth), hdg
    print(f"Armed. Setpoints init: depth {depth_sp:.2f} m, "
          f"heading {heading_sp:.0f} deg. B=freeze, Back=quit.")

    depth_pid = PID(**SIM_DEPTH_GAINS)
    heading_pid = PID(**SIM_HEADING_GAINS)
    log = CsvLogger("teleop_assisted",
                    ["t", "depth", "depth_sp", "heading", "heading_sp",
                     "forward", "v_eff", "y_eff"])
    period = 1.0 / LOOP_HZ

    try:
        while True:
            pygame.event.pump()
            if js.get_button(BTN_BACK):
                print("\nBack — quitting.")
                break

            # ── Read vehicle state ──────────────────────────────────
            d = io.get_depth()
            att = io.get_attitude()
            if d is None or att is None:
                io.send_manual_control()   # stale data -> neutral
                continue
            hdg = math.degrees(att[2]) % 360.0

            # ── Sticks adjust setpoints ─────────────────────────────
            if js.get_button(BTN_B):       # freeze
                forward = 0.0
                depth_sp, heading_sp = d, hdg
                depth_pid.reset()
                heading_pid.reset()
            else:
                forward = deadzone(-js.get_axis(AXIS_LY)) * MAX_FORWARD
                slew = deadzone(js.get_axis(AXIS_LX))
                heading_sp = (heading_sp
                              + slew * HEADING_RATE_DEG_S * period) % 360.0
                if js.get_button(BTN_RB):
                    depth_sp = min(DEPTH_MAX_M,
                                   depth_sp + DEPTH_RATE_M_S * period)
                if js.get_button(BTN_LB):
                    depth_sp = max(DEPTH_MIN_M,
                                   depth_sp - DEPTH_RATE_M_S * period)

            # ── PIDs fly the setpoints ──────────────────────────────
            v_eff = SIM_DEPTH_DIRECTION * depth_pid.update(depth_sp - d)
            y_eff = SIM_HEADING_DIRECTION * heading_pid.update(
                wrap_error_deg(heading_sp, hdg))
            io.send_manual_control(forward=forward,
                                   vertical=v_eff, yaw=y_eff)

            log.row(depth=round(d, 3), depth_sp=round(depth_sp, 2),
                    heading=round(hdg, 1), heading_sp=round(heading_sp, 1),
                    forward=round(forward, 2), v_eff=round(v_eff, 3),
                    y_eff=round(y_eff, 3))
            print(f"d={d:5.2f}/{depth_sp:4.1f} m  "
                  f"hdg={hdg:5.1f}/{heading_sp:5.1f}  fwd={forward:+4.2f}",
                  end="\r")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nCtrl+C — quitting.")
    finally:
        try:
            io.send_manual_control()
            io.disarm()
            print("Neutralled and disarmed.")
        except Exception:
            print("WARNING: could not neutral/disarm — check vehicle!")
        log.close()
        pygame.quit()


if __name__ == "__main__":
    main()
