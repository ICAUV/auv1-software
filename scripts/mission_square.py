"""First autonomous mission: dive, fly a square, surface.

Run in Ubuntu against SITL:   python3 scripts/mission_square.py
(or on Windows with the 14552 output:  py scripts/mission_square.py udpin:0.0.0.0:14552)

A mission is a LIST OF STEPS, each with targets and a completion
condition. The runner is the same 10 Hz loop as the hold scripts —
both PIDs run every tick against the current step's targets; the only
new machinery is deciding when a step is "done":

- "stable-for" conditions (dive/turn): the target must be met
  continuously for HOLD_S seconds — a single lucky reading mustn't
  advance the mission.
- timed conditions (cruise legs): hold targets while the clock runs.

Watch it on the QGC map (the sim reaches Windows QGC on 14550): the
vehicle should trace a square, then surface. Everything logs to
logs/mission_square_*.csv.
"""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from auv1.mavlink_io import MavlinkIO
from auv1.controllers import (PID, wrap_error_deg,
                              SIM_DEPTH_GAINS, SIM_HEADING_GAINS,
                              SIM_DEPTH_DIRECTION, SIM_HEADING_DIRECTION)
from auv1.telemetry_log import CsvLogger

# ── The mission ──────────────────────────────────────────────────────
CRUISE_FWD = 0.4      # forward effort on legs
LEG_TIME_S = 20.0     # length of each side of the square
MISSION_DEPTH = 2.0

MISSION = [
    # (name, targets{depth,hdg,forward}, done_when{...})
    ("dive",     dict(depth=MISSION_DEPTH),
                 dict(depth_within=0.2, hold_s=3.0)),
    ("turn N",   dict(depth=MISSION_DEPTH, hdg=0.0),
                 dict(hdg_within=5.0, hold_s=2.0)),
    ("leg N",    dict(depth=MISSION_DEPTH, hdg=0.0,   forward=CRUISE_FWD),
                 dict(time_s=LEG_TIME_S)),
    ("leg E",    dict(depth=MISSION_DEPTH, hdg=90.0,  forward=CRUISE_FWD),
                 dict(time_s=LEG_TIME_S)),
    ("leg S",    dict(depth=MISSION_DEPTH, hdg=180.0, forward=CRUISE_FWD),
                 dict(time_s=LEG_TIME_S)),
    ("leg W",    dict(depth=MISSION_DEPTH, hdg=270.0, forward=CRUISE_FWD),
                 dict(time_s=LEG_TIME_S)),
    ("surface",  dict(depth=0.2),
                 dict(depth_within=0.3, hold_s=2.0)),
]

LOOP_HZ = 10


def step_done(done, t_in_step, depth_err, hdg_err, stable_since):
    """Evaluate a step's completion condition.

    Returns (done?, new_stable_since). stable_since tracks how long the
    'within tolerance' condition has held continuously (None = not
    currently met).
    """
    now = time.monotonic()
    if "time_s" in done:
        return t_in_step >= done["time_s"], None

    ok = True
    if "depth_within" in done:
        ok = ok and abs(depth_err) <= done["depth_within"]
    if "hdg_within" in done:
        ok = ok and abs(hdg_err) <= done["hdg_within"]

    if not ok:
        return False, None                    # streak broken
    if stable_since is None:
        return False, now                     # streak starts
    return (now - stable_since) >= done.get("hold_s", 0.0), stable_since


def main() -> None:
    conn = sys.argv[1] if len(sys.argv) > 1 else "udpin:0.0.0.0:14551"

    io = MavlinkIO(conn)
    print(f"Listening on {conn} — waiting for heartbeat...")
    if not io.wait_heartbeat():
        print("No heartbeat. Is the sim up (and outputting to this port)?")
        return
    if not io.set_mode("MANUAL"):
        print("Could not set MANUAL mode.")
        return
    print("Arming...")
    if not io.arm():
        print("Arming refused — check MAVProxy console.")
        return

    depth_pid = PID(**SIM_DEPTH_GAINS)
    heading_pid = PID(**SIM_HEADING_GAINS)
    log = CsvLogger("mission_square",
                    ["t", "step", "depth", "depth_sp", "heading",
                     "heading_sp", "forward", "v_eff", "y_eff"])
    period = 1.0 / LOOP_HZ

    step_i = 0
    step_t0 = time.monotonic()
    stable_since = None
    hold_hdg = None      # heading to hold when a step doesn't specify one
    print(f"Mission start: {len(MISSION)} steps. Ctrl+C aborts safely.")

    try:
        while step_i < len(MISSION):
            name, targets, done = MISSION[step_i]

            d = io.get_depth()
            att = io.get_attitude()
            if d is None or att is None:
                io.send_manual_control()
                continue
            hdg = math.degrees(att[2]) % 360.0

            if hold_hdg is None:
                hold_hdg = hdg               # first tick: remember heading

            depth_sp = targets.get("depth", 0.0)
            hdg_sp = targets.get("hdg", hold_hdg)
            forward = targets.get("forward", 0.0)

            depth_err = depth_sp - d
            hdg_err = wrap_error_deg(hdg_sp, hdg)

            v_eff = SIM_DEPTH_DIRECTION * depth_pid.update(depth_err)
            y_eff = SIM_HEADING_DIRECTION * heading_pid.update(hdg_err)
            io.send_manual_control(forward=forward,
                                   vertical=v_eff, yaw=y_eff)

            t_in_step = time.monotonic() - step_t0
            finished, stable_since = step_done(done, t_in_step,
                                               depth_err, hdg_err,
                                               stable_since)
            log.row(step=name, depth=round(d, 3), depth_sp=depth_sp,
                    heading=round(hdg, 1), heading_sp=round(hdg_sp, 1),
                    forward=forward, v_eff=round(v_eff, 3),
                    y_eff=round(y_eff, 3))
            print(f"[{step_i + 1}/{len(MISSION)} {name:9s}] "
                  f"d={d:5.2f}/{depth_sp:4.1f}  "
                  f"hdg={hdg:5.1f}/{hdg_sp:5.1f}  t={t_in_step:5.1f}s ",
                  end="\r")

            if finished:
                print(f"\nStep done: {name} ({t_in_step:.1f} s)")
                step_i += 1
                step_t0 = time.monotonic()
                stable_since = None
                hold_hdg = hdg               # carry current heading forward
                depth_pid.reset()            # fresh integrators per step
                heading_pid.reset()

            time.sleep(period)

        print("MISSION COMPLETE.")
    except KeyboardInterrupt:
        print("\nCtrl+C — mission aborted.")
    finally:
        try:
            io.send_manual_control()
            io.disarm()
            print("Neutralled and disarmed.")
        except Exception:
            print("WARNING: could not neutral/disarm — check vehicle!")
        log.close()


if __name__ == "__main__":
    main()
