"""Manual teleoperation with an Xbox controller.

Drop into scripts/ and run:  python scripts/teleop.py
First run:  python scripts/teleop.py --debug   (shows raw stick/button
numbers so you can verify the AXIS_*/BTN_* constants below match your
controller/driver — numbering can vary).

Controls (defaults):
  Left stick  up/down    surge (stern thruster, MAIN 8)
  Left stick  left/right yaw (fins + bow tunnel thruster, AUX 1)
  Right stick up/down    pitch (fins)
  Right stick left/right roll (fins)
  LB / RB                depth setpoint shallower / deeper   (VBS — TBD)
  LT / RT                pitch-hold setpoint nose down / up  (VBS — TBD)
  B button               EMERGENCY NEUTRAL (everything to centre/stop)
  Back button            quit (neutrals everything first)

The depth and pitch-hold setpoints are placeholders: they are adjusted
and printed, but actuate nothing until the buoyancy engine (VBS)
hardware exists. When it does, feed them to the VBS controller.

Safety:
  - THRUSTERS_ENABLED starts False: thruster outputs are forced to 1500
    (stop) no matter what the sticks do. Set True only when you're ready,
    props off.
  - On any exit (Back button, Ctrl+C, crash, controller unplugged) all
    outputs are sent to neutral.
"""

import sys
import time
from pathlib import Path

import pygame

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from auv1.mavlink_io import MavlinkIO
from auv1.fin_mixer import mix, deflection_to_pwm

# ── Configuration ────────────────────────────────────────────────────
THRUSTERS_ENABLED = False   # <-- flip to True only when ready (props off)

FIN_OUTPUTS = {"top": 4, "bottom": 5, "left": 6, "right": 7}
STERN_THRUSTER_OUTPUT = 8   # MAIN 8
BOW_THRUSTER_OUTPUT = 9     # AUX 1

THRUSTER_STOP = 1500
THRUSTER_RANGE = 100        # gentle: +/-100 us for first tests (max 400)

DEADZONE = 0.12             # sticks rarely rest at exactly zero
LOOP_HZ = 20

# VBS setpoint tuning (placeholders until buoyancy engine exists)
DEPTH_RATE_M_S = 0.2        # metres/second while LB/RB held
DEPTH_MAX_M = 10.0
PITCH_RATE_DEG_S = 10.0     # degrees/second while LT/RT held
PITCH_MAX_DEG = 30.0

# pygame ids — verify with --debug, they vary by controller/driver
AXIS_LX, AXIS_LY = 0, 1
AXIS_RX, AXIS_RY = 2, 3
AXIS_LT, AXIS_RT = 4, 5
BTN_B, BTN_BACK = 1, 6
BTN_LB, BTN_RB = 4, 5

# ── Helpers ──────────────────────────────────────────────────────────

def deadzone(x: float) -> float:
    return 0.0 if abs(x) < DEADZONE else x


def trigger_01(raw: float) -> float:
    """Triggers idle at -1 and reach +1 when pulled; map to 0..1."""
    return (raw + 1.0) / 2.0


def neutral_all(io: MavlinkIO) -> None:
    for out in FIN_OUTPUTS.values():
        io.set_servo_pwm(out, deflection_to_pwm(0.0))
    io.set_servo_pwm(STERN_THRUSTER_OUTPUT, THRUSTER_STOP)
    io.set_servo_pwm(BOW_THRUSTER_OUTPUT, THRUSTER_STOP)


def main() -> None:
    debug = "--debug" in sys.argv

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No controller found. Plug in / pair the Xbox controller.")
        return
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller: {js.get_name()}  "
          f"({js.get_numaxes()} axes, {js.get_numbuttons()} buttons)")

    if debug:
        # Controller identification needs NO vehicle: pure pygame loop.
        print("DEBUG mode — move one control at a time. Ctrl+C to stop.")
        try:
            while True:
                pygame.event.pump()
                axes = [round(js.get_axis(i), 2)
                        for i in range(js.get_numaxes())]
                btns = [js.get_button(i)
                        for i in range(js.get_numbuttons())]
                print(f"axes={axes} buttons={btns}", end="\r")
                time.sleep(1.0 / LOOP_HZ)
        except KeyboardInterrupt:
            print("\nDebug done.")
        finally:
            pygame.quit()
        return

    io = MavlinkIO()
    print("Waiting for heartbeat...")
    if not io.wait_heartbeat():
        print("No heartbeat. Check BlueOS endpoint (UDP Client -> PC:14551).")
        return
    print("Connected. Teleop running — B = neutral, Back = quit.")
    if not THRUSTERS_ENABLED:
        print("NOTE: thrusters DISABLED (forced to stop). "
              "Edit THRUSTERS_ENABLED when ready.")

    period = 1.0 / LOOP_HZ
    depth_target_m = 0.0     # VBS placeholder setpoints
    pitch_target_deg = 0.0
    try:
        while True:
            pygame.event.pump()  # let pygame refresh controller state

            if debug:
                axes = [round(js.get_axis(i), 2) for i in range(js.get_numaxes())]
                btns = [js.get_button(i) for i in range(js.get_numbuttons())]
                print(f"axes={axes} buttons={btns}", end="\r")
                time.sleep(period)
                continue

            if js.get_button(BTN_BACK):
                print("\nBack pressed — quitting.")
                break
            if js.get_button(BTN_B):
                neutral_all(io)
                time.sleep(period)
                continue

            # Read sticks (pygame Y axes: up = -1, so negate for "up = +")
            surge = deadzone(-js.get_axis(AXIS_LY))
            yaw_cmd = deadzone(js.get_axis(AXIS_LX))
            pitch_cmd = deadzone(-js.get_axis(AXIS_RY))
            roll_cmd = deadzone(js.get_axis(AXIS_RX))

            # VBS setpoints (placeholders — printed, not yet actuated)
            changed = False
            if js.get_button(BTN_RB):   # deeper
                depth_target_m = min(DEPTH_MAX_M,
                                     depth_target_m + DEPTH_RATE_M_S * period)
                changed = True
            if js.get_button(BTN_LB):   # shallower
                depth_target_m = max(0.0,
                                     depth_target_m - DEPTH_RATE_M_S * period)
                changed = True
            trim = trigger_01(js.get_axis(AXIS_RT)) - trigger_01(js.get_axis(AXIS_LT))
            if abs(trim) > 0.05:        # RT = nose up, LT = nose down
                pitch_target_deg += trim * PITCH_RATE_DEG_S * period
                pitch_target_deg = max(-PITCH_MAX_DEG,
                                       min(PITCH_MAX_DEG, pitch_target_deg))
                changed = True
            if changed:
                print(f"[VBS setpoints] depth={depth_target_m:5.2f} m  "
                      f"pitch={pitch_target_deg:+6.1f} deg", end="\r")

            # Fins: desires -> deflections -> PWM
            fins = mix(pitch_cmd=pitch_cmd, roll_cmd=roll_cmd, yaw_cmd=yaw_cmd)
            for name, deflection in fins.items():
                io.set_servo_pwm(FIN_OUTPUTS[name], deflection_to_pwm(deflection))

            # Thrusters (bow tunnel assists yaw at low speed)
            if THRUSTERS_ENABLED:
                stern = int(THRUSTER_STOP + surge * THRUSTER_RANGE)
                bow = int(THRUSTER_STOP + yaw_cmd * THRUSTER_RANGE)
            else:
                stern = bow = THRUSTER_STOP
            io.set_servo_pwm(STERN_THRUSTER_OUTPUT, stern)
            io.set_servo_pwm(BOW_THRUSTER_OUTPUT, bow)

            time.sleep(period)
    except KeyboardInterrupt:
        print("\nCtrl+C — quitting.")
    finally:
        # Whatever happens, leave the vehicle limp and safe.
        try:
            neutral_all(io)
            print("All outputs neutralled.")
        except Exception:
            print("WARNING: could not neutral outputs — check vehicle!")
        pygame.quit()


if __name__ == "__main__":
    main()
