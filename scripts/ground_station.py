"""Ground station — runs ON THE LAPTOP. The Xbox half of teleop.

    Xbox pad -> THIS SCRIPT --UDP setpoints--> vehicle_daemon.py (Pi) -> Pixhawk

Reads the controller exactly like scripts/teleop.py does, but instead of
talking MAVLink it sends one tiny setpoint packet per tick to the Pi and
prints the telemetry the Pi sends back. No pymavlink needed here.

    py scripts/ground_station.py                      # vehicle at 192.168.2.2
    py scripts/ground_station.py --vehicle 127.0.0.1  # daemon on this PC (rehearsal)

Controls:
  Left stick  up/down    surge (stern thruster)
  Left stick  left/right yaw   (fins + bow tunnel)
  Right stick up/down    pitch (fins)
  Right stick left/right roll  (fins)
  Start (Menu) button    toggle ARM / DISARM
  B button (hold)        EMERGENCY NEUTRAL — vehicle puts everything to centre/stop
  Back (View) button     quit (sends disarm + neutral first)

Losing this script, the cable, or the laptop is safe by design: the Pi
neutrals after 0.7 s of silence and disarms after 5 s.
"""

import argparse
import sys
import time
from pathlib import Path

import pygame

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from auv1.setpoint_link import SetpointSender, DEFAULT_PORT

# ── Configuration (mirrors teleop.py) ────────────────────────────────
DEFAULT_VEHICLE_IP = "192.168.2.2"     # BlueOS default over the tether
DEADZONE = 0.12
LOOP_HZ = 20
TELEMETRY_TIMEOUT_S = 2.0

# pygame ids — verify with  py scripts/teleop.py --debug  (they vary)
AXIS_LX, AXIS_LY = 0, 1
AXIS_RX, AXIS_RY = 2, 3
BTN_B, BTN_BACK, BTN_START = 1, 6, 7


def deadzone(x: float) -> float:
    return 0.0 if abs(x) < DEADZONE else x


def fmt(v, spec="+6.1f"):
    return "  --  " if v is None else format(v, spec)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--vehicle", default=DEFAULT_VEHICLE_IP,
                    help=f"IP of the Pi running vehicle_daemon (default {DEFAULT_VEHICLE_IP})")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No controller found. Plug in / pair the Xbox controller.")
        return
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller: {js.get_name()}  "
          f"({js.get_numaxes()} axes, {js.get_numbuttons()} buttons)")

    link = SetpointSender(args.vehicle, args.port)
    print(f"Sending setpoints to {args.vehicle}:{args.port} at {LOOP_HZ} Hz. "
          "Start = arm/disarm, B = neutral, Back = quit.")

    period = 1.0 / LOOP_HZ
    arm = False
    start_was_down = False
    telem = None
    try:
        while True:
            tick = time.monotonic()
            pygame.event.pump()

            if js.get_button(BTN_BACK):
                print("\nBack pressed — quitting.")
                break

            # Start button: toggle on the press edge, not while held
            start_down = bool(js.get_button(BTN_START))
            if start_down and not start_was_down:
                arm = not arm
                print(f"\n{'ARM' if arm else 'DISARM'} requested.")
            start_was_down = start_down

            estop = bool(js.get_button(BTN_B))

            # pygame Y axes: up = -1, so negate for "up = +"
            link.send(
                surge=deadzone(-js.get_axis(AXIS_LY)),
                yaw=deadzone(js.get_axis(AXIS_LX)),
                pitch=deadzone(-js.get_axis(AXIS_RY)),
                roll=deadzone(js.get_axis(AXIS_RX)),
                arm=arm,
                estop=estop,
            )

            t = link.poll_telemetry()
            if t is not None:
                telem = t
            if link.telemetry_age() > TELEMETRY_TIMEOUT_S:
                status = "NO TELEMETRY from vehicle (daemon running? cable? IP?)"
            else:
                status = (f"{'ARMED ' if telem.get('armed') else 'disarmed'} "
                          f"| link {fmt(telem.get('link_age'), '4.2f')}s "
                          f"| roll {fmt(telem.get('roll_deg'))} "
                          f"pitch {fmt(telem.get('pitch_deg'))} "
                          f"yaw {fmt(telem.get('yaw_deg'), '5.1f')} "
                          f"| depth {fmt(telem.get('depth_m'), '5.2f')} m "
                          f"| thrusters {'ON ' if telem.get('thrusters') else 'off'}")
            if estop:
                status = "E-STOP HELD — " + status
            print(status.ljust(110), end="\r")

            time.sleep(max(0.0, period - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        print("\nCtrl+C — quitting.")
    finally:
        # Tell the vehicle to stand down before we go (it would failsafe
        # anyway after 0.7 s / 5 s, but explicit is kinder).
        for _ in range(5):
            link.send(arm=False, estop=True)
            time.sleep(0.05)
        link.close()
        pygame.quit()
        print("Sent disarm + neutral. Bye.")


if __name__ == "__main__":
    main()
