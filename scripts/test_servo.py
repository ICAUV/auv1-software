"""Bench test: write path. Sweeps fin servo on MAIN 3 (safe if nothing wired).

Run:  python scripts/test_servo.py
With no servo attached, a COMMAND_ACK result of 0 (ACCEPTED) still proves
the command chain works. With a BEC-powered servo on MAIN 3, it sweeps.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from auv1.mavlink_io import MavlinkIO

OUTPUT = 7  # MAIN 3 = first fin servo per SYS00 output mapping


def main():
    io = MavlinkIO()
    print("Waiting for heartbeat...")
    if not io.wait_heartbeat():
        print("No heartbeat. Check BlueOS endpoint (UDP Client -> this PC:14551).")
        return

    for pwm in (1500, 1600, 1400, 1500):
        print(f"MAIN {OUTPUT} -> {pwm} us")
        io.set_servo_pwm(OUTPUT, pwm)
        ack = io.wait_command_ack()
        if ack is None:
            print("  no ACK (timeout)")
        else:
            ok = "ACCEPTED" if ack.result == 0 else f"result={ack.result}"
            print(f"  ACK: {ok}")
        time.sleep(1.0)


if __name__ == "__main__":
    main()
