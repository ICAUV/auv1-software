"""Bench test: read path. Prints live roll/pitch/yaw from the vehicle.

Run:  python scripts/read_attitude.py
Tilt the Pixhawk and watch the numbers follow. Ctrl+C to stop.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from auv1.mavlink_io import MavlinkIO


def main():
    io = MavlinkIO()
    print("Waiting for heartbeat...")
    if not io.wait_heartbeat():
        print("No heartbeat. Check BlueOS endpoint (UDP Client -> this PC:14551).")
        return
    print("Connected. Streaming attitude (Ctrl+C to stop):")
    while True:
        att = io.get_attitude()
        if att:
            roll, pitch, yaw = att
            print(f"roll={roll:+.3f}  pitch={pitch:+.3f}  yaw={yaw:+.3f}", end="\r")


if __name__ == "__main__":
    main()
