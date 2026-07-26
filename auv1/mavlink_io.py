"""Thin MAVLink I/O layer.

The ONLY file in this project that imports pymavlink. Everything else
(mixer, controllers) is plain Python so it can be reused unchanged after
a future migration to ROS2/MAVROS.
"""

# ── Pixhawk 6C output mapping (AUV1, custom control — no frame mixing) ──
# Our code owns all actuator outputs via DO_SET_SERVO. ArduSub does no
# mixing for us; it provides sensors, telemetry, arming, and failsafes.
#
#  Output  Servo#  Assignment
#  MAIN 1   1      (reserved by frame: Motor1 — leave unconnected)
#  MAIN 2   2      (reserved by frame: Motor2 — leave unconnected)
#  MAIN 3   3      (reserved by frame: Motor3 — leave unconnected)
#  MAIN 4   4      fin servo — top
#  MAIN 5   5      fin servo — bottom
#  MAIN 6   6      fin servo — left
#  MAIN 7   7      fin servo — right
#  MAIN 8   8      stern T200 thruster ESC (surge)
#  AUX 1    9      bow tunnel T200 thruster ESC (low-speed yaw)
#                  (was ArduSub default Lights1=181 — set to Disabled)
#  AUX 2    10     unused — left at ArduSub default Mount1Pitch
#                  (camera tilt if we ever add one)
#
# Notes:
# - DO_SET_SERVO only works on outputs whose SERVOn_FUNCTION = 0
#   (Disabled); anything else answers COMMAND_ACK result=4 FAILED.
# - ArduSub RE-APPLIES the frame's motor functions to MAIN 1-3 on every
#   boot (SimpleROV-3 frame). Setting them Disabled does NOT survive
#   restart — do not try to reclaim these outputs.
# - Servo numbering continues across connectors: AUX n = servo 8+n.

from pymavlink import mavutil


class MavlinkIO:
    """Connection to the vehicle via BlueOS's UDP endpoint."""

    def __init__(self, connection_string: str = "udpin:0.0.0.0:14551"):
        self.conn = mavutil.mavlink_connection(connection_string)

    def wait_heartbeat(self, timeout: float = 10.0) -> bool:
        """Block until the autopilot's heartbeat is seen."""
        hb = self.conn.wait_heartbeat(timeout=timeout)
        return hb is not None

    def get_attitude(self, timeout: float = 1.0):
        """Return (roll, pitch, yaw) in radians, or None on timeout."""
        msg = self.conn.recv_match(type="ATTITUDE", blocking=True, timeout=timeout)
        if msg is None:
            return None
        return msg.roll, msg.pitch, msg.yaw

    def get_depth(self, timeout: float = 1.0):
        """Return depth in metres (positive down), or None on timeout.

        Uses VFR_HUD.alt, which ArduSub reports as negative below surface.
        """
        msg = self.conn.recv_match(type="VFR_HUD", blocking=True, timeout=timeout)
        if msg is None:
            return None
        return -msg.alt

    def set_servo_pwm(self, output: int, pwm_us: int) -> None:
        """Command a servo output (1-based, e.g. MAIN 3 -> output=3).

        pwm_us: 1100-1900, centre 1500. Caller is responsible for limits.
        """
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            output, pwm_us, 0, 0, 0, 0, 0,
        )

    def wait_command_ack(self, timeout: float = 3.0):
        """Return the next COMMAND_ACK message, or None on timeout.

        The ack's .result field (MAV_RESULT) tells you the command's fate:
          0  ACCEPTED             executed successfully
          1  TEMPORARILY_REJECTED can't right now (busy) — retrying may work
          2  DENIED               command is invalid — will never work as sent
          3  UNSUPPORTED          autopilot doesn't know this command
          4  FAILED               understood but couldn't execute
                                  (e.g. DO_SET_SERVO on a mixer-owned output)
          5  IN_PROGRESS          started, not finished — more acks follow
          6  CANCELLED            aborted
        """
        return self.conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
