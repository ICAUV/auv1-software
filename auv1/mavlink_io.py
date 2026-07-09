"""Thin MAVLink I/O layer.

The ONLY file in this project that imports pymavlink. Everything else
(mixer, controllers) is plain Python so it can be reused unchanged after
a future migration to ROS2/MAVROS.
"""

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
        """Return the next COMMAND_ACK message, or None on timeout."""
        return self.conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
