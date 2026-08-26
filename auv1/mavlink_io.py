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
#  MAIN 8   8      stern thruster ESC (surge) — T100 interim, T200/T500 later
#  AUX 1    9      bow tunnel thruster ESC (low-speed yaw) — T100 interim
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

        Uses GLOBAL_POSITION_INT.relative_alt: altitude relative to HOME
        in millimetres, negative below the surface. Chosen over VFR_HUD.alt
        because that one is altitude above sea level — in SITL, home sits
        584 m up in Australia, which made the vehicle "584 m above the
        water" and sent the depth PID to full thrust. (Bug found night 2.)
        """
        msg = self.conn.recv_match(type="GLOBAL_POSITION_INT",
                                   blocking=True, timeout=timeout)
        if msg is None:
            return None
        return -msg.relative_alt / 1000.0

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

    def set_mode(self, mode_name: str) -> bool:
        """Switch flight mode by name, e.g. "MANUAL", "ALT_HOLD".

        Returns False if the autopilot doesn't know that mode.
        """
        mapping = self.conn.mode_mapping()
        if mapping is None or mode_name not in mapping:
            return False
        self.conn.set_mode(mapping[mode_name])
        return True

    def arm(self, timeout: float = 5.0) -> bool:
        """Arm the vehicle (enables outputs). Returns True when confirmed."""
        self.conn.arducopter_arm()
        try:
            self.conn.motors_armed_wait(timeout=timeout)
        except TypeError:
            # older pymavlink versions take no timeout argument
            self.conn.motors_armed_wait()
        return bool(self.conn.motors_armed())

    def disarm(self) -> None:
        """Disarm the vehicle (outputs safe)."""
        self.conn.arducopter_disarm()

    def send_manual_control(self, forward: float = 0.0, lateral: float = 0.0,
                            vertical: float = 0.0, yaw: float = 0.0) -> None:
        """Send a MANUAL_CONTROL message (what a joystick sends via QGC).

        All inputs are -1..+1. ArduSub's own mixer turns them into motor
        commands for whatever frame it's configured with.

        Used for SITL development: the simulated vehicle is a standard
        thruster ROV, so DO_SET_SERVO on our custom outputs won't move it —
        this will. On the real vehicle, our controllers will instead feed
        the fin mixer / VBS via DO_SET_SERVO.

        Wire format quirks: x/y/r are -1000..1000 (0 = neutral), but z is
        0..1000 with 500 = neutral (up = more, down = less).
        """
        def clamp(v):
            return max(-1.0, min(1.0, v))

        self.conn.mav.manual_control_send(
            self.conn.target_system,
            int(clamp(forward) * 1000),
            int(clamp(lateral) * 1000),
            int(500 + clamp(vertical) * 500),
            int(clamp(yaw) * 1000),
            0,  # no buttons pressed
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
