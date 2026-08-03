"""Flight control for the REAL AUV1 airframe (torpedo + fins).

The SITL work (depth_hold.py etc.) drove a simulated ROV that has a
vertical thruster, so "go deeper" was one command. AUV1 has no
vertical thruster: depth is controlled by PITCHING THE NOSE DOWN AND
DRIVING FORWARD ("pitch-and-drive"). That requires a CASCADE — two
loops stacked:

    depth error ──[outer PID]──> target pitch angle (deg, limited)
    pitch error ──[inner PID]──> fin pitch command (-1..1)

The outer loop runs on slow depth dynamics; the inner loop tracks the
pitch target against fast attitude dynamics. Cascades are standard
practice everywhere in vehicle control (drone altitude→climb-rate,
car cruise→throttle) — this is our first.

Also here:
- roll stabilisation (hold 0 deg — passive stability helps, fins trim)
- heading loop with SPEED-BLENDED yaw authority: fins do the work at
  cruise, the bow tunnel thruster at low/zero speed, cross-faded so
  yaw never goes dead in the middle.

HONESTY BOX: pitch-and-drive produces no depth authority at zero
forward speed. Until the VBS exists (currently deferred — foam +
weights trim), depth control only works while moving. The mission
layer must not command "hover deep and stationary".

All pure Python, no MAVLink. The caller (bench/vehicle script) feeds
sensor values in and sends the returned commands out through
fin_mixer.deflection_to_pwm + MavlinkIO.set_servo_pwm.

ALL GAINS BELOW ARE STARTING GUESSES for the bench/pool sessions —
nothing here has met real water yet. Tune inner loops first (pitch,
roll, heading on the bench stand), outer depth loop last (pool).
"""

from auv1.controllers import PID, wrap_error_deg
from auv1.fin_mixer import mix

# ── Starting-guess gains (bench/pool tune; units in comments) ────────
DEPTH_TO_PITCH_GAINS = dict(kp=1.0, ki=0.05, kd=0.5,      # m err -> frac
                            output_limit=1.0,             # x MAX_PITCH
                            integral_limit=0.3)
# kp=1.0 means: 1 m of depth error requests the full MAX_PITCH angle.
PITCH_GAINS = dict(kp=0.04, ki=0.0, kd=0.02,              # deg err -> cmd
                   output_limit=1.0, integral_limit=10.0)
ROLL_GAINS = dict(kp=0.02, ki=0.0, kd=0.01,               # deg err -> cmd
                  output_limit=0.5, integral_limit=10.0)
HEADING_GAINS = dict(kp=0.02, ki=0.0, kd=0.02,            # deg err -> cmd
                     output_limit=0.6, integral_limit=60.0)

MAX_PITCH_DEG = 25.0     # never command steeper than this nose angle
# Sign: to go DEEPER (positive depth error), pitch NOSE DOWN (negative
# pitch). Flip on the bench if the vehicle climbs when told to dive.
DEPTH_TO_PITCH_SIGN = -1

# Bow tunnel yaw authority fades out as forward effort rises: full
# authority at standstill, none beyond BOW_FADE_FORWARD. Fins carry
# yaw at speed (flow over the rudders), the tunnel carries it at rest.
BOW_FADE_FORWARD = 0.4


def bow_authority(forward_effort: float) -> float:
    """0..1 weighting of the bow tunnel thruster for yaw."""
    f = min(abs(forward_effort), BOW_FADE_FORWARD)
    return 1.0 - f / BOW_FADE_FORWARD


class VehicleFlightController:
    """One update() per control tick. Feed sensors, get actuator demands.

    Inputs (SI-ish): depth m (+down), pitch/roll/heading deg
    (pitch + = nose up, heading 0..360), setpoints likewise, forward
    effort -1..1 commanded by pilot/mission.

    Output dict:
      fins:  {"top","bottom","left","right"} deflections -1..1
      stern: stern thruster effort -1..1
      bow:   bow tunnel effort -1..1
      pitch_sp: the cascade's internal pitch target (for logging)
    """

    def __init__(self):
        self.depth_to_pitch = PID(**DEPTH_TO_PITCH_GAINS)
        self.pitch = PID(**PITCH_GAINS)
        self.roll = PID(**ROLL_GAINS)
        self.heading = PID(**HEADING_GAINS)

    def reset(self) -> None:
        for pid in (self.depth_to_pitch, self.pitch, self.roll,
                    self.heading):
            pid.reset()

    def update(self, *, depth_m: float, pitch_deg: float, roll_deg: float,
               heading_deg: float, depth_sp_m: float, heading_sp_deg: float,
               forward: float, now: float | None = None) -> dict:
        # Outer loop: depth error -> pitch target (bounded)
        depth_err = depth_sp_m - depth_m          # +ve = need deeper
        pitch_sp = (DEPTH_TO_PITCH_SIGN * MAX_PITCH_DEG
                    * self.depth_to_pitch.update(depth_err, now=now))

        # Inner loops: attitude errors -> normalized commands
        pitch_cmd = self.pitch.update(pitch_sp - pitch_deg, now=now)
        roll_cmd = self.roll.update(0.0 - roll_deg, now=now)
        yaw_cmd = self.heading.update(
            wrap_error_deg(heading_sp_deg, heading_deg), now=now)

        fins = mix(pitch_cmd=pitch_cmd, roll_cmd=roll_cmd, yaw_cmd=yaw_cmd)
        return {
            "fins": fins,
            "stern": max(-1.0, min(1.0, forward)),
            "bow": yaw_cmd * bow_authority(forward),
            "pitch_sp": pitch_sp,
        }
