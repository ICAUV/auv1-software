"""Fin mixing for the "+" stern fin configuration.

Pure Python, no MAVLink. Fin layout viewed from behind the vehicle:

        TOP
   LEFT  +  RIGHT
       BOTTOM

Convention (to be VERIFIED on the bench, then update SIGNS below):
- TOP and BOTTOM act as rudders -> yaw
- LEFT and RIGHT act as stern planes -> pitch
- All four differentially -> roll
"""

# Flip entries to -1 on the bench if a fin moves the wrong way.
SIGNS = {
    "top":    {"yaw": +1, "roll": +1},
    "bottom": {"yaw": +1, "roll": -1},
    "left":   {"pitch": +1, "roll": +1},
    "right":  {"pitch": +1, "roll": -1},
}

PWM_CENTRE = 1500
PWM_RANGE = 400  # max deflection = centre +/- range -> 1100..1900


def _clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, x))


def mix(pitch_cmd: float, roll_cmd: float, yaw_cmd: float) -> dict:
    """Map normalised commands (-1..+1) to fin deflections (-1..+1).

    Returns {"top": t, "bottom": b, "left": l, "right": r}.
    """
    s = SIGNS
    return {
        "top":    _clamp(s["top"]["yaw"] * yaw_cmd + s["top"]["roll"] * roll_cmd),
        "bottom": _clamp(s["bottom"]["yaw"] * yaw_cmd + s["bottom"]["roll"] * roll_cmd),
        "left":   _clamp(s["left"]["pitch"] * pitch_cmd + s["left"]["roll"] * roll_cmd),
        "right":  _clamp(s["right"]["pitch"] * pitch_cmd + s["right"]["roll"] * roll_cmd),
    }


def deflection_to_pwm(deflection: float) -> int:
    """Convert a -1..+1 deflection to a servo PWM value in microseconds."""
    return int(PWM_CENTRE + _clamp(deflection) * PWM_RANGE)
