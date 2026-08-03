"""PID controllers for depth-hold and heading-hold (roadmap week 6).

Pure Python, no MAVLink.

Also home to shared control helpers (angle wrapping) and the SIM-tuned
gain sets, so every script builds its PIDs from one place. These gains
were tuned against SITL (2026-07-26/27) — the real vehicle will be
retuned at the pool and should get its own gain sets here.
"""

import time

# Gains tuned in SITL. Units matter: depth error is metres, heading
# error is degrees — gains and clamps are NOT interchangeable.
SIM_DEPTH_GAINS = dict(kp=0.5, ki=0.2, kd=0.7,
                       output_limit=0.6, integral_limit=1.0)
SIM_HEADING_GAINS = dict(kp=0.02, ki=0.005, kd=0.02,
                         output_limit=0.5, integral_limit=60.0)

# Empirically verified output-direction signs (SITL):
# vertical effort is multiplied by DEPTH_DIRECTION before sending.
SIM_DEPTH_DIRECTION = -1
SIM_HEADING_DIRECTION = +1


def wrap_error_deg(target_deg: float, current_deg: float) -> float:
    """Shortest signed angular difference, -180..+180 degrees.

    Headings live on a circle: 350 -> 10 is +20 (turn right), not -340.
    """
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


class PID:
    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0,
                 output_limit: float = 1.0, integral_limit: float = 0.5):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = None
        self._prev_time = None

    def update(self, error: float, now: float | None = None) -> float:
        """Return control output for the given error. Call at a steady rate."""
        now = time.monotonic() if now is None else now
        d_term = 0.0
        if self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0:
                self._integral += error * dt
                self._integral = max(-self.integral_limit,
                                     min(self.integral_limit, self._integral))
                if self._prev_error is not None:
                    d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error
        self._prev_time = now

        out = self.kp * error + self.ki * self._integral + d_term
        return max(-self.output_limit, min(self.output_limit, out))
