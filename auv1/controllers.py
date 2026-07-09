"""PID controllers for depth-hold and heading-hold (roadmap week 6).

Pure Python, no MAVLink.
"""

import time


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
