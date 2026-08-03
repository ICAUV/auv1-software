"""Unit tests for the PID controller and angle helpers.

Time is injected via update(..., now=...) so tests run instantly and
deterministically — no real sleeping.
"""

from auv1.controllers import PID, wrap_error_deg


def test_p_term_proportional():
    pid = PID(kp=2.0)
    assert pid.update(0.25, now=0.0) == 0.5


def test_output_clamped():
    pid = PID(kp=10.0, output_limit=0.6)
    assert pid.update(5.0, now=0.0) == 0.6
    assert pid.update(-5.0, now=1.0) == -0.6


def test_integral_accumulates_and_clamps():
    pid = PID(kp=0.0, ki=1.0, integral_limit=0.5, output_limit=10.0)
    pid.update(1.0, now=0.0)
    out = pid.update(1.0, now=0.3)          # integral = 0.3
    assert abs(out - 0.3) < 1e-9
    for i in range(1, 20):                   # keep pouring: must clamp
        out = pid.update(1.0, now=0.3 + i)
    assert out == 0.5                        # ki * integral_limit


def test_derivative_opposes_fast_change():
    pid = PID(kp=0.0, kd=1.0, output_limit=10.0)
    pid.update(0.0, now=0.0)
    out = pid.update(1.0, now=0.1)           # error rising fast
    assert out > 0                            # d-term follows d(error)/dt
    out2 = pid.update(0.5, now=0.2)           # error falling
    assert out2 < 0


def test_reset_clears_memory():
    pid = PID(kp=0.0, ki=1.0, integral_limit=5.0, output_limit=10.0)
    pid.update(1.0, now=0.0)
    pid.update(1.0, now=2.0)
    pid.reset()
    assert pid.update(0.0, now=3.0) == 0.0    # no leftover integral


def test_first_call_has_no_i_or_d():
    pid = PID(kp=1.0, ki=99.0, kd=99.0, output_limit=100.0)
    assert pid.update(0.5, now=0.0) == 0.5    # pure P on first tick


def test_wrap_error_shortest_way():
    assert wrap_error_deg(10, 350) == 20      # across North, rightward
    assert wrap_error_deg(350, 10) == -20     # across North, leftward
    assert wrap_error_deg(90, 90) == 0
    assert abs(wrap_error_deg(180, 0)) == 180
    assert -180 <= wrap_error_deg(0.0, 137.4) <= 180
