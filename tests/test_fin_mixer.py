"""Unit tests for the "+" fin mixer.

These encode the PHYSICAL CONTRACT of the fin layout: which fins are
allowed to respond to which command. If a SIGNS edit ever breaks that
contract (e.g. a pitch command moving the rudders), these fail before
the vehicle does.
"""

from auv1.fin_mixer import mix, deflection_to_pwm, PWM_CENTRE, PWM_RANGE


def test_pitch_only_moves_horizontal_fins():
    out = mix(pitch_cmd=0.5, roll_cmd=0.0, yaw_cmd=0.0)
    assert out["top"] == 0.0 and out["bottom"] == 0.0
    assert out["left"] != 0.0 and out["right"] != 0.0


def test_yaw_only_moves_vertical_fins():
    out = mix(pitch_cmd=0.0, roll_cmd=0.0, yaw_cmd=0.7)
    assert out["left"] == 0.0 and out["right"] == 0.0
    assert out["top"] != 0.0 and out["bottom"] != 0.0


def test_roll_moves_all_four():
    out = mix(pitch_cmd=0.0, roll_cmd=0.5, yaw_cmd=0.0)
    assert all(v != 0.0 for v in out.values())
    # NOTE: we deliberately do NOT assert numeric sign relationships
    # (e.g. top == -bottom). SIGNS is an EMPIRICAL table that absorbs
    # each servo's mounting orientation — two fins can be physically
    # opposed while sharing the same numeric sign. Physical direction
    # correctness is verified on the bench, not in unit tests.


def test_outputs_clamped():
    out = mix(pitch_cmd=1.0, roll_cmd=1.0, yaw_cmd=1.0)
    assert all(-1.0 <= v <= 1.0 for v in out.values())


def test_zero_in_zero_out():
    out = mix(pitch_cmd=0.0, roll_cmd=0.0, yaw_cmd=0.0)
    assert all(v == 0.0 for v in out.values())


def test_pwm_conversion():
    assert deflection_to_pwm(0.0) == PWM_CENTRE
    assert deflection_to_pwm(1.0) == PWM_CENTRE + PWM_RANGE
    assert deflection_to_pwm(-1.0) == PWM_CENTRE - PWM_RANGE
    # out-of-range input clamps rather than exceeding servo limits
    assert deflection_to_pwm(5.0) == PWM_CENTRE + PWM_RANGE
