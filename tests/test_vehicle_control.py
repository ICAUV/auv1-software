"""Unit tests for the real-airframe flight controller (the cascade).

These verify the CONVENTIONS, not the tuning: signs, limits, blending.
Gains will change at the pool; these contracts must not.
"""

from auv1.vehicle_control import (VehicleFlightController, bow_authority,
                                  MAX_PITCH_DEG, BOW_FADE_FORWARD)


def make():
    return VehicleFlightController()


def test_deeper_means_nose_down():
    c = make()
    out = c.update(depth_m=0.0, pitch_deg=0.0, roll_deg=0.0,
                   heading_deg=0.0, depth_sp_m=2.0, heading_sp_deg=0.0,
                   forward=0.4, now=0.0)
    # need to go deeper -> pitch target must be nose DOWN (negative)
    assert out["pitch_sp"] < 0
    # and the horizontal fins must be commanded (pitch authority)
    assert out["fins"]["left"] != 0.0 and out["fins"]["right"] != 0.0


def test_pitch_target_limited():
    c = make()
    out = c.update(depth_m=0.0, pitch_deg=0.0, roll_deg=0.0,
                   heading_deg=0.0, depth_sp_m=50.0, heading_sp_deg=0.0,
                   forward=0.4, now=0.0)
    assert abs(out["pitch_sp"]) <= MAX_PITCH_DEG + 1e-9


def test_at_depth_level_pitch_no_command():
    c = make()
    out = c.update(depth_m=2.0, pitch_deg=0.0, roll_deg=0.0,
                   heading_deg=90.0, depth_sp_m=2.0, heading_sp_deg=90.0,
                   forward=0.0, now=0.0)
    assert out["pitch_sp"] == 0.0
    assert all(v == 0.0 for v in out["fins"].values())
    assert out["bow"] == 0.0


def test_roll_stabilisation_responds_to_roll():
    c = make()
    out = c.update(depth_m=0.0, pitch_deg=0.0, roll_deg=10.0,
                   heading_deg=0.0, depth_sp_m=0.0, heading_sp_deg=0.0,
                   forward=0.0, now=0.0)
    # rolled -> all four fins engage (numeric signs are mounting-
    # dependent, see note in test_fin_mixer)
    assert all(v != 0.0 for v in out["fins"].values())


def test_yaw_uses_shortest_way_across_north():
    c = make()
    out = c.update(depth_m=0.0, pitch_deg=0.0, roll_deg=0.0,
                   heading_deg=350.0, depth_sp_m=0.0, heading_sp_deg=10.0,
                   forward=0.0, now=0.0)
    # +20 deg error -> some yaw command, and bow assists at zero speed
    assert out["fins"]["top"] != 0.0
    assert out["bow"] != 0.0


def test_bow_authority_fades_with_speed():
    assert bow_authority(0.0) == 1.0
    assert bow_authority(BOW_FADE_FORWARD) == 0.0
    assert bow_authority(1.0) == 0.0
    mid = bow_authority(BOW_FADE_FORWARD / 2)
    assert 0.0 < mid < 1.0
    # symmetric for reverse
    assert bow_authority(-BOW_FADE_FORWARD) == 0.0


def test_stern_clamped():
    c = make()
    out = c.update(depth_m=0.0, pitch_deg=0.0, roll_deg=0.0,
                   heading_deg=0.0, depth_sp_m=0.0, heading_sp_deg=0.0,
                   forward=3.0, now=0.0)
    assert out["stern"] == 1.0
