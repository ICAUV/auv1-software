"""Loopback tests for the UDP setpoint link (no vehicle, no controller)."""

import math
import time

from auv1.setpoint_link import SetpointSender, SetpointReceiver, NEUTRAL

PORT = 47999   # out of the way of a real daemon


def _wait_for(rx, predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        latest = rx.poll()
        if predicate(latest):
            return latest
        time.sleep(0.005)
    return rx.poll()


def test_roundtrip_and_defaults():
    rx = SetpointReceiver(PORT)
    tx = SetpointSender("127.0.0.1", PORT)
    try:
        assert rx.poll() is None
        assert math.isinf(rx.age())

        tx.send(surge=0.5, arm=True)
        got = _wait_for(rx, lambda p: p is not None)
        assert got["surge"] == 0.5
        assert got["arm"] is True
        assert got["yaw"] == NEUTRAL["yaw"]      # unspecified -> neutral
        assert got["estop"] is False
        assert rx.age() < 0.5
    finally:
        tx.close()
        rx.close()


def test_newest_packet_wins():
    rx = SetpointReceiver(PORT)
    tx = SetpointSender("127.0.0.1", PORT)
    try:
        for i in range(10):
            tx.send(yaw=i / 10)
        got = _wait_for(rx, lambda p: p is not None and p["seq"] == 10)
        assert got["seq"] == 10
        assert got["yaw"] == 0.9
    finally:
        tx.close()
        rx.close()


def test_telemetry_goes_back_to_sender():
    rx = SetpointReceiver(PORT)
    tx = SetpointSender("127.0.0.1", PORT)
    try:
        rx.send_telemetry(armed=True)           # no peer yet -> silently dropped
        assert tx.poll_telemetry() is None

        tx.send()
        _wait_for(rx, lambda p: p is not None)
        rx.send_telemetry(armed=True, link_age=0.01)

        deadline = time.monotonic() + 1.0
        t = None
        while t is None and time.monotonic() < deadline:
            t = tx.poll_telemetry()
            time.sleep(0.005)
        assert t == {"armed": True, "link_age": 0.01}
        assert tx.telemetry_age() < 0.5
    finally:
        tx.close()
        rx.close()


def test_age_grows_when_silent():
    rx = SetpointReceiver(PORT)
    tx = SetpointSender("127.0.0.1", PORT)
    try:
        tx.send()
        _wait_for(rx, lambda p: p is not None)
        a1 = rx.age()
        time.sleep(0.1)
        assert rx.age() >= a1 + 0.09
    finally:
        tx.close()
        rx.close()
