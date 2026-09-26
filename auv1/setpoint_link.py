"""UDP setpoint link: ground station (laptop)  <-->  vehicle daemon (Pi).

Plain Python, no MAVLink. This is the "thin wire" between the two halves
of teleop once the control loop lives on the vehicle:

    laptop: SetpointSender.send(surge=.., yaw=.., pitch=.., roll=.., ...)
                      |  UDP, one small JSON packet per tick (20 Hz)
                      v
    Pi:     SetpointReceiver.poll()  -> latest packet (or None)
            SetpointReceiver.age()   -> seconds since the last packet
            SetpointReceiver.send_telemetry(...) -> back to the laptop

Why UDP and not TCP: a stale stick position is worthless, so we never
want the network to queue up old packets and replay them. UDP just drops
what doesn't arrive; the receiver keeps the newest packet only.

Why the laptop needs no fixed IP: the Pi replies to whatever address the
setpoints came from. The BlueOS "stream to 192.168.2.1" gotcha from the
bench does not apply here, and later the same code runs over a radio.

Packet (JSON dict): seq, surge, yaw, pitch, roll (-1..+1 floats),
arm (bool), estop (bool). Anything missing is filled from NEUTRAL.
"""

import json
import math
import socket
import time

DEFAULT_PORT = 47001      # vehicle listens here for setpoints
MAX_BYTES = 2048

NEUTRAL = {
    "surge": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0,
    "arm": False, "estop": False,
}


def _encode(d: dict) -> bytes:
    return json.dumps(d, separators=(",", ":")).encode("utf-8")


def _decode(b: bytes):
    try:
        d = json.loads(b.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return None
    return d if isinstance(d, dict) else None


class SetpointSender:
    """Laptop side: sends setpoints to the vehicle, receives telemetry back."""

    def __init__(self, vehicle_ip: str, port: int = DEFAULT_PORT):
        self.addr = (vehicle_ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.seq = 0
        self._last_telemetry_at = None   # time.monotonic()

    def send(self, **fields) -> dict:
        """Send one setpoint packet. Unspecified fields are neutral."""
        packet = dict(NEUTRAL)
        packet.update(fields)
        self.seq += 1
        packet["seq"] = self.seq
        try:
            self.sock.sendto(_encode(packet), self.addr)
        except OSError:
            pass    # e.g. cable unplugged — keep going, the vehicle failsafes
        return packet

    def poll_telemetry(self):
        """Return the newest telemetry dict received since last call, or None."""
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(MAX_BYTES)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                # Windows raises ConnectionResetError on ICMP "port unreachable"
                # (vehicle not listening yet). Ignore; try again next tick.
                break
            d = _decode(data)
            if d is not None:
                latest = d
                self._last_telemetry_at = time.monotonic()
        return latest

    def telemetry_age(self) -> float:
        if self._last_telemetry_at is None:
            return math.inf
        return time.monotonic() - self._last_telemetry_at

    def close(self) -> None:
        self.sock.close()


class SetpointReceiver:
    """Vehicle side: keeps the newest setpoint; knows how old it is."""

    def __init__(self, port: int = DEFAULT_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.latest = None
        self.peer = None                  # (ip, port) of the ground station
        self._last_rx_at = None           # time.monotonic()
        self._last_seq = -1

    def poll(self):
        """Drain everything waiting; return the newest packet (or the
        previous newest if nothing new arrived; None if never anything)."""
        while True:
            try:
                data, addr = self.sock.recvfrom(MAX_BYTES)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            d = _decode(data)
            if d is None:
                continue
            seq = d.get("seq", 0)
            if seq < self._last_seq and self._last_seq - seq < 1000:
                continue                  # out-of-order straggler: ignore
            self._last_seq = seq
            packet = dict(NEUTRAL)
            packet.update(d)
            self.latest = packet
            self.peer = addr
            self._last_rx_at = time.monotonic()
        return self.latest

    def age(self) -> float:
        """Seconds since the last packet arrived (inf if none yet).
        This is the link-health number the failsafe watches."""
        if self._last_rx_at is None:
            return math.inf
        return time.monotonic() - self._last_rx_at

    def send_telemetry(self, **fields) -> None:
        """Send a small status dict back to wherever setpoints came from."""
        if self.peer is None:
            return
        try:
            self.sock.sendto(_encode(fields), self.peer)
        except OSError:
            pass

    def close(self) -> None:
        self.sock.close()
