#!/usr/bin/env python3
"""
gary_lidar_io.py -- shared connection helper for Gary's LiDAR viewer scripts
(gary_lidar_view.py, gary_lidar_map.py, gary_live_dashboard.py and
gary_web_dashboard.py all import this).

Supports two ways to reach Gary:
  - A serial port name, e.g. "COM15" or "/dev/ttyUSB0"  (USB cable)
  - "tcp:HOST:PORT", e.g. "tcp:192.168.1.42:3333"        (WiFi -- Gary prints
    his IP over Serial at boot once WiFi connects)

Both connection types expose the same things the viewer scripts need:
    .in_waiting   -- truthy when a full line is ready to read
    .readline()   -- returns bytes, including the trailing newline if present
    .write(bytes) -- send control chars back to Gary (see set_scanner_mode)
    .close()
"""

import socket

# Matches GARY_SERIAL_BAUD in GARY_v3_autoscan.ino. Bumped from 115200 so the
# ~1.8KB SCAN: frames cost ~20ms of TX instead of ~150ms -- the old rate was
# starving the firmware's main loop (and with it the voice-command poll).
DEFAULT_BAUD = 921600


class TCPLineSource:
    """Wraps a TCP socket so it looks enough like a pyserial.Serial object
    for the viewer scripts (in_waiting / readline / write / close)."""

    def __init__(self, host, port, connect_timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=connect_timeout)
        self.sock.settimeout(0.05)  # short poll timeout, matches the UI tick
        self._buf = b""

    def _pump(self):
        try:
            chunk = self.sock.recv(65536)
            if chunk:
                self._buf += chunk
        except socket.timeout:
            pass
        except OSError:
            pass

    @property
    def in_waiting(self):
        self._pump()
        return 1 if b"\n" in self._buf else 0

    def readline(self):
        self._pump()
        if b"\n" not in self._buf:
            return b""
        line, self._buf = self._buf.split(b"\n", 1)
        return line + b"\n"

    def write(self, data):
        # The firmware's poll_console() reads the WiFi client as well as USB
        # serial, so '1' / '0' / 's' work over TCP just like over the cable.
        try:
            self.sock.sendall(data)
        except OSError:
            pass

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def set_scanner_mode(src, on=True):
    """Ask Gary to start (or stop) streaming SCAN: frames.

    Scanner mode is OFF at boot: the 360-point dump is the single biggest
    consumer of firmware loop time, so it is opt-in rather than always-on.
    Every viewer script calls this on connect -- without it you connect
    successfully and then sit there receiving nothing, which looks exactly
    like a dead link.

    '1' / '0' are idempotent (unlike 's', which toggles), so this is safe to
    send on every connect and reconnect.
    """
    try:
        src.write(b"1" if on else b"0")
    except Exception:
        pass  # port opened read-only, or link already gone -- not fatal


def open_source(spec, baud=DEFAULT_BAUD, serial_timeout=0.5):
    """spec is either a serial port name or 'tcp:HOST:PORT'. Returns an object
    with .in_waiting / .readline() / .write() / .close(), raises on failure
    (OSError for TCP, serial.SerialException for serial)."""
    if spec.lower().startswith("tcp:"):
        _, host, port_str = spec.split(":")
        return TCPLineSource(host, int(port_str))
    else:
        import serial
        return serial.Serial(spec, baud, timeout=serial_timeout)
