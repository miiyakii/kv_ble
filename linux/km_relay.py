#!/usr/bin/env python3
"""
km_relay.py — KM Switch Linux relay

Reads km_proto frames from Central Dongle (USB CDC-ACM), injects HID events
into Linux via uinput, detects Shift+F1/F2 hotkeys to switch target PC,
and forwards frames to the corresponding Peripheral Dongle.

Frame format: [0xAA][TYPE:1B][LEN:1B][PAYLOAD:LEN][CRC8-CCITT:1B]

Types:
  0x01  Central→Linux  Keyboard report (8 bytes: mod, rsvd, key1..key6)
  0x02  Central→Linux  Mouse report    (4 bytes: buttons, dx, dy, wheel)
  0x10  Linux→Central  Switch notify   (1 byte: target 1 or 2)

Usage:
  python3 km_relay.py --central /dev/ttyACM0 --pc1 /dev/ttyACM1 --pc2 /dev/ttyACM2
"""

import argparse
import logging
import select
import serial
import struct
import sys
import time

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("km_relay")

# ── km_proto constants ────────────────────────────────────────────────────────
MAGIC        = 0xAA
TYPE_KB      = 0x01
TYPE_MOUSE   = 0x02
TYPE_SWITCH  = 0x10
CRC_INIT     = 0xFF
PAYLOAD_MAX  = 16


# ── CRC8-CCITT ────────────────────────────────────────────────────────────────
def crc8_ccitt(data: bytes, init: int = CRC_INIT) -> int:
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

# ── Frame builder ─────────────────────────────────────────────────────────────
def build_frame(ftype: int, payload: bytes) -> bytes:
    crc = crc8_ccitt(payload)
    return bytes([MAGIC, ftype, len(payload)]) + payload + bytes([crc])

# ── Frame parser (state machine) ──────────────────────────────────────────────
class FrameParser:
    ST_MAGIC, ST_TYPE, ST_LEN, ST_PAYLOAD, ST_CRC = range(5)

    def __init__(self, callback):
        """callback(ftype, payload) called on each valid frame."""
        self._cb = callback
        self._reset()

    def _reset(self):
        self._state   = self.ST_MAGIC
        self._ftype   = 0
        self._flen    = 0
        self._payload = bytearray()

    def feed(self, data: bytes):
        for byte in data:
            self._process(byte)

    def _process(self, byte: int):
        s = self._state

        if s == self.ST_MAGIC:
            if byte == MAGIC:
                self._state = self.ST_TYPE

        elif s == self.ST_TYPE:
            self._ftype = byte
            self._state = self.ST_LEN

        elif s == self.ST_LEN:
            if byte == 0 or byte > PAYLOAD_MAX:
                log.warning("Bad frame len=%d, resyncing", byte)
                self._reset()
                return
            self._flen    = byte
            self._payload = bytearray()
            self._state   = self.ST_PAYLOAD

        elif s == self.ST_PAYLOAD:
            self._payload.append(byte)
            if len(self._payload) >= self._flen:
                self._state = self.ST_CRC

        elif s == self.ST_CRC:
            expected = crc8_ccitt(bytes(self._payload))
            if byte == expected:
                log.debug("Frame type=0x%02x payload=%s",
                          self._ftype, self._payload.hex())
                self._cb(self._ftype, bytes(self._payload))
            else:
                log.warning("CRC mismatch: got 0x%02x expected 0x%02x", byte, expected)
            self._reset()

# ── uinput keyboard/mouse injector ───────────────────────────────────────────
try:
    import evdev
    from evdev import UInput, ecodes as e

    # Key table: HID keycode → evdev keycode
    # Covers modifier keys + F1..F12 + standard keys used in practice
    HID_TO_EVDEV = {
        # Modifiers
        0x01: e.KEY_UNKNOWN,
        0x04: e.KEY_A, 0x05: e.KEY_B, 0x06: e.KEY_C, 0x07: e.KEY_D,
        0x08: e.KEY_E, 0x09: e.KEY_F, 0x0A: e.KEY_G, 0x0B: e.KEY_H,
        0x0C: e.KEY_I, 0x0D: e.KEY_J, 0x0E: e.KEY_K, 0x0F: e.KEY_L,
        0x10: e.KEY_M, 0x11: e.KEY_N, 0x12: e.KEY_O, 0x13: e.KEY_P,
        0x14: e.KEY_Q, 0x15: e.KEY_R, 0x16: e.KEY_S, 0x17: e.KEY_T,
        0x18: e.KEY_U, 0x19: e.KEY_V, 0x1A: e.KEY_W, 0x1B: e.KEY_X,
        0x1C: e.KEY_Y, 0x1D: e.KEY_Z,
        0x1E: e.KEY_1, 0x1F: e.KEY_2, 0x20: e.KEY_3, 0x21: e.KEY_4,
        0x22: e.KEY_5, 0x23: e.KEY_6, 0x24: e.KEY_7, 0x25: e.KEY_8,
        0x26: e.KEY_9, 0x27: e.KEY_0,
        0x28: e.KEY_ENTER,  0x29: e.KEY_ESC,   0x2A: e.KEY_BACKSPACE,
        0x2B: e.KEY_TAB,    0x2C: e.KEY_SPACE,  0x2D: e.KEY_MINUS,
        0x2E: e.KEY_EQUAL,  0x2F: e.KEY_LEFTBRACE, 0x30: e.KEY_RIGHTBRACE,
        0x31: e.KEY_BACKSLASH, 0x33: e.KEY_SEMICOLON, 0x34: e.KEY_APOSTROPHE,
        0x35: e.KEY_GRAVE,  0x36: e.KEY_COMMA,  0x37: e.KEY_DOT,
        0x38: e.KEY_SLASH,  0x39: e.KEY_CAPSLOCK,
        0x3A: e.KEY_F1,  0x3B: e.KEY_F2,  0x3C: e.KEY_F3,  0x3D: e.KEY_F4,
        0x3E: e.KEY_F5,  0x3F: e.KEY_F6,  0x40: e.KEY_F7,  0x41: e.KEY_F8,
        0x42: e.KEY_F9,  0x43: e.KEY_F10, 0x44: e.KEY_F11, 0x45: e.KEY_F12,
        0x4F: e.KEY_RIGHT, 0x50: e.KEY_LEFT, 0x51: e.KEY_DOWN, 0x52: e.KEY_UP,
        0x4A: e.KEY_HOME,  0x4D: e.KEY_END,
        0x4B: e.KEY_PAGEUP, 0x4E: e.KEY_PAGEDOWN,
        0x4C: e.KEY_DELETE, 0x49: e.KEY_INSERT,
        0x53: e.KEY_NUMLOCK,
    }

    MOD_BITS = [
        (0x01, e.KEY_LEFTCTRL),
        (0x02, e.KEY_LEFTSHIFT),
        (0x04, e.KEY_LEFTALT),
        (0x08, e.KEY_LEFTMETA),
        (0x10, e.KEY_RIGHTCTRL),
        (0x20, e.KEY_RIGHTSHIFT),
        (0x40, e.KEY_RIGHTALT),
        (0x80, e.KEY_RIGHTMETA),
    ]

    _ALL_KEYS = set(HID_TO_EVDEV.values()) | {v for _, v in MOD_BITS}
    _ALL_KEYS.discard(e.KEY_UNKNOWN)

    _uinput_kb = UInput(
        {e.EV_KEY: list(_ALL_KEYS)},
        name="km-relay-keyboard",
        version=0x1,
    )
    _uinput_mouse = UInput(
        {
            e.EV_KEY: [e.BTN_LEFT, e.BTN_RIGHT, e.BTN_MIDDLE, e.BTN_SIDE, e.BTN_EXTRA],
            e.EV_REL: [e.REL_X, e.REL_Y, e.REL_WHEEL],
        },
        name="km-relay-mouse",
        version=0x1,
    )
    HAS_EVDEV = True
    log.info("uinput devices created")

except ImportError:
    HAS_EVDEV = False
    log.warning("evdev not available — HID injection disabled (install python3-evdev)")


class HIDInjector:
    """Injects keyboard/mouse HID reports into Linux uinput."""

    def __init__(self):
        self._prev_keys = set()
        self._prev_mods = 0

    def inject_keyboard(self, report: bytes):
        if not HAS_EVDEV or len(report) < 8:
            return
        mod   = report[0]
        keys  = set(report[2:8]) - {0}

        # Modifier changes
        for bit, evkey in MOD_BITS:
            was = bool(self._prev_mods & bit)
            now = bool(mod & bit)
            if was != now:
                _uinput_kb.write(e.EV_KEY, evkey, 1 if now else 0)

        # Key releases
        for hid in self._prev_keys - keys:
            evkey = HID_TO_EVDEV.get(hid)
            if evkey:
                _uinput_kb.write(e.EV_KEY, evkey, 0)

        # Key presses
        for hid in keys - self._prev_keys:
            evkey = HID_TO_EVDEV.get(hid)
            if evkey:
                _uinput_kb.write(e.EV_KEY, evkey, 1)

        _uinput_kb.syn()
        self._prev_keys = keys
        self._prev_mods = mod

    def inject_mouse(self, report: bytes):
        if not HAS_EVDEV or len(report) < 4:
            return
        # report: [buttons(1), dx(1), dy(1), wheel(1), ...]
        # buttons byte: bit0=left, bit1=right, bit2=middle, bit3=back, bit4=forward, bit5=side
        buttons = report[0]
        dx    = struct.unpack_from("b", report, 1)[0]
        dy    = struct.unpack_from("b", report, 2)[0]
        wheel = struct.unpack_from("b", report, 3)[0] if len(report) > 3 else 0

        _uinput_mouse.write(e.EV_KEY, e.BTN_LEFT,   1 if buttons & 0x01 else 0)
        _uinput_mouse.write(e.EV_KEY, e.BTN_RIGHT,  1 if buttons & 0x02 else 0)
        _uinput_mouse.write(e.EV_KEY, e.BTN_MIDDLE, 1 if buttons & 0x04 else 0)
        _uinput_mouse.write(e.EV_KEY, e.BTN_SIDE,   1 if buttons & 0x08 else 0)
        _uinput_mouse.write(e.EV_KEY, e.BTN_EXTRA,  1 if buttons & 0x10 else 0)
        if dx:
            _uinput_mouse.write(e.EV_REL, e.REL_X, dx)
        if dy:
            _uinput_mouse.write(e.EV_REL, e.REL_Y, dy)
        if wheel:
            _uinput_mouse.write(e.EV_REL, e.REL_WHEEL, wheel)
        _uinput_mouse.syn()

    def release_all(self):
        """Send all-keys-up (used when switching target)."""
        if not HAS_EVDEV:
            return
        for hid in self._prev_keys:
            evkey = HID_TO_EVDEV.get(hid)
            if evkey:
                _uinput_kb.write(e.EV_KEY, evkey, 0)
        for _, evkey in MOD_BITS:
            _uinput_kb.write(e.EV_KEY, evkey, 0)
        _uinput_kb.syn()
        self._prev_keys = set()
        self._prev_mods = 0


# ── Hotkey detector ───────────────────────────────────────────────────────────
# Mouse button bitmask for the side button (bit5 = 0x20)
MOUSE_BTN_SWITCH = 0x20

class HotkeyDetector:
    """
    Detects single press of mouse side button (0x20) to trigger PC switch.
    Triggers on the rising edge (button down).
    """

    def __init__(self):
        self._btn_was_pressed = False

    def check_mouse(self, report: bytes) -> bool:
        """Returns True on the rising edge of the side button."""
        if not report:
            return False
        pressed = bool(report[0] & MOUSE_BTN_SWITCH)
        triggered = pressed and not self._btn_was_pressed
        self._btn_was_pressed = pressed
        return triggered


# ── Mouse payload converter ───────────────────────────────────────────────────
def _mouse_7to5(payload: bytes) -> bytes:
    """
    Convert 7-byte Central mouse payload to 5-byte Peripheral format.
    Central (HOGP raw):  [buttons, dx_lo, dx_hi, dy_lo, dy_hi, wheel, pan]
    Peripheral expects:  [buttons, x, y, wheel, pan]  (x/y = int8, clamped)
    """
    if len(payload) < 5:
        return payload
    buttons = payload[0]
    dx = struct.unpack_from("<h", payload, 1)[0] if len(payload) >= 3 else 0
    dy = struct.unpack_from("<h", payload, 3)[0] if len(payload) >= 5 else 0
    wheel = struct.unpack_from("b", payload, 5)[0] if len(payload) >= 6 else 0
    pan   = struct.unpack_from("b", payload, 6)[0] if len(payload) >= 7 else 0
    # Clamp to int8 range
    x = max(-128, min(127, dx))
    y = max(-128, min(127, dy))
    return struct.pack("Bbbbb", buttons, x, y, wheel, pan)


# ── Main relay ────────────────────────────────────────────────────────────────
class KMRelay:
    def __init__(self, central_port: str, pc1_port: str | None, pc2_port: str | None):
        self._target = 1  # active target PC (1 or 2)
        self._injector = HIDInjector()
        self._hotkey   = HotkeyDetector()

        log.info("Opening central: %s", central_port)
        self._central = serial.Serial(central_port, baudrate=115200, timeout=0)
        self._central.dtr = True

        self._pc = {}
        if pc1_port:
            log.info("Opening PC1: %s", pc1_port)
            s = serial.Serial(pc1_port, baudrate=115200, timeout=0)
            s.dtr = True
            self._pc[1] = s
        if pc2_port:
            log.info("Opening PC2: %s", pc2_port)
            s = serial.Serial(pc2_port, baudrate=115200, timeout=0)
            s.dtr = True
            self._pc[2] = s

        self._parser = FrameParser(self._on_frame)
        log.info("Relay started, active target=PC%d", self._target)

    def _on_frame(self, ftype: int, payload: bytes):
        if ftype == TYPE_KB:
            self._injector.inject_keyboard(payload)
            self._forward(ftype, payload)

        elif ftype == TYPE_MOUSE:
            if self._hotkey.check_mouse(payload):
                next_target = 2 if self._target == 1 else 1
                self._switch(next_target)
                return  # swallow the trigger click

            self._injector.inject_mouse(payload)
            # Peripheral expects 5-byte mouse payload: [buttons, x, y, wheel, pan]
            # Central sends 7-byte: [buttons, dx_lo, dx_hi, dy_lo, dy_hi, wheel, pan]
            # Convert 16-bit dx/dy → clamped int8
            periph_payload = _mouse_7to5(payload)
            self._forward(ftype, periph_payload)

        else:
            log.debug("Unknown frame type 0x%02x", ftype)

    def _forward(self, ftype: int, payload: bytes):
        port = self._pc.get(self._target)
        if port and port.is_open:
            frame = build_frame(ftype, payload)
            n = port.write(frame)
            log.debug("Forwarded type=0x%02x %d bytes to PC%d", ftype, n, self._target)
        else:
            log.debug("Forward skipped: no port for PC%d", self._target)

    def _switch(self, target: int):
        log.info("Switching to PC%d", target)
        self._injector.release_all()
        self._target = target

        # Notify Central (LED feedback)
        frame = build_frame(TYPE_SWITCH, bytes([target]))
        self._central.write(frame)

        # Send key-up to old and new peripheral
        release = build_frame(TYPE_KB, bytes(8))
        for port in self._pc.values():
            if port and port.is_open:
                port.write(release)

        log.info("Active target: PC%d", self._target)

    def run(self):
        fds = [self._central]
        while True:
            try:
                readable, _, _ = select.select(fds, [], [], 1.0)
                for fd in readable:
                    data = fd.read(256)
                    if data:
                        self._parser.feed(data)
            except KeyboardInterrupt:
                log.info("Interrupted, shutting down")
                break
            except serial.SerialException as ex:
                log.error("Serial error: %s", ex)
                time.sleep(1)

        self._central.close()
        for port in self._pc.values():
            port.close()


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="KM Switch relay")
    parser.add_argument("--central", default="/dev/ttyACM0",
                        help="Central Dongle serial port")
    parser.add_argument("--pc1",    default=None,
                        help="Peripheral Dongle for PC1 (optional)")
    parser.add_argument("--pc2",    default=None,
                        help="Peripheral Dongle for PC2 (optional)")
    parser.add_argument("--debug",  action="store_true",
                        help="Enable debug logging")
    args = parser.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    relay = KMRelay(args.central, args.pc1, args.pc2)
    relay.run()


if __name__ == "__main__":
    main()
