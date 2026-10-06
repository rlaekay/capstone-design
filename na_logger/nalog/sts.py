"""Feetech STS3215 bus access (the servos in SO-101).

Protocol: FF FF | ID | LEN | INSTR | PARAMS | CHK, little-endian words, 1 Mbps half-duplex.
Signed registers use sign-magnitude: bit 15 for position/velocity/current, bit 10 for load
(same decoding as SCServo_Linux SMS_STS::ReadPos/ReadSpeed/ReadLoad/ReadCurrent).
"""
from __future__ import annotations

import time

import scservo_sdk as scs

from .clock import now_ns

ADDR_RETURN_DELAY = 7
ADDR_MIN_LIMIT = 9
ADDR_MAX_LIMIT = 11
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_VOLTAGE = 62

# Two sync reads per cycle: state/telemetry (56..70) then command (40..43).
# A single 40..70 read of six servos corrupted ID 4's reply in ~10 % of cycles on the real
# arm (bad checksum); the two shorter reads were error-free (0 / 1500) and cost ~0.2 ms more.
STATE_START, STATE_LEN = 56, 15
CMD_START, CMD_LEN = 40, 4
# EEPROM 0..39: read once per connect and saved with each trial for reproducibility.
CONFIG_LEN = 40


def sm_decode(value: int, sign_bit: int) -> int:
    if value & (1 << sign_bit):
        return -(value & ~(1 << sign_bit))
    return value


def _word(b: bytes, off: int) -> int:
    return b[off] | (b[off + 1] << 8)


def decode_state(b: bytes) -> dict:
    """Registers 56..70: position, velocity, load, voltage, temperature, status, current."""
    o = STATE_START
    return {
        "pos": sm_decode(_word(b, 56 - o), 15),
        "vel": sm_decode(_word(b, 58 - o), 15),
        "load": sm_decode(_word(b, 60 - o), 10),
        "volt": b[62 - o],
        "temp": b[63 - o],
        "status": b[65 - o],
        "moving": b[66 - o],
        "cur": sm_decode(_word(b, 69 - o), 15),
    }


def decode_cmd(b: bytes) -> dict:
    """Registers 40..43: torque enable, acceleration, goal position."""
    o = CMD_START
    return {"torque": b[40 - o], "accel": b[41 - o], "goal": sm_decode(_word(b, 42 - o), 15)}


def merge_reads(state: bytes | None, cmd: bytes | None) -> dict | None:
    """One servo's sample; None if its state read failed (command fields may be None)."""
    if state is None:
        return None
    d = decode_state(state)
    d.update(decode_cmd(cmd) if cmd is not None else {"torque": None, "accel": None, "goal": None})
    return d


def _packet(sid: int, instr: int, params: list[int]) -> bytes:
    body = [sid, len(params) + 2, instr, *params]
    return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def parse_status_packets(buf: bytes, length: int) -> dict[int, bytes]:
    """All well-formed status packets in buf with a payload of `length` bytes, by servo ID.
    A corrupt or missing reply only loses that servo (the Feetech SDK drops every servo after it)."""
    out: dict[int, bytes] = {}
    i, n = 0, len(buf)
    while i + 6 <= n:
        if buf[i] == 0xFF and buf[i + 1] == 0xFF and buf[i + 2] != 0xFF:
            sid, ln = buf[i + 2], buf[i + 3]
            end = i + 4 + ln
            if ln == length + 2 and end <= n:
                body = buf[i + 2:end - 1]
                if (~sum(body)) & 0xFF == buf[end - 1]:   # error flags don't invalidate data
                    out[sid] = bytes(buf[i + 5:end - 1])
                    i = end
                    continue
        i += 1
    return out


def decode_config(b: bytes) -> dict:
    """Decode the EEPROM settings that shape actuator behaviour (registers 0..39)."""
    return {
        "firmware": f"{b[0]}.{b[1]}",
        "model": _word(b, 3),
        "return_delay": b[7],
        "min_limit": _word(b, 9),
        "max_limit": _word(b, 11),
        "max_torque": _word(b, 16),
        "p_coef": b[21],
        "d_coef": b[22],
        "i_coef": b[23],
        "protection_current": _word(b, 28),
        "homing_offset": sm_decode(_word(b, 31), 11),
        "operating_mode": b[33],
        "overload_torque": b[36],
    }


class StsBus:
    """One serial port with a chain of STS servos. Not thread-safe: one owner thread."""

    def __init__(self, port: str, baudrate: int, ids: list[int]):
        self.port = port
        self.baudrate = baudrate
        self.ids = list(ids)
        self._ph = scs.PortHandler(port)
        self._pk = scs.PacketHandler(0)  # 0 = STS/SMS little-endian

    def open(self) -> None:
        if not self._ph.openPort():
            raise OSError(f"cannot open {self.port}")
        if not self._ph.setBaudRate(self.baudrate):
            raise OSError(f"cannot set baudrate on {self.port}")

    def close(self) -> None:
        try:
            self._ph.closePort()
        except Exception:
            pass

    def ping_all(self) -> list[int]:
        found = []
        for sid in self.ids:
            for _ in range(2):
                _, res, _ = self._pk.ping(self._ph, sid)
                if res == scs.COMM_SUCCESS:
                    found.append(sid)
                    break
        self.set_ids(found)
        return found

    def set_ids(self, ids: list[int]) -> None:
        self.ids = list(ids)

    def read(self, sid: int, addr: int, length: int) -> bytes | None:
        data, res, _ = self._pk.readTxRx(self._ph, sid, addr, length)
        if res == scs.COMM_SUCCESS and len(data) == length:
            return bytes(data)
        return None

    def read_config(self, sid: int) -> dict | None:
        b = self.read(sid, 0, CONFIG_LEN)
        return decode_config(b) if b else None

    def sync_read(self, addr: int, length: int, timeout_s: float = 0.006):
        """Sync read `length` bytes at `addr` from every servo. Returns ({id: bytes|None},
        t_request_ns, t_last_byte_ns)."""
        ser = self._ph.ser
        expect = len(self.ids) * (6 + length)
        ser.reset_input_buffer()
        t0 = now_ns()
        ser.write(_packet(0xFE, 0x82, [addr, length, *self.ids]))
        buf = bytearray()
        deadline = time.perf_counter() + timeout_s + expect * 10e-6
        while len(buf) < expect and time.perf_counter() < deadline:
            chunk = ser.read(expect - len(buf))
            if chunk:
                buf += chunk
            else:
                time.sleep(0.0002)
        t1 = now_ns()
        got = parse_status_packets(buf, length)
        return {sid: got.get(sid) for sid in self.ids}, t0, t1

    def sync_read_block(self):
        """{id: decoded sample or None}, t_request_ns, t_response_ns (of the state read)."""
        state, t0, t1 = self.sync_read(STATE_START, STATE_LEN)
        cmd, _, _ = self.sync_read(CMD_START, CMD_LEN)
        return {sid: merge_reads(state[sid], cmd[sid]) for sid in self.ids}, t0, t1

    def sync_read_pos(self):
        raw, t0, t1 = self.sync_read(ADDR_PRESENT_POSITION, 2)
        return {sid: (sm_decode(_word(b, 0), 15) if b else None) for sid, b in raw.items()}, t0, t1

    def sync_write_goal(self, goals: dict[int, int | None]) -> None:
        g = scs.GroupSyncWrite(self._ph, self._pk, ADDR_GOAL_POSITION, 2)
        for sid, v in goals.items():
            if v is None or sid not in self.ids:
                continue
            v = max(0, min(4095, int(v)))
            g.addParam(sid, [v & 0xFF, (v >> 8) & 0xFF])
        g.txPacket()

    def write_byte_all(self, addr: int, value: int) -> bool:
        """Write one byte to every servo and confirm by reading it back."""
        ok = True
        for sid in self.ids:
            self._pk.write1ByteTxRx(self._ph, sid, addr, value)
            back = self.read(sid, addr, 1)
            ok &= back is not None and back[0] == value
        return ok

    def write_torque(self, on: bool) -> bool:
        return self.write_byte_all(ADDR_TORQUE_ENABLE, 1 if on else 0)
