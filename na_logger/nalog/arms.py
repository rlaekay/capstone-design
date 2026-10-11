"""SO-101 leader/follower: port discovery, telemetry loop and leader->follower teleop.

One thread owns both buses. Each cycle:
  1. sync-read leader positions            (if a leader is connected)
  2. sync-write follower Goal_Position     (if teleop is active)
  3. sync-read follower registers 40..70   (goal, position, velocity, load, V, T, current)
The follower sample is stamped with the midpoint of its read request/response on the host clock.
"""
from __future__ import annotations

import glob
import logging
import threading
import time
from dataclasses import dataclass, field

import serial

from . import units
from .clock import now_ns
from .config import Config
from .sts import (ADDR_ACCELERATION, ADDR_TORQUE_ENABLE, ADDR_D_COEF, ADDR_I_COEF, ADDR_LOCK, ADDR_P_COEF,
                  ADDR_PRESENT_VOLTAGE, StsBus)

log = logging.getLogger(__name__)

PORT_GLOBS = ["/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/cu.wchusbserial*",
              "/dev/ttyACM*", "/dev/ttyUSB*"]
FIELDS = ["torque", "goal", "pos", "vel", "load", "volt", "temp", "moving", "cur"]
BUS_ERRORS = (serial.SerialException, OSError)


@dataclass
class ArmCalib:
    """Per-joint range written to servo EEPROM (Min/Max_Position_Limit) by LeRobot calibration."""
    mins: list[int]
    maxs: list[int]

    @property
    def mids(self) -> list[float]:
        return [(a + b) / 2 for a, b in zip(self.mins, self.maxs)]

    @property
    def calibrated(self) -> bool:
        return not all(a == 0 and b == 4095 for a, b in zip(self.mins, self.maxs))

    @property
    def valid(self) -> bool:
        return all(b > a for a, b in zip(self.mins, self.maxs))

    def as_dict(self) -> dict:
        return {"range_min": self.mins, "range_max": self.maxs, "mid": self.mids,
                "calibrated": self.calibrated}


def map_range(raw: list, src: ArmCalib, dst: ArmCalib) -> list:
    """LeRobot RANGE_M100_100 / RANGE_0_100 normalise on the leader, unnormalise on the follower."""
    out = []
    for r, lo, hi, dlo, dhi in zip(raw, src.mins, src.maxs, dst.mins, dst.maxs):
        if r is None or hi <= lo:
            out.append(None)
            continue
        frac = (min(max(r, lo), hi) - lo) / (hi - lo)
        out.append(int(round(dlo + frac * (dhi - dlo))))
    return out


def map_degrees(raw: list, src: ArmCalib, dst: ArmCalib) -> list:
    out = []
    for r, smid, dmid, dlo, dhi in zip(raw, src.mids, dst.mids, dst.mins, dst.maxs):
        out.append(None if r is None else int(round(min(max(r - smid + dmid, dlo), dhi))))
    return out


def build_sample(seq, t0, t1, blocks, leader_raw, leader_t, target_raw, teleop,
                 f_calib: ArmCalib, l_calib: ArmCalib | None) -> dict:
    """blocks: per joint decoded register dict or None. Raw values + physical units."""
    n = len(blocks)
    s = {"t": (t0 + t1) // 2, "t0": t0, "t1": t1, "seq": seq, "teleop": teleop,
         "leader_t": leader_t, "ok": [b is not None for b in blocks]}
    for k in FIELDS:
        s[k] = [b[k] if b is not None else None for b in blocks]
    s["leader"] = list(leader_raw) if leader_raw is not None else [None] * n
    s["target"] = list(target_raw) if target_raw is not None else [None] * n
    mids = f_calib.mids
    s["pos_deg"] = [units.pos_deg(p, m) for p, m in zip(s["pos"], mids)]
    s["goal_deg"] = [units.pos_deg(p, m) for p, m in zip(s["goal"], mids)]
    s["target_deg"] = [units.pos_deg(p, m) for p, m in zip(s["target"], mids)]
    s["vel_dps"] = [units.scaled(v, units.VEL_DPS_PER_STEP) for v in s["vel"]]
    s["load_pct"] = [units.scaled(v, units.LOAD_PCT_PER_UNIT) for v in s["load"]]
    s["cur_ma"] = [units.scaled(v, units.CUR_MA_PER_UNIT) for v in s["cur"]]
    s["volt_v"] = [units.scaled(v, units.VOLT_V_PER_UNIT) for v in s["volt"]]
    return s


@dataclass
class Arm:
    bus: StsBus
    role: str
    port: str
    found: list[int]
    calib: ArmCalib
    voltage: float | None
    servo_config: dict = field(default_factory=dict)

    def info(self, all_ids: list[int]) -> dict:
        return {"connected": True, "port": self.port, "ids": self.found,
                "missing": [i for i in all_ids if i not in self.found],
                "voltage": self.voltage, "calibrated": self.calib.calibrated}


class ArmManager:
    kind = "hw"

    def __init__(self, cfg: Config, hub):
        self.cfg = cfg
        self.hub = hub
        self.ids = list(cfg.servo_ids)
        self.follower: Arm | None = None
        self.leader: Arm | None = None
        self.teleop = "off"          # off | active | hold
        self.message = "SO-101 포트 검색 중"
        self.port_notes: dict[str, str] = {}
        self._bad: dict[str, float] = {}
        self._pending: str | None = None
        self._cmd_lock = threading.Lock()
        self._manual_stop = False
        self._soft_until = 0.0
        self._prev_goal: list | None = None
        self._script = None              # (goal_fn(t_s)->deg list, duration_s, t_start_monotonic)
        self._script_req = None
        self._gain_req = None             # ({id: {"p","d","i"}}, persist)
        self.gain_result: dict | None = None
        self.script_t0_ns: int | None = None
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._scan_loop, name="arm-scan", daemon=True),
                         threading.Thread(target=self._loop, name="arm-loop", daemon=True)]

    # ---------------------------------------------------------------- public
    def start(self):
        for t in self._threads:
            t.start()

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)
        for a in (self.follower, self.leader):
            if a:
                a.bus.close()

    def command(self, cmd: str) -> None:
        """'start' | 'hold' | 'torque_off' — executed by the loop thread."""
        with self._cmd_lock:
            self._pending = cmd

    def start_script(self, goal_fn, duration_s: float) -> None:
        """Drive the follower along goal_fn(t) [logger degrees, 6 joints] for duration_s.
        Starts from the present pose with torque on; 'hold' aborts at the current goal."""
        with self._cmd_lock:
            self._script_req = (goal_fn, float(duration_s))
            self._pending = "script"

    def set_gains(self, gains: dict, persist: bool = False) -> None:
        """Write follower P/D/I coefficients ({id: {"p": .., "d": .., "i": ..}}) from the loop thread.
        persist=False keeps the servo EEPROM lock closed: the values act now and are gone after a
        power cycle. Result lands in gain_result."""
        with self._cmd_lock:
            self.gain_result = None
            self._gain_req = ({int(k): v for k, v in gains.items()}, bool(persist))
            self._pending = "gains"

    def status(self) -> dict:
        f, l = self.follower, self.leader
        in_use = {a.port for a in (f, l) if a}
        return {
            "mode": self.kind,
            "follower": f.info(self.ids) if f else {"connected": False},
            "leader": l.info(self.ids) if l else {"connected": False,
                                                  "disabled": self.cfg.leader_port == "none"},
            "teleop": self.teleop,
            "soft_start": self.teleop == "active" and time.monotonic() < self._soft_until,
            "script_t0_ns": self.script_t0_ns if self.teleop == "script" else None,
            "script_progress": (min(1.0, (time.monotonic() - self._script[2]) / self._script[1])
                                if self.teleop == "script" and self._script else None),
            "message": self.message,
            "port_notes": {p: n for p, n in self.port_notes.items() if p not in in_use},
        }

    def calib_info(self) -> dict:
        out = {}
        for role, a in (("follower", self.follower), ("leader", self.leader)):
            if a:
                out[role] = {"port": a.port, "ids": a.found, "voltage": a.voltage,
                             **a.calib.as_dict(), "servo_config": a.servo_config}
        return out

    # ------------------------------------------------------------- discovery
    def _candidates(self, role: str) -> list[str]:
        explicit = self.cfg.follower_port if role == "follower" else self.cfg.leader_port
        if explicit == "none":
            return []
        if explicit not in ("", "auto"):
            return [explicit]
        return sorted({p for g in PORT_GLOBS for p in glob.glob(g)})

    def _scan_loop(self):
        while not self._stop.is_set():
            need_f = self.follower is None
            need_l = self.leader is None and self.cfg.leader_port != "none"
            if need_f or need_l:
                in_use = {a.port for a in (self.follower, self.leader) if a}
                ports: set[str] = set()
                if need_f:
                    ports |= set(self._candidates("follower"))
                if need_l:
                    ports |= set(self._candidates("leader"))
                if not ports and need_f:
                    self.message = "SO-101 USB 포트 없음 (보드 USB 연결 확인)"
                for p in sorted(ports - in_use):
                    if self._stop.is_set():
                        break
                    if time.monotonic() - self._bad.get(p, -1e9) < 5.0:
                        continue
                    self._probe(p)
            self._stop.wait(1.0)

    def _probe(self, port: str):
        bus = StsBus(port, self.cfg.baudrate, self.ids)
        try:
            bus.open()
        except Exception as e:
            self.port_notes[port] = f"포트 열기 실패: {e}"
            self._bad[port] = time.monotonic()
            return
        try:
            found = bus.ping_all()
            if not found:
                self.port_notes[port] = "서보 응답 없음 (서보 전원 확인)"
                raise _Skip()
            v = bus.read(found[0], ADDR_PRESENT_VOLTAGE, 1)
            volt = v[0] * 0.1 if v else None
            vtxt = f"{volt:.1f} V" if volt is not None else "전압 ?"
            if port == self.cfg.follower_port:
                role = "follower"
            elif port == self.cfg.leader_port:
                role = "leader"
            else:
                role = "follower" if (volt or 0) >= self.cfg.follower_min_voltage else "leader"
            if getattr(self, role) is not None:
                self.port_notes[port] = (f"{vtxt} → {role}로 판별됐지만 이미 연결됨. "
                                         "config에서 follower_port/leader_port 지정 필요")
                raise _Skip()
            cfgs = {sid: bus.read_config(sid) for sid in found}
            by_id = {sid: c for sid, c in cfgs.items() if c}
            mins = [by_id[i]["min_limit"] if i in by_id else 0 for i in self.ids]
            maxs = [by_id[i]["max_limit"] if i in by_id else 4095 for i in self.ids]
            arm = Arm(bus, role, port, found, ArmCalib(mins, maxs), volt,
                      {str(k): v for k, v in by_id.items()})
            setattr(self, role, arm)
            self.port_notes.pop(port, None)
            missing = [i for i in self.ids if i not in found]
            self.message = (f"{role} 연결: {port} ({vtxt})"
                            + (f", 응답 없는 ID {missing}" if missing else ""))
            log.info(self.message)
        except _Skip:
            bus.close()
            self._bad[port] = time.monotonic()
        except Exception as e:
            bus.close()
            self.port_notes[port] = f"검사 실패: {e}"
            self._bad[port] = time.monotonic()

    def _drop(self, role: str, reason: str):
        arm = getattr(self, role)
        setattr(self, role, None)
        if arm:
            arm.bus.close()
        self._manual_stop = False
        if role == "follower":
            self.teleop = "off"
        elif self.teleop == "active":
            self.teleop = "hold"
        self.message = f"{role} 연결 끊김: {reason}"
        log.warning(self.message)

    # ------------------------------------------------------------------ loop
    def _to_list(self, by_id: dict) -> list:
        return [by_id.get(i) for i in self.ids]

    def _map(self, leader_raw, lc: ArmCalib, fc: ArmCalib):
        fn = map_degrees if self.cfg.teleop_mapping == "degrees" else map_range
        return fn(leader_raw, lc, fc)

    def _next_goal(self, target: list, period_s: float) -> list:
        prev = self._prev_goal or target
        goal = [p if t is None else t for t, p in zip(target, prev)]
        if time.monotonic() < self._soft_until:
            step = max(1, int(self.cfg.soft_start_max_dps * period_s / units.POS_DEG_PER_STEP))
            goal = [g if p is None else p + max(-step, min(step, g - p)) for g, p in zip(goal, prev)]
            close = all(t is None or g is None or abs(t - g) * units.POS_DEG_PER_STEP < 2.0
                        for t, g in zip(target, goal))
            if close:
                self._soft_until = 0.0
        self._prev_goal = goal
        return goal

    def _enable_from_present(self, f: Arm) -> bool:
        """Goal := present position, then torque on (no jump)."""
        blocks, _, _ = f.bus.sync_read_block()
        present = [b["pos"] if b else None for b in self._to_list(blocks)]
        if any(p is None for i, p in zip(self.ids, present) if i in f.found):
            self.message = "팔로워 현재 위치를 읽지 못함"
            return False
        self._prev_goal = present
        f.bus.sync_write_goal(dict(zip(self.ids, present)))
        if self.cfg.follower_acceleration is not None:
            f.bus.write_byte_all(ADDR_ACCELERATION, self.cfg.follower_acceleration)
        if not f.bus.write_torque(True):
            self.message = "경고: 팔로워 토크 켜기 확인 실패"
        return True

    def _handle_command(self, f: Arm | None, l: Arm | None):
        with self._cmd_lock:
            cmd, self._pending = self._pending, None
            script_req, self._script_req = self._script_req, None
        if cmd == "gains":
            req = getattr(self, "_gain_req", None)
            self._gain_req = None
            if not f or not req:
                self.gain_result = {"ok": False, "error": "팔로워 없음"}
                return
            gains, persist = req
            out, ok = {}, True
            for sid, g in gains.items():
                if sid not in f.found:
                    ok = False
                    continue
                lock_ok = f.bus.write_byte(sid, ADDR_LOCK, 0 if persist else 1)
                for key, addr in (("p", ADDR_P_COEF), ("d", ADDR_D_COEF), ("i", ADDR_I_COEF)):
                    if key in g:
                        ok &= f.bus.write_byte(sid, addr, max(0, min(254, int(g[key]))))
                if persist:
                    f.bus.write_byte(sid, ADDR_LOCK, 1)
                c = f.bus.read_config(sid)
                if c:
                    f.servo_config[str(sid)] = c
                    out[str(sid)] = {"p": c["p_coef"], "d": c["d_coef"], "i": c["i_coef"]}
                ok &= lock_ok
            self.gain_result = {"ok": ok, "persist": persist, "gains": out}
            self.message = f"게인 변경 {'(EEPROM 저장)' if persist else '(임시)'}: {out}"
            return
        if cmd == "script":
            if not f:
                self.message = "스크립트 시작 불가: 팔로워 없음"
                return
            if self._enable_from_present(f):
                fn, dur = script_req
                self._script = (fn, dur, time.monotonic())
                self.script_t0_ns = now_ns()
                self.teleop = "script"
                self._manual_stop = True          # do not auto-restart teleop afterwards
                self.message = "자동 동작 실행 중"
            return
        if (cmd is None and self.cfg.teleop_autostart and self.teleop == "off"
                and f and l and not self._manual_stop):
            cmd = "start"
        if cmd == "start":
            if not (f and l):
                self.message = "텔레옵 시작 불가: 리더와 팔로워가 모두 연결돼야 함"
                return
            if not (f.calib.valid and l.calib.valid):
                self.message = "텔레옵 시작 불가: 캘리브레이션 범위(Min/Max limit)가 비정상"
                return
            l.bus.write_torque(False)
            if not self._enable_from_present(f):
                return
            self._soft_until = time.monotonic() + self.cfg.soft_start_timeout_s
            self.teleop = "active"
            self._manual_stop = False
            self.message = "텔레옵 시작 (소프트 스타트)"
        elif cmd == "hold" and self.teleop in ("active", "script"):
            was = self.teleop
            self.teleop = "hold"
            self._manual_stop = True
            self.message = "자동 동작 중단: 현재 자세 유지" if was == "script" else "텔레옵 정지: 팔로워 자세 유지 중"
        elif cmd == "release_gripper" and f:
            # gripper (last id) limp so a payload can be placed; other joints keep holding
            gid = self.ids[-1]
            ok = f.bus.write_byte(gid, ADDR_TORQUE_ENABLE, 0)
            self.message = "그리퍼 토크 풀림: 물건을 쥐어주세요" if ok else "그리퍼 토크 끄기 확인 실패"
        elif cmd == "torque_off" and f:
            f.bus.write_torque(False)
            self.teleop = "off"
            self._manual_stop = True
            self.message = "팔로워 토크 꺼짐"

    def _loop(self):
        period = 1.0 / self.cfg.arm_rate_hz
        next_t = time.monotonic()
        seq = 0
        miss_f = miss_l = 0
        fail_limit = max(10, int(0.5 / period))
        while not self._stop.is_set():
            f, l = self.follower, self.leader
            try:
                self._handle_command(f, l)
            except BUS_ERRORS as e:
                self.message = f"명령 실패: {e}"
            if f is None:
                self.teleop = "off"
                time.sleep(0.05)
                next_t = time.monotonic()
                continue

            leader_raw = leader_t = target = None
            if l is not None:
                try:
                    pos, t0, t1 = l.bus.sync_read_pos()
                    lst = self._to_list(pos)
                    if any(v is not None for v in lst):
                        leader_raw, leader_t, miss_l = lst, (t0 + t1) // 2, 0
                    else:
                        miss_l += 1
                except BUS_ERRORS as e:
                    self._drop("leader", str(e))
                    l = None
                if l is not None and miss_l > fail_limit:
                    self._drop("leader", "서보 응답 없음")
                    l = None
            if l is not None and leader_raw is not None:
                target = self._map(leader_raw, l.calib, f.calib)

            if self.teleop == "script" and self._script:
                fn, dur, t_start = self._script
                el = time.monotonic() - t_start
                if el > dur:
                    self.teleop = "hold"
                    self.message = "자동 동작 완료: 자세 유지"
                else:
                    mids = f.calib.mids
                    deg = fn(el)
                    raw = [int(round(d / units.POS_DEG_PER_STEP + m)) for d, m in zip(deg, mids)]
                    # never move faster than 300 deg/s whatever the script asks
                    step = int(300.0 * period / units.POS_DEG_PER_STEP)
                    prev = self._prev_goal or raw
                    goal = [p + max(-step, min(step, r - p)) if p is not None else r
                            for r, p in zip(raw, prev)]
                    self._prev_goal = goal
                    try:
                        f.bus.sync_write_goal(dict(zip(self.ids, goal)))
                    except BUS_ERRORS as e:
                        self._drop("follower", str(e))
                        continue
            elif self.teleop == "active":
                if target is None:
                    if l is None:
                        self.teleop = "hold"
                        self.message = "리더 신호 끊김 → 팔로워 자세 유지"
                else:
                    try:
                        f.bus.sync_write_goal(dict(zip(self.ids, self._next_goal(target, period))))
                    except BUS_ERRORS as e:
                        self._drop("follower", str(e))
                        continue

            try:
                blocks, t0, t1 = f.bus.sync_read_block()
            except BUS_ERRORS as e:
                self._drop("follower", str(e))
                continue
            dec = self._to_list(blocks)
            if any(d is not None for d in dec):
                miss_f = 0
                self.hub.push_arm(build_sample(seq, t0, t1, dec, leader_raw, leader_t, target,
                                               self.teleop, f.calib, l.calib if l else None))
                seq += 1
            else:
                miss_f += 1
                if miss_f > fail_limit:
                    self._drop("follower", "서보 응답 없음 (전원 확인)")
                    continue

            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -period:
                next_t = time.monotonic()


class _Skip(Exception):
    pass
