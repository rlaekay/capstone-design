"""Crazyflie IMU source over Crazyradio, with automatic reconnect.

Log blocks (firmware minimum period 10 ms):
  imu : acc.x/y/z [g], gyro.x/y/z [deg/s]
  att : stateEstimate.roll/pitch/yaw [deg]
  bat : pm.vbat [V] (slow)
Each packet is stamped with its host receive time the moment the callback fires; the
Crazyflie's own timestamp is kept alongside so the clock mapping can be refit offline.
"""
from __future__ import annotations

import logging
import threading
from pathlib import Path

from .clock import now_ns
from .config import Config

log = logging.getLogger(__name__)

HINTS = {
    "Cannot find a Crazyradio Dongle": "Crazyradio 동글 없음 (또는 cfclient가 동글 사용 중 — cfclient 종료)",
    "Too many packets lost": "라디오 연결 끊김 (드론 전원·배터리·거리 확인)",
    "No driver found": "URI 형식 오류",
}


def _hint(msg: str) -> str:
    for k, v in HINTS.items():
        if k in msg:
            return v
    return msg


class CrazyflieSource:
    kind = "hw"

    def __init__(self, cfg: Config, hub):
        self.cfg = cfg
        self.hub = hub
        self.state = "idle"          # idle | connecting | connected | disconnected
        self.message = ""
        self.battery_v: float | None = None
        self.link_quality: float | None = None
        self.has_att = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="crazyflie", daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=3)

    def status(self) -> dict:
        return {"mode": self.kind, "state": self.state, "uri": self.cfg.cf_uri,
                "message": self.message, "battery_v": self.battery_v,
                "link_quality": self.link_quality, "has_att": self.has_att}

    # ------------------------------------------------------------------
    def _run(self):
        import cflib.crtp
        from cflib.crazyflie import Crazyflie

        cflib.crtp.init_drivers()
        cache = Path(__file__).resolve().parent.parent / ".cfcache"
        cache.mkdir(exist_ok=True)
        while not self._stop.is_set():
            self.state = "connecting"
            done = threading.Event()
            cf = Crazyflie(rw_cache=str(cache))

            def on_connected(uri, cf=cf, done=done):
                try:
                    self._setup_logs(cf)
                    self.state = "connected"
                    self.message = f"연결됨 {uri}"
                    log.info("Crazyflie connected: %s", uri)
                except Exception as e:  # noqa: BLE001
                    self.message = f"로그 설정 실패: {e}"
                    done.set()

            def on_failed(uri, msg, done=done):
                self.message = _hint(str(msg))
                done.set()

            cf.connected.add_callback(on_connected)
            cf.connection_failed.add_callback(on_failed)
            cf.connection_lost.add_callback(on_failed)
            cf.disconnected.add_callback(lambda uri, done=done: done.set())
            try:
                cf.link_statistics.link_quality_updated.add_callback(self._on_link_quality)
            except AttributeError:
                pass
            try:
                cf.open_link(self.cfg.cf_uri)
            except Exception as e:  # noqa: BLE001
                self.message = _hint(str(e))
                done.set()
            while not done.wait(0.2):
                if self._stop.is_set():
                    break
            try:
                cf.close_link()
            except Exception:  # noqa: BLE001
                pass
            self.state = "disconnected"
            self.link_quality = None
            self._stop.wait(2.0)

    def _setup_logs(self, cf):
        from cflib.crazyflie.log import LogConfig

        period = self.cfg.imu_period_ms
        imu = LogConfig(name="imu", period_in_ms=period)
        for v in ("acc.x", "acc.y", "acc.z", "gyro.x", "gyro.y", "gyro.z"):
            imu.add_variable(v, "float")
        cf.log.add_config(imu)
        imu.data_received_cb.add_callback(self._on_imu)
        imu.start()

        self.has_att = False
        try:
            att = LogConfig(name="att", period_in_ms=period)
            for v in ("stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw"):
                att.add_variable(v, "float")
            cf.log.add_config(att)
            att.data_received_cb.add_callback(self._on_att)
            att.start()
            self.has_att = True
        except (KeyError, AttributeError) as e:
            log.warning("attitude log unavailable: %s", e)

        bat = LogConfig(name="bat", period_in_ms=500)
        bat.add_variable("pm.vbat", "float")
        cf.log.add_config(bat)
        bat.data_received_cb.add_callback(self._on_bat)
        bat.start()

    def _on_imu(self, ts, data, _lc):
        rx = now_ns()
        self.hub.push_imu({"rx": rx, "cf_ms": ts,
                           "acc": [data["acc.x"], data["acc.y"], data["acc.z"]],
                           "gyro": [data["gyro.x"], data["gyro.y"], data["gyro.z"]]})

    def _on_att(self, ts, data, _lc):
        rx = now_ns()
        self.hub.push_att({"rx": rx, "cf_ms": ts,
                           "rpy": [data["stateEstimate.roll"], data["stateEstimate.pitch"],
                                   data["stateEstimate.yaw"]]})

    def _on_bat(self, _ts, data, _lc):
        self.battery_v = data["pm.vbat"]

    def _on_link_quality(self, q):
        self.link_quality = q
