"""python -m nalog [--sim] [--teleop] [--config config.toml] [--port 8080]"""
from __future__ import annotations

import argparse
import logging
import os
import subprocess

from aiohttp import web

from .config import Config
from .recorder import Hub
from .server import make_app


def main():
    ap = argparse.ArgumentParser(prog="nalog", description=__doc__)
    ap.add_argument("--config", help="TOML file overriding defaults in nalog/config.py")
    ap.add_argument("--sim", action="store_true", help="simulated SO-101 + Crazyflie")
    ap.add_argument("--teleop", action="store_true", help="start teleop when both arms are found")
    ap.add_argument("--port", type=int)
    ap.add_argument("--data-dir")
    ap.add_argument("--no-cf", action="store_true", help="run without the Crazyflie")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    for noisy in ("cflib", "aiohttp.access"):
        logging.getLogger(noisy).setLevel(logging.WARNING)
    # cflib logs a full traceback on every failed reconnect; the GUI/status already shows why
    logging.getLogger("cflib.crazyflie").setLevel(logging.CRITICAL)

    cfg = Config.load(args.config, port=args.port, data_dir=args.data_dir,
                      teleop_autostart=True if args.teleop else None,
                      cf_enabled=False if args.no_cf else None)
    hub = Hub(cfg)
    if args.sim:
        from .sim import SimArmSource, SimImuSource
        arm = SimArmSource(cfg, hub)
        imu = SimImuSource(cfg, hub, arm)
    else:
        from .arms import ArmManager
        from .imu import CrazyflieSource
        arm = ArmManager(cfg, hub)
        imu = CrazyflieSource(cfg, hub) if cfg.cf_enabled else None
    hub.attach(arm, imu)

    # keep the Mac awake while logging (display may sleep; the process keeps running)
    caff = subprocess.Popen(["caffeinate", "-is", "-w", str(os.getpid())])

    arm.start()
    if imu:
        imu.start()
    app = make_app(hub)

    async def cleanup(_app):
        hub.shutdown()
        arm.stop()
        if imu:
            imu.stop()

    app.on_cleanup.append(cleanup)
    print("\n  NA logger" + ("  [SIMULATION]" if args.sim else ""))
    print(f"  이 Mac:        http://localhost:{cfg.port}")
    for u in app["urls"]:
        print(f"  같은 Wi-Fi:    {u}")
    print(f"  데이터 폴더:   {cfg.data_dir}\n")
    try:
        web.run_app(app, host=cfg.host, port=cfg.port, print=None)
    finally:
        caff.terminate()


if __name__ == "__main__":
    main()
