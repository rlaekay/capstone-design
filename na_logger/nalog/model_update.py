"""Edit the user's MJCF: remove the placeholder mount/deck blocks and mount the Crazyflie where
alignid says it is.

  * removes body crazyflie_bracket (black mount_block) and the blue battery-holder / expansion-deck
    boxes (visual + collision) plus contact entries that reference them — the drone's battery-pack
    panel is taped straight onto the fixed jaw, there is no bracket or deck.
  * crazyflie body + crazyflie_mount site: pose from the calibration (IMU frame = Crazyflie frame)
  * imu site: moved from the board centre to the BMI088 chip (+4.6 mm y, +0.5 mm z)

python -m nalog.model_update [--calib data/calib/current.json] [--export-dir ~/Downloads]
"""
from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from .kin import BMI088_IN_CF, MODEL_DIR, mat_to_quat

REMOVE_BODIES = {"crazyflie_bracket"}
REMOVE_GEOMS = {"mount_block", "cf_visual_battery_holder", "cf_visual_expansion_deck",
                "cf_collision_battery_holder", "cf_collision_expansion_deck"}


def _fmt(v) -> str:
    return " ".join(f"{x:.6g}" for x in v)


def edit_model(src: Path, dst: Path, calib: dict | None = None) -> dict:
    tree = ET.parse(src)
    root = tree.getroot()
    parent = {c: p for p in root.iter() for c in p}
    removed = []
    for el in list(root.iter()):
        name = el.get("name")
        if (el.tag == "body" and name in REMOVE_BODIES) or (el.tag == "geom" and name in REMOVE_GEOMS):
            parent[el].remove(el)
            removed.append(name)
    gone_bodies, gone_geoms = REMOVE_BODIES, REMOVE_GEOMS
    for contact in root.iter("contact"):
        for el in list(contact):
            refs = {el.get("body1"), el.get("body2"), el.get("geom1"), el.get("geom2")}
            if refs & (gone_bodies | gone_geoms):
                contact.remove(el)
    info = {"removed": sorted(set(removed))}
    if calib:
        R = np.asarray(calib["R_gi"], float)
        p = np.asarray(calib["p_gi"], float)
        body_pos = p - R @ BMI088_IN_CF
        quat = mat_to_quat(R)
        for el in root.iter():
            if el.tag == "body" and el.get("name") == "crazyflie":
                el.set("pos", _fmt(body_pos))
                el.set("quat", _fmt(quat))
            elif el.tag == "site" and el.get("name") == "crazyflie_mount":
                el.set("pos", _fmt(body_pos))
                el.set("quat", _fmt(quat))
            elif el.tag == "site" and el.get("name") == "imu":
                el.set("pos", _fmt(BMI088_IN_CF))
        info.update(calib=calib.get("id"), body_pos=body_pos.tolist(), quat=quat.tolist())
    tree.write(dst, encoding="utf-8", xml_declaration=True)
    return info


def main():
    ap = argparse.ArgumentParser(prog="nalog.model_update")
    ap.add_argument("--calib", help="alignid result json (omit: only remove the blocks)")
    ap.add_argument("--export-dir", help="also write *_aligned.xml copies next to the originals "
                                         "(original mesh paths) into this folder")
    ap.add_argument("--originals", default=str(Path.home() / "Downloads"))
    a = ap.parse_args()
    calib = json.loads(Path(a.calib).read_text()) if a.calib else None
    for name in ("scene.xml", "torque_scene.xml"):
        path = MODEL_DIR / name
        info = edit_model(path, path, calib)
        mujoco.MjModel.from_xml_path(str(path))            # must still load
        print(f"{path.name}: {info}")
        if a.export_dir:
            src = Path(a.originals) / name
            if src.exists():
                out = Path(a.export_dir).expanduser() / name.replace(".xml", "_aligned.xml")
                edit_model(src, out, calib)
                print(f"  -> {out}")


if __name__ == "__main__":
    main()
