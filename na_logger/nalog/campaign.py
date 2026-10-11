"""Payload data collection, one mass at a time (run through ./log.sh).

Per mass:  grams -> gripper released -> payload placed, Enter -> 30 trajectories
(paper_timewarp_10, excitation_v2_set_1, excitation_v2_set_2) -> gripper released again.
Each trajectory is a logger trial, copied to data/loaded/payload_<g>g/<trial>_<set>_<motion>/.
Ctrl+C during a run stops the arm where it is (gripper keeps holding).
"""
from __future__ import annotations

import json
import shutil
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

URL = "http://localhost:8080"
ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
SETS = ["paper_timewarp_10", "excitation_v2_set_1", "excitation_v2_set_2"]
# no extra squeeze: the payload is taped to the gripper, the gripper just holds where it is
# (squeezing past contact stalls the gripper servo; it reached 70 C on 2026-10-11)


def call(path: str, body: dict | None = None, timeout: float = 10.0) -> dict:
    req = urllib.request.Request(URL + path, method="GET" if body is None else "POST",
                                 data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode() or "{}")


def ask(prompt: str, default: str = "") -> str:
    try:
        v = input(prompt).strip()
    except EOFError:
        v = "q"
    return v or default


def wait_devices() -> None:
    t0 = time.time()
    last = ""
    while True:
        try:
            st = call("/api/status")
            f, cf = st["arm"].get("follower", {}), st["cf"]
            arm_ok = f.get("connected") and st["arm"].get("rate_hz", 0) > 50
            cf_ok = cf.get("state") == "connected" and cf.get("rate_hz", 0) > 50
            line = (f"  팔로워 {'✓' if arm_ok else '…'}   드론 {'✓' if cf_ok else '…'} "
                    f"(배터리 {cf.get('battery_v') or 0:.2f} V)")
            if line != last:
                print(line, flush=True)
                last = line
            if arm_ok and cf_ok:
                bat = cf.get("battery_v") or 0
                if bat and bat < 3.5:
                    print(f"  ⚠ 드론 배터리 {bat:.2f} V — 중간에 끊길 수 있어요")
                return
        except (urllib.error.URLError, OSError, KeyError):
            pass
        if time.time() - t0 > 60:
            print("  ⚠ 60초 안에 팔/드론이 연결되지 않았어요. 전원과 cfclient 종료를 확인한 뒤 다시 실행하세요.")
            sys.exit(1)
        time.sleep(1)


def mass_folder(g: float) -> Path:
    return DATA / "loaded" / f"payload_{g:g}g"


def done_motions(g: float) -> set[str]:
    out = set()
    for meta in mass_folder(g).glob("*/meta.json"):
        try:
            m = json.loads(meta.read_text())
            out.add(f"{m['set']}/{m['motion']}")
        except (OSError, json.JSONDecodeError, KeyError):
            pass
    return out


def joints() -> dict:
    return call("/api/status")["joints"]


def label_trial(tid: str, suffix: str) -> None:
    try:
        meta = json.loads((DATA / tid / "meta.json").read_text())
        call(f"/api/trials/{tid}/label", {"label": (meta.get("label", "") + " " + suffix).strip()})
    except Exception:  # noqa: BLE001
        pass


def delete_session(g: float, trials: list[str]) -> None:
    for tid in trials:
        shutil.rmtree(DATA / tid, ignore_errors=True)
        for d in mass_folder(g).glob(f"{tid[:5]}_*"):
            shutil.rmtree(d, ignore_errors=True)
    if mass_folder(g).exists() and not any(mass_folder(g).iterdir()):
        mass_folder(g).rmdir()


def run_mass(g: float, squeeze: float, motions: list[str]) -> tuple[list[dict], bool]:
    """Returns (done entries, aborted)."""
    base = len(call("/api/replay/status")["done"])     # done list spans every round since logger start
    r = call("/api/replay/start", {"motions": motions, "payload_g": g, "tag": f"payload {g:g}g",
                                   "grip_squeeze_deg": squeeze})
    if not r.get("ok"):
        print("  ✗ 시작 실패:", r.get("problems"))
        return [], True
    n, last, t0 = len(motions), None, time.time()
    try:
        while True:
            time.sleep(1)
            d = call("/api/replay/status")
            d["done"] = d["done"][base:]
            key = (d["state"], d["motion"], len(d["done"]))
            if key != last:
                last = key
                if d["state"] == "running":
                    temp = max(x for x in joints()["temp"] if x is not None)
                    el = time.time() - t0
                    print(f"  [{len(d['done']) + 1:2d}/{n}] {d['motion']:<34} 경과 {el / 60:4.1f}분  최고온도 {temp} °C",
                          flush=True)
            if d["state"] not in ("running", "checking"):
                break
    except KeyboardInterrupt:
        print("\n  ■ 중단 요청 → 동작 정지 후 전체 토크를 풉니다 (팔이 처질 수 있어요)")
        call("/api/replay/abort", {})
        time.sleep(0.5)
        call("/api/teleop/torque_off", {})
        time.sleep(1.0)
        d = call("/api/replay/status")
        return d["done"][base:], True
    if d["state"] == "error":
        print(f"  ✗ 오류로 멈춤: {d['message']}")
        return d["done"], True
    return d["done"], False


def wait_exports(entries: list[dict], timeout: float = 90.0) -> list[dict]:
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = call("/api/replay/status")["done"]
        mine = [x for x in d if x.get("trial") in {e.get("trial") for e in entries} and x.get("ok")]
        if all(("export" in x or "export_error" in x) for x in mine):
            return mine
        time.sleep(1)
    return [x for x in call("/api/replay/status")["done"] if x.get("ok")]


def summary(g: float) -> None:
    rows = []
    for meta in sorted(mass_folder(g).glob("*/meta.json")):
        m = json.loads(meta.read_text())
        q = m["quality_training_window"]
        rows.append((m["set"], q.get("imu_vs_fk_corr_3hz"), max(q["load_max_pct"][:5])))
    for s in SETS:
        r = [x for x in rows if x[0] == s]
        if r:
            cs = sorted(x[1] for x in r if x[1] is not None)
            med = cs[len(cs) // 2] if cs else float("nan")
            print(f"    {s:<22} {len(r):2d}개   IMU-FK r(중앙값) {med:.3f}   최대 부하 {max(x[2] for x in r):.0f}%")


def main() -> None:
    print("━━ 페이로드 데이터 수집 ━━  (Ctrl+C: 동작 중이면 정지, 대기 중이면 종료)")
    wait_devices()
    try:
        call("/api/auto", {"on": False})          # no auto trials while the payload is handled
    except Exception:  # noqa: BLE001
        pass
    allm = call("/api/replay/list")["motions"]
    plan = [m for s in SETS for m in allm if m.startswith(s + "/")]
    while True:
        print()
        call("/api/teleop/release_gripper", {})
        print("⓪ 6번(그리퍼) 토크를 풀었어요. 물체를 빼거나 바꾸세요. (1~5번은 자세 유지)")
        v = ask("① 페이로드 무게 (g, q=종료): ")
        if v.lower() in ("q", "quit", "exit"):
            break
        try:
            g = float(v)
            assert g >= 0
        except (ValueError, AssertionError):
            print("  숫자로 입력하세요 (예: 135)")
            continue
        squeeze = 0.0
        have = done_motions(g)
        todo = [m for m in plan if m not in have]
        if have:
            a = ask(f"  {g:g} g 데이터가 이미 {len(have)}개 있어요. 이어서(Enter) / 처음부터(r) / 취소(c): ").lower()
            if a == "c":
                continue
            if a == "r":
                shutil.rmtree(mass_folder(g), ignore_errors=True)
                todo = list(plan)
        if not todo:
            print(f"  {g:g} g는 30개 모두 완료돼 있어요.")
            continue
        call("/api/teleop/release_gripper", {})
        print("② 6번(그리퍼) 토크 풀림 확인. 물체를 테이프로 고정하세요 (그리퍼는 지금 자리 그대로 유지, 추가 조임 없음).")
        if ask("   쥐었으면 Enter (c=취소): ").lower() == "c":
            continue
        wait_devices()
        j = joints()
        print(f"③ 시작: {g:g} g, 그리퍼 {j['pos'][5]:.1f}°에 고정, 궤적 {len(todo)}개 "
              f"(약 {len(todo) * 23 / 60:.0f}분)")
        entries, aborted = run_mass(g, squeeze, todo)
        session = [e["trial"] for e in entries if e.get("trial")]
        failed = [x for x in entries if not x.get("ok")]
        if aborted:
            a = ask("  이번 중량에서 방금 기록한 데이터를 지울까요? (물체가 떨어졌으면 y) [y/N]: ").lower()
            # the stopped trial itself is the newest trial folder
            newest = sorted(DATA.glob("T0*"))[-1].name if list(DATA.glob("T0*")) else None
            if a == "y":
                wait_exports(entries, timeout=30)
                delete_session(g, session + ([newest] if newest and newest not in session else []))
                print(f"  지웠어요 ({len(session) + (1 if newest and newest not in session else 0)}개 회차).")
            else:
                if newest:
                    label_trial(newest, "[ABORTED]")
                print(f"  남겨뒀어요. 완료된 {len(session)}개는 {mass_folder(g)} 에 있어요. 다음에 같은 무게로 이어서 할 수 있어요.")
            print("  (Ctrl+C로 멈췄으면 전체 토크가 풀려 있어요. 오류로 멈췄으면 자세 유지 중)")
            continue
        ok = wait_exports(entries)
        bad = [x for x in ok if "export_error" in x]
        print(f"④ 완료: {len(ok)}개 저장 → {mass_folder(g)}" + (f"  (내보내기 오류 {len(bad)}개)" if bad else ""))
        if failed:
            print(f"  실패 {len(failed)}개: {[x['motion'] for x in failed]}")
        summary(g)
        call("/api/teleop/release_gripper", {})
        print("  그리퍼 토크를 풀었어요. 다음 물체로 바꾸세요.")
    print("종료. 데이터:", DATA / "loaded")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        try:                                   # Ctrl+C anywhere: arm limp
            call("/api/replay/abort", {})
            call("/api/teleop/torque_off", {})
            print("\n전체 토크를 풀었어요. 종료.")
        except Exception:  # noqa: BLE001
            print("\n⚠ 로거에 연결할 수 없어 토크를 못 풀었어요. ./log.sh를 다시 실행하세요. 종료.")
