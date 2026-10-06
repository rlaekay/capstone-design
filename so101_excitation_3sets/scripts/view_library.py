#!/usr/bin/env python3
"""Browse previously verified open-gripper physics recordings. No hardware I/O."""
import argparse,json,time
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import mujoco
import mujoco.viewer
import numpy as np
from planning.core import load_scene,load_trajectory,compiled_model_sha256,make_checker,validate_trajectory
from planning.telemetry import load_recording


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library',type=Path,default=ROOT/'outputs/excitation_library')
    p.add_argument('--index',type=int,default=0)
    p.add_argument('--speed',type=float,default=1.)
    p.add_argument('--auto',action='store_true',help='Advance through the bank; otherwise loop selected motion')
    p.add_argument('--training-only',action='store_true',help='Play only the saved training window, excluding approach/return')
    p.add_argument('--headless',action='store_true',help='Validate selected recording without opening a window')
    args=p.parse_args()
    if args.speed<=0 or not np.isfinite(args.speed):p.error('Speed must be positive')
    manifest=json.loads((args.library/'manifest.json').read_text())
    entries=[r for r in manifest['trajectories'] if not r.get('failed')]
    lookup={r['index']:i for i,r in enumerate(entries)}
    if args.index not in lookup:p.error('Motion index is absent from this bank')
    selected=lookup[args.index]
    folder=args.library/f"motion_{entries[selected]['index']:03d}"
    _,meta=load_trajectory(folder/'trajectory.npz')
    model_path=Path(meta['model_path'])
    if not model_path.is_absolute():model_path=ROOT/model_path
    model=load_scene(model_path,meta['config'].get('obstacles',[]))
    model_hash=compiled_model_sha256(model)
    def load(index):
        entry=entries[index];folder=args.library/f"motion_{entry['index']:03d}"
        reference,metadata=load_trajectory(folder/'trajectory.npz')
        if metadata['compiled_model_sha256']!=model_hash or metadata['mujoco_version']!=mujoco.__version__:
            raise ValueError('Model/version changed; regenerate bank')
        info=json.loads((ROOT/metadata['config']['collision_policy_file']).read_text())
        policy={k:info.get(k,[]) for k in ('planning_excluded_body_pairs','planning_excluded_geom_pairs')}
        if policy!=metadata['collision_policy']:raise ValueError('Collision policy changed')
        checker=make_checker(model,metadata['config'],policy)
        validate_trajectory(checker,reference,metadata['config']['max_velocity_rad_s'],metadata['config']['max_acceleration_rad_s2'])
        arrays,_=load_recording(folder/'imu.npz',metadata)
        window=metadata['config'].get('training_window_s',[0.,float(arrays['t'][-1])])
        if args.training_only:
            mask=(arrays['t']>=window[0]-1e-8)&(arrays['t']<=window[1]+1e-8)
            arrays={k:v[mask].copy() for k,v in arrays.items()}
            arrays['t']-=window[0]
            window=[0.,window[1]-window[0]]
        entry={**entry,'playback_training_window_s':window}
        print(f"Motion {entry['index']:03d} | {entry['family']} | {arrays['t'][-1]:.1f}s | jaw=1.2 rad",flush=True)
        return arrays,entry
    arrays,entry=load(selected)
    if args.headless:return
    commands=[]
    def key(k):commands.append(k)
    data=mujoco.MjData(model);data.qpos[:]=arrays['qpos'][0];mujoco.mj_forward(model,data)
    paused=False;elapsed=0.;last=time.monotonic()
    with mujoco.viewer.launch_passive(model,data,key_callback=key) as viewer:
        viewer.cam.lookat[:]=[.04,0,.22];viewer.cam.distance=1.20;viewer.cam.azimuth=140;viewer.cam.elevation=-22
        viewer.opt.geomgroup[3]=0
        while viewer.is_running():
            now=time.monotonic()
            if not paused:elapsed+=(now-last)*args.speed
            last=now;change=0
            while commands:
                k=commands.pop(0)
                if k in (ord('N'),262):change=1
                elif k in (ord('P'),263):change=-1
                elif k==32:paused=not paused
                elif k==ord('R'):elapsed=0.
                elif k==ord('C'):viewer.opt.geomgroup[3]^=1
            if elapsed>arrays['t'][-1]:
                if args.auto:change=1
                else:elapsed=0.
            if change:
                selected=(selected+change)%len(entries);arrays,entry=load(selected);elapsed=0.;last=time.monotonic()
            with viewer.lock():
                data.qpos[:]=[np.interp(elapsed,arrays['t'],arrays['qpos'][:,j]) for j in range(model.nq)]
                data.qvel[:]=0;data.time=elapsed;mujoco.mj_forward(model,data)
            window=entry['playback_training_window_s']
            phase='APPROACH' if elapsed<window[0] else ('RETURN' if elapsed>window[1] else 'TRAINING')
            acc=[np.interp(elapsed,arrays['t'],arrays['imu_acceleration_world_m_s2'][:,j]) for j in range(3)]
            viewer.set_texts((None,None,
                f"{entry['index']:03d} / {len(entries)}  {entry['family']}\nRecorded physics | gripper 1.2 rad | {phase}\nt={elapsed:.2f}/{arrays['t'][-1]:.2f}s\nWorld a: {acc[0]:+.2f}, {acc[1]:+.2f}, {acc[2]:+.2f} m/s2\nN/P: next/previous  Space: pause  R: restart\nC: collision shapes | no hardware",''))
            viewer.sync();time.sleep(1/60)
if __name__=='__main__':main()
