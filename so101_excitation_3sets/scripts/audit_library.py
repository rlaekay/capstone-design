#!/usr/bin/env python3
"""Independent exhaustive-distance spot audit of the bank's physics states."""
import argparse,json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np
from planning.core import load_scene,make_checker,load_trajectory,compiled_model_sha256
from planning.telemetry import load_recording


def audit(folder):
    folder=Path(folder);_,meta=load_trajectory(folder/'trajectory.npz')
    modelpath=Path(meta['model_path'])
    if not modelpath.is_absolute():modelpath=ROOT/modelpath
    model=load_scene(modelpath,meta['config'].get('obstacles',[]))
    if compiled_model_sha256(model)!=meta['compiled_model_sha256']:raise ValueError('Model hash changed')
    c={**meta['config'],'broad_phase':False};checker=make_checker(model,c,meta['collision_policy'])
    a,_=load_recording(folder/'imu.npz',meta)
    # Every 0.5 s, last pose, and extrema of each acceleration axis.
    indices=np.unique(np.concatenate([np.arange(0,len(a['t']),50),[len(a['t'])-1],
        np.argmin(a['imu_acceleration_world_m_s2'],axis=0),np.argmax(a['imu_acceleration_world_m_s2'],axis=0)]))
    minimum=1.
    for i in indices:
        report=checker.check_full_qpos(a['qpos'][i],full_report=True)
        if not report.valid:raise ValueError(f'{folder.name} fails at {a["t"][i]}: {report}')
        minimum=min(minimum,report.minimum_distance_m)
    return {'motion':folder.name,'samples':len(indices),'minimum_clearance_m':minimum}


def main():
    p=argparse.ArgumentParser();p.add_argument('library',type=Path);p.add_argument('--workers',type=int,default=6);args=p.parse_args()
    m=json.loads((args.library/'manifest.json').read_text())
    folders=[str(args.library/f'motion_{r["index"]:03d}') for r in m['trajectories'] if not r.get('failed')]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:results=list(pool.map(audit,folders))
    out={'motions':len(results),'exhaustive_audit_samples':sum(r['samples'] for r in results),
         'minimum_clearance_m':min(r['minimum_clearance_m'] for r in results),'passed':True,
         'scope':'Every 0.5 s and acceleration extrema; additional to 2 ms AABB-assisted physics collision checks.',
         'results':results}
    (args.library/'independent_audit.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({k:v for k,v in out.items() if k!='results'},indent=2))
if __name__=='__main__':main()
