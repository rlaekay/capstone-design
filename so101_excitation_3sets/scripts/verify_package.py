#!/usr/bin/env python3
"""Verify checksums, frozen model and all thirty recordings; no hardware access."""
from pathlib import Path
import hashlib,json,sys
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from planning.core import load_scene,compiled_model_sha256,load_trajectory
from planning.telemetry import load_recording

def main():
 hashes=json.loads((ROOT/'CHECKSUMS.json').read_text())
 for rel,expected in hashes.items():
  if hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()!=expected:raise ValueError('File changed: '+rel)
 if mujoco.__version__!='3.3.2':raise ValueError('This package requires mujoco==3.3.2')
 model=load_scene(ROOT/'models/bench/scene.mjb');xml=load_scene(ROOT/'models/bench/scene.xml');compared=0
 # File-path byte tables/addresses change on relocation. Every physical/numerical array must match.
 ignore={'paths','mesh_pathadr','hfield_pathadr','tex_pathadr'}
 for name in dir(model):
  if name.startswith('_') or name in ignore:continue
  a=getattr(model,name);b=getattr(xml,name)
  if isinstance(a,np.ndarray):
   if not np.array_equal(a,b):raise ValueError('Portable XML differs from frozen model: '+name)
   compared+=1
 h=compiled_model_sha256(model);count=0;seconds=0
 for bank in sorted((ROOT/'outputs').glob('excitation_v2_set_*')):
  manifest=json.loads((bank/'manifest.json').read_text())
  assert manifest['completed']==10 and manifest['failed']==0
  for row in manifest['trajectories']:
   folder=bank/f'motion_{row["index"]:03d}';a,meta=load_trajectory(folder/'trajectory.npz')
   assert meta['compiled_model_sha256']==h
   assert (ROOT/meta['model_path']).is_file()
   recording,_=load_recording(folder/'imu.npz',meta)
   clip,cm=load_trajectory(folder/'training_trajectory.npz');start,end=meta['config']['training_window_s']
   assert np.isclose(end-start,12) and np.isclose(clip['t'][-1],12) and len(clip['t'])==1201
   for j in range(6):assert np.allclose(clip['qpos'][:,j],np.interp(clip['t']+start,a['t'],a['qpos'][:,j]),atol=1e-9)
   with np.load(folder/'training_imu.npz') as f:
    assert np.isclose(f['t'][0],0,atol=1e-8) and np.isclose(f['t'][-1],12,atol=1e-8)
   count+=1;seconds+=end-start
 assert count==30
 print(json.dumps({'passed':True,'checksummed_files':len(hashes),'matching_model_arrays':compared,'motions':count,'training_seconds':seconds,'model_sha256':h},indent=2))
if __name__=='__main__':main()
