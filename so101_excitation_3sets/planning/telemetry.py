"""Synchronous, ideal simulated IMU telemetry. No mass or force labels."""
import json
from pathlib import Path
import mujoco
import numpy as np


def site_measurement(model, data, site_name, sensor_name):
    site = model.site(site_name).id
    rotation = data.site_xmat[site].reshape(3, 3)
    specific = data.sensor(sensor_name).data.copy()
    jac = np.zeros((3, model.nv)); rotjac = np.zeros_like(jac)
    mujoco.mj_jacSite(model, data, jac, rotjac, site)
    quat = np.empty(4); mujoco.mju_mat2Quat(quat, rotation.ravel())
    return {'position_world_m':data.site_xpos[site].copy(),
            'quaternion_world_wxyz':quat,
            'velocity_world_m_s':jac @ data.qvel,
            'acceleration_world_m_s2':rotation @ specific + model.opt.gravity,
            'specific_force_local_m_s2':specific}


class ImuRecorder:
    def __init__(self, model, period_s=0.01):
        if not np.isfinite(period_s) or period_s < model.opt.timestep:
            raise ValueError('Recording period must be finite and at least the physics timestep')
        ratio = period_s/model.opt.timestep
        if not np.isclose(ratio, round(ratio)):
            raise ValueError('Recording period must be an integer multiple of physics timestep')
        for name in ('imu_accelerometer','imu_gyro','drone_com_accelerometer','payload_com_accelerometer'):
            model.sensor(name)
        self.model=model; self.period=period_s; self.next_time=0.; self.rows=[]

    def observe(self, data, reference):
        if data.time+1e-9 < self.next_time:
            return
        self.next_time += self.period
        row={'t':float(data.time),'qpos':data.qpos.copy(),'qvel':data.qvel.copy(),
             'qacc':data.qacc.copy(),'reference_qpos':reference.copy()}
        for site in ('imu','drone_com','payload_com'):
            sensor='imu_accelerometer' if site=='imu' else site+'_accelerometer'
            row.update({site+'_'+k:v for k,v in site_measurement(self.model,data,site,sensor).items()})
        row['imu_specific_force_local_g']=row['imu_specific_force_local_m_s2']/9.80665
        row['imu_gyro_local_rad_s']=data.sensor('imu_gyro').data.copy()
        row['imu_gyro_local_deg_s']=np.rad2deg(row['imu_gyro_local_rad_s'])
        self.rows.append(row)

    def save(self, path, metadata, physics_report):
        path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
        arrays={key:np.asarray([r[key] for r in self.rows]) for key in self.rows[0]}
        info={**metadata,'telemetry_version':1,'telemetry_mode':'simulated_physics_ideal_imu',
              'sample_period_s':self.period,'gravity_world_m_s2':self.model.opt.gravity.tolist(),
              'physics_report':physics_report,
              'conventions':{'world':'base/world, +Z up','quaternion':'wxyz, local to world',
                 'linear_acceleration':'a_world = R_world_from_site @ specific_force_local + gravity_world',
                 'specific_force':'ideal accelerometer a-g in local frame; nonzero when resting',
                 'payload_com':'placeholder site; no held object modeled',
                 'force_labels':'none; no payload mass assumed',
                 'noise':'none; real IMU bias/filter/time delay not simulated'}}
        np.savez_compressed(path,**arrays,metadata_json=json.dumps(info))
        columns=[]; values=[]
        for key,a in arrays.items():
            if a.ndim==1:
                columns.append('time_s' if key=='t' else key); values.append(a[:,None]); continue
            labels=list('wxyz') if 'quaternion' in key else (list('xyz') if a.shape[1]==3 else [str(i+1) for i in range(a.shape[1])])
            columns.extend(key+'_'+label for label in labels);values.append(a)
        np.savetxt(path.with_suffix('.csv'),np.concatenate(values,axis=1),delimiter=',',
                   header=','.join(columns),comments='',fmt='%.10g')
        c=metadata['config']; start=c.get('initial_hold_s',0.)
        active=(arrays['t']>=start)&(arrays['t']<=arrays['t'][-1]-c.get('final_hold_s',0.))
        accel=arrays['imu_acceleration_world_m_s2'][active]
        summary={'samples':len(arrays['t']),'sample_period_s':self.period,
                 'active_acceleration_min_xyz_m_s2':accel.min(axis=0).tolist() if len(accel) else None,
                 'active_acceleration_max_xyz_m_s2':accel.max(axis=0).tolist() if len(accel) else None,
                 'active_acceleration_rms_xyz_m_s2':np.sqrt(np.mean(accel**2,axis=0)).tolist() if len(accel) else None,
                 'per_segment':[]}
        if len(arrays['t']) > 2 and len(accel):
            differentiated = np.gradient(arrays['imu_velocity_world_m_s'], arrays['t'], axis=0)
            residual = arrays['imu_acceleration_world_m_s2'][active]-differentiated[active]
            summary['velocity_difference_check'] = {
                'rms_residual_xyz_m_s2':np.sqrt(np.mean(residual**2,axis=0)).tolist(),
                'note':'Central differences of sampled velocity; finite timestep/sampling error expected.'}
        for segment in c.get('segments',[]):
            select=(arrays['t']>=start)&(arrays['t']<start+segment['duration_s'])
            a=arrays['imu_acceleration_world_m_s2'][select]
            if not len(a):
                break
            summary['per_segment'].append({'name':segment['name'],'start_s':start,
                'end_s':start+segment['duration_s'],'min_xyz_m_s2':a.min(axis=0).tolist(),
                'max_xyz_m_s2':a.max(axis=0).tolist(),'rms_xyz_m_s2':np.sqrt(np.mean(a*a,axis=0)).tolist()})
            start+=segment['duration_s']
        info['statistics']=summary
        path.with_suffix('.json').write_text(json.dumps(info,indent=2)+'\n')
        return summary


def load_recording(path, trajectory_metadata):
    with np.load(path,allow_pickle=False) as f:
        info=json.loads(str(f['metadata_json']))
        arrays={k:f[k] for k in f.files if k!='metadata_json'}
    if info.get('telemetry_version')!=1 or info.get('telemetry_mode')!='simulated_physics_ideal_imu':
        raise ValueError('Unknown telemetry format')
    for key in ('compiled_model_sha256','config','collision_policy','mujoco_version'):
        if info[key]!=trajectory_metadata[key]:
            raise ValueError('Telemetry/model/trajectory configuration mismatch: '+key)
    if not info['physics_report']['completed']:
        raise ValueError('Recording is incomplete; inspect physics report')
    if len(arrays['t'])<2 or not np.all(np.diff(arrays['t'])>0) or not all(np.isfinite(a).all() for a in arrays.values()):
        raise ValueError('Invalid recording arrays')
    return arrays,info
