"""Independent offline position-only FK minimization; no ROS connection."""
import contextlib
import io
import json
import math
from pathlib import Path
import numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import audit_tcp_geometric_reach as a

names=['ur_shoulder_pan_joint','ur_shoulder_lift_joint','ur_elbow_joint',
       'ur_wrist_1_joint','ur_wrist_2_joint','ur_wrist_3_joint']
chain=[];link='tcp'
while link!='mir_base_footprint':
    j=a.by_child[link];chain.append(j);link=j.find('parent').get('link')
chain=list(reversed(chain))
spec=[]
for j in chain:
    spec.append((a.origin(j),names.index(j.get('name')) if j.get('name') in names else None))
limits=[]
for n in names:
    l=a.joints[n].find('limit');limits.append((float(l.get('lower')),float(l.get('upper'))))
limits=np.array(limits)
def fk(q, need_jacobian=True):
    T=np.eye(4);axes=[];pivots=[]
    for O,index in spec:
        T=T@O
        if index is not None:
            axes.append(T[:3,2].copy());pivots.append(T[:3,3].copy())
            Q=np.eye(4);Q[:3,:3]=a.rz(q[index]);T=T@Q
    J=np.stack([np.cross(axis,T[:3,3]-pivot) for axis,pivot in zip(axes,pivots)],axis=1)
    return T,J

joint_positions=dict(zip(a.data['joint_state']['name'],a.data['joint_state']['position']))
current_q=np.array([joint_positions[n] for n in names])
control_T,_=fk(current_q)
assert np.linalg.norm(control_T[:3,3]-a.fks['tcp'][:3,3])<1e-8
assert np.linalg.norm(control_T[:3,:3]-a.fks['tcp'][:3,:3])<1e-8
target=a.pose(a.data['controls'][1]['pose']['pose'])[:3,3]

def optimize(seed):
    q=seed.copy();damping=.005
    for iteration in range(500):
        T,J=fk(q);e=target-T[:3,3];error=np.linalg.norm(e)
        if error<1e-8:
            return q,T,error,iteration
        step=J.T@np.linalg.solve(J@J.T+np.eye(3)*damping*damping,e)
        norm=np.linalg.norm(step)
        if norm>.25:step*=.25/norm
        improved=False
        for scale in [1,.5,.25,.125]:
            trial=np.clip(q+step*scale,limits[:,0],limits[:,1])
            trial_T,_=fk(trial)
            trial_error=np.linalg.norm(target-trial_T[:3,3])
            if trial_error<error-1e-12:
                q=trial;damping=max(damping*.8,1e-6);improved=True;break
        if not improved:
            damping=min(1,damping*3)
            if damping>=1:
                return q,T,error,iteration
    T,_=fk(q)
    return q,T,float(np.linalg.norm(target-T[:3,3])),iteration

rng=np.random.default_rng(20261022)
seeds=[current_q]+[np.clip(current_q+rng.normal(0,1,6),limits[:,0],limits[:,1]) for _ in range(19)]
seeds += [rng.uniform(limits[:,0],limits[:,1]) for _ in range(80)]
results=[]
for i,seed in enumerate(seeds):
    q,T,error,it=optimize(seed)
    results.append({'seed':i,'error_m':float(error),'iterations':it,
                    'joint_positions_rad':dict(zip(names,[float(x) for x in q])),
                    'fk':T.tolist()})
    if i==0 or error<1e-8:
        print(json.dumps({'seed':i,'error_m':float(error),'q':q.tolist()}),flush=True)
best=min(results,key=lambda x:x['error_m'])
report={'scope':'Offline independent position-only FK optimization; no collision checking and no orientation target.',
        'control_fk_error_m':float(np.linalg.norm(control_T[:3,3]-a.fks['tcp'][:3,3])),
        'target_global_m':target.tolist(),'best':best,'seed_count':len(seeds),
        'minimum_errors_m':sorted([x['error_m'] for x in results]),
        'limitation':'Numerical multi-start minimization does not prove global impossibility. A found zero-error solution establishes kinematic position feasibility only; collisions and physical parity are not assessed.'}
out=Path(__file__).parent/'tcp_position_optimization_20261022.json'
out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'best':best,'seed_count':len(seeds),'max_error_m':max(x['error_m'] for x in results)},indent=2))
