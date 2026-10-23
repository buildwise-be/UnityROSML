"""Offline same-position/same-normal reach proof, with tool twist unconstrained."""
import contextlib
import io
import json
import math
from pathlib import Path
import numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import audit_tcp_geometric_reach as a

O2=a.origin(a.joints['ur_shoulder_lift_joint'])
O3=a.origin(a.joints['ur_elbow_joint'])
O4=a.origin(a.joints['ur_wrist_1_joint'])
O5=a.origin(a.joints['ur_wrist_2_joint'])
O6=a.O6
v3=O3[:3,:3].T@O3[:3,3]
t4=O4[:3,3]
upper_arm_forearm_max=math.hypot(v3[2]+t4[2],
    np.linalg.norm(v3[:2])+np.linalg.norm(t4[:2]))
shoulder_to_wrist1_max=float(np.linalg.norm(O2[:3,3])+upper_arm_forearm_max)

# For the wrist1 location implied by any q5,q6 at a specified TCP pose:
# p_w1 = p_w3 - R_w3 Rz(-q6) [v6 + O6.R.T Rz(-q5) v5].
# The v5.xy contribution has length <= residual_radius. The remaining
# q6-dependent term is an exact circle about wrist3.Z == tcp.Z.
v5=O5[:3,:3].T@O5[:3,3]
u=a.v+O6[:3,:3].T@np.array([0,0,v5[2]])
circle_radius=float(np.linalg.norm(u[:2]))
residual_radius=float(np.linalg.norm(v5[:2]))

# The fixed transform is Rz(-90deg), t=(0,-34mm,150mm). Therefore its
# inverse wrist3-origin correction is -34mm*tcp.X -150mm*tcp.Z.
assert np.allclose(a.wrist_to_tcp[:3,3],[0,-.034,.15],atol=1e-12)
assert np.allclose(a.wrist_to_tcp[:3,:3],a.rz(-math.pi/2),atol=1e-12)
cases=[]
for item in a.data['controls']:
    T=a.pose(item['pose']['pose']);normal=T[:3,2]
    centre=T[:3,3]-(.15+u[2])*normal
    d=centre-a.shoulder
    axial=float(d@normal)
    radial=float(np.linalg.norm(d-axial*normal))
    # Free tool twist sweeps the 34mm TCP eccentricity about the same
    # normal. A disk of radius circle_radius+34mm over-approximates all
    # these circles, making this a conservative necessary lower bound.
    minimum=math.hypot(axial,max(0,radial-circle_radius-.034))-residual_radius
    cases.append({'name':item['name'],'tcp_position_global_m':T[:3,3].tolist(),
        'normal_global':normal.tolist(),'clocking':'fully unconstrained',
        'minimum_required_shoulder_to_wrist1_m':minimum,
        'maximum_possible_shoulder_to_wrist1_m':shoulder_to_wrist1_max,
        'excess_m':minimum-shoulder_to_wrist1_max,
        'violates_necessary_reach_constraint':minimum>shoulder_to_wrist1_max})

# Independent position+normal numerical minimization. It is corroboration,
# not the proof: the inequality above covers all joint and tool-twist values.
names=['ur_shoulder_pan_joint','ur_shoulder_lift_joint','ur_elbow_joint',
       'ur_wrist_1_joint','ur_wrist_2_joint','ur_wrist_3_joint']
chain=[];link='tcp'
while link!='mir_base_footprint':
    j=a.by_child[link];chain.append(j);link=j.find('parent').get('link')
spec=[(a.origin(j),names.index(j.get('name')) if j.get('name') in names else None)
      for j in reversed(chain)]
limits=np.array([(float(a.joints[n].find('limit').get('lower')),
                  float(a.joints[n].find('limit').get('upper'))) for n in names])
def fk(q):
    T=np.eye(4);axes=[];pivots=[]
    for O,index in spec:
        T=T@O
        if index is not None:
            axes.append(T[:3,2].copy());pivots.append(T[:3,3].copy())
            Q=np.eye(4);Q[:3,:3]=a.rz(q[index]);T=T@Q
    Jp=np.stack([np.cross(axis,T[:3,3]-p) for axis,p in zip(axes,pivots)],axis=1)
    Jn=np.stack([np.cross(axis,T[:3,2]) for axis in axes],axis=1)
    return T,Jp,Jn
qmap=dict(zip(a.data['joint_state']['name'],a.data['joint_state']['position']))
current=np.array([qmap[n] for n in names])
target=a.pose(a.data['controls'][1]['pose']['pose'])
position=target[:3,3];normal=target[:3,2]
weight=.3
def residual(T):
    return np.r_[position-T[:3,3],weight*(normal-T[:3,2])]
def optimize(seed):
    q=seed.copy();damping=.005
    for iteration in range(500):
        T,Jp,Jn=fk(q);e=residual(T);error=float(np.linalg.norm(e))
        if error<1e-9:break
        J=np.vstack([Jp,weight*Jn])
        step=J.T@np.linalg.solve(J@J.T+np.eye(6)*damping*damping,e)
        norm=np.linalg.norm(step)
        if norm>.25:step*=.25/norm
        improved=False
        for scale in [1,.5,.25,.125]:
            trial=np.clip(q+step*scale,limits[:,0],limits[:,1])
            trial_T,_,_=fk(trial)
            if np.linalg.norm(residual(trial_T))<error-1e-12:
                q=trial;damping=max(damping*.8,1e-6);improved=True;break
        if not improved:
            damping=min(1,damping*3)
            if damping>=1:break
    T,_,_=fk(q)
    return {'weighted_residual':float(np.linalg.norm(residual(T))),
        'position_error_m':float(np.linalg.norm(position-T[:3,3])),
        'normal_error_degrees':math.degrees(math.acos(np.clip(normal@T[:3,2],-1,1))),
        'joint_positions_rad':dict(zip(names,[float(x) for x in q])),'iterations':iteration}
rng=np.random.default_rng(20261022)
seeds=[current]+[np.clip(current+rng.normal(0,1,6),limits[:,0],limits[:,1]) for _ in range(19)]
seeds += [rng.uniform(limits[:,0],limits[:,1]) for _ in range(80)]
results=[optimize(seed) for seed in seeds]
best=min(results,key=lambda x:x['weighted_residual'])
report={'scope':'Same original TCP position and +Z normal, any tool twist. Entirely offline.',
    'proof':'Triangle/circle bounds derived from the actual calibrated URDF. Shoulder-to-wrist1 upper bound maximizes the elbow circle exactly, then adds the small shoulder-lift offset. Wrist1 lower bound includes arbitrary q5,q6 and tool twist plus full calibrated residual error balls. Positive lower-minus-upper proves infeasibility independently of numerical IK.',
    'upper_arm_forearm_max_m':upper_arm_forearm_max,
    'shoulder_to_wrist1_max_m':shoulder_to_wrist1_max,
    'wrist_circle_radius_m':circle_radius,'wrist_residual_radius_m':residual_radius,
    'combined_axial_offset_m':float(.15+u[2]),'cases':cases,
    'numerical_check':{'seeds':len(seeds),'normal_weight_m':weight,'best':best},
    'limitation':'This proves a necessary constraint for this captured URDF. Physical tool mounting and model parity are not established by offline calculations.'}
out=Path(__file__).parent/'tcp_normal_constraint_20261022.json'
out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,indent=2))
