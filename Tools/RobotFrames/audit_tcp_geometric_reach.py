"""Offline necessary reach bound from captured URDF; never connects to ROS."""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
models = json.loads((ROOT/'planning_models_diagnostic_20261022.json').read_text(encoding='utf-8-sig'))
data = json.loads((ROOT/'planning_ik_diagnostic_20261022.json').read_text(encoding='utf-8-sig'))
urdf = ET.fromstring(models['/move_group/get_parameters']['response']['values'][0]['string_value'])
joints = {j.get('name'):j for j in urdf.findall('joint')}
by_child = {j.find('child').get('link'):j for j in joints.values()}

def vector(text):
    return np.array([float(x) for x in text.split()])

def rz(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def origin(joint):
    o=joint.find('origin')
    t=vector(o.get('xyz','0 0 0')) if o is not None else np.zeros(3)
    r,p,y=vector(o.get('rpy','0 0 0')) if o is not None else np.zeros(3)
    rx=np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    ry=np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    T=np.eye(4);T[:3,:3]=rz(y)@ry@rx;T[:3,3]=t
    return T

def pose(msg):
    p=msg['position'];q=msg['orientation']
    x,y,z,w=np.array([q[a] for a in ['x','y','z','w']])/np.linalg.norm([q[a] for a in ['x','y','z','w']])
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=[p[a] for a in ['x','y','z']]
    return T

chain=[];child='tcp'
while child!='ur_wrist_3_link':
    j=by_child[child]
    assert j.get('type')=='fixed'
    chain.append(j);child=j.find('parent').get('link')
wrist_to_tcp=np.eye(4)
for j in reversed(chain):
    wrist_to_tcp=wrist_to_tcp@origin(j)

fks=dict(zip(data['fk']['fk_link_names'],[pose(p['pose']) for p in data['fk']['pose_stamped']]))
shoulder=fks['ur_shoulder_link'][:3,3]
prefix=['ur_shoulder_lift_joint','ur_elbow_joint','ur_wrist_1_joint','ur_wrist_2_joint']
lengths={name:float(np.linalg.norm(origin(joints[name])[:3,3])) for name in prefix}
maximum=sum(lengths.values())
O6=origin(joints['ur_wrist_3_joint'])
v=O6[:3,:3].T@O6[:3,3]
radius=float(np.linalg.norm(v[:2]))

results=[]
for item in data['controls']:
    Ttcp=pose(item['pose']['pose'])
    Tw3=Ttcp@np.linalg.inv(wrist_to_tcp)
    centre=Tw3[:3,3]-Tw3[:3,:3]@np.array([0,0,v[2]])
    d=centre-shoulder
    n=Tw3[:3,2]
    axial=float(d@n)
    planar=float(np.linalg.norm(d-axial*n))
    minimum=math.hypot(axial,planar-radius)
    results.append({'name':item['name'],'tcp_position_global_m':Ttcp[:3,3].tolist(),
        'tcp_plus_z_global':Ttcp[:3,2].tolist(),
        'required_wrist3_global_m':Tw3[:3,3].tolist(),
        'possible_wrist2_circle_centre_global_m':centre.tolist(),
        'possible_wrist2_circle_radius_m':radius,
        'minimum_possible_shoulder_to_wrist2_m':minimum,
        'maximum_possible_shoulder_to_wrist2_m':maximum,
        'excess_over_conservative_bound_m':minimum-maximum,
        'violates_necessary_reach_bound':minimum>maximum})

# Control the derived q6 circle against the actual current FK point.
positions=dict(zip(data['joint_state']['name'],data['joint_state']['position']))
Tw3=fks['ur_wrist_3_link']
predicted_w2=Tw3[:3,3]-Tw3[:3,:3]@rz(-positions['ur_wrist_3_joint'])@v
actual_w2=fks['ur_wrist_2_link'][:3,3]
report={'schema_version':1,'method':'Necessary bound using triangle inequality for shoulder-to-wrist2 joint-origin chain. For a fixed TCP pose, wrist2 lies on the q6 circle derived exactly from wrist3 joint origin; compare its minimum shoulder distance to maximum chain length. A positive excess proves infeasibility in this URDF without relying on IK search.',
    'shoulder_global_m':shoulder.tolist(),'prefix_translation_lengths_m':lengths,
    'wrist3_to_tcp':wrist_to_tcp.tolist(),'joint6_vector_in_rotated_frame':v.tolist(),
    'current_wrist2_reconstruction_error_m':float(np.linalg.norm(predicted_w2-actual_w2)),
    'cases':results,
    'limitation':'This proves a necessary reach constraint for the configured URDF, not the physical robot/tool. A physical mounting, model, capture or alignment mismatch can still explain a disagreement with observation.'}
assert report['current_wrist2_reconstruction_error_m']<1e-8
(ROOT/'tcp_geometric_reach_20261022.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,indent=2))
