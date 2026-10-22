"""Compact upright ring slots: gravity-supported V cradles, five per ring size.

No free-standing 600 mm stalks or hanging pins. CPU CAD/IK screening only;
measured three-part assembly and timing remain Isaac PhysX acceptance gates.
"""
from pathlib import Path
import sys,json,itertools,hashlib,os
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import search_low_rack as lib
e=lib.e;cc=lib.cc;fe=lib.fe;fcl=lib.fcl
report=dict(scope=__doc__,table=lib.TABLE,storage_capacity={'ring1':5,'ring2':5,'pin':5},full_task_pass=False,cycle_time_s=None,cases=[]);OUT=HERE/os.environ.get('M80_RACK_OUTPUT','vertical-rack-search.json');TOP=lib.TABLE['top_height'];DY=lib.TABLE['center'][1]+.18
def make(centers,yaws,pin_pitch):
    rows=[];solids=[];stocks=[]
    def box(name,P,size,R=np.eye(3)):
        rows.append(e.ct.shape_box(name,P,size,R));solids.append(dict(name=name,type='box',position=list(P),size=list(size),R=R.tolist()))
    def cyl(name,P,r,h,R=np.eye(3)):
        rows.append((name,fcl.CollisionObject(fcl.Cylinder(r,h),fcl.Transform(R,np.array(P)))));solids.append(dict(name=name,type='cylinder',position=list(P),radius=r,height=h,R=R.tolist()))
    for part,h,radius in [('ring1',.0155,.0375),('ring2',.014,.031)]:
        P,R=lib.source_pose(part,centers[part],yaws[part]);R=Rotation.from_euler('y',90,degrees=True).as_matrix()@R;n=R[:,2];up=np.array([0.,0.,1.]);up-=n*np.dot(up,n);up/=np.linalg.norm(up);right=np.cross(up,n);A=np.column_stack([right,up,n])
        for i in range(5):
            C=P-[i*.026,0,0];stocks.append(dict(part=part,index=i,P=C.tolist(),R=R.tolist()))
            for j,sign in enumerate([-1,1]):
                radial=-up*np.cos(np.deg2rad(35))+sign*right*np.sin(np.deg2rad(35));roller=C+radial*(radius+.003)
                cyl(f'{part}_slot{i}_roller{j}',roller,.003,h+.004,A);top=roller[2]-.003;cyl(f'{part}_slot{i}_post{j}',[roller[0],roller[1],(TOP+top)/2],.003,top-TOP)
                # Short cheeks constrain axial tipping, without attaching a ring.
                for k,s in enumerate([-1,1]):
                    cheek=C+radial*(radius-.003)+s*n*(h/2+.002);box(f'{part}_slot{i}_cheek{j}{k}',cheek,[.008,.008,.002],A)
    P,R=lib.source_pose('pin',centers['pin'],yaws['pin']);R=Rotation.from_euler('y',pin_pitch,degrees=True).as_matrix()@R
    from stock_fixture_geometry import pin_stock_sleeve
    v,faces,_=pin_stock_sleeve();shape=cc.g.mesh_shape(v,faces)
    for i in range(5):
        C=P+[i*.035,0,0];stocks.append(dict(part='pin',index=i,P=C.tolist(),R=R.tolist()));floor=C+R@np.array([0,0,-.01975]);box('pin_floor_'+str(i),floor,[.020,.020,.004],R);center=C+R@np.array([0,0,-.01475]);rows.append(('pin_sleeve_'+str(i),fcl.CollisionObject(shape,fcl.Transform(R,center))));solids.append(dict(name='pin_sleeve_'+str(i),type='hollow_sleeve',position=center.tolist(),inner_radius=.0026,outer_radius=.005,height=.006,R=R.tolist()));low=float(floor[2]-.002*abs(R[2,2])-.01*abs(R[2,0]));box('pin_support_'+str(i),[floor[0],floor[1],(TOP+low)/2],[.020,.020,low-TOP])
    return rows,solids,stocks

table_hits=[]
for name,obj in lib.base[:-1]:
    data=fcl.CollisionData();fe._baseline.collide(obj,data,fcl.defaultCollisionCallback)
    if data.result.is_collision:table_hits.append(name)
    data=fcl.CollisionData();fe._base.collide(obj,data,fcl.defaultCollisionCallback)
    if data.result.is_collision:table_hits.append(name+'_Duo_mount')
report['table_M80_intersections']=table_hits
if table_hits:OUT.write_text(json.dumps(report,indent=2));print('TABLE_M80_REJECT',table_hits,flush=True);raise SystemExit(2)
for lowlevel,lowpin,pitch in itertools.product([.86,.84,.82],[.88,.86],[0,-30]):
    level=lowlevel+TOP-.7;pinlevel=lowpin+TOP-.7
    if os.environ.get('M80_SIDE_RACK')=='1':
        tx,ty,_=lib.TABLE['center'];centers={'ring1':[tx+.20,ty+.08,level],'ring2':[tx+.04,ty-.07,level],'pin':[tx+.26,ty-.10,pinlevel]}
    else:centers={'ring1':[.05,-.18+DY,level],'ring2':[-.40,-.18+DY,level],'pin':[-.64,-.24+DY,pinlevel]}
    yaws={'ring1':135,'ring2':60,'pin':-45};label=f'vertical_ring{round(level*1000)}_pin{round(pinlevel*1000)}_pitch{pitch}';record=dict(label=label,parts={},failures=[],centers=centers,pin_pitch_deg=pitch,full_task_pass=False,cycle_time_s=None)
    for part in ['ring1','ring2','pin']:
        wins=[]
        for yaw in ([135,-45,90,180]if part=='ring1'else[60,240,0,90]if part=='ring2'else[-45,0,45,90,180]):
            angles=dict(yaws,**{part:yaw});props,solids,stocks=make(centers,angles,pitch);token=hashlib.sha256(json.dumps([centers,angles,pitch]).encode()).hexdigest();world=lib.scene(props,stocks,part,token);s=next(s for s in stocks if s['part']==part and s['index']==0);c,why=lib.pick(part,np.array(s['P']),np.array(s['R']),world)
            print('VERTICAL_RACK_PICK',label,part,yaw,c is not None,why,flush=True)
            if c:wins.append(dict(c,stock_yaw_deg=yaw))
        if not wins:record['failures'].append(dict(part=part,stage='pickup_and_extraction'));break
        record['parts'][part]=min(wins,key=lambda c:c['endpoint_motion_score_s']);yaws[part]=record['parts'][part]['stock_yaw_deg']
    props,solids,stocks=make(centers,yaws,pitch);record.update(geometry=solids,stock_parts=stocks,stock_yaws_deg=yaws,maximum_loaded_height_above_table_m=max(level+.0375,pinlevel+.019)-TOP)
    collisions=[]
    for name,obj in props:
        data=fcl.CollisionData();fe._baseline.collide(obj,data,fcl.defaultCollisionCallback)
        if data.result.is_collision:collisions.append(name)
    record['static_M80_intersections']=collisions;record['all_pickups_screened']=len(record['parts'])==3 and not collisions and record['maximum_loaded_height_above_table_m']<=.200001
    report['cases'].append(record);OUT.write_text(json.dumps(report,indent=2));print('VERTICAL_RACK_CASE',label,record['all_pickups_screened'],collisions,flush=True)
    if sum(c['all_pickups_screened']for c in report['cases'])>=3:break
print('VERTICAL_RACK_SEARCH_DONE',flush=True)
