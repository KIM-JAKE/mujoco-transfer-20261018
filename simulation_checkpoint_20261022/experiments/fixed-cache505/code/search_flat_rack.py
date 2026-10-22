"""Flat five-set trays and bottom-supported pin sockets, under 200 mm tall."""
from pathlib import Path
import sys,os,json,itertools,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import search_low_rack as lib
e=lib.e;cc=lib.cc;fe=lib.fe;fcl=lib.fcl
report=dict(scope=__doc__,table=lib.TABLE,storage_capacity={'ring1':5,'ring2':5,'pin':5},full_task_pass=False,cycle_time_s=None,cases=[],layout='Individual ring nests, three bottom supports per ring; five independent pin sleeves and floors')
OUT=HERE/'flat-rack-search.json'
def make(centers,yaws,pitch):
    rows=[];solids=[];stocks=[]
    def box(name,P,size,R=np.eye(3)):
        rows.append(e.ct.shape_box(name,P,size,R));solids.append(dict(name=name,type='box',position=list(P),size=list(size),R=R.tolist()))
    def cylinder(name,P,r,h,R=np.eye(3)):
        rows.append((name,fcl.CollisionObject(fcl.Cylinder(r,h),fcl.Transform(R,np.array(P)))));solids.append(dict(name=name,type='cylinder',position=list(P),radius=r,height=h,R=R.tolist()))
    for part,dx in [('ring1',.100),('ring2',.086)]:
        P,R=lib.source_pose(part,centers[part],yaws[part]);radius=.0375 if part=='ring1'else .031;inner=.031685 if part=='ring1'else .025255;half=.00775 if part=='ring1'else .007
        offsets=[[0,0,0],[-dx/2,.070,0],[-dx,0,0],[-1.5*dx,.070,0],[-2*dx,0,0]]if part=='ring1'else[[0,0,0],[dx/2,.070,0],[dx,0,0],[1.5*dx,.070,0],[2*dx,0,0]]
        for i,delta in enumerate(offsets):
            C=P+delta;stocks.append(dict(part=part,index=i,P=C.tolist(),R=R.tolist()))
            for j,angle in enumerate([90,210,330]):
                a=np.deg2rad(angle);direction=np.array([np.cos(a),np.sin(a),0]);post=C+direction*(inner+.0018);bottom=.7;top=C[2]-half
                cylinder(f'{part}_nest{i}_support{j}',[post[0],post[1],(bottom+top)/2],.002,top-bottom)
                # A short radial ledge and an inner-hole locating nib, below
                # the finger contact region. The part remains physically free.
                A=Rotation.from_euler('z',angle,degrees=True).as_matrix();cap=C+direction*(inner+.001);cap[2]=top-.001
                box(f'{part}_nest{i}_ledge{j}',cap,[.008,.006,.002],A)
                nib=C+direction*(inner-.0015);nib[2]=top+.0015;cylinder(f'{part}_nest{i}_locator{j}',nib,.0005,.003)
    P,R=lib.source_pose('pin',centers['pin'],yaws['pin']);R=Rotation.from_euler('y',pitch,degrees=True).as_matrix()@R
    from stock_fixture_geometry import pin_stock_sleeve
    v,faces,original=pin_stock_sleeve();shape=cc.g.mesh_shape(v,faces)
    for i in range(5):
        C=P+[i*.035,0,0];stocks.append(dict(part='pin',index=i,P=C.tolist(),R=R.tolist()));floor=C+R@np.array([0,0,-.01975]);box('pin_floor_'+str(i),floor,[.020,.020,.004],R)
        center=C+R@np.array([0,0,-.01475]);rows.append(('pin_sleeve_'+str(i),fcl.CollisionObject(shape,fcl.Transform(R,center))));solids.append(dict(name='pin_sleeve_'+str(i),type='hollow_sleeve',position=center.tolist(),inner_radius=.0026,outer_radius=.005,height=.006,R=R.tolist()))
        low=float(floor[2]-.002*np.abs(R[2,2])-.01*np.abs(R[2,0]));supportheight=low-.7
        box('pin_support_'+str(i),[floor[0],floor[1],.7+supportheight/2],[.020,.020,supportheight])
    return rows,solids,stocks

for level,pinlevel,pitch in itertools.product([.88,.85,.82],[.88,.85],[-30,0]):
    centers={'ring1':[.09,-.215,level],'ring2':[-.40,-.215,level],'pin':[-.64,-.18,pinlevel]};yaws={'ring1':225,'ring2':150,'pin':-45};label=f'flat_ring{round(level*1000)}_pin{round(pinlevel*1000)}_pitch{pitch}';record=dict(label=label,parts={},failures=[],centers=centers,pin_pitch_deg=pitch,full_task_pass=False,cycle_time_s=None)
    for part in ['ring1','ring2','pin']:
        wins=[]
        for yaw in ([225,180,270,0]if part=='ring1'else[150,180,90,0]if part=='ring2'else[-45,0,45,90,180]):
            angles=dict(yaws,**{part:yaw});props,solids,stocks=make(centers,angles,pitch);token=hashlib.sha256(json.dumps([centers,angles,pitch]).encode()).hexdigest();world=lib.scene(props,stocks,part,token)
            source=next(s for s in stocks if s['part']==part and s['index']==0);candidate,why=lib.pick(part,np.array(source['P']),np.array(source['R']),world)
            print('FLAT_RACK_PICK',label,part,yaw,candidate is not None,why,flush=True)
            if candidate:wins.append(dict(candidate,stock_yaw_deg=yaw))
        if not wins:record['failures'].append(dict(part=part,stage='pickup_and_extraction'));break
        record['parts'][part]=min(wins,key=lambda c:c['endpoint_motion_score_s']);yaws[part]=record['parts'][part]['stock_yaw_deg']
    props,solids,stocks=make(centers,yaws,pitch);record.update(geometry=solids,stock_parts=stocks,stock_yaws_deg=yaws,rack_height_above_table_m=level+.008-.7)
    collisions=[]
    for name,obj in props:
        query=fcl.CollisionData();fe._baseline.collide(obj,query,fcl.defaultCollisionCallback)
        if query.result.is_collision:collisions.append(name)
    record['static_M80_intersections']=collisions;record['all_pickups_screened']=len(record['parts'])==3 and not collisions and record['rack_height_above_table_m']<=.200001
    report['cases'].append(record);OUT.write_text(json.dumps(report,indent=2));print('FLAT_RACK_CASE',label,record['all_pickups_screened'],collisions,flush=True)
    if sum(c['all_pickups_screened']for c in report['cases'])>=3:break
print('FLAT_RACK_SEARCH_DONE',flush=True)
