"""Low independent five-set rack: geometry and grasp/path screening.

No measured task time is produced here. Original insertion grasps are retained;
stock azimuth changes rotate the stocked part as well as the hand. The full
assembly still has to run, with free parts, in Isaac Sim PhysX.
"""
from pathlib import Path
import os,sys,json,copy,itertools,hashlib,faulthandler
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE));os.environ['M80_IMPORT_ENV_ONLY']='1'
faulthandler.dump_traceback_later(120,repeat=True)
import prepare_collision_targets as e
from roas_kinematics import solve
from audit_five_geometry import carrying
from payload_shapes import shape_for
cc=e.cc;fe=e.fe;fcl=cc.fcl
TABLE=dict(center=[float(os.environ.get('M80_RACK_TABLE_X','-.26')),float(os.environ.get('M80_RACK_TABLE_Y','-.18')),float(os.environ.get('M80_RACK_TABLE_HEIGHT','.7'))],size=[float(os.environ.get('M80_RACK_TABLE_WIDTH','.8')),float(os.environ.get('M80_RACK_TABLE_DEPTH','.15')),.025],top_height=float(os.environ.get('M80_RACK_TABLE_HEIGHT','.7')))
OUT=HERE/'low-rack-search.json';report=dict(scope=__doc__,storage_capacity={'ring1':5,'ring2':5,'pin':5},maximum_rack_height_above_table_m=.2,table=TABLE,duo_centered_on_ROAS=True,full_task_pass=False,cycle_time_s=None,cases=[])
base=e.ct.table(*TABLE['center'],w=TABLE['size'][0],d=TABLE['size'][1])
base.append(e.ct.shape_box('ROAS_closed_cabinet_envelope',[*e.spec['duo_world_position'][:2],.36],[.68,.66,.6]))
raw=json.load(open(HERE/'collision-global-centered.json'))['rows']
def cylinder(name,p,r,h):return(name,fcl.CollisionObject(fcl.Cylinder(r,h),fcl.Transform(np.eye(3),np.array(p))))
def save():OUT.write_text(json.dumps(report,indent=2))
def source_pose(part,P,yaw):
    s=e.ot.source['parts'][part];R=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@cc.g.checker.rot(s['quaternion']);return np.array(P),R
def geometry(centers,yaws):
    rows=[];solids=[];stocks=[]
    def box(name,p,size):rows.append(e.ct.shape_box(name,p,size));solids.append(dict(name=name,type='box',position=list(p),size=list(size)))
    def cyl(name,p,r,h):rows.append(cylinder(name,p,r,h));solids.append(dict(name=name,type='cylinder',position=list(p),radius=r,height=h))
    for part,h in [('ring1',.0155),('ring2',.014)]:
        P,R=source_pose(part,centers[part],yaws[part]);bottom=P[2]-4*h-h/2;plate_z=bottom-.003
        box(part+'_plate',[P[0],P[1],plate_z],[.09,.09,.006]);postheight=bottom-.006-.7
        box(part+'_post',[P[0],P[1],.7+postheight/2],[.02,.02,postheight]);tip=P[2]+.010
        cyl(part+'_guide',[P[0],P[1],(bottom+tip)/2],.010,tip-bottom)
        for i in range(5):stocks.append(dict(part=part,index=i,P=(P-[0,0,i*h]).tolist(),R=R.tolist()))
    P,R=source_pose('pin',centers['pin'],yaws['pin']);floor_top=P[2]-.01775;bank=np.array(P)-[.044,0,0];plate_top=floor_top-.004
    box('pin_bank_plate',[bank[0],bank[1],plate_top-.0025],[.116,.030,.005]);postheight=plate_top-.005-.7
    box('pin_bank_post',[bank[0],bank[1],.7+postheight/2],[.02,.02,postheight])
    # Passive hollow sleeves and bottom floors support every pin. No hanging.
    from stock_fixture_geometry import pin_stock_sleeve
    v,faces,original_center=pin_stock_sleeve();shape=cc.g.mesh_shape(v,faces)
    for i in range(5):
        C=P-[i*.022,0,0];stocks.append(dict(part='pin',index=i,P=C.tolist(),R=R.tolist()));box('pin_floor_'+str(i),[C[0],C[1],floor_top-.002],[.02,.02,.004])
        center=C+[0,0,-.01475];rows.append(('pin_sleeve_'+str(i),fcl.CollisionObject(shape,fcl.Transform(np.eye(3),center))));solids.append(dict(name='pin_sleeve_'+str(i),type='hollow_sleeve',position=center.tolist(),inner_radius=.0026,outer_radius=.005,height=.006))
    return rows,solids,stocks
def scene(props,stocks,held,token):
    rows=base+props
    for item in stocks:
        if item['part']==held and item['index']==0:continue
        rows.append(('stored_'+item['part']+'_'+str(item['index']),fcl.CollisionObject(shape_for(item['part']),fcl.Transform(np.array(item['R']),np.array(item['P'])))))
    for part,(P,R)in e.original_goals.items():
        if part!=held:rows.append(('installed_'+part,fcl.CollisionObject(shape_for(part),fcl.Transform(R,P))))
    return fe.FixedScene(rows,('low_rack_v1',token,held))
def screen_path(side,path,gap):
    for a,b in zip(path[:-1],path[1:]):
        okay,why=cc.path_clear(side,[a,b],gap,max(3,int(np.ceil(np.max(abs(a-b))/.015))+1))
        if not okay:return False,why
    return True,None
def pick(part,P,Rp,world):
    r=e.ot.original_recipe(part);side=r['side'];R=Rp@np.array(r['localR']);tcp=P+Rp@np.array(r['localP']);opening=.077 if part=='ring1'else .08 if part=='ring2'else .02
    seeds=[np.array(c['q'])for row in raw if row['part']==part and row['stage']=='pickup'for c in row['candidates']]
    rng=np.random.default_rng(31);seeds += [np.array([0,-.785,0,-2.356,0,1.571,.785])]+[rng.uniform(cc.LOWER+.05,cc.UPPER-.05)for _ in range(40 if part=='pin'else 16)]
    fe.set_env(world);best=None;failures=[]
    for seed in seeds:
        q=solve(side,tcp,R,seed)
        if q is None:continue
        okay,why=fe.clear(side,q,opening)
        if not okay:failures.append(why);continue
        path=[q]
        for rise in np.linspace(.002,.025,9):
            b=solve(side,tcp+(Rp[:,2]if part=='pin'else np.array([0,0,1]))*rise,R,path[-1])
            if b is None:break
            path.append(b)
        if len(path)!=10:continue
        okay,why=screen_path(side,path,opening)
        if not okay:failures.append(why);continue
        r.update(source_part_P=P.tolist(),source_part_R=Rp.tolist(),tcp=tcp.tolist(),R=R.tolist())
        with carrying(r):okay,why=screen_path(side,path[1:],r['gap'])
        if not okay:failures.append(why);continue
        # Optimistic endpoint score ranks candidates only; no transfer pass.
        score=min(cc.segment_time(path[-1],np.array(c['q']))for row in raw if row['part']==part and row['stage']=='insert'for c in row['candidates'])
        candidate=dict(recipe=r,q_pick=q.tolist(),q_pre=path[-1].tolist(),lift_q=[q.tolist()for q in path],opening_gap_m=opening,endpoint_motion_score_s=score,full_transfer_validated=False)
        if best is None or score<best['endpoint_motion_score_s']:best=candidate
        if best is not None:break
    return best,failures[-3:]
def main():
 for level,y in itertools.product([.88,.85,.82],[-.18,-.20,-.16]):
    centers={'ring1':[0,y,level],'ring2':[-.35,y,level],'pin':[-.52,y,.88]};yaws={p:0 for p in centers};record=dict(label=f'low_z{round(level*1000)}_y{round(y*1000)}',centers=centers,parts={},failures=[],physics_pass=False,cycle_time_s=None)
    for part in ['ring1','ring2','pin']:
        wins=[]
        for yaw in [0,45,-45,90,-90,180]:
            angles=dict(yaws,**{part:yaw});props,solids,stocks=geometry(centers,angles);token=hashlib.sha256(json.dumps([centers,angles]).encode()).hexdigest();world=scene(props,stocks,part,token)
            c,why=pick(part,*source_pose(part,centers[part],yaw),world)
            print('LOW_RACK_PICK',record['label'],part,yaw,c is not None,why,flush=True)
            if c:wins.append(dict(c,stock_yaw_deg=yaw))
        if not wins:record['failures'].append(dict(part=part,stage='pickup_and_extraction'));break
        record['parts'][part]=min(wins,key=lambda c:c['endpoint_motion_score_s']);yaws[part]=record['parts'][part]['stock_yaw_deg']
    props,solids,stocks=geometry(centers,yaws);record.update(geometry=solids,stock_parts=stocks,stock_yaws_deg=yaws,rack_height_above_table_m=max(c['position'][2]+(c.get('height',0)/2 if c['type']!='box'else c['size'][2]/2)for c in solids)-.7)
    record['all_pickups_screened']=len(record['parts'])==3 and record['rack_height_above_table_m']<=.200001
    report['cases'].append(record);save();print('LOW_RACK_CASE',record['label'],record['all_pickups_screened'],flush=True)
    if sum(c['all_pickups_screened']for c in report['cases'])>=3:break
 print('LOW_RACK_SEARCH_DONE',flush=True)
if __name__=='__main__':main()
