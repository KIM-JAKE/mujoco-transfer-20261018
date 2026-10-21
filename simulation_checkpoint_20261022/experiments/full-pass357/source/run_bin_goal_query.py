"""One Isaac PhysX diagnostic: are the requested CAD assembly goals occupied?

Only invisible query geometry is placed at goals. Physical workpieces and the
original machine remain untouched. Core overlaps distinguish gross intrusion
from a mere boundary/contact overlap; they do not measure penetration depth.
"""
from pathlib import Path
import os,sys,json,traceback
import numpy as np
from pxr import Usd,UsdGeom,UsdUtils,PhysicsSchemaTools,Sdf,Gf
import omni.kit.app,omni.physx
HERE=Path(__file__).resolve().parent;PORT=HERE.parents[1];OUT=PORT/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
sys.path.insert(0,str(HERE))
from isaac_kinematics import world,frame
try:
    source=PORT/'runs/native-roas-table-bin-mixed-sequence09';stage=Usd.Stage.Open(str(source/'scene-authored.usda'));paths=json.load(open(source/'body-paths.json'));plan=json.load(open(source/'arm-plan.json'));rows=[]
    for part in ['ring1','ring2','pin']:
        body=paths['workpiece_'+part];root=stage.GetPrimAtPath(body+'/CPUConvexContact');goal=plan['goals'][part];Tgoal=frame(goal['P'],np.array(goal['R']));base=np.linalg.inv(world(stage,body))
        for prim in root.GetChildren():
            if not prim.IsA(UsdGeom.Mesh):continue
            mesh=UsdGeom.Mesh(prim);v=np.asarray(mesh.GetPointsAttr().Get(),float);local=base@world(stage,str(prim.GetPath()));v=v@local[:3,:3].T+local[:3,3]
            for scale in [1.,.5]:
                center=v.mean(0);vertices=center+(v-center)*scale;path='/World/GoalQueries/'+part+'_'+prim.GetName()+'_'+str(int(scale*100));m=UsdGeom.Mesh.Define(stage,path);m.CreatePointsAttr(vertices.astype(np.float32));m.CreateFaceVertexCountsAttr(mesh.GetFaceVertexCountsAttr().Get());m.CreateFaceVertexIndicesAttr(mesh.GetFaceVertexIndicesAttr().Get());m.CreateSubdivisionSchemeAttr('none');m.CreateVisibilityAttr('invisible');UsdGeom.Xformable(m).AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Matrix4d(*Tgoal.T.ravel().tolist()));rows.append((part,scale,path,PhysicsSchemaTools.encodeSdfPath(Sdf.Path(path))))
    ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);sim.simulate(1/480,0);sim.fetch_results();query=omni.physx.get_physx_scene_query_interface();hits=[]
    for part,scale,path,encoded in rows:
        def callback(hit):
            actor=str(hit.rigid_body)
            if actor.startswith('/World/Environment/m80/'):
                hits.append(dict(part=part,sector_scale=scale,query=path,actor=actor,collider=str(hit.collision)));return False
            return True
        query.overlap_mesh(*encoded,callback,False)
    report=dict(engine='Isaac Sim Omni PhysX CPU',source_scene=str(source),physical_workpieces_moved=False,original_machine_geometry_preserved=True,query_count=len(rows),overlaps=hits,parts={p:dict(boundary_overlap_sectors=sum(h['part']==p and h['sector_scale']==1 for h in hits),core_overlap_sectors=sum(h['part']==p and h['sector_scale']==.5 for h in hits),core_overlap_machine_objects=sorted({h['actor']for h in hits if h['part']==p and h['sector_scale']==.5}))for p in ['ring1','ring2','pin']},scope='Goal occupation diagnostic, not a complete assembly or penetration-depth measurement')
    (OUT/'goal-occupation-results.json').write_text(json.dumps(report,indent=2));sim.detach_stage();print('ISAAC_GOAL_OCCUPATION',report['parts'],flush=True);omni.kit.app.get_app().post_quit(0)
except Exception:
    traceback.print_exc();omni.kit.app.get_app().post_quit(1)
