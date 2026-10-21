import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import copy
import shutil
from unittest.mock import patch
from field_demo.stabilization import inspect_run, import_run, ROOT, create_profile, load_profile, bind_recipe, revalidate_binding, ENGINE_FILES, PROFILE_ID


class StabilizationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.run=self.root/'completed-run';self.run.mkdir()
        code=self.run/'code_snapshot';code.mkdir();manifest={}
        for name in ('rgbd_scattered_rings.py','rgbd_geometry.py'):
            data=(ROOT/'stabilization/v119'/name).read_bytes();(code/name).write_bytes(data)
            manifest[name]=hashlib.sha256(data).hexdigest()
        (self.run/'code-manifest.json').write_text(json.dumps(manifest))
        self.report=self.run/'pipeline-results.json'
        self.report.write_text(json.dumps(dict(full_task_pass=True,hardware_tested=False,wall_verified_elapsed_s=52.3)))

    def test_import_preserves_original_and_executes_no_source(self):
        before={p:hashlib.sha256(p.read_bytes()).hexdigest() for p in self.run.rglob('*') if p.is_file()}
        result=import_run(self.run,self.root/'imported')
        self.assertTrue(result['packaged_detector_matches'])
        self.assertFalse(result['source_code_executed'])
        self.assertFalse(result['simulator_joint_trace_used_for_robot_motion'])
        self.assertEqual(before,{p:hashlib.sha256(p.read_bytes()).hexdigest() for p in before})
        self.assertTrue((self.root/'imported/IMPORT.json').is_file())

    def test_corrupt_hash_and_incomplete_result_rejected_without_output(self):
        p=self.run/'code_snapshot/rgbd_geometry.py';old=p.read_bytes();p.write_bytes(old+b'\n')
        with self.assertRaises(ValueError):import_run(self.run,self.root/'rejected')
        self.assertFalse((self.root/'rejected').exists());p.write_bytes(old)
        self.report.write_text(json.dumps(dict(full_task_pass=False,hardware_tested=False)))
        with self.assertRaises(ValueError):inspect_run(self.run)

    def test_existing_import_is_not_overwritten_and_symlink_is_not_followed(self):
        import_run(self.run,self.root/'imported')
        with self.assertRaises(FileExistsError):import_run(self.run,self.root/'imported')
        p=self.run/'code_snapshot/rgbd_geometry.py';p.unlink();p.symlink_to(ROOT/'stabilization/v119/rgbd_geometry.py')
        with self.assertRaises(ValueError):inspect_run(self.run)

    def test_new_changed_detector_is_data_not_automatically_activated(self):
        p=self.run/'code_snapshot/rgbd_geometry.py';p.write_bytes(p.read_bytes()+b'\n# newer compatible-looking source\n')
        manifest=json.loads((self.run/'code-manifest.json').read_text());manifest[p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
        (self.run/'code-manifest.json').write_text(json.dumps(manifest))
        result=import_run(self.run,self.root/'newer')
        self.assertFalse(result['packaged_detector_matches']);self.assertFalse(result['source_code_executed'])


class FieldProfileTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        self.engine=self.root/'engine'
        for name in ENGINE_FILES:
            target=self.engine/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes((ROOT/name).read_bytes())
        source=self.root/'native-roas-table-scatter-speed-v128-closer-transfer-445'
        (source/'code_snapshot').mkdir(parents=True);manifest={}
        for name in ('rgbd_scattered_rings.py','rgbd_geometry.py'):
            data=(ROOT/'stabilization/v119'/name).read_bytes();(source/'code_snapshot'/name).write_bytes(data)
            manifest[name]=hashlib.sha256(data).hexdigest()
        (source/'code-manifest.json').write_text(json.dumps(manifest))
        (source/'pipeline-results.json').write_text(json.dumps(dict(full_task_pass=True,hardware_tested=False,wall_verified_elapsed_s=43.533158828970045)))
        (source/'trial-config.json').write_text(json.dumps(dict(parallel_transfer_approach=False,path_local_timing=True,prefer_near_handover_branch=True)))
        self.imported=self.root/'v128';import_run(source,self.imported)
        create_profile(self.imported,engine_root=self.engine)
        self.profile=self.imported/'field-profile.json'
        self.recipe=dict(_root=str(self.root),parts=[dict(part=part,pickup_side='left',insert_side='right' if part!='pin' else 'left') for part in ('ring1','ring2','pin')])
        self.selection=dict(path='v128/field-profile.json',sha256=hashlib.sha256(self.profile.read_bytes()).hexdigest())

    def test_verified_profile_binds_source_engine_and_role_independent_recipe(self):
        value=load_profile(self.profile,engine_root=self.engine)
        self.assertEqual(value['profile_id'],PROFILE_ID)
        self.assertFalse(value['physical_cycle_time_claimed'])
        self.assertFalse(value['simulation_coordinates_applied'])
        self.assertFalse(value['simulation_joint_trace_replayed'])
        binding=bind_recipe(self.recipe,self.selection,engine_root=self.engine)
        self.assertEqual(binding['source_run'],'native-roas-table-scatter-speed-v128-closer-transfer-445')
        self.assertGreater(len(binding['dependency_files']),len(ENGINE_FILES))
        self.assertEqual(revalidate_binding(binding,engine_root=self.engine)['profile_id'],PROFILE_ID)

    def test_changed_engine_source_or_selected_profile_rejected(self):
        engine=self.engine/ENGINE_FILES[0];engine.write_bytes(engine.read_bytes()+b'\n# edited engine\n')
        with self.assertRaisesRegex(ValueError,'dependency changed'):load_profile(self.profile,engine_root=self.engine)
        engine.write_bytes((ROOT/ENGINE_FILES[0]).read_bytes())
        source=self.imported/'rgbd_geometry.py';source.write_bytes(source.read_bytes()+b'\n# edited reference\n')
        with self.assertRaisesRegex(ValueError,'dependency changed'):load_profile(self.profile,engine_root=self.engine)
        source.write_bytes((ROOT/'stabilization/v119/rgbd_geometry.py').read_bytes())
        self.profile.write_bytes(self.profile.read_bytes()+b'\n')
        with self.assertRaisesRegex(ValueError,'profile changed'):bind_recipe(self.recipe,self.selection,engine_root=self.engine)

    def test_same_detector_hash_is_required_even_after_declared_engine_hash_updated(self):
        detector=self.engine/'src/field_demo/stabilized/rgbd_geometry.py';detector.write_bytes(detector.read_bytes()+b'\n# different detector\n')
        p=json.loads(self.profile.read_text());p['runtime_files']['src/field_demo/stabilized/rgbd_geometry.py']=hashlib.sha256(detector.read_bytes()).hexdigest()
        self.profile.write_text(json.dumps(p))
        with self.assertRaisesRegex(ValueError,'executing port'):load_profile(self.profile,engine_root=self.engine)

    def test_incompatible_part_order_or_transfer_roles_cannot_claim_v128(self):
        bad=copy.deepcopy(self.recipe);bad['parts'][0]['insert_side']='left'
        with self.assertRaisesRegex(ValueError,'handover'):bind_recipe(bad,self.selection,engine_root=self.engine)
        bad=copy.deepcopy(self.recipe);bad['parts'][2]['insert_side']='right'
        with self.assertRaisesRegex(ValueError,'direct'):bind_recipe(bad,self.selection,engine_root=self.engine)
        bad=copy.deepcopy(self.recipe);bad['parts'].reverse()
        with self.assertRaisesRegex(ValueError,'phase order'):bind_recipe(bad,self.selection,engine_root=self.engine)

    def test_source_time_and_false_hardware_claims_are_checked(self):
        original=json.loads(self.profile.read_text())
        for key,value in [('physical_cycle_time_claimed',True),('simulation_wall_elapsed_s',1.0)]:
            modified=copy.deepcopy(original);modified[key]=value;self.profile.write_text(json.dumps(modified))
            with self.assertRaises(ValueError):load_profile(self.profile,engine_root=self.engine)

    def test_runtime_observation_records_exact_selected_profile_dependencies(self):
        from field_demo.random_cycle import LiveRuntime
        binding=bind_recipe(self.recipe,self.selection,engine_root=self.engine)
        runtime=LiveRuntime.__new__(LiveRuntime);runtime.recipe={'_stabilization_binding':binding}
        runtime.output=self.root;runtime.installed_parts=[]
        record=runtime._record(dict(state='test_observation'))
        saved=json.loads(Path(record['_path']).read_text())
        self.assertEqual(saved['stabilization_profile']['profile_id'],PROFILE_ID)
        self.assertEqual(saved['stabilization_dependency_files'],binding['dependency_files'])
        self.assertEqual(hashlib.sha256(Path(record['_path']).read_bytes()).hexdigest(),record['_sha256'])



if __name__=='__main__':unittest.main()
