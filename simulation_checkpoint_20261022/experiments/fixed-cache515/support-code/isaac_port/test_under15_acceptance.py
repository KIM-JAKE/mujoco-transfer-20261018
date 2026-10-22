"""A physically valid 22.88s rollout must fail the new strict 15s request."""
from pathlib import Path
from tempfile import TemporaryDirectory
import json
from evaluate_contact import evaluate
root=Path(__file__).resolve().parent;base=root/'runs/contact-fr3-95-04'
with TemporaryDirectory()as directory:
    run=Path(directory)
    for name in ['trace.json','contacts.json','status.json','fr3-limits.json','button-trace.json','recording']:
        if(base/name).exists():(run/name).symlink_to(base/name,target_is_directory=(base/name).is_dir())
    manifest=json.loads((base/'run-manifest.json').read_text());manifest['arguments']['target_15']=True
    (run/'run-manifest.json').write_text(json.dumps(manifest))
    result=evaluate(run)
    assert result['fr3_kinematic_limits_pass']and all(v['passed']for v in result['parts'].values())and result['button']['passed']
    assert result['cycle_time_required']and result['cycle_target_s']==15 and not result['cycle_time_pass']and not result['physical_simulation_pass']
    print('PASS: otherwise valid 22.88s rollout rejected by strict under-15 acceptance')
