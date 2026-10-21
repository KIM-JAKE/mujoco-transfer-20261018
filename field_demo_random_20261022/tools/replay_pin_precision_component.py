"""Portable SIM308 formula replay only; no SDK, camera or robot connection."""
from pathlib import Path
import json
import numpy as np
from field_demo.pin_precision import correction,errors

def main():
    root=Path(__file__).resolve().parents[1]
    fixture=json.loads((root/'replay/pin-precision308/frame-fixtures.json').read_text())
    differences=[];records=[]
    for row in fixture['rows']:
        evidence=errors(row['RGBD_CAD_body_pose'],fixture['goal_body'])
        if not evidence['aligned']:
            target,_=correction(row['command_SIM_encoder_FK_TCP'],row['actual_SIM_encoder_TCP'],row['RGBD_body_G'],row['RGBD_CAD_body_pose'],fixture['goal_body'],gain=.5)
            delta=float(np.max(abs(target-np.asarray(row['expected_command_target_from_SIM308_formula']))));assert delta<1e-9;differences.append(delta)
        records.append(dict(image_index=row['image_index'],**evidence))
    assert records[-1]['aligned']
    print(json.dumps(dict(scope='Recorded SIM308 pixel-fitted poses/encoder frames; field ICP not executed; no actual SDK/hardware',hardware_tested=False,field_ready=False,formula_replayed=True,maximum_matrix_error=max(differences),prealignment_frames=records,source_SIM_pin_component=fixture['SIM_pin_component_state'],source_SIM_full_task_pass=fixture['SIM_source_full_task_pass'],source_SIM_postinsert_axial_depth_corrections=fixture['SIM_postinsert_axial_depth_corrections'],physical_field_convergence_verified=False),indent=2))

if __name__=='__main__':main()
