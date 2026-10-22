"""Restore source robot collision exclusions without disabling self collision."""
import json
from pxr import UsdPhysics

def apply(stage,root,body_paths):
 data=json.loads((root/'generated/contact_task/robot-filters.json').read_text())
 for row in data['pairs']:
  UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(body_paths[row['body0']])).CreateFilteredPairsRel().AddTarget(body_paths[row['body1']])
 return dict(robot_collision_exclusions=len(data['pairs']),robot_self_collision=True)
