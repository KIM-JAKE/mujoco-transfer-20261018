import ast,copy,json
from pathlib import Path
O=Path(__file__).parent
base=ast.parse(Path('/tmp/fixed521-pin-entry-axis-view/overrides/direct_pin_skill.py').read_text());tree=ast.parse((O/'overrides/direct_pin_skill.py').read_text())
class Remove(ast.NodeTransformer):
 def visit_FunctionDef(self,n):
  if n.name in ['_pin_carry_aperture_domain','_pin_check_carry_aperture','_pin_admit_measured_carry','carry_opening']:return None
  return self.generic_visit(n)
 def visit_Assign(self,n):
  if any(isinstance(t,ast.Name)and t.id=='carry_domain'for t in n.targets):return None
  return self.generic_visit(n)
 def visit_If(self,n):
  if any(isinstance(x,ast.Name)and x.id=='carry_domain'for x in ast.walk(n.test)):return None
  if any(isinstance(x,ast.Constant)and x.value=='pin_measured_carry_aperture_guard'for x in ast.walk(n.test)):return None
  return self.generic_visit(n)
 def visit_Call(self,n):
  if isinstance(n.func,ast.Name)and n.func.id=='carry_opening':return ast.Constant(value=.0105)
  return self.generic_visit(n)
assert ast.dump(Remove().visit(tree))==ast.dump(base)
(O/'archival-AST-equivalence.json').write_text(json.dumps({'PASS':True,'entire_base_AST_restored_after_removing_new_guard_helpers_and_optin_branches':True,'camera_GM_two_stage_original_protected':True},indent=2)+'\n')
print('PASS original whole-source AST')
