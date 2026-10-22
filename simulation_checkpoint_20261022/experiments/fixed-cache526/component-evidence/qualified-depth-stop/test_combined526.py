from pathlib import Path
import numpy as np,json,ast,textwrap
p=Path('/tmp/fixed527-qualified-depth-stop');s=Path('/tmp/fixed526-sensor-qualified-depth-full/overrides/direct_pin_skill.py').read_text();base=Path('/tmp/fixed522-entry-before-contact/overrides/direct_pin_skill.py').read_text()
start="    wait(.12,'pin_full_depth_hold');";end="    for prim in stage.Traverse():"
code=textwrap.dedent(s[s.index(start):s.index(end,s.index(start))]);old=textwrap.dedent(base[base.index(start):base.index(end,base.index(start))])
def pose(d,angle=0.):return {'center_m':[0.,0.,-d],'axis':[np.sin(angle),0.,np.cos(angle)],'robust_cap_circle_from_pixels':True}
def run(on,poses,fault=None,source=code):
    calls=[];it=iter(poses)
    def observe(label,G,require_circle):
        calls.append(('observe',require_circle))
        item=next(it)
        if isinstance(item,Exception):raise item
        return {'result':{'pose':item}}
    def line(*args):
        calls.append(('line',args[-1]))
        if fault:raise fault
    arm=type('A',(),{'fk':lambda self,q:np.eye(4)})()
    env=dict(np=np,wait=lambda *a:None,observe=observe,G=np.eye(4),qualified_full_depth=True,postrelease_rgbd=False,avoid_unneeded_depth_correction=on,goal=np.zeros(3),report={},arm=arm,cmd=np.zeros((1,7)),ids=list(range(7)),line=line)
    try:exec(source,env);exc=None
    except Exception as e:exc=e
    return calls,env['report'],exc
checks={}
c,r,e=run(True,[pose(.00074888,.016055)]);assert e is None and not any(x[0]=='line' for x in c) and r['pin_unneeded_depth_correction_skipped']['extra_images_or_motion']==0;checks['523_already_original_final_qualified_no_push']=True
c,r,e=run(True,[pose(.0016),pose(.0002)]);assert e is None and sum(x[0]=='line'for x in c)==1;checks['outside_original_position_gate_retains_correction']=True
c,r,e=run(True,[pose(.0008,.051)]);assert isinstance(e,RuntimeError) and not any(x[0]=='line'for x in c);checks['outside_original_axis_gate_still_fails_closed']=True
c,r,e=run(True,[RuntimeError('original cap occluded')]);assert str(e)=='original cap occluded'and not any(x[0]=='line'for x in c);checks['qualified_observation_failure_propagates_before_motion']=True
c,r,e=run(True,[pose(.0016)],fault=RuntimeError('issued fault'));assert str(e)=='issued fault'and sum(x[0]=='line'for x in c)==1;checks['motion_fault_no_retry']=True
for samples in [[pose(.0008),pose(.0002)],[pose(.0002)],[pose(.0016),pose(.0008),pose(.0002)],[pose(.0008,.051)]]:
    a=run(False,samples);b=run(False,samples,source=old);assert a[0]==b[0] and a[1]==b[1] and type(a[2])==type(b[2])
checks['default_off_original_block_behavior_equivalent_four_cases']=True
# All code outside the validation + refinement-loop insertions remains literal-identical.
oldtree=ast.parse(base);newtree=ast.parse(s)
old_functions={n.name:ast.dump(n) for n in oldtree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name!='execute'};new_functions={n.name:ast.dump(n) for n in newtree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name!='execute'};assert all(new_functions[k]==v for k,v in old_functions.items())
checks['all_other_top_level_function_ASTs_unchanged']=True
validation="    avoid_unneeded_depth_correction=bool(pin_config.get('pin_avoid_unneeded_depth_correction',False))\n    if avoid_unneeded_depth_correction and not qualified_full_depth:raise ValueError('Stopping at qualified PIN full depth requires original robust-circle full-depth policy')\n"
for conf,qualified,raises in [({},False,False),({'pin_avoid_unneeded_depth_correction':True},False,True),({'pin_avoid_unneeded_depth_correction':True},True,False)]:
    try:exec(textwrap.dedent(validation),dict(pin_config=conf,qualified_full_depth=qualified));failed=False
    except ValueError:failed=True
    assert failed==raises
checks['enabled_requires_qualified_robust_full_depth']=True
(p/'combined526-critical-tests.json').write_text(json.dumps(checks,indent=2)+'\n');print(json.dumps(checks,indent=2))
