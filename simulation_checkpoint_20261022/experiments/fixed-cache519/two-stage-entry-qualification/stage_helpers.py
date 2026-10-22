from pathlib import Path
import ast,difflib,hashlib,json
O=Path(__file__).parent;B=Path('/tmp/fixed517-preclose-gain-full/overrides');H=Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/native_hand_layout/roas-table');records={}
def save(name,base,s):
 old=base.read_text();ast.parse(s);p=O/'overrides'/name;p.write_text(s);(O/(name+'.diff')).write_text(''.join(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(base),tofile=str(p))));records[name]=dict(base=str(base),base_sha256=hashlib.sha256(old.encode()).hexdigest(),sha256=hashlib.sha256(s.encode()).hexdigest())
p=H/'pin_terminal_geometry.py';s=p.read_text();s=s.replace('def check_terminal(ctx,arm,checker,q,depth,opening,selected,margin=.03):','def check_terminal(ctx,arm,checker,q,depth,opening,selected,margin=.03,*,entry_stage=False,withdraw_H=None,requested_s=.8):')
s=s.replace("    if q.shape!=(7,)or not np.isfinite(q).all()or not .035<=depth<=.055 or not .0<=opening<=.08:raise ValueError('Invalid terminal stroke input')",'''    depth_valid=.014<=depth<=.022 if entry_stage else .035<=depth<=.055
    if q.shape!=(7,)or not np.isfinite(q).all()or not depth_valid or not .0<=opening<=.08:raise ValueError('Invalid terminal stroke input')
    if entry_stage:
        withdraw_H=np.asarray(withdraw_H,float);requested_s=float(requested_s)
        if withdraw_H.shape!=(4,4)or not np.isfinite(withdraw_H).all()or not .01<requested_s<=.8:raise ValueError('Entry terminal requires original saved far withdrawal target and bounded stroke timing')
    elif withdraw_H is not None or requested_s!=.8:raise ValueError('Original far terminal policy retains original timing/withdrawal')''')
s=s.replace('    def issued(points,fingers):\n        curve,_,_=retime(points,.8,DT,','    def issued(points,fingers,requested=.8):\n        curve,_,_=retime(points,requested,DT,')
s=s.replace("    stroke=issued(rows,width);withdraw=issued(rows[::-1],np.full(2,.009))",'''    stroke=issued(rows,width,requested_s)
    if entry_stage:
        seated=arm.fk(stroke[-1]);withdraw_rows=[stroke[-1].copy()]
        for u in np.arange(1,7)/6:
            target=seated.copy();target[:3,3]=(1-u)*seated[:3,3]+u*withdraw_H[:3,3]
            nextq=arm.solve(target[:3,3],seated[:3,:3],withdraw_rows[-1])
            if nextq is None:return False,dict(reason='full_far_withdraw_IK',fraction=float(u))
            withdraw_rows.append(nextq)
        withdraw=issued(withdraw_rows,np.full(2,.009),.65)
        withdrawal_margin=float(np.minimum(withdraw-bounds[:,0],bounds[:,1]-withdraw).min())
        if withdrawal_margin<margin:return False,dict(reason='far_withdraw_joint_margin',minimum_rad=withdrawal_margin,required_rad=float(margin))
    else:withdraw=issued(rows[::-1],np.full(2,.009))''')
s=s.replace("all_issued_FR3_array_checks_retained=True,uses_free_body_poses=False,robot_encoder_other_arm=True,actual_execution_not_performed=True)","all_issued_FR3_array_checks_retained=True,uses_free_body_poses=False,robot_encoder_other_arm=True,actual_execution_not_performed=True,entry_stage=bool(entry_stage),saved_far_withdrawal_checked=bool(entry_stage))")
save(p.name,p,s)
p=B/'pin_current_terminal.py';s=p.read_text();s=s.replace('def assess(ctx,body_G,depth,q=None):','def assess(ctx,body_G,depth,q=None,*,entry_stage=False,withdraw_H=None,requested_s=.8):');s=s.replace("ok,evidence=check_terminal(ctx,arm,checker,q,float(depth),float(max(widths)),ctx['paths']['workpiece_pin'],margin=.03)","ok,evidence=check_terminal(ctx,arm,checker,q,float(depth),float(max(widths)),ctx['paths']['workpiece_pin'],margin=.03,**(dict(entry_stage=True,withdraw_H=withdraw_H,requested_s=requested_s)if entry_stage else{}))");s=s.replace('field,timing=_timed(ctx,knots,.8,False)','field,timing=_timed(ctx,knots,requested_s,False)');s=s.replace('def plan_rebranch(ctx,body_G,depth):','def plan_rebranch(ctx,body_G,depth,*,entry_stage=False,withdraw_H=None,requested_s=.8):');s=s.replace('ok,terminal=assess(ctx,body_G,depth,q)','ok,terminal=assess(ctx,body_G,depth,q,**(dict(entry_stage=True,withdraw_H=withdraw_H,requested_s=requested_s)if entry_stage else{}))');save(p.name,p,s)
(O/'helper-manifest.json').write_text(json.dumps(records,indent=2)+'\n')
