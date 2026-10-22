"""Replan the pin with its measured friction grasp in the live Isaac cell."""
import numpy as np
import json,os
from pathlib import Path
from isaac_kinematics import frame,exp
from isaac_motion import duration
from isaac_collision_queries import Checker
from pin_axis_ik import solve as solve_axis

def plan(stage,arm,commands,clock,indices,k,G,paths,opening,reference,part_index):
    q=commands[:,indices].copy();original=q.copy();tt=reference['time_s'];parts=reference['parts'][:,part_index];routes=[]
    checker=Checker(stage,arm,'right');checker.ignored_future_actors={str(p.GetPath())for p in stage.Traverse()if p.HasAPI(__import__('pxr').UsdPhysics.RigidBodyAPI)and('left_arm_'in str(p.GetPath())or'left_NativeHand'in str(p.GetPath()))}|{paths['workpiece_pin']}
    def at(st):return int(np.searchsorted(clock,st))
    def part_position(st):return np.array([np.interp(st,tt,parts[:,j])for j in range(3)])
    pre=at(79.20);contact=at(81.10);release=at(81.82);withdraw=at(83.89);empty=at(85.5);bevel=at(93.50)
    P=part_position(79.30);endP=part_position(81.50);here=Path(__file__).resolve().parent;goalxy=json.load(open(here.parents[1]/'runs'/os.environ['M80_ROAS_RUN']/'arm-plan.json'))['goals']['pin']['P'];P[:2]=goalxy[:2];endP[:2]=goalxy[:2]
    if os.environ.get('M80_ROAS_RGB_ONLY')=='1':P[1]-=.00025;endP[1]-=.00025
    seed=original[pre];choices=[]
    for yaw in [0.,-.3,.3,-.6,.6,-1.,1.,-1.5,1.5,-2.,2.,np.pi]:
        target=frame(P,exp(np.array([0.,0.,yaw])))@np.linalg.inv(G);goal=arm.solve(target[:3,3],target[:3,:3],seed)
        if goal is None:continue
        endtarget=frame(endP,exp(np.array([0.,0.,yaw])))@np.linalg.inv(G);end=arm.solve(endtarget[:3,3],endtarget[:3,:3],goal)
        if end is None:continue
        if not checker.clear(goal,opening,paths['workpiece_pin'])[0]or not checker.clear(end,opening,paths['workpiece_pin'])[0]:continue
        choices.append((duration([commands[k,indices],goal],.1),yaw,goal,end))
    if not choices:raise RuntimeError('No collision-free vertical-pin preinsert/partial-insert IK')
    found=None
    for score,yaw,goal,end in sorted(choices,key=lambda v:v[0]):
        checker.attach_payload(paths['workpiece_pin'],frame([0,0,.1034],np.eye(3))@G);route=checker.route(commands[k,indices],goal,opening,paths['workpiece_pin'],seconds=5);checker.detach_payload()
        if route is not None:found=(score,yaw,goal,end,route);break
    if found is None:raise RuntimeError('No live pin carry route')
    score,yaw,goal,end,route=found
    def assign(a,b,path,label):
        path=np.array(path);weights=np.array([max(.025,np.linalg.norm(B-A))for A,B in zip(path[:-1],path[1:])]);knots=np.r_[0,np.cumsum(weights)];knots/=knots[-1];u=np.linspace(0,1,b-a+1);segment=np.clip(np.searchsorted(knots,u,side='right')-1,0,len(path)-2);local=(u-knots[segment])/(knots[segment+1]-knots[segment]);h=local**3*(10-15*local+6*local*local);q[a:b+1]=path[segment]+(path[segment+1]-path[segment])*h[:,None];q[b+1:]=path[-1];routes.append(dict(label=label,start_source_s=float(clock[a]),end_source_s=float(clock[b]),waypoints=path.tolist()))
    assign(k,pre,route,'pin_carry_with_observed_grasp')
    # 5 mm shaft insertion, vertical axis; the pneumatic surrogate holds it.
    line=[];last=goal.copy()
    for u in np.linspace(0,1,10):
        H=frame(P*(1-u)+endP*u,exp(np.array([0.,0.,yaw])))@np.linalg.inv(G);sol=arm.solve(H[:3,3],H[:3,:3],last)
        if sol is None or not checker.clear(sol,opening,paths['workpiece_pin'])[0]:raise RuntimeError('Vertical pin insertion robot clearance failed')
        line.append(sol);last=sol
    end=line[-1];assign(pre,contact,line,'pin_partial_vertical_insert');q[contact:release+1]=end
    # Keep the fingers open before withdrawing along the pin axis.
    H=arm.fk(end);down=H.copy();down[:3,3]+=[0,0,-.025];downq=arm.solve(down[:3,3],down[:3,:3],end)
    if downq is None:raise RuntimeError('Pin axial withdrawal IK')
    if not checker.path([end,downq],.022,paths['workpiece_pin'])[0]:raise RuntimeError('Pin axial withdrawal robot clearance')
    assign(at(82.39),withdraw,[end,downq],'pin_clear_down');q[withdraw:empty+1]=downq
    route=checker.route(downq,original[bevel],.08,paths['workpiece_pin'],seconds=8)
    if route is None:raise RuntimeError('Pin empty hand route to final pusher')
    assign(empty,bevel,route,'pin_empty_to_original_pusher');q[bevel+1:]=original[bevel+1:]
    return q,dict(planning_only=True,full_task_pass=False,pin_target_axis='vertical',pin_grasp_measured=True,selected_yaw_rad=yaw,routes=routes)
