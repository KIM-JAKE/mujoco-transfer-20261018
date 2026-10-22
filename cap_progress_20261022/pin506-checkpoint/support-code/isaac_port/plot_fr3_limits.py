"""Plot unsmoothed command and measured kinematics against official FR3 bounds."""
from pathlib import Path
import argparse,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from fr3_motion_control import position_velocity_bounds

p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();run=args.run
z=np.load(run/'joint-motion.npz');t=z['time_s'];dt=np.diff(t)
report=json.loads((run/'fr3-limits.json').read_text())
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True,constrained_layout=True)
for mode,color in [('command','#2368ad'),('actual','#df641c')]:
    q=z[mode+'_position'];v=z[mode+'_velocity'];a=z['command_acceleration']if mode=='command'else np.diff(v,axis=0)/dt[:,None]
    jerk=np.diff(a,axis=0)/dt[-(len(a)-1):,None]
    lo,hi=position_velocity_bounds(q.reshape(-1,2,7));lo=lo.reshape(-1,14);hi=hi.reshape(-1,14)
    cap=np.where(v>=0,hi,-lo);vr=np.divide(abs(v),cap,out=np.zeros_like(v,dtype=float),where=cap>1e-8)
    axes[0].plot(t,np.max(vr,axis=1),color=color,lw=1,label=mode)
    axes[1].plot(t[-len(a):],np.max(abs(a),axis=1)/10,color=color,lw=1)
    axes[2].plot(t[-len(jerk):],np.max(abs(jerk),axis=1)/5000,color=color,lw=1)
for ax,label in zip(axes,['Velocity / position-dependent limit','Acceleration / 10 rad/s²','Jerk / 5000 rad/s³']):
    ax.axhline(1,color='#b62626',ls='--',lw=1.3);ax.set_ylabel(label);ax.grid(alpha=.2);ax.set_ylim(bottom=0)
    if report.get('planning_budget',{}):ax.axhline(report['planning_budget']['fraction'],color='#737373',ls=':',lw=1,label='95% command ceiling'if ax is axes[0]else None)
axes[0].legend(loc='upper right');axes[-1].set_xlabel('Measured simulation time (s), including initial settling and final retention check')
fig.suptitle(f'FR3 joint limits — {report["state"].upper()} | maximum across both arms, 480 Hz, no smoothing')
fig.savefig(run/'fr3-limits.png',dpi=180);fig.savefig(run/'fr3-limits.pdf');plt.close(fig)
print(run/'fr3-limits.png')
