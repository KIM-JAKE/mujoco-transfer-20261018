"""GPU proposal scores only; final RGB-D size/sign/annulus gates stay on CPU."""
import numpy as np,time
import warp as wp
wp.set_module_options({'enable_backward':False,'fast_math':False,'fuse_fp':False})
@wp.kernel
def _planes(points:wp.array2d(dtype=wp.float64),normals:wp.array2d(dtype=wp.float64),levels:wp.array(dtype=wp.float64),scores:wp.array(dtype=wp.int32)):
    i,j=wp.tid()
    d=points[i,0]*normals[j,0]+points[i,1]*normals[j,1]+points[i,2]*normals[j,2]-levels[j]
    if wp.abs(d)<wp.float64(0.00007):wp.atomic_add(scores,j,1)
@wp.kernel
def _circles(edge:wp.array2d(dtype=wp.float64),hyp:wp.array2d(dtype=wp.float64),radius:wp.float64,scores:wp.array(dtype=wp.int32)):
    i,j=wp.tid()
    dx=edge[j,0]-hyp[i,0];dy=edge[j,1]-hyp[i,1]
    d=wp.abs(wp.sqrt(dx*dx+dy*dy)-radius)
    if d<wp.float64(0.0012):wp.atomic_add(scores,i,1)
_CACHE={};STATS=dict(plane_calls=0,circle_calls=0,gpu_call_s=0.,cpu_fallback_calls=0)
def _array(key,a):
    a=np.ascontiguousarray(a,dtype=np.float64);k=(key,a.shape)
    if k not in _CACHE:_CACHE[k]=wp.array(a,dtype=wp.float64,device='cuda:0')
    else:_CACHE[k].assign(a)
    return _CACHE[k]
def _scores(n):
    k=('scores',n)
    if k not in _CACHE:_CACHE[k]=wp.zeros(n,dtype=wp.int32,device='cuda:0')
    else:_CACHE[k].zero_()
    return _CACHE[k]
def plane_scores(points,normals,levels):
    start=time.perf_counter()
    try:
        p=_array('points',points);n=_array('normals',normals);l=_array('levels',levels);s=_scores(len(normals));wp.launch(_planes,(len(points),len(normals)),inputs=[p,n,l,s],device='cuda:0');v=s.numpy();STATS['plane_calls']+=1;STATS['gpu_call_s']+=time.perf_counter()-start;return v
    except Exception:
        STATS['cpu_fallback_calls']+=1;return (abs(points@normals.T-levels)<.00007).sum(0)
def circle_scores(edge,hyp,radius):
    start=time.perf_counter()
    try:
        e=_array('edge',edge);h=_array('hyp',hyp);s=_scores(len(hyp));wp.launch(_circles,(len(hyp),len(edge)),inputs=[e,h,wp.float64(radius),s],device='cuda:0');v=s.numpy();STATS['circle_calls']+=1;STATS['gpu_call_s']+=time.perf_counter()-start;return v
    except Exception:
        STATS['cpu_fallback_calls']+=1;return (abs(np.linalg.norm(edge[None,:,:]-hyp[:,None,:],axis=2)-radius)<.0012).sum(1)
