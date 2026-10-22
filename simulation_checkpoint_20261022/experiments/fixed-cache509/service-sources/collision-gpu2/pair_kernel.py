"""Conservative original native convex-proxy CLEAR certificates, float64 GPU.

A false value means unknown, never collision. Only original15 OBB axes and
original convex face-normal separation are used, with1nm outward reserve.
"""
import torch,numpy as np
RESERVE=1e-9
MARGIN=.0005
class Geometry:
 def __init__(self,rows,device):
  if not 1<=len(rows)<=128:raise ValueError('1..128 original geometry rows required')
  tensors=[]
  for r in rows:
   c,a,e,v,n=[np.asarray(r[k],float)for k in ['center','axes','extents','vertices','normals']]
   if c.shape!=(3,)or a.shape!=(3,3)or e.shape!=(3,)or v.ndim!=2 or v.shape[1]!=3 or n.ndim!=2 or n.shape[1]!=3 or not len(v)or not len(n):raise ValueError('Invalid original convex proxy')
   if not np.isfinite(np.r_[c,a.ravel(),e,v.ravel(),n.ravel()]).all()or np.any(e<0)or max(abs(c).max(),abs(e).max(),abs(v).max())>10 or abs(a).max()>1.1 or abs(n).max()>2:raise ValueError('Proxy outside bounded certificate domain')
   # A subset of ORIGINAL face directions can prove separation. Missing
   # directions only cause CPU fallback, never false CLEAR.
   idx=np.unique(np.linspace(0,len(n)-1,min(64,len(n)),dtype=int));tensors.append((c,a,e,v,n[idx]))
  V=max(len(x[3])for x in tensors);F=max(len(x[4])for x in tensors);N=len(rows)
  c=np.array([x[0]for x in tensors]);a=np.array([x[1]for x in tensors]);e=np.array([x[2]for x in tensors]);v=np.zeros((N,V,3));n=np.zeros((N,F,3));vm=np.zeros((N,V),bool);nm=np.zeros((N,F),bool)
  for i,x in enumerate(tensors):v[i,:len(x[3])]=x[3];n[i,:len(x[4])]=x[4];vm[i,:len(x[3])]=True;nm[i,:len(x[4])]=True
  self.count=N
  self.c,self.a,self.e,self.v,self.n=[torch.tensor(x,device=device,dtype=torch.float64)for x in [c,a,e,v,n]]
  self.vm=torch.tensor(vm,device=device);self.nm=torch.tensor(nm,device=device)
 def transform(self,T):
  R=T[:,:,:3,:3];p=T[:,:,:3,3]
  c=torch.einsum('nlij,lj->nli',R,self.c)+p;a=R@self.a[None]
  v=torch.einsum('nlij,lvj->nlvi',R,self.v)+p[:,:,None,:];n=torch.einsum('nlij,lfj->nlfi',R,self.n)
  if torch.any(abs(v)>10).item():raise ValueError('Transformed vertices outside10m certificate domain')
  return c,a,self.e,v,n

def prove(left,right,LT,RT,device='cuda:0'):
 L=torch.as_tensor(LT,device=device,dtype=torch.float64);R=torch.as_tensor(RT,device=device,dtype=torch.float64)
 if L.ndim!=4 or R.ndim!=4 or L.shape[0]!=R.shape[0]or L.shape[1:]!=(left.count,4,4)or R.shape[1:]!=(right.count,4,4)or not 0<len(L)<=16384:raise ValueError('Invalid bounded batched transform shape')
 for T in [L,R]:
  rot=T[:,:,:3,:3]
  if not torch.isfinite(T).all().item()or torch.max(abs(T[:,:,:3,3])).item()>10 or torch.max(abs(rot)).item()>1.1 or torch.max(abs(rot.transpose(-1,-2)@rot-torch.eye(3,device=device,dtype=torch.float64))).item()>1e-5 or torch.max(abs(T[:,:,3,:]-torch.tensor([0.,0.,0.,1.],device=device,dtype=torch.float64))).item()>1e-12:raise ValueError('Transforms outside finite rigid certificate domain')
 out=[];obb_count=face_count=0
 for start in range(0,len(L),64):
  lc,la,le,lv,ln=left.transform(L[start:start+64]);rc,ra,re,rv,rn=right.transform(R[start:start+64])
  rot=la[:,:,None,:,:].transpose(-1,-2)@ra[:,None,:,:,:];ab=abs(rot)+1e-10;delta=torch.einsum('nlij,nlrj->nlri',la.transpose(-1,-2),rc[:,None,:,:]-lc[:,:,None,:]);ea=le[None,:,None,:]+MARGIN;eb=re[None,None,:,:]+MARGIN
  sep=(abs(delta)>ea+(ab@eb[...,None]).squeeze(-1)+RESERVE).any(-1)
  db=(delta[...,None,:]@rot).squeeze(-2);sep|=(abs(db)>eb+(ab.transpose(-1,-2)@ea[...,None]).squeeze(-1)+RESERVE).any(-1)
  for i in range(3):
   i1,i2=(i+1)%3,(i+2)%3
   for j in range(3):
    j1,j2=(j+1)%3,(j+2)%3;distance=abs(delta[...,i2]*rot[...,i1,j]-delta[...,i1]*rot[...,i2,j]);radius_a=ea[...,i1]*ab[...,i2,j]+ea[...,i2]*ab[...,i1,j];radius_b=eb[...,j1]*ab[...,i,j2]+eb[...,j2]*ab[...,i,j1];sep|=distance>radius_a+radius_b+RESERVE
  obb_count+=int(sep.sum().item());unknown=torch.nonzero(~sep,as_tuple=False)
  for k in range(0,len(unknown),128):
   ix=unknown[k:k+128];bi,ai,bj=ix[:,0],ix[:,1],ix[:,2];av=lv[bi,ai];bv=rv[bi,bj];clear=torch.zeros(len(ix),device=device,dtype=torch.bool)
   for normals,nmask in [(ln[bi,ai],left.nm[ai]),(rn[bi,bj],right.nm[bj])]:
    ap=av@normals.transpose(-1,-2);bp=bv@normals.transpose(-1,-2);amax=ap.masked_fill(~left.vm[ai,:,None],-torch.inf).amax(1);amin=ap.masked_fill(~left.vm[ai,:,None],torch.inf).amin(1);bmax=bp.masked_fill(~right.vm[bj,:,None],-torch.inf).amax(1);bmin=bp.masked_fill(~right.vm[bj,:,None],torch.inf).amin(1)
    clear|=(((amax+2*MARGIN+RESERVE<bmin)|(bmax+2*MARGIN+RESERVE<amin))&nmask).any(-1)
   face_count+=int(clear.sum().item());sep[bi,ai,bj]|=clear
  out.extend(sep.flatten(1).all(1).cpu().tolist())
 return out,dict(OBB_pairs_certified=obb_count,face_pairs_certified=face_count,total_rows=len(L),local_left_shapes=left.count,local_right_shapes=right.count,outward_reserve_m=RESERVE,original_pair_margin_m=2*MARGIN,float64=True)
