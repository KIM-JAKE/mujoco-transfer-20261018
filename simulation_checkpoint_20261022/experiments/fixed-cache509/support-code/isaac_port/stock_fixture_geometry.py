"""Assumed rigid tip sleeve for the source's upright pin supply socket."""
import numpy as np

def pin_stock_sleeve():
    n=48;inner=.0026;outer=.005;height=.006
    rings=[]
    for z,r in [(-height/2,inner),(-height/2,outer),(height/2,inner),(height/2,outer)]:
        angles=np.arange(n)*2*np.pi/n;rings.extend(np.c_[r*np.cos(angles),r*np.sin(angles),np.full(n,z)])
    faces=[]
    for j in range(n):
        k=(j+1)%n;ib,ob,it,ot=j,n+j,2*n+j,3*n+j;ik,ok,tk,qk=k,n+k,2*n+k,3*n+k
        faces.extend([[ob,ok,qk],[ob,qk,ot],[ib,it,tk],[ib,tk,ik],[ib,ok,ob],[ib,ik,ok],[it,ot,qk],[it,qk,tk]])
    return np.asarray(rings),np.asarray(faces,dtype=np.int32),np.array([-.55475,-.16,1.28525])
