"""Exact NumPy float64 geometry identity and bounded pickle5 socket framing."""
import hashlib,pickle,struct
import numpy as np
LIMIT=268435456
SCHEMA='native-pair-numpy-float64-v1'
KEYS=('center','axes','extents','vertices','normals')

def geometry_sha256(left,right):
    h=hashlib.sha256(SCHEMA.encode())
    for side,rows in [('left',left),('right',right)]:
        h.update(side.encode());h.update(struct.pack('<I',len(rows)))
        for row in rows:
            for key in KEYS:
                a=np.ascontiguousarray(row[key],dtype='<f8')
                h.update(key.encode());h.update(struct.pack('<I',a.ndim));h.update(np.asarray(a.shape,dtype='<i8').tobytes());h.update(a.tobytes())
    return h.hexdigest()

def encode(value):
    data=pickle.dumps(value,protocol=5)
    if len(data)>LIMIT:raise ValueError('Pair request exceeds256MiB')
    return data

def recv_exact(conn,n):
    data=bytearray(n);view=memoryview(data);at=0
    while at<n:
        count=conn.recv_into(view[at:])
        if not count:raise ValueError('Incomplete binary pair frame')
        at+=count
    return data

def receive(conn,maximum=LIMIT):
    n=struct.unpack('!Q',recv_exact(conn,8))[0]
    if not 0<n<=maximum:raise ValueError('Oversized binary pair frame')
    data=recv_exact(conn,n)
    return pickle.loads(data),n

def send(conn,data):
    conn.sendall(struct.pack('!Q',len(data)));conn.sendall(data)
