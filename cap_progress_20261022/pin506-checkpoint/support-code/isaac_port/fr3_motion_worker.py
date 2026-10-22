"""Binary pipe worker: no Isaac/PhysX shared libraries are loaded here."""
import sys,json
import numpy as np
from fr3_motion_control import MotionLimiter,OffsetLimiter

stream=sys.stdin.buffer;output=sys.stdout.buffer
header=json.loads(stream.readline());kind=OffsetLimiter if header.get('mode')=='offset'else MotionLimiter;limiter=kind(header['names'],header['initial'],header['dt'])
output.write(b'R');output.flush()
while True:
    data=bytearray()
    while len(data)<28*8:
        chunk=stream.read(28*8-len(data))
        if not chunk:raise SystemExit(0)
        data.extend(chunk)
    values=np.frombuffer(data,dtype='<f8').reshape(2,14)
    limiter.set_target(values[0],values[1]);result=limiter.step()
    output.write(np.asarray(result,dtype='<f8').tobytes());output.flush()
