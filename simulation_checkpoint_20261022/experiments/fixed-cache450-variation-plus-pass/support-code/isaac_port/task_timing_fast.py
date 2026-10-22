"""Concurrent transport with time reserved for measured precision insertion."""
LEFT_WAIT=4.886333333333333
LEFT_DELAY=1.2
RIGHT_WAIT=7.4
LEFT_RELEASED=7.303+LEFT_DELAY
RIGHT_DELAY=LEFT_RELEASED-RIGHT_WAIT
BASE_TOTAL=8.060333333333332+RIGHT_DELAY
EARLY_RATE=1.6
INSERT_RATE=.6
POST_RATE=1.9
ALIGN_DWELL=.15
RATE=EARLY_RATE
B1=6.486333333333333+LEFT_DELAY
B2=7.153+LEFT_DELAY
PICK_BEGIN=4.4
PICK_END=LEFT_WAIT
PICK_RATE=.8
PICK_T1=PICK_BEGIN/EARLY_RATE
PICK_T2=PICK_T1+(PICK_END-PICK_BEGIN)/PICK_RATE
PICK_DELAY=(PICK_END-PICK_BEGIN)*(1/PICK_RATE-1/EARLY_RATE)
T1=B1/EARLY_RATE+PICK_DELAY
T2=T1+ALIGN_DWELL+(B2-B1)/INSERT_RATE
TOTAL=T2+(BASE_TOTAL-B2)/POST_RATE
TIMING_PARAMETERS=dict(transport_source_rate=EARLY_RATE,insertion_source_rate=INSERT_RATE,button_source_rate=POST_RATE,pin_alignment_dwell_s=ALIGN_DWELL,pin_pickup_source_interval=[PICK_BEGIN,PICK_END],pin_pickup_source_rate=PICK_RATE)

def base_time(t):
    if t<PICK_T1:return t*EARLY_RATE
    if t<PICK_T2:return PICK_BEGIN+(t-PICK_T1)*PICK_RATE
    if t<T1:return (t-PICK_DELAY)*EARLY_RATE
    if t<T1+ALIGN_DWELL:return B1
    if t<T2:return B1+(t-T1-ALIGN_DWELL)*INSERT_RATE
    return min(B2+(t-T2)*POST_RATE,BASE_TOTAL)

def reference_time(n,start=False):
    if n<PICK_BEGIN:return n/EARLY_RATE
    if n<PICK_END:return PICK_T1+(n-PICK_BEGIN)/PICK_RATE
    if n<B1-1e-8:return n/EARLY_RATE+PICK_DELAY
    if abs(n-B1)<1e-8:return T1+ALIGN_DWELL if start else T1
    if n<B2:return T1+ALIGN_DWELL+(n-B1)/INSERT_RATE
    return T2+(n-B2)/POST_RATE

def source_time(side,t):
    n=base_time(t)
    if side=='left':return min(n,LEFT_WAIT)if n<LEFT_WAIT+LEFT_DELAY else min(n-LEFT_DELAY,8.060333333333332)
    return min(n,RIGHT_WAIT)if n<LEFT_RELEASED else min(n-RIGHT_DELAY,8.060333333333332)

def event_time(side,t,start=False):
    boundary=LEFT_WAIT if side=='left'else RIGHT_WAIT
    delay=LEFT_DELAY if side=='left'else RIGHT_DELAY
    n=t+delay if t>boundary+1e-8 or(start and abs(t-boundary)<1e-8)else t
    return reference_time(n,start)

BREAKPOINTS=[reference_time(n)for n in [LEFT_WAIT,LEFT_WAIT+LEFT_DELAY,RIGHT_WAIT,LEFT_RELEASED,B1,B2,PICK_BEGIN,PICK_END]]+[T1+ALIGN_DWELL]
