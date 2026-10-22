"""Serialize the empty right-hand return and left pin transport."""
LEFT_WAIT=4.886333333333333
RIGHT_WAIT=7.4
LEFT_DELAY=RIGHT_WAIT-LEFT_WAIT
LEFT_RELEASED=7.303+LEFT_DELAY
RIGHT_DELAY=LEFT_RELEASED-RIGHT_WAIT
TOTAL=8.060333333333332+RIGHT_DELAY

def source_time(side,t):
    if side=='left':
        return min(t,LEFT_WAIT)if t<RIGHT_WAIT else min(t-LEFT_DELAY,8.060333333333332)
    return min(t,RIGHT_WAIT)if t<LEFT_RELEASED else min(t-RIGHT_DELAY,8.060333333333332)

def event_time(side,t,start=False):
    boundary=LEFT_WAIT if side=='left'else RIGHT_WAIT
    delay=LEFT_DELAY if side=='left'else RIGHT_DELAY
    return t+delay if t>boundary+1e-8 or(start and abs(t-boundary)<1e-8)else t
