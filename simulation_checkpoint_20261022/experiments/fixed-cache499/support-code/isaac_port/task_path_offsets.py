"""Explicit robot-path corrections; never edits measured workpiece states."""
import math

def smooth(u):
    u=max(0.,min(1.,float(u)))
    return u*u*(3-2*u)

def pin_clearance_z(t):
    # Keep the pin head below the rigid mating fixture until XY is aligned.
    return -.004*smooth((t-5.8)/.5)*(1-smooth((t-6.4863333333)/.18))

def empty_right_offset(t):
    # Withdraw the empty hand below/front of the rail before approaching button.
    u=max(0.,min(1.,(t-5.95)/1.45))
    w=math.sin(math.pi*u)**2
    v=max(0.,min(1.,(t-5.95)/.55))
    return [0.,-.050*w,-.060*math.sin(math.pi*v)**2]


def pin_grasp_depth(t):
    """Grip the cylindrical neck 2.5mm deeper, clear of the concave tail."""
    def ease(x):
        u=min(1.,max(0.,x));return u*u*(3-2*u)
    return .0025*ease((t-3.394333333333333)/.3)*(1-ease((t-7.303)/.2))


def pin_grasp_weight(t):
    def ease(x):
        u=min(1.,max(0.,x));return u*u*(3-2*u)
    return ease((t-3.394333333333333)/.3)
