import math
from types import SimpleNamespace
from hnu2027_sentry_autonomy.sensors import split_rotation,filter_scan

def product(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return (w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z)

def test_planar_frame_preserves_full_body_orientation():
    raw=(.1,-.2,.3,.9)
    norm=math.sqrt(sum(v*v for v in raw));raw=tuple(v/norm for v in raw)
    sy,cy,body=split_rotation(SimpleNamespace(**dict(zip(("x","y","z","w"),raw))))
    actual=product((0,0,sy,cy),body)
    assert all(abs(a-b)<1e-12 for a,b in zip(actual,raw))
    assert abs(body[0])+abs(body[1])>.1

def test_self_returns_are_unknown_not_free_space():
    assert math.isnan(filter_scan([.405],-.9,.1,True)[0])
    assert filter_scan([.405],-.9,.1,False)[0]==.405
    assert math.isnan(filter_scan([.2],0,.1,False)[0])
    assert filter_scan([.8],-.9,.1,True)[0]==.8
    assert math.isinf(filter_scan([math.inf],0,.1,True)[0])
