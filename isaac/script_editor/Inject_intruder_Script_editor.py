import sys, importlib
import omni.usd
from pxr import Usd, UsdGeom

p = "/home/mohammad/watchdog/WatchDog_V1/isaac/script_editor"
if p not in sys.path:
    sys.path.insert(0, p)
import inject_intruder as ii
importlib.reload(ii)
print("loaded:", ii.__file__)





stage = omni.usd.get_context().get_stage()
print("metersPerUnit:", UsdGeom.GetStageMetersPerUnit(stage))   # must be 1.0

cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
parts = ["base1", "base2", "u_green", "u_brown", "u_orange", "inverted_u_brown",
         "inverted_u_yellow", "line_green", "hex_blue", "hex_red"]
lows = []
for n in parts:
    prim = stage.GetPrimAtPath(f"/World/{n}")
    if not prim.IsValid():
        print(f"{n:18s} missing"); continue
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    lows.append(mn[2])
    print(f"{n:18s} x[{mn[0]: .3f},{mx[0]: .3f}]  y[{mn[1]: .3f},{mx[1]: .3f}]  z[{mn[2]: .3f},{mx[2]: .3f}]")

BENCH_Z = min(lows)
print("bench surface ≈", round(BENCH_Z, 3))




SIZE = 0.05
X, Y = -0.50, 0.1   # EDIT: a free spot >=10 cm from every part box above,
                     # and within ~0.4 m of the camera's (0.25, -0.2) so it is in view

ii.inject("intruder_l2", shape="cube", size_m=SIZE,
          position_xyz_world=(X, Y, BENCH_Z + SIZE / 2 + 0.005),
          color_rgb=(0.9, 0.1, 0.1), physics=True,
          label="L0 mechanics test", expected_impact="benign",
          notes="L0: injection mechanics, no agents running")
print(ii.list_injections())





prim = stage.GetPrimAtPath("/World/WatchdogIntruders/intruder_l2")
print("valid:", prim.IsValid())
if prim.IsValid():
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    print("USD pose:", xf.ExtractTranslation())
    
    
    ii.remove("intruder_l2")
print(ii.list_injections())
    
