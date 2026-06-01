from pxr import UsdGeom, Gf, Sdf, Vt
import omni.usd

try:
    from omni.kit.viewport.utility import get_active_viewport
except Exception:
    get_active_viewport = None

stage = omni.usd.get_context().get_stage()

camera_path = "/OmniverseKit_Persp"

# Get or create the perspective camera
cam_prim = stage.GetPrimAtPath(camera_path)
if not cam_prim.IsValid():
    cam = UsdGeom.Camera.Define(stage, Sdf.Path(camera_path))
    cam_prim = cam.GetPrim()
else:
    cam = UsdGeom.Camera(cam_prim)

# Directly force a simple transform stack.
# No AddTranslateOp / AddScaleOp, so no duplicate-op error.
cam_prim.CreateAttribute(
    "xformOp:translate",
    Sdf.ValueTypeNames.Double3,
    custom=False
).Set(Gf.Vec3d(1.5, -3.0, 1.5))

cam_prim.CreateAttribute(
    "xformOp:rotateXYZ",
    Sdf.ValueTypeNames.Float3,
    custom=False
).Set(Gf.Vec3f(60.0, 0.0, 25.0))

cam_prim.CreateAttribute(
    "xformOp:scale",
    Sdf.ValueTypeNames.Float3,
    custom=False
).Set(Gf.Vec3f(1.0, 1.0, 1.0))

cam_prim.CreateAttribute(
    "xformOpOrder",
    Sdf.ValueTypeNames.TokenArray,
    custom=False
).Set(Vt.TokenArray([
    "xformOp:translate",
    "xformOp:rotateXYZ",
    "xformOp:scale",
]))

# Reset camera optical properties
cam.CreateFocalLengthAttr().Set(24.0)
cam.CreateFocusDistanceAttr().Set(400.0)
cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000000.0))
cam.CreateHorizontalApertureAttr().Set(20.955)

# Make active viewport use this camera
if get_active_viewport is not None:
    vp = get_active_viewport()
    if vp is not None:
        try:
            vp.camera_path = camera_path
        except Exception:
            try:
                vp.set_camera_path(camera_path)
            except Exception as e:
                print("Could not set active viewport camera:", e)

print("Perspective camera transform overwritten directly.")
print("Camera path:", camera_path)
print("Now click viewport and press A, not F.")