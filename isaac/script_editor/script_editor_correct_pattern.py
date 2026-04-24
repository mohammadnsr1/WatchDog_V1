import asyncio
import omni.timeline
import omni.kit.app

from isaacsim.core.prims import Articulation
from isaacsim.core.api.simulation_context import SimulationContext


async def test():
    robot_prim_path = "/World/ur5e"   # change later if needed

    print("Creating SimulationContext...")
    sim = SimulationContext()
    await sim.initialize_simulation_context_async()

    print("Binding articulation...")
    robot = Articulation(prim_paths_expr=robot_prim_path, name="ur5e")

    print("Starting timeline...")
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    print("Resetting sim...")
    await sim.reset_async()

    # allow backend to initialize
    for _ in range(5):
        await omni.kit.app.get_app().next_update_async()

    print("Trying to read joint names...")
    try:
        print("Joint names:", robot.joint_names)
        print("SUCCESS: articulation handle acquired")
    except Exception as e:
        print("FAILED:", e)


asyncio.ensure_future(test())
