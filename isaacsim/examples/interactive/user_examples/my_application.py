from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World()
world.scene.add_default_ground_plane()

fancy_cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/fancy_cube",
        name="fancy_cube",
        position=np.array([0.0, 0.0, 1.0]),
        scale= np.array([0.5, 0.5, 0.5]),
        color=np.array([1.0, 0.0, 0.0]),
    )
)

world.reset()

for i in range(500):
    position, orientation = fancy_cube.get_world_pose()
    linear_velocity = fancy_cube.get_linear_velocity()

    print("Cube Position: " + str(position))
    print("Cube Orientation: " + str(orientation))
    print("Cube Linear Velocity: " + str(linear_velocity))

    world.step(render=True)

simulation_app.close()