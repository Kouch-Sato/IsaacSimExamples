from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot
import numpy as np
import carb

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("assets_root_pathがないよ！")
            return
            
        asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/jetbot_01")

        jetbot_01 = world.scene.add(
            Robot(
                prim_path="/World/jetbot_01",
                name="jetbot_01",
            )
        )

        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._jetbot_01 = self._world.scene.get_object("jetbot_01")
        
        self._jetbot_01_articulation_controller = self._jetbot_01.get_articulation_controller()
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        return
    
    def send_robot_actions(self, step_size):
        self._jetbot_01_articulation_controller.apply_action(
            ArticulationAction(
                joint_positions=None,
                joint_efforts=None,
                joint_velocities=np.array([5, 5])
            )
        )

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
