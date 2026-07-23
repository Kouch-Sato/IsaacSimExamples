import array

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.api.objects import DynamicCuboid
import numpy as np
import carb

SO101_PARENT_PATH = r"C:/Users/USER/Documents/kouch_projects/SO-ARM100/Simulation/SO101"

class SO101PickPlace(BaseSample):
    def __init__(self) -> None:
        super().__init__()

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()

        self._cube = world.scene.add(
            DynamicCuboid(
                prim_path = "/World/Cube",
                name = "cube",
                position = np.array([0.20, 0.0, 0.015]),
                scale = np.array([0.03, 0.03, 0.03]),
                color = np.array([0.0, 0.2, 1.0]),
                mass = 0.02
            )
        )

        usd_path = SO101_PARENT_PATH + "/so101_new_calib/so101_new_calib.usd"
        prim_path = "/World/SO101"

        add_reference_to_stage(
            usd_path = usd_path,
            prim_path = prim_path
        )

        self._so101 = world.scene.add(
            SingleArticulation(
                prim_path = prim_path,
                name = "so101"
            )
        )

    async def setup_post_load(self):
        self._world = self.get_world()
        await self._world.reset_async()

        self._articulation_controller = (
            self._so101.get_articulation_controller()
        )

        self._world.add_physics_callback(
            "so101_test",
            callback_fn = self.physics_step
        )
    
    async def setup_post_reset(self):
        self._time = 0.0
        await self._world.play_async()
    
    def physics_step(self, step_size):
        self.open_gripper()
    
    def send_robot_actions(self, step_size):
        return  
        
    async def setup_pre_reset(self):
        return

    def world_cleanup(self):
        return
    
    def open_gripper(self):
        action = ArticulationAction(
            joint_positions = np.array([0.5]),
            joint_indices = np.array([5]),
        )
                
        self._articulation_controller.apply_action(action)

    def close_gripper(self):
        action = ArticulationAction(
            joint_positions = np.array([0]),
            joint_indices = np.array([5]),
        )
                
        self._articulation_controller.apply_action(action)