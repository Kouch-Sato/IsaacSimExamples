import array

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.api.controllers import BaseController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.robot.manipulators.examples.franka.tasks import PickPlace
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController
import numpy as np
import carb

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.add_task(PickPlace(
            name="my_task",
            cube_initial_position=np.array([-0.3, 0.2, 0.26])
        ))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        task_params = self._world.get_task("my_task").get_params()

        # task_paramsの中身
        # {
        #     'cube_position':  {'value': array([0.3, 0.3, 0.3], dtype=float32), 'modifiable': True}, 
        #     'cube_orientation': {'value': array([1., 0., 0., 0.], dtype=float32), 'modifiable': True},
        #     'target_position': {'value': array([-0.3    , -0.3    ,  0.02575]), 'modifiable': True},
        #     'cube_name': {'value': 'cube', 'modifiable': False}, 
        #     'robot_name': {'value': 'my_franka', 'modifiable': False}
        # }

        self._franka = self._world.scene.get_object(task_params["robot_name"]["value"])
        self._cube = self._world.scene.get_object(task_params["cube_name"]["value"])
        self._controller = PickPlaceController(
            name="my_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
        )
        self._world.add_physics_callback("sending_actions", callback_fn=self.physics_step)
        await self._world.play_async()
        return
    
    async def setup_post_reset(self):
        self._controller.reset()
        await self._world.play_async()
        return
    
    def physics_step(self, step_size):
        current_observations = self._world.get_observations()

        actions = self._controller.forward(
            picking_position=current_observations[self._cube.name]["position"],
            placing_position=current_observations[self._cube.name]["target_position"],
            current_joint_positions=current_observations[self._franka.name]["joint_positions"],
        )

        self._franka.apply_action(actions)

        if self._controller.is_done():
            self._world.pause()
        return
    
    def send_robot_actions(self, step_size):
        return  
        
    async def setup_pre_reset(self):
        return

    def world_cleanup(self):
        return
