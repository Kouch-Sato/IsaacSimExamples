# 0703 controllerを使って書いてみる
# https://docs.isaacsim.omniverse.nvidia.com/4.5.0/core_api_tutorials/tutorial_core_adding_controller.html

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.api.controllers import BaseController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
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
            
        jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
       
        world.scene.add(
            WheeledRobot(
                prim_path="/World/jetbot_01",
                name="jetbot_01",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_asset_path,
                position=np.array([0.0, 0.0, 0.1]),
            )
        )

        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._jetbot_01 = self._world.scene.get_object("jetbot_01")

        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        self._my_controller = WheelBasePoseController(
            name="my_controller",
            open_loop_wheel_controller=DifferentialController(
                name="simple_differential_controller",
                wheel_radius=0.03,
                wheel_base=0.1125,
            ),
            is_holonomic=False,
        )
        return
    
    def send_robot_actions(self, step_size):
        position, orientation = self._jetbot_01.get_world_pose()
        self._jetbot_01.apply_wheel_actions(
            self._my_controller.forward(
                start_position=position,
                start_orientation=orientation,
                goal_position=np.array([-5, 5, 0]),
            )
        )

        print(self._my_controller.forward(
            start_position=position,
            start_orientation=orientation,
            goal_position=np.array([-5, 5, 0]),
        ))

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
