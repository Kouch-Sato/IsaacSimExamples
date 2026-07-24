import array

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver, 
    ArticulationKinematicsSolver
)
import numpy as np
import carb

SO101_PARENT_PATH = r"C:/Users/USER/Documents/kouch_projects/SO-ARM100/Simulation/SO101"

class SO101PickPlace(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._state = "approach"

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()

        self._cube = world.scene.add(
            DynamicCuboid(
                prim_path = "/World/Cube",
                name = "cube",
                position = np.array([0.25, 0.0, 0.015]),
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

        self._home_joint_positions = np.array([
            0.0,
            -1.74533,
            1.65632,
            0.98262,
            0.0,
            0.0,
        ])

        self._so101.set_joints_default_state(
            positions = self._home_joint_positions,
            velocities = np.zeros(6)
        )

        self._so101.set_joint_positions(self._home_joint_positions)
        self._so101.set_joint_velocities(np.zeros(6))

        self._articulation_controller = (
            self._so101.get_articulation_controller()
        )

        urdf_path = SO101_PARENT_PATH + "/so101_new_calib.urdf"
        descripter_path = SO101_PARENT_PATH + "/so101_new_calib_descriptor.yaml"

        self._lula_solver = LulaKinematicsSolver(
            robot_description_path = descripter_path,
            urdf_path = urdf_path
        )

        self._ik_solver = ArticulationKinematicsSolver(
            robot_articulation = self._so101,
            kinematics_solver = self._lula_solver,
            end_effector_frame_name = "gripper_frame_link"
        )

        so101_base_position, so101_base_orientation = self._so101.get_world_pose()
        self._cube_position, cube_orientation = self._cube.get_world_pose()
        self._target_position = self._cube_position + np.array([-0.02, 0.0, 0.08])

        self._world.add_physics_callback(
            "so101_test",
            callback_fn = self.physics_step
        )
    
    async def setup_post_reset(self):
        self._time = 0.0
        await self._world.play_async()

    # apply_actionするのは基本的にここ
    def physics_step(self, step_size):
        self.open_gripper()

        # if self._state == "approach" and self.has_reached_positon():
        #     self._target_position = self._cube_position + np.array([-0.02, 0.0, 0.03])
            
        #     self._state = "descend"
        #     print("apporachから次に進んだよ")

        action = self.forward(step_size)
        if action is not None:
            self._articulation_controller.apply_action(action)
    
    def send_robot_actions(self, step_size):
        return  
        
    async def setup_pre_reset(self):
        return

    def world_cleanup(self):
        return
    
    def open_gripper(self):
        action = ArticulationAction(
            joint_positions = np.array([0.7]),
            joint_indices = np.array([5]),
        )
                
        self._articulation_controller.apply_action(action)

    def close_gripper(self):
        action = ArticulationAction(
            joint_positions = np.array([0]),
            joint_indices = np.array([5]),
        )
                
        self._articulation_controller.apply_action(action)

    def has_reached_positon(self, tolerance = 0.01):
        current_position, _ = self._ik_solver.compute_end_effector_pose()

        distance = np.linalg.norm(current_position - self._target_position)

        return distance < tolerance

    # ここでは目標座標からIKを解いてactionを出す
    def forward(self, step_size):
        action, success = self._ik_solver.compute_inverse_kinematics(
            target_position = self._target_position
        )
        
        return action   
            