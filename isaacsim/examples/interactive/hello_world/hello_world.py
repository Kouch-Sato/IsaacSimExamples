from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot
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

        print(str(jetbot_01.num_dof))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._jetbot_01 = self._world.scene.get_object("jetbot_01")
        
        print(str(self._jetbot_01.num_dof))
        print(str(self._jetbot_01.get_joint_positions()))
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
