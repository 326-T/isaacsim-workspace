from omni.isaac.core.articulations import Articulation
from omni.isaac.motion_generation import (
    ArticulationMotionPolicy,
    MotionPolicyController,
)
from omni.isaac.motion_generation.lula.motion_policies import RmpFlow


class RMPFlowController(MotionPolicyController):
    """
    RMPFlowを用いたモーションポリシーコントローラ
    作成したArticulationMotionPolicyをスーパークラスに渡す

    Args:
        MotionPolicyController (MotionPolicyController): スーパークラス
    """

    def __init__(
        self,
        robot_articulation: Articulation,
        name: str = "RMPFlowController",
        physics_dt: float = 1.0 / 60.0,
        robot_description_path: str = "isaacsim_ext/static/rmpflow/robot_descriptor.yml",
        rmpflow_config_path: str = "isaacsim_ext/static/rmpflow/ur5e_rmpflow_common.yml",
        urdf_path: str = "isaacsim_ext/static/urdf/ur5e.urdf",
    ) -> None:
        """
        Initialize RMPFlowController

        Args:
            robot_articulation (Articulation):
                Isaac Sim Articulation
            name (str, optional):
                インスタンスの名前.
                Defaults to "RMPFlowController".
            physics_dt (float, optional):
                タイムステップ.
                Defaults to 1.0/60.0.
            robot_description_path (str, optional):
                操作するロボットの設定ファイルのパス.
                Defaults to "isaacsim_ext/static/rmpflow/robot_descriptor.yml".
            rmpflow_config_path (str, optional):
                RMPFlowの設定ファイルのパス.
                Defaults to "isaacsim_ext/static/rmpflow/ur5e_rmpflow_common.yml".
            urdf_path (str, optional):
                ロボットのURDFファイルのパス.
                Defaults to "isaacsim_ext/static/urdf/ur5e.urdf".
        """
        self.rmp_flow = RmpFlow(
            robot_description_path=robot_description_path,
            rmpflow_config_path=rmpflow_config_path,
            urdf_path=urdf_path,
            end_effector_frame_name="tool0",
            maximum_substep_size=0.00334,
        )

        self.articulation_rmp = ArticulationMotionPolicy(
            robot_articulation=robot_articulation,
            motion_policy=self.rmp_flow,
            default_physics_dt=physics_dt,
        )

        super().__init__(name=name, articulation_motion_policy=self.articulation_rmp)
        (
            self._default_position,
            self._default_orientation,
        ) = self._articulation_motion_policy._robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )
        return
