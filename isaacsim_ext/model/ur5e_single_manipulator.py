from typing import Dict, Optional

import numpy as np
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.sensor import ContactSensor
from omni.physx.scripts import utils

from .factory import create_contact_sensor, create_parallel_gripper


class UR5eSingleManipulator(SingleManipulator):
    """
    DoF: 12

    6 for the UR5e manipulator and 2 for the parallel gripper.
    - shoulder_pan_joint
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint
    - finger_joint
    - right_outer_knuckle_joint
    - left_inner_finger_joint (not controlled)
    - left_inner_knuckle_joint (not controlled)
    - right_inner_finger_joint (not controlled)
    - right_inner_knuckle_joint (not controlled)
    """

    def __init__(
        self,
        name: str = "UR5e",
        usd_path: str = "isaacsim_ext/static/usd/ur5e_with_gripper_and_frame.usd",
        prim_path: str = "/World/ur5e_with_gripper/ur5e/base_link",
        end_effector_prim_path: str = "/World/ur5e_with_gripper/robotiq_2f140/robotiq_arg2f_base_link",
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize UR5e

        Args:
        name (str, optional):
            ロボットアームの名前. シーン内で一意である必要がある.
            Defaults to "UR5e".
        usd_path (str, optional):
            UR5eのUSDファイルのパス.
            Defaults to "static/usd/ur5e_with_gripper_and_frame.usd".
        prim_path (str, optional):
            UR5eのプリムパス.
        end_effector_prim_path (str, optional):
            エンドエフェクタのプリムパス.
            Defaults to "/World/ur5e_with_gripper/robotiq_2f140/robotiq_arg2f_base_link".
        position (Optional[np.ndarray], optional):
            初期位置.
            Defaults to None.
        orientation (Optional[np.ndarray], optional):
            初期姿勢.
            Defaults to None.
        """

        add_reference_to_stage(
            usd_path=usd_path,
            prim_path="/World",
        )
        self._gripper: ParallelGripper = create_parallel_gripper(
            end_effector_prim_path=self._end_effector_prim_path
        )
        super().__init__(
            prim_path=prim_path,
            name=name,
            end_effector_prim_name="robotiq_arg2f_base_link",
            gripper=self._gripper,
            position=position,
            orientation=orientation,
        )
        self._sensors: list[ContactSensor] = [
            create_contact_sensor(
                name="left_contact_sensor",
                link_path="/World/ur5e_with_gripper/robotiq_2f140/left_inner_finger_pad",
            ),
            create_contact_sensor(
                name="right_contact_sensor",
                link_path="/World/ur5e_with_gripper/robotiq_2f140/right_inner_finger_pad",
            ),
        ]

        for path in [
            "/World/ur5e_with_gripper/robotiq_2f140/left_inner_finger_pad",
            "/World/ur5e_with_gripper/robotiq_2f140/right_inner_finger_pad",
        ]:
            utils.setRigidBody(
                prim=get_prim_at_path(prim_path=path),
                approximationShape="convexDecomposition",
                kinematic=True,
            )
            utils.addRigidBodyMaterial(
                stage=get_current_stage(),
                path=path,
                staticFriction=1.0,
                dynamicFriction=0.7,
            )

    @property
    def gripper(self) -> ParallelGripper:
        """
        グリッパーのGetter

        Returns:
            ParallelGripper: グリッパー
        """
        return self._gripper

    @property
    def sensors(self) -> list[ContactSensor]:
        """
        センサのGetter

        Returns:
            list[ContactSensor]: センサのリスト
        """
        return self._sensors

    def initialize(self, physics_sim_view=None) -> None:
        """
        Initialize UR5e

        Args:
            physics_sim_view (_type_, optional):
                Physics simulation view.
                Defaults to None.
        """
        super().initialize(physics_sim_view=physics_sim_view)
        self._gripper.initialize(
            physics_sim_view=physics_sim_view,
            articulation_apply_action_func=self.apply_action,
            get_joint_positions_func=self.get_joint_positions,
            set_joint_positions_func=self.set_joint_positions,
            dof_names=self.dof_names,
        )
        return

    def post_reset(self) -> None:
        """
        ロボットを初期状態にリセットする
        """
        super().post_reset()
        self._gripper.post_reset()
        return

    def get_contact(self) -> Dict:
        """
        コンタクトセンサーの値を取得する

        Returns:
            Dict: コンタクトセンサーの値
        """
        return {sensor.name: sensor.get_current_frame() for sensor in self._sensors}
