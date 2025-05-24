from typing import Any, Dict

import numpy as np
from omni.isaac.core.utils.extensions import enable_extension
from scipy.spatial.transform import Rotation as R

enable_extension("srl.spacemouse")
from srl.spacemouse.buttons import ButtonStateStruct
from srl.spacemouse.device import DEVICE_SPECS
from srl.spacemouse.spacemouse import SpaceMouse, SpaceMouseData
from srl.spacemouse.spacemousefilter import SpaceMouseFilter

from .base_controller import ActionBuffer, BaseController


class SpaceMouseController(BaseController):
    """
    スペースマウスでロボットアームのエンドエフェクタの位置と姿勢を変更するコントローラ
    """

    def __init__(
        self,
        device_type: str = "SpaceMouse Compact",
        trans_gain: float = 0.1,
        rotate_gain: float = 0.1,
    ) -> None:
        """
        Initialize SpaceMouseController

        Args:
            device_type (str, optional): スペースマウスの種類. Defaults to "SpaceMouse Compact".
            trans_gain (float, optional): 移動量のゲイン. Defaults to 0.1.
            rotate_gain (float, optional): 回転量のゲイン. Defaults to 0.1.

        .. seealso::
            - :py:class:`srl.spacemouse.device.DEVICE_SPECS`
        """
        self._trans_gain: float = trans_gain
        self._rotate_gain: float = rotate_gain
        filter: SpaceMouseFilter = SpaceMouseFilter(
            smoothing_factor=0.5,
            softmax_temp=0.85,
            translation_modifier=0.1,
            rotation_modifer=0.1,
            translation_deadband=0.1,
            rotation_deadband=0.1,
            translation_enabled=True,
            rotation_enabled=True,
        )
        self._device: SpaceMouse = SpaceMouse(spec=DEVICE_SPECS[device_type])
        self._device.set_position_callback(callback=filter._translation_modifier)
        self._device.set_rotation_callback(callback=filter._rotation_modifier)
        self._device.run()
        self._action_buffer: ActionBuffer = ActionBuffer()

    def forward(
        self, position: np.ndarray, orientation: np.ndarray
    ) -> tuple[np.ndarray, Any]:
        """
        スペースマウスの入力によってエンドエフェクタの位置と姿勢を変更する

        Args:
            position (np.ndarray): 現在の位置
            orientation (np.ndarray): 現在の姿勢

        Returns:
            tuple[np.ndarray, Any]: 次のステップの位置と姿勢
        """
        stick_state: SpaceMouseData | None = self._device.get_controller_state()
        xyz_delta: np.ndarray = self._trans_gain * np.array(
            [stick_state.xyz[1], -stick_state.xyz[0], stick_state.xyz[2]]
        )
        euler_delta: np.ndarray = self._rotate_gain * np.array(
            [stick_state.rpy[0], stick_state.rpy[1], -stick_state.rpy[2]]
        )
        rot_delta: R = R.from_euler(seq="xyz", angles=euler_delta, degrees=False)

        self._action_buffer.push(key="position", value=xyz_delta.tolist())
        self._action_buffer.push(key="orientation", value=rot_delta.as_quat().tolist())

        next_position = position + xyz_delta
        current_rot: R = R.from_quat(quat=orientation)
        next_orientation: np.ndarray = (rot_delta * current_rot).as_quat()
        return next_position, next_orientation

    def get_gripper_action(self) -> str | None:
        """
        スペースマウスのボタンの状態を取得する

        Returns:
            str | None: "open" or "close" or None
        """
        button_state: ButtonStateStruct | None = self._device.get_button_state()
        if button_state.value is None:
            self._action_buffer.push(key="gripper", value=0)
            return None
        if button_state.value == 1:
            self._action_buffer.push(key="gripper", value=1)
            return "close"
        if button_state.value == 2:
            self._action_buffer.push(key="gripper", value=-1)
            return "open"

    def get_applied_action(self) -> Dict:
        """
        Get applied action

        Returns:
            Dict: action
        """
        return self._action_buffer.flush()
