from typing import Any, Dict

import carb
import numpy as np
import omni
from scipy.spatial.transform import Rotation as R

from .base_controller import ActionBuffer, BaseController


class KeyboardController(BaseController):
    """
    キーボードの入力によってロボットアームのエンドエフェクタの位置と姿勢を変更するコントローラ
    """

    def __init__(
        self,
        delta: float = 0.03,
    ) -> None:
        """
        Initialize KeyboardController

        Args:
            delta (float, optional): 移動量. Defaults to 0.03.
        """
        self._delta = delta
        self._reset()
        appwindow = omni.appwindow.get_default_app_window()
        input = carb.input.acquire_input_interface()
        input.subscribe_to_keyboard_events(
            appwindow.get_keyboard(),
            lambda event, *args, **kwargs: self._sub_keyboard_event(event),
        )
        carb.settings.get_settings().set(
            "persistent/app/omniverse/gamepadCameraControl", False
        )
        self._action_buffer: ActionBuffer = ActionBuffer()

    def _sub_keyboard_event(self, event) -> None:
        """
        キーボードの入力によって関節角を変更する

        Args:
            event (_type_): キーボードの入力イベント
        """
        self._reset()
        if (
            event.type != carb.input.KeyboardEventType.KEY_PRESS
            and event.type != carb.input.KeyboardEventType.KEY_REPEAT
        ):
            return

        match event.input:
            # Translate
            case carb.input.KeyboardInput.W:
                self._translate[0] = self._delta
            case carb.input.KeyboardInput.S:
                self._translate[0] = -self._delta
            case carb.input.KeyboardInput.E:
                self._translate[1] = self._delta
            case carb.input.KeyboardInput.D:
                self._translate[1] = -self._delta
            case carb.input.KeyboardInput.R:
                self._translate[2] = self._delta
            case carb.input.KeyboardInput.F:
                self._translate[2] = -self._delta
            # Rotate
            case carb.input.KeyboardInput.U:
                self._rotate[0] = self._delta
            case carb.input.KeyboardInput.J:
                self._rotate[0] = -self._delta
            case carb.input.KeyboardInput.I:
                self._rotate[1] = self._delta
            case carb.input.KeyboardInput.K:
                self._rotate[1] = -self._delta
            case carb.input.KeyboardInput.O:
                self._rotate[2] = self._delta
            case carb.input.KeyboardInput.L:
                self._rotate[2] = -self._delta
            # Gripper
            case carb.input.KeyboardInput.G:
                self._gripper_action = "close"
            case carb.input.KeyboardInput.H:
                self._gripper_action = "open"

    def forward(
        self, position: np.ndarray, orientation: np.ndarray
    ) -> tuple[np.ndarray, Any]:
        """
        各ステップで次のエンドエフェクタの位置と姿勢を計算する

        Args:
            position (np.ndarray): 現在のエンドエフェクタの位置
            orientation (np.ndarray): 現在のエンドエフェクタの姿勢

        Returns:
            tuple[np.ndarray, Any]: 次のエンドエフェクタの位置と姿勢

            Example:
                >>> position = np.array([0.0, 0.0, 0.0])
                >>> orientation = np.array([0.0, 0.0, 0.0, 1.0])
                >>> next_position, next_orientation = keyboard_controller.forward(position, orientation)
        """
        next_position = position + self._translate
        delta_rot: R = R.from_euler(seq="xyz", angles=self._rotate, degrees=False)

        self._action_buffer.push(key="position", value=self._translate.tolist())
        self._action_buffer.push(key="orientation", value=delta_rot.as_quat().tolist())

        current_rot: R = R.from_quat(quat=orientation)
        next_orientation: np.ndarray = (delta_rot * current_rot).as_quat()
        return next_position, next_orientation

    def get_gripper_action(self) -> str | None:
        """
        各ステップでのグリッパーのアクションを取得する

        Returns:
            str | None: "open" or "close" or None
        """
        if self._gripper_action is None:
            self._action_buffer.push(key="gripper", value=0)
        elif self._gripper_action == "open":
            self._action_buffer.push(key="gripper", value=-1)
        elif self._gripper_action == "close":
            self._action_buffer.push(key="gripper", value=1)
        return self._gripper_action

    def get_applied_action(self) -> Dict:
        """
        Get applied action

        Returns:
            Dict: action
        """
        return self._action_buffer.flush()

    def _reset(self):
        """
        アクションが適用されたらバファをリセットする
        """
        self._translate = np.zeros(shape=3)
        self._rotate = np.zeros(shape=3)
        self._gripper_action: str = None
        return
