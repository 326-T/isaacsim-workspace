import carb
import numpy as np
import omni
from omni.isaac.core.utils.types import ArticulationAction


@DeprecationWarning
class JointController:
    """
    関節角を直接指定するコントローラ
    キーボードの入力によって関節角を変更する
    """

    def __init__(self, dofs: int = 12) -> None:
        """
        Initialize JointController

        Args:
            dofs (int, optional):
                ロボットアームの自由度数. Defaults to 12.
        """
        self._dofs = dofs
        self._action = np.zeros(shape=self._dofs)
        appwindow = omni.appwindow.get_default_app_window()
        input = carb.input.acquire_input_interface()
        input.subscribe_to_keyboard_events(
            appwindow.get_keyboard(),
            lambda event, *args, **kwargs: self._sub_keyboard_event(event),
        )

    def _sub_keyboard_event(self, event) -> None:
        """
        キーボードの入力によって関節角を変更する

        Args:
            event (Any): キーボードの入力イベント
        """
        self._action = np.zeros(shape=self._dofs)
        if (
            event.type != carb.input.KeyboardEventType.KEY_PRESS
            and event.type != carb.input.KeyboardEventType.KEY_REPEAT
        ):
            return

        match event.input:
            case carb.input.KeyboardInput.Q:
                self._action[0] = 0.1
            case carb.input.KeyboardInput.A:
                self._action[0] = -0.1
            case carb.input.KeyboardInput.W:
                self._action[1] = 0.1
            case carb.input.KeyboardInput.S:
                self._action[1] = -0.1
            case carb.input.KeyboardInput.E:
                self._action[2] = 0.1
            case carb.input.KeyboardInput.D:
                self._action[2] = -0.1
            case carb.input.KeyboardInput.R:
                self._action[3] = 0.1
            case carb.input.KeyboardInput.F:
                self._action[3] = -0.1
            case carb.input.KeyboardInput.T:
                self._action[4] = 0.1
            case carb.input.KeyboardInput.G:
                self._action[4] = -0.1
            case carb.input.KeyboardInput.Y:
                self._action[5] = 0.1
            case carb.input.KeyboardInput.H:
                self._action[5] = -0.1
            case carb.input.KeyboardInput.U:
                self._action[6] = 0.1
            case carb.input.KeyboardInput.J:
                self._action[6] = -0.1
            case carb.input.KeyboardInput.I:
                self._action[7] = 0.1
            case carb.input.KeyboardInput.K:
                self._action[7] = -0.1

    def forward(self, current_position: np.ndarray) -> ArticulationAction:
        """
        各ステップで次の関節角を計算する

        Args:
            current_position (np.ndarray): 現在の関節角

        Returns:
            ArticulationAction: 次の関節角
        """
        return ArticulationAction(joint_positions=self._action + current_position)
