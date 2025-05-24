from typing import Any, Dict, List

import cv2
import hydra
import numpy as np
import torch
from diffusion_policy.common.pytorch_util import dict_apply
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.sensor import Camera
from scipy.spatial.transform import Rotation as R

from .base_controller import ActionBuffer, BaseController


class SimpleInference:
    def __init__(self, checkpoint_path: str) -> None:
        # チェックポイントからモデルと設定をロード
        ckpt_data = torch.load(checkpoint_path, map_location="cuda:0")
        self.cfg = ckpt_data["cfg"]
        self.policy = hydra.utils.instantiate(self.cfg.policy)
        # EMAモデルの重みをロードして評価モードに設定
        self.policy.load_state_dict(ckpt_data["state_dicts"]["ema_model"])
        self.policy.cuda()
        self.policy.eval()

        # 設定から時系列長とジョイント次元を取得
        self.n_obs_steps = self.cfg.n_obs_steps

        # cfg.task.image_shape は [4, 240, 320]（RGBA, H, W）なので、
        # cv2.resize 用に (W, H) を取得。さらに、reshape 用に H,W を記録
        self.image_shape = self.cfg.task.image_shape  # [4, 240, 320]
        # image_size は (W, H)
        self.image_size = tuple(self.cfg.task.image_shape[1:][::-1])  # (320, 240)
        self.H, self.W = (
            self.cfg.task.image_shape[1],
            self.cfg.task.image_shape[2],
        )  # 240, 320

        # 入力履歴のキャッシュ（時系列データ）
        self.image_history = []  # 各要素: ndarray (H, W, 4)
        self.joint_history = []  # 各要素: ndarray (joint_dim,)

    def update_history(self, image: np.ndarray, joint: np.ndarray) -> None:
        """
        image: ndarray of shape (N, 4) で、N = w * h
        joint: ndarray of shape (joint_dim,)

        → image を (H, W, 4) に reshape して履歴に追加する
        """
        # 入力画像を (H, W, 4) に reshape する
        image_reshaped = image.reshape((self.H, self.W, 3))

        # image の履歴更新（スライディングウィンドウ）
        if len(self.image_history) == 0:
            self.image_history = [image_reshaped.copy()] * self.n_obs_steps
        else:
            self.image_history.pop(0)
            self.image_history.append(image_reshaped.copy())

        # joint の履歴更新（同様にスライディングウィンドウ）
        if len(self.joint_history) == 0:
            self.joint_history = [joint.copy()] * self.n_obs_steps
        else:
            self.joint_history.pop(0)
            self.joint_history.append(joint.copy())

    def infer_action(self, image: np.ndarray, joint: np.ndarray) -> List[float]:
        """
        各ステップごとに image (ndarray, shape=(N, 4)) と
        joint (ndarray, shape=(joint_dim,)) を受け取り、内部キャッシュを更新します。
        キャッシュが n_obs_steps に達していれば、時系列入力としてモデルに渡し、
        推論したアクションを返します。
        """
        self.update_history(image=image, joint=joint)

        # 画像履歴: (n_obs_steps, H, W, 4)
        img_history = np.array(self.image_history).astype(np.float32)
        # 正規化（0～1にスケーリング）
        img_history = img_history / 255.0
        # 軸入れ替え: (n_obs_steps, 4, H, W)
        img_history = np.moveaxis(img_history, -1, 1)
        # バッチ次元追加: (1, n_obs_steps, 4, H, W)
        img_input = np.expand_dims(img_history, 0)

        # ジョイント履歴: (n_obs_steps, joint_dim)
        joint_history = np.array(self.joint_history).astype(np.float32)
        # バッチ次元追加: (1, n_obs_steps, joint_dim)
        joint_input = np.expand_dims(joint_history, 0)

        # 入力辞書の作成
        obs_dict = {"image": img_input, "joint": joint_input}
        obs_dict = dict_apply(
            obs_dict, lambda x: torch.from_numpy(x).to(self.policy.device)
        )

        with torch.no_grad():
            output = self.policy.predict_action(obs_dict)
        output = dict_apply(output, lambda x: x.detach().cpu().numpy())

        # 出力の形状が (1, T, action_dim) と仮定し、最初のタイムステップのアクションを返す
        if output["action"].ndim == 3:
            action = output["action"][0, 0]
        else:
            action = output["action"][0]
        return action.tolist()


class DiffusionPolicyController:
    """
    Diffusion Policy によってロボットアームのエンドエフェクタの位置と姿勢を変更するコントローラ
    """

    def __init__(self, checkpoint_path: str, cameras: List[Camera]) -> None:
        """ """
        self._cameras: List[Camera] = cameras
        self._inference = SimpleInference(checkpoint_path=checkpoint_path)
        self._action_buffer: ActionBuffer = ActionBuffer()

    def forward(
        self, joint: np.ndarray, position: np.ndarray, orientation: np.ndarray
    ) -> tuple[np.ndarray, Any]:
        """ """
        action_delta: List[float] = self._inference.infer_action(
            image=self._proceed_image(), joint=joint
        )
        xyz_delta = np.array(action_delta[:3])
        rot_delta = R.from_quat(action_delta[3:7])
        gripper = action_delta[7]

        self._action_buffer.push(key="position", value=xyz_delta)
        self._action_buffer.push(key="orientation", value=rot_delta.as_quat().tolist())
        self._action_buffer.push(key="gripper", value=gripper)

        gripper_str = None
        if gripper > 0.5:
            gripper_str = "close"
        elif gripper < -0.5:
            gripper_str = "open"

        return (
            position + xyz_delta,
            (rot_delta * R.from_quat(orientation)).as_quat(),
            gripper_str,
        )

    def _proceed_image(self) -> np.ndarray:
        """ """
        image_arrays = []
        for camera in self._cameras:
            rgb: np.ndarray = camera.get_rgb()
            resolution: tuple[int, int] = camera.get_resolution()
            image_array: np.ndarray[Any, np.dtype[np.unsignedinteger]] = np.frombuffer(
                buffer=np.ascontiguousarray(rgb), dtype=np.uint8
            )
            image_array = image_array.reshape((resolution[1], resolution[0], 3))
            # drop alpha channel
            image_arrays.append(image_array)
        return np.array(
            object=[
                cv2.resize(
                    src=cv2.hconcat(
                        [
                            cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
                            for image_array in image_arrays
                        ]
                    ),
                    dsize=(
                        self._inference.image_size[0],
                        self._inference.image_size[1],
                    ),
                )
            ]
        )

    def get_gripper_action(self) -> str | None:
        """ """
        self._action_buffer.push(key="gripper", value=0)

    def get_applied_action(self) -> Dict:
        """ """
        return self._action_buffer.flush()
