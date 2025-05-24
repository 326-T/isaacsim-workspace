import os
import shutil
import threading
from datetime import datetime
from typing import Any, Callable, Dict, List, cast

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.loggers.data_logger import DataLogger
from isaacsim.core.api.prims import XFormPrim
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks.base_task import BaseTask
from omni.isaac.sensor import Camera, ContactSensor
from PIL import Image

from isaacsim_ext.env import get_settings


class Logger:
    """
    ロギングに関する処理を行うクラス
    """

    def __init__(
        self,
        world: World,
        robot_names: List[str] = ["UR5e"],
        x_form_prim_names: List[str] = ["target"],
        contact_sensor_names: List[str] = [
            "left_contact_sensor",
            "right_contact_sensor",
        ],
        camera_names: List[str] = ["left_camera", "right_camera"],
        action_ref: Callable[[], Dict] = lambda: {},
        output_dir: str = ".data/isaacsim_trial",
    ) -> None:
        """
        ロガークラス

        Args:
            world (World): IsaacSim World
            robot_names (List[str], optional):
                ログを取得するRobotの名前の一覧.
                Defaults to ["UR5e"].
            x_form_prim_names (List[str], optional):
                ログを取得するXFormPrimの名前の一覧.
                Defaults to ["target"].
            contact_sensor_names (List[str], optional):
                ログを取得するContactSensorの名前の一覧.
                Defaults to [ "left_contact_sensor", "right_contact_sensor", ].
            camera_names (List[str], optional):
                ログを取得するCameraの名前の一覧.
                Defaults to ["left_camera", "right_camera"].

        .. seealso::
            - :py:class:`omni.isaac.core.World`
            - :py:class:`omni.isaac.core.robots.robot.Robot`
            - :py:class:`omni.isaac.core.prims.XFormPrim`
            - :py:class:`omni.isaac.sensor.scene.Camera`
            - :py:class:`omni.isaac.sensor.scene.ContactSensor`
        """
        self._output_dir: str = output_dir
        self._temp_dir: str = f"{output_dir}/recording"
        self._world: World = world
        self._robot_names: str = robot_names
        self._x_form_prim_names: str = x_form_prim_names
        self._contact_sensor_names: List[str] = contact_sensor_names
        self._camera_names: List[str] = camera_names
        self._action_ref: Callable[[], Dict] = action_ref
        self._data_logger: DataLogger = world.get_data_logger()
        self._data_logger.add_data_frame_logging_func(func=self._frame_logging_func)
        self._reset_temp_dir()
        self._phase: int = 0

    def start(self) -> None:
        """
        再生ボタンを押した際に呼び出されるロギングに関する処理
        """
        self._data_logger.start()

    def pause(self) -> None:
        """
        一時停止ボタンを押した際に呼び出されるロギングに関する処理
        フェーズを一つ進める
        """
        self._data_logger.pause()
        self._phase += 1

    def save_and_reset(self) -> None:
        """
        停止ボタンを押した際に呼び出されるロギングに関する処理
        ログを保存してリセットする
        """
        json_path: str = os.path.join(self._temp_dir, "trajectory.json")
        self._data_logger.save(log_path=json_path)
        self._data_logger.reset()
        os.rename(
            src=self._temp_dir,
            dst=f"{self._output_dir}/{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}",
        )
        self._phase = 0
        self._reset_temp_dir()

    def _reset_temp_dir(self) -> None:
        """
        ログを保存するディレクトリを自動作成する
        カメラに関しては名前でネストする形でサブディレクトリを作成する
        """
        if os.path.exists(path=self._temp_dir):
            shutil.rmtree(self._temp_dir)
        os.makedirs(name=self._temp_dir)
        for camera_name in self._camera_names:
            os.makedirs(name=os.path.join(self._temp_dir, camera_name))

    def _frame_logging_func(
        self, tasks: List[BaseTask], scene: Scene
    ) -> Dict[str, Any]:
        """
        フレームごとのロギングに関する処理
        WorldのDataLoggerにCallbackとして登録される

        Args:
            tasks (List[BaseTask]): 現在実行されているタスク（使用しない）
            scene (Scene): IsaacSim Scene

        Returns:
            Dict[str, Any]: JSON形式のログデータ

            Examples:
                {
                    "phase": 0,
                    "robots": {
                        "UR5e": {
                            "joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            "applied_joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                        }
                    },
                    "x_form_prims": {
                        "target": {
                            "position": [0.0, 0.0, 0.0],
                            "orientation": [0.0, 0.0, 0.0, 0.0]
                        }
                    },
                    "contact_sensors": {
                        "left_contact_sensor": 0.0,
                        "right_contact_sensor": 0.0
                    },
                    "cameras": {
                        "left_camera": "./left_camera/000000.png",
                        "right_camera": "./right_camera/000000.png"
                    },
                    "action": {
                        Dictionary of action data
                    }
                }
        """
        if not self._world.is_playing():
            return {}
        robots: Dict[str, Robot] = {
            name: cast(Robot, scene.get_object(name=name)) for name in self._robot_names
        }
        x_form_prims: Dict[str, XFormPrim] = {
            name: cast(XFormPrim, scene.get_object(name=name))
            for name in self._x_form_prim_names
        }
        contact_sensors: Dict[str, ContactSensor] = {
            name: cast(ContactSensor, scene.get_object(name=name))
            for name in self._contact_sensor_names
        }
        cameras: Dict[str, Camera] = {
            name: cast(Camera, scene.get_object(name=name))
            for name in self._camera_names
        }
        return {
            "phase": self._phase,
            "robots": {
                name: {
                    "joint_positions": robot.get_joint_positions().tolist(),
                    "applied_joint_positions": robot.get_applied_action().joint_positions.tolist(),
                }
                for name, robot in robots.items()
            },
            "x_form_prims": {
                name: {
                    "position": x_form_prim.get_world_pose()[0].tolist(),
                    "orientation": x_form_prim.get_world_pose()[1].tolist(),
                }
                for name, x_form_prim in x_form_prims.items()
            },
            "contact_sensors": {
                name: sensor.get_current_frame()
                for name, sensor in contact_sensors.items()
            },
            "cameras": {
                name: self._render(camera=camera) for name, camera in cameras.items()
            },
            "action": self._action_ref(),
        }

    @staticmethod
    def _reshape_and_save(
        rgb: np.ndarray, resolution: tuple[int, int], path: str
    ) -> None:
        """
        フレームの画像を保存する

        Args:
            rgb (np.ndarray): カメラから取得されたRGB画素配列
            resolution (tuple[int, int]): 解像度
            path (str): 画像の保存先
        """
        image_array: np.ndarray[Any, np.dtype[np.unsignedinteger]] = np.frombuffer(
            buffer=np.ascontiguousarray(rgb), dtype=np.uint8
        )
        image_array = image_array.reshape((resolution[1], resolution[0], 3))
        image: Image.Image = Image.fromarray(obj=image_array)
        image.save(fp=path)

    def _render(self, camera: Camera) -> str:
        """
        各フレームの画像保存に関する処理

        画像の保存は別スレッドで行われる

        Args:
            camera (Camera): Sceneから取得したCameraインスタンス

        Returns:
            str: Cameraの名前
        """
        if (
            self._world.current_time_step_index % get_settings().save_frame_interval
            != 0
        ):
            return ""
        path: str = os.path.join(
            self._temp_dir,
            camera.name,
            f"{self._world.current_time_step_index:06}.png",
        )
        image_data: np.ndarray = camera.get_rgb()
        threading.Thread(
            target=self._reshape_and_save,
            args=(image_data, camera.get_resolution(), path),
        ).start()
        return path
