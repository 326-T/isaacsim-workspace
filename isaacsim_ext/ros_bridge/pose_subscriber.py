from operator import itemgetter
from typing import List, Optional

import numpy as np
import rclpy
from interfaces.msg import SpaceMouseData
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.manipulators.grippers import ParallelGripper
from pydantic import BaseModel
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from isaacsim_ext.controller.rmpflow_controller import RMPFlowController


class DesiredState(BaseModel):
    position: np.ndarray = np.zeros(3)
    orientation: np.ndarray = np.array([0, 0, 0, 1])  # Quaternion (x, y, z, w)
    gripper: List[int] = [0, 0]  # [close, open] state of the gripper


class Action(BaseModel):
    arm_action: ArticulationAction
    gripper_action: Optional[ArticulationAction] = None


def multiply_quaternions(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    r1 = R.from_quat(q1)
    r2 = R.from_quat(q2)
    r_result = r1 * r2
    return r_result.as_quat()


class PoseSubscriber(Node):
    """
    SpaseMouseや推論モデルの出力を受け取り、ロボットアームへの指令値を生成するROSノード

    FIXME: 本当はロボットアームの現在のステートもROS経由で購読すべきだが,
    IsaacSimのRMPFlow実装を利用しているため, Articulationを直接参照している.
    できれば, ここのROSブリッジではArticulationActionそのものを購読するようにしたい.
    """

    def __init__(
        self, robot_arm: Articulation, target: XFormPrim, gripper: ParallelGripper
    ):
        super().__init__("pose_subscriber")
        self.subscription = self.create_subscription(
            SpaceMouseData, "spacemouse/data", self.listener_callback, 10
        )
        self.buffer: DesiredState = DesiredState()
        self.robot_arm = robot_arm
        self.target = target
        self.gripper = gripper
        self.rmpflow_controller = RMPFlowController(robot_articulation=self.robot_arm)

    def get_applied_action(self) -> Action:
        current_position, current_orientation = self.target.get_world_pose()
        next_position = current_position + self.buffer.position
        next_orientation = multiply_quaternions(
            current_orientation, self.buffer.orientation
        )
        self.target.set_world_pose(position=next_position, orientation=next_orientation)
        arm_action: ArticulationAction = self.rmpflow_controller.forward(
            target_end_effector_position=next_position,
            target_end_effector_orientation=next_orientation,
        )

        if self.buffer.gripper == [1, 0]:
            gripper_action = self.gripper.forward("close")
        elif self.buffer.gripper == [0, 1]:
            gripper_action = self.gripper.forward("open")
        else:
            return Action(arm_action=arm_action)

        gripper_action = ArticulationAction(
            joint_positions=itemgetter(7, 9)(gripper_action.joint_positions),
            joint_velocities=itemgetter(7, 9)(gripper_action.joint_velocities),
            joint_efforts=itemgetter(7, 9)(gripper_action.joint_efforts),
        )

        return Action(arm_action=arm_action, gripper_action=gripper_action)

    def listener_callback(self, msg: SpaceMouseData):
        self.get_logger().debug(
            f"Received data: position={msg.pose.position}, orientation={msg.pose.orientation}, buttons={msg.gripper}"
        )
        self.buffer = DesiredState(
            position=np.array(
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            ),
            orientation=np.array(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ),
            gripper=[int(msg.gripper[0]), int(msg.gripper[1])],
        )

    def spin(self):
        rclpy.init()
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            self.destroy_node()
            rclpy.shutdown()
