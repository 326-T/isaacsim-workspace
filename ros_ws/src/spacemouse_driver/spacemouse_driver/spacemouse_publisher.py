from typing import NamedTuple

import pyspacemouse
import rclpy
from interfaces.msg import SpaceMouseData
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R


class SpaceMousePublisher(Node):
    def __init__(self):
        super().__init__("spacemouse_publisher")

        # パブリッシャーの作成
        self.publisher = self.create_publisher(SpaceMouseData, "spacemouse/data", 10)

        # 10Hzでパブリッシュ
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 初期化
        success = pyspacemouse.open(
            dof_callback=pyspacemouse.print_state,
            button_callback=pyspacemouse.print_buttons,
        )
        if not success:
            self.get_logger().error("Failed to open SpaceMouse device")
            rclpy.shutdown()
            return

        self.get_logger().info("SpaceMouse publisher started")

    def timer_callback(self):
        state: NamedTuple = pyspacemouse.read()

        # メッセージの作成
        msg = SpaceMouseData()
        msg.pose.position.x = float(state.x)
        msg.pose.position.y = float(state.y)
        msg.pose.position.z = float(state.z)
        yaw = float(state.yaw)
        pitch = float(state.pitch)
        roll = float(state.roll)
        # Use scipy to convert Euler angles (roll, pitch, yaw) to quaternion
        quat = R.from_euler(
            seq="xyz", angles=[roll, pitch, yaw], degrees=False
        ).as_quat()
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        msg.gripper = [state.buttons[0] > 0, state.buttons[1] > 0]

        # パブリッシュ
        self.publisher.publish(msg)

        # ログ出力（1秒ごと）
        self.get_logger().debug(
            f"Publishing: pos=({msg.pose.position.x:.3f}, "
            f"{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}), "
            f"orient=({msg.pose.orientation.x:.3f}, "
            f"gripper={msg.gripper}"
        )


def main(args=None):
    rclpy.init(args=args)

    spacemouse_publisher = SpaceMousePublisher()

    try:
        rclpy.spin(spacemouse_publisher)
    except KeyboardInterrupt:
        pass

    spacemouse_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
