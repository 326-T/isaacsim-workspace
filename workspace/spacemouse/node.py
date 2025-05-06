import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class SpacemouseNode(Node):
    def __init__(self):
        super().__init__("spacemouse")
        self.pub = self.create_publisher(Float32, "spacemouse/input", 10)
        # 5Hz でランダム入力を publish
        self.create_timer(1 / 5, self.timer_cb)

    def timer_cb(self):
        # ダミー入力：-1.0～1.0 のランダム値
        v = random.uniform(-1.0, 1.0)
        self.pub.publish(Float32(data=v))
        self.get_logger().info(f"[spacemouse] input={v:.2f}")


def main():
    rclpy.init()
    node = SpacemouseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
