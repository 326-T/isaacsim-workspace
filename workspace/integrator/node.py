import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Empty


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")
        self.latest_state = None
        self.latest_input = None

        # 各トピックを購読
        self.create_subscription(Float32, "sim/state", self.state_cb, 10)
        self.create_subscription(Float32, "spacemouse/input", self.input_cb, 10)
        self.create_subscription(Empty, "sim/step", self.step_cb, 10)

        # 目標値 publisher
        self.pub_target = self.create_publisher(Float32, "controller/target", 10)

    def state_cb(self, msg):
        self.latest_state = msg.data

    def input_cb(self, msg):
        self.latest_input = msg.data

    def step_cb(self, _: Empty):
        # sim/step が来たタイミングで最新の state と input を使って
        # 次の目標値を計算
        if self.latest_state is None or self.latest_input is None:
            self.get_logger().warn("[controller] missing state or input → skip")
            return

        # ダミー統合：単純に足し算
        target = self.latest_state + self.latest_input
        self.pub_target.publish(Float32(data=target))
        self.get_logger().info(f"[controller] publish target={target:.2f}")


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
