import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from message_filters import Subscriber, ApproximateTimeSynchronizer


class IntegratorNode(Node):
    def __init__(self):
        super().__init__("sync_approx")
        sub_t = Subscriber(self, String, "human_input")
        sub_p = Subscriber(self, String, "isaacsim_state")
        ats = ApproximateTimeSynchronizer(
            [sub_t, sub_p],
            queue_size=10,
            slop=0.05,  # 最大50msのズレを許容
            allow_headerless=True,
        )
        ats.registerCallback(self.callback)
        self.pub = self.create_publisher(String, "integrated", 10)

    def callback(self, human_input, isaacsim_state):
        msg = String()
        msg.data = f"Integrated: {human_input.data}, {isaacsim_state.data}"
        self.pub.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")


def main():
    rclpy.init()
    subscriber = IntegratorNode()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
