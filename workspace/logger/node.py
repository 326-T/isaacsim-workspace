#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LoggerSubscriber(Node):
    def __init__(self):
        super().__init__("logger_node")
        self.create_subscription(String, "human_input", self.human_input_cb, 10)
        self.create_subscription(String, "integrated", self.integrated_cb, 10)
        self.create_subscription(String, "isaacsim_state", self.isaac_cb, 10)
        self.get_logger().info("Logger node has been started")

    def human_input_cb(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}" from human_input"')

    def integrated_cb(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}" from integrated"')

    def isaac_cb(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}" from isaacsim_state"')


def main():
    rclpy.init()
    subscriber = LoggerSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
