#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class LoggerSubscriber(Node):
    def __init__(self):
        super().__init__("logger_node")
        self.create_subscription(Float32, "spacemouse/input", self.input_cb, 10)
        self.create_subscription(Float32, "controller/target", self.controller_cb, 10)
        self.create_subscription(Float32, "sim/state", self.isaac_cb, 10)
        self.get_logger().info("Logger node has been started")

    def input_cb(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}" from spacemouse/input"')

    def controller_cb(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}" from controller/target"')

    def isaac_cb(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}" from sim/state"')


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
