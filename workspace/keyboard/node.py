#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardNode(Node):
    def __init__(self):
        super().__init__("keyboard_node")
        self.publisher = self.create_publisher(String, "human_input", 10)
        self.timer = self.create_timer(5.0, self.publish_message)
        self.get_logger().info("Keyboard node has been started")

    def publish_message(self):
        msg = String()
        msg.data = "Hello"
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main():
    rclpy.init()
    publisher = KeyboardNode()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
