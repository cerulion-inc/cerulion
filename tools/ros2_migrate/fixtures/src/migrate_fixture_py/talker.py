# SPDX-License-Identifier: AGPL-3.0-only
# rclpy fixture: REPORT-ONLY. rclpy has no loaned-message API upstream, so
# the migrate verb never rewrites Python — it reports the node and the
# copy floor (one copy per side; two copies become one under the
# launcher's heap hook).
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PyTalker(Node):
    def __init__(self):
        super().__init__("py_talker")
        self.pub = self.create_publisher(String, "py_chatter", 10)

    def tick(self):
        msg = String()
        msg.data = "hello from rclpy"
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PyTalker()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
