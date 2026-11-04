# SPDX-License-Identifier: AGPL-3.0-only
# A ROS-time timer at 10 Hz publishes a counter on /a (run with use_sim_time:=true).
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from common import CbLog


class Talker(Node):
    def __init__(self):
        super().__init__("talker_a")
        self.pub = self.create_publisher(String, "/a", 10)
        self.n = 0
        self.log = CbLog("talker_a")
        self.create_timer(0.1, self.tick)

    def tick(self):
        now = self.get_clock().now().nanoseconds
        if now == 0:
            return
        self.n += 1
        m = String()
        m.data = f"a:{self.n}"
        self.pub.publish(m)
        self.log.log(cb="timer", now=now, out=m.data)


def main():
    rclpy.init(args=sys.argv)
    n = Talker()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == "__main__":
    main()
