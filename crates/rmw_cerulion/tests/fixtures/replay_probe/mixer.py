# SPDX-License-Identifier: AGPL-3.0-only
# Subscribes /a; a ROS-time timer at 7 Hz publishes sha256(state) on /out, where
# state folds every input in arrival order and every timer fire's ROS time, so
# the output is a pure function of the callback order and the inputs.
#   MIXER_WALLCLOCK=1  also folds time.time_ns() at each fire (the wall-clock control)
#   --mt               spins a MultiThreadedExecutor (2 threads) instead of rclpy.spin
import hashlib, os, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from common import CbLog

WALL = os.environ.get("MIXER_WALLCLOCK") == "1"


class Mixer(Node):
    def __init__(self):
        super().__init__("mixer")
        self.state = b"\0" * 32
        self.pub = self.create_publisher(String, "/out", 10)
        self.create_subscription(String, "/a", self.on_a, 10)
        self.log = CbLog("mixer")
        self.create_timer(1.0 / 7.0, self.tick)

    def on_a(self, m):
        self.state = hashlib.sha256(self.state + b"a" + m.data.encode()).digest()
        self.log.log(cb="a", inp=m.data)

    def tick(self):
        now = self.get_clock().now().nanoseconds
        if now == 0:
            return
        extra = str(time.time_ns()).encode() if WALL else b""
        self.state = hashlib.sha256(self.state + b"t" + str(now).encode() + extra).digest()
        m = String()
        m.data = self.state.hex()
        self.pub.publish(m)
        self.log.log(cb="timer", now=now, out=m.data)


def main():
    rclpy.init(args=sys.argv)
    n = Mixer()
    try:
        if "--mt" in sys.argv:
            from rclpy.executors import MultiThreadedExecutor

            ex = MultiThreadedExecutor(num_threads=2)
            ex.add_node(n)
            ex.spin()
        else:
            rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == "__main__":
    main()
