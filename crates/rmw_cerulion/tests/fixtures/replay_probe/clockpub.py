# SPDX-License-Identifier: AGPL-3.0-only
# /clock at a CLOCK_PERIOD wall cadence; the VALUES are deterministic (k * 5 ms of
# ROS time). Starts ticking once CLOCK_SUBS subscribers match /clock; stops after
# TICKS ticks (the process stays up until SIGINT) and writes <PROBE_OUT>/clockpub.done.
import os, sys
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from common import CbLog

NSUBS = int(os.environ.get("CLOCK_SUBS", "2"))
TICKS = int(os.environ.get("TICKS", "4000"))
PERIOD = float(os.environ.get("CLOCK_PERIOD", "0.0005"))


class ClockPub(Node):
    def __init__(self):
        super().__init__("clockpub")
        self.pub = self.create_publisher(Clock, "/clock", 10)
        self.k = 0
        self.started = False
        self.log = CbLog("clockpub")
        self.create_timer(PERIOD, self.tick)  # a wall timer: this node is the clock SOURCE

    def tick(self):
        if not self.started:
            if self.pub.get_subscription_count() < NSUBS:
                return
            self.started = True
            self.log.log(cb="start", subs=self.pub.get_subscription_count())
            return
        if self.k >= TICKS:
            return
        self.k += 1
        ns = self.k * 5_000_000
        m = Clock()
        m.clock.sec = ns // 1_000_000_000
        m.clock.nanosec = ns % 1_000_000_000
        self.pub.publish(m)
        if self.k == TICKS:
            open(os.path.join(os.environ.get("PROBE_OUT", "."), "clockpub.done"), "w").close()


def main():
    rclpy.init(args=sys.argv)
    n = ClockPub()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == "__main__":
    main()
