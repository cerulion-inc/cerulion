# SPDX-License-Identifier: AGPL-3.0-only
# Subscribes /a; a ROS-time timer at 7 Hz publishes sha256(state) on /out, where
# state folds every input in arrival order and every timer fire's ROS time, so
# the output is a pure function of the callback order and the inputs.
#   MIXER_WALLCLOCK=1  also folds time.time_ns() at each fire (the wall-clock control)
#   --mt               spins a MultiThreadedExecutor (2 threads) instead of rclpy.spin
#   MIXER_SIDE_THREAD=1 also publishes /side from a plain threading.Thread every
#                      10 ms of wall time (a thread outside every executor)
#   MIXER_RAW=1        also subscribes /a with raw=True (a serialized take)
#   MIXER_EXIT_AFTER_A=N exits after its Nth /a callback (set on a replay only:
#                      the recording's later takes are never made)
import hashlib, os, sys, threading, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from common import CbLog

WALL = os.environ.get("MIXER_WALLCLOCK") == "1"
SIDE = os.environ.get("MIXER_SIDE_THREAD") == "1"
RAW = os.environ.get("MIXER_RAW") == "1"
EXIT_AFTER_A = int(os.environ.get("MIXER_EXIT_AFTER_A", "0"))


class Mixer(Node):
    def __init__(self):
        super().__init__("mixer")
        self.state = b"\0" * 32
        self.pub = self.create_publisher(String, "/out", 10)
        self.create_subscription(String, "/a", self.on_a, 10)
        self.log = CbLog("mixer")
        self.create_timer(1.0 / 7.0, self.tick)
        if RAW:
            self.create_subscription(String, "/a", self.on_raw, 10, raw=True)
        if SIDE:
            self.side = self.create_publisher(String, "/side", 10)
            threading.Thread(target=self.side_loop, daemon=True).start()

    def side_loop(self):
        n = 0
        while rclpy.ok():
            n += 1
            m = String()
            m.data = f"side:{n}"
            self.side.publish(m)
            time.sleep(0.01)

    def on_raw(self, b):
        self.log.log(cb="raw", inp=str(len(b)))

    def on_a(self, m):
        self.state = hashlib.sha256(self.state + b"a" + m.data.encode()).digest()
        self.log.log(cb="a", inp=m.data)
        self.a_seen = getattr(self, "a_seen", 0) + 1
        if EXIT_AFTER_A and self.a_seen >= EXIT_AFTER_A:
            raise SystemExit(0)

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
