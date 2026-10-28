# SPDX-License-Identifier: AGPL-3.0-only
# Two nodes, each spun by its own SingleThreadedExecutor on its own thread:
#   left : subscribes /a, folds it, publishes the fold on /left; a 7 Hz ROS-time
#          timer folds its fire time
#   right: subscribes /left, folds it, publishes the fold on /right; a 5 Hz
#          ROS-time timer folds its fire time
# --block: right's /left callback waits until left runs its next callback (a
#          callback blocked on another executor's thread: the watchdog case).
import hashlib, sys, threading
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from common import CbLog

BLOCK = "--block" in sys.argv
LEFT_RAN = threading.Event()


class Folder(Node):
    def __init__(self, name, inp, out, hz):
        super().__init__(name)
        self.state = b"\0" * 32
        self.pub = self.create_publisher(String, out, 10)
        self.create_subscription(String, inp, self.on_in, 10)
        self.log = CbLog(name)
        self.create_timer(1.0 / hz, self.tick)

    def fold(self, tag, data):
        self.state = hashlib.sha256(self.state + tag + data).digest()
        m = String()
        m.data = self.state.hex()
        self.pub.publish(m)
        return m.data

    def on_in(self, m):
        if self.get_name() == "left":
            LEFT_RAN.set()
        elif BLOCK:
            LEFT_RAN.clear()
            LEFT_RAN.wait()
        self.log.log(cb="in", out=self.fold(b"i", m.data.encode()))

    def tick(self):
        now = self.get_clock().now().nanoseconds
        if now == 0:
            return
        if self.get_name() == "left":
            LEFT_RAN.set()
        self.log.log(cb="timer", now=now, out=self.fold(b"t", str(now).encode()))


def spin(node):
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except Exception:
        pass


def main():
    rclpy.init(args=sys.argv)
    left = Folder("left", "/a", "/left", 7.0)
    right = Folder("right", "/left", "/right", 5.0)
    threads = [threading.Thread(target=spin, args=(n,), daemon=True) for n in (left, right)]
    for t in threads:
        t.start()
    try:
        for t in threads:
            while t.is_alive():
                t.join(0.2)
    except KeyboardInterrupt:
        pass
    # Stop both executors through rcl (their waits wake on the shutdown guard),
    # so the process exits through a clean rmw_shutdown.
    LEFT_RAN.set()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    for t in threads:
        t.join(2.0)


if __name__ == "__main__":
    main()
