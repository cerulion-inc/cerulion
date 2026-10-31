# SPDX-License-Identifier: AGPL-3.0-only
# A service pair on ROS time (run both with use_sim_time:=true).
#   svc.py server : node `adder` serves /add (example_interfaces/AddTwoInts),
#                   answering a + b + (the number of requests served before)
#   svc.py client : node `asker`; a 10 Hz ROS-time timer sends a = n, b = n * n
#                   once /add is available and no request is pending (REQUESTS
#                   requests, default 30); each response is folded into a hash
#                   published on /out
import hashlib, os, sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
from common import CbLog

REQUESTS = int(os.environ.get("REQUESTS", "30"))


class Adder(Node):
    def __init__(self):
        super().__init__("adder")
        self.served = 0
        self.log = CbLog("adder")
        self.create_service(AddTwoInts, "/add", self.on_add)

    def on_add(self, req, resp):
        resp.sum = req.a + req.b + self.served
        self.served += 1
        self.log.log(cb="add", inp=f"{req.a},{req.b}", out=str(resp.sum))
        return resp


class Asker(Node):
    def __init__(self):
        super().__init__("asker")
        self.state = b"\0" * 32
        self.n = 0
        self.pending = None
        self.cli = self.create_client(AddTwoInts, "/add")
        self.pub = self.create_publisher(String, "/out", 10)
        self.log = CbLog("asker")
        self.create_timer(0.1, self.tick)

    def tick(self):
        if self.get_clock().now().nanoseconds == 0 or self.n >= REQUESTS:
            return
        if self.pending is not None or not self.cli.service_is_ready():
            return
        self.n += 1
        req = AddTwoInts.Request()
        req.a = self.n
        req.b = self.n * self.n
        self.pending = self.cli.call_async(req)
        self.pending.add_done_callback(self.on_resp)
        self.log.log(cb="send", out=f"{req.a},{req.b}")

    def on_resp(self, fut):
        self.pending = None
        s = fut.result().sum
        self.state = hashlib.sha256(self.state + str(s).encode()).digest()
        m = String()
        m.data = self.state.hex()
        self.pub.publish(m)
        self.log.log(cb="resp", inp=str(s), out=m.data)


def main():
    rclpy.init(args=sys.argv)
    n = Adder() if "server" in sys.argv[1:2] else Asker()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == "__main__":
    main()
