import cerulion
from cerulion.msgs.geometry_msgs import Vector3


@cerulion.node(period_ms=1)
class Builtin:
    out = cerulion.output(Vector3)

    def tick(self):
        out = self.out
        out.x = 1.5
        out.y = -2.0
        out.z = 0.25
