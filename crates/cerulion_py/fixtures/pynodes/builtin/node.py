import cerulion as cer


@cer.node(period_ms=1)
class Builtin:
    out = cer.output("geometry_msgs/Vector3")

    def tick(self):
        out = self.out
        out.x = 1.5
        out.y = -2.0
        out.z = 0.25
