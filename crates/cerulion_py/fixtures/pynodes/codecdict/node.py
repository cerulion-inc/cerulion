import cerulion
from cerulion.msgs import Probe


@cerulion.node(period_ms=10)
class CodecDict:
    inp = cerulion.input(Probe, depth=1)
    out = cerulion.output(Probe)

    def tick(self):
        # The received message read back as a dict, republished from one.
        if self.inp is not None:
            seen = self.inp.to_dict()
            self.publish_dict("out", {"value": seen["value"] + 100})
