import cerulion
from cerulion.msgs import Probe


@cerulion.node(period_ms=10)
class WrongMeta:
    inp = cerulion.input(Probe)
    out = cerulion.output(Probe)

    def tick(self):
        self.out.value = self.inp.value
