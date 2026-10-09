import cerulion as cer
import helpers


@cer.node(period_ms=10)
class Counter:
    inp = cer.input("Probe", depth=1)
    out = cer.output("Probe")

    def tick(self):
        if self.inp is not None:
            self.out.value = helpers.transform(self.inp.value)
