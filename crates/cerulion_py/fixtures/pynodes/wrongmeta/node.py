import cerulion as cer


@cer.node(period_ms=10)
class WrongMeta:
    inp = cer.input("Probe")
    out = cer.output("Probe")

    def tick(self):
        self.out.value = self.inp.value
