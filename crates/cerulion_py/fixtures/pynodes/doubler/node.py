import cerulion as cer


@cer.node(period_ms=10)
class Doubler:
    inp = cer.input("Probe")
    out = cer.output("Probe")

    def tick(self):
        if self.inp is not None:
            import helpers

            self.out.value = helpers.transform(self.inp.value)
