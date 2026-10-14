import cerulion as cer


@cer.node(period_ms=10)
class VarLen:
    out = cer.output("Samples", max_slice_len_default=96)

    def tick(self):
        # Variable lengths are ELEMENT counts: five characters, three doubles.
        out = self.loan("out", name=5, samples=3)
        out.name = "laser"
        out.samples = [1.5, -2.0, 0.25]
