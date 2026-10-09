import cerulion as cer


@cer.node(period_ms=10)
class Counter:
    inp = cer.input("Probe", depth=1)
    out = cer.output("Probe")

    def init(self, ctx):
        self.helpers = None

    def tick(self):
        if self.inp is not None:
            import helpers

            self.out.value = helpers.transform(self.inp.value)
            self.helpers = helpers

    def shutdown(self):
        # The very module object the ticks used: another node type's
        # `helpers`, or a fresh copy of this one without its state, is not it.
        import helpers

        if self.helpers is not None and helpers is not self.helpers:
            raise RuntimeError(
                f"shutdown imported a foreign or reloaded helpers: {helpers.__file__}"
            )
