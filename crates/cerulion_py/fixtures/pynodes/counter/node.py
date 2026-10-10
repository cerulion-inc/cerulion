import cerulion as cer


@cer.node(period_ms=10)
class Counter:
    inp = cer.input("Probe", depth=1)
    out = cer.output("Probe")

    def init(self, ctx):
        # Imported at init, so a second instance of this type loading later in
        # the same process must leave this very module in place for the ticks.
        import helpers

        self.helpers = helpers

    def tick(self):
        if self.inp is not None:
            import helpers

            if helpers is not self.helpers:
                raise RuntimeError(
                    f"tick imported a foreign or reloaded helpers: {helpers.__file__}"
                )
            self.out.value = helpers.transform(self.inp.value)

    def shutdown(self):
        # The very module object init and the ticks used: another node type's
        # `helpers`, or a fresh copy of this one without its state, is not it.
        import helpers

        if helpers is not self.helpers:
            raise RuntimeError(
                f"shutdown imported a foreign or reloaded helpers: {helpers.__file__}"
            )
