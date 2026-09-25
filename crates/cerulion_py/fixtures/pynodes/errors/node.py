import os
import threading
import time

import cerulion as cer

if os.environ.get("CERULION_PYNODE_CASE") == "import_error":
    raise ImportError("fixture import failure")

if os.environ.get("CERULION_PYNODE_CASE") == "missing_tick":
    @cer.node(period_ms=10)
    class Errors:
        inp = cer.input("Probe")
        out = cer.output("Probe")
        out2 = cer.output("Probe")
else:
    @cer.node(period_ms=10)
    class Errors:
        inp = cer.input("Probe")
        out = cer.output("Probe")
        out2 = cer.output("Probe")

        def init(self, ctx):
            if os.environ.get("CERULION_PYNODE_CASE") == "spawn_thread":
                threading.Thread(target=time.sleep, args=(5,), daemon=True).start()
                time.sleep(0.02)
            if os.environ.get("CERULION_PYNODE_CASE") == "env_lookup":
                seen = (
                    ctx.env("CERULION_PYNODE_ABSENT_KEY"),
                    ctx.env("CERULION_PYNODE_ABSENT_KEY", "fallback"),
                    ctx.env("CERULION_PYNODE_CASE"),
                )
                if seen != (None, "fallback", "env_lookup"):
                    raise RuntimeError(f"unexpected env lookups {seen!r}")

        def tick(self):
            case = os.environ.get("CERULION_PYNODE_CASE")
            if case == "loan_length_type":
                for length in (1.9, True, -1, "2"):
                    try:
                        self.loan("out", value=length)
                    except TypeError as error:
                        if "must be a non-negative int" not in str(error):
                            raise
                    else:
                        raise RuntimeError(f"loan accepted length {length!r}")
            if case == "tick_exception":
                raise RuntimeError("fixture tick failure")
            if case == "retain_view" and self.inp is not None:
                self.kept = memoryview(self.inp._payload)
            if case == "retain_then_raise" and self.inp is not None:
                self.kept = memoryview(self.inp._payload)
                raise RuntimeError("fixture retained input then raised")
            if case == "retain_loan":
                if hasattr(self, "kept_loan_view"):
                    if bytes(self.kept_loan_view)[:4] != bytes((0xD4, 0xC3, 0xB2, 0xA1)):
                        raise RuntimeError("retained loan bytes changed")
                    self.kept_loan_view.release()
                else:
                    self.out.value = 0xA1B2C3D4
                    self.kept_loan_view = memoryview(self.out._payload)
                    return
            if case == "retained_tick":
                if not hasattr(self, "kept_tick"):
                    self.kept_tick = self._cer_tick
                    return
                self.kept_tick.loan("out", [])
            if case == "retained_tick_loan_after_error":
                if not hasattr(self, "kept_tick"):
                    self.kept_tick = self._cer_tick
                    return
                if not hasattr(self, "failed"):
                    self.failed = True
                    raise RuntimeError("fixture retained tick loan failure")
                self.kept_tick.loan("out", [])
            if case == "retain_second_output":
                self.out.value = self.inp.value if self.inp is not None else 0
                self.kept = memoryview(self.out2._payload)
            if self.inp is not None:
                self.out.value = self.inp.value

        def shutdown(self):
            if os.environ.get("CERULION_PYNODE_CASE") == "shutdown_exception":
                raise RuntimeError("fixture shutdown failure")
