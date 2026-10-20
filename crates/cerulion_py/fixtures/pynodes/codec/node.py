import os

import cerulion as cer


@cer.node(period_ms=10)
class Codec:
    out = cer.output("Samples", max_slice_len_default=96)

    def tick(self):
        case = os.environ.get("CERULION_PYNODE_CASE")
        if case == "bad_yaml":
            self.publish_yaml("out", "name: 1\nsamples: []\n")
        elif case == "touched_twice":
            self.out.samples
            self.publish_yaml("out", 'name: ""\nsamples: []\n')
        elif case == "unknown_output":
            self.publish_dict("nope", {})
        elif case == "caught_mismatch":
            # A frame longer than its offset table says cannot fill the
            # loan exactly; the node catches the error, so nothing is sent.
            frame = self._cer_schemas.yaml_to_message("Samples", 'name: ""\nsamples: []\n')
            try:
                self._cer_publish_frame("out", frame + bytes(8))
            except cer.EncodeError as err:
                if err.kind != "FrameLoanSizeMismatch":
                    raise
        else:
            # A whole message from message YAML, no field packed by hand.
            self.publish_yaml("out", 'name: "laser"\nsamples: [1.5, -2.0, 0.25]\n')
