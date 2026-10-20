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
        else:
            # A whole message from message YAML, no field packed by hand.
            self.publish_yaml("out", 'name: "laser"\nsamples: [1.5, -2.0, 0.25]\n')
