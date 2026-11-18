# SPDX-License-Identifier: AGPL-3.0-only
"""What the env host reads of the body's own sensors beyond one value per control step (round six, the design notes
C5 and C11): the depth picture of a camera and the wrist force sensor at the physics rate. Runs inside the env host
(bench/ext/libero/envhost.py), on its numpy and its model handles: this file imports nothing.

- depth_facts(m, np, gl): the depth buffer's own facts: the near and far planes (m), the buffer's step turned to
  metres at 1 m (the error a simulator states for its depth: it grows with the square of the distance) and,
  where it was measured for the renderer in use, WHERE INSIDE A PIXEL the depth picture samples its ray
  ("sample_px": [du, dv], the design rulings): the depth at [v, u] is the depth along the ray through
  (u + du, v + dv), u to the right and v down, the pixel being the square [u, u + 1) x [v, v + 1). A renderer
  with no measured entry states none, and a reader then takes the pixel's middle and says so.
- depth_metres(buf, facts, np): the buffer's values (0 at the near plane, 1 at the far one, row 0 at the bottom as
  the renderer gives them) as metres along the optical axis, row 0 at the top.
- FastWatch(sim, d, np, force_adr, vel_idx): wraps sim.step and keeps, for the control step in hand, the raw wrist
  force sensor and the arm's joint speeds after every physics step, the joint speed before the first of them and
  the force's mean. Reading changes nothing in the physics.
"""

DEPTH_BITS = 24   # the renderer's depth buffer: its values step by 2 ** -DEPTH_BITS of the near to far range
# Where inside a pixel the depth picture samples its ray, per renderer (MUJOCO_GL): (du, dv), u right and v down.
# MEASURED, not assumed (tests/r6_p2_g1_probe.py sample; checked by tests/test_r6_gate_instruments.py --only masks):
# on the software renderer the host's own ray cast through (u + 0.375, v + 0.875) agrees with the depth picture
# within 0.01 mm on 90 percent of 600 pixels of each camera, where the ray through the middle is 1.3 mm off in the
# median: the depth is the first of the renderer's four sub-samples, the colour their mean. No entry: not measured.
DEPTH_SAMPLE = {"osmesa": (0.375, 0.875)}


def depth_facts(m, np, gl=None):
    near, far = float(m.vis.map.znear * m.stat.extent), float(m.vis.map.zfar * m.stat.extent)
    k = 1.0 - near / far
    out = {"near_m": near, "far_m": far, "step_m_at_1m": 2.0 ** -DEPTH_BITS * k / near,
           "unit": "m along the optical axis", "grows": "square",
           "source": f"the simulator's depth buffer: its own step ({DEPTH_BITS} bits between the near and far planes)"}
    at = DEPTH_SAMPLE.get(str(gl or "").lower())
    if at is not None:
        out["sample_px"] = [at[0], at[1]]
        out["sample_source"] = (f"measured for this renderer ({gl}): the depth at pixel [v, u] is the depth along "
                                f"the ray through (u + {at[0]}, v + {at[1]}), u right, v down; the colour is the "
                                "pixel's mean")
    return out


def depth_metres(buf, facts, np):
    near, far = facts["near_m"], facts["far_m"]
    return (near / (1.0 - np.asarray(buf[::-1], dtype=np.float64) * (1.0 - near / far))).astype(np.float32)


class FastWatch:
    def __init__(self, sim, d, np, force_adr, vel_idx):
        self.np, self.d, self.adr, self.vel = np, d, int(force_adr), vel_idx
        self.mean = np.zeros(3)
        self.begin()
        step = sim.step

        def watched():
            step()
            self.force.append(np.array(d.sensordata[self.adr:self.adr + 3]))
            self.qd.append(np.array(d.qvel[self.vel]))
        sim.step = watched

    def begin(self):
        """Before a control step: forget the last one's samples, keep the joint speed the step starts from."""
        self.qd0 = self.np.array(self.d.qvel[self.vel])
        self.force, self.qd = [], []

    def end(self):
        """After a control step: the mean raw force over its physics steps."""
        if self.force:
            self.mean = self.np.mean(self.force, axis=0)
        return self.mean

    def block(self, dt):
        return {"dt": float(dt), "wrist_force_raw": self.force, "qd": self.qd, "qd0": self.qd0}
