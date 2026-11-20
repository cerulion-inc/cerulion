# SPDX-License-Identifier: AGPL-3.0-only
"""What a guard's contact says of the speed it was met at, and the place error of a
stored contact. No robot is named; nothing is typed.

    met(ex, contact, before) -> the contact, its tip speed read where the tip MET it
"""

import numpy as np

MM = 1000.0


def met(ex, contact, before):
    """The contact with its tip speed read where the tip MET it: the larger of the speed at the first loaded
    reading and at the reading before it (the last one under the threshold: the contact may already have braked
    the tip by the first). A contact stored at speed carries that speed's place error."""
    if contact is None or before is None or getattr(before, "qd", None) is None:
        return contact
    v = float(np.linalg.norm(ex.model.jac(before.q)[:3] @ before.qd)) * MM
    if contact.get("tip_speed_mm_s") is None or v > float(contact["tip_speed_mm_s"]):
        contact["tip_speed_mm_s"] = round(v, 1)
    return contact
