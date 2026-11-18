# SPDX-License-Identifier: AGPL-3.0-only
"""The body contract (ARCHITECTURE.md section 2.3, C0) and a shim over today's backend. Nothing else lives here.

    from rcpx.body import contract, checks, exits      # the eight calls, the shape checks, the one exit table
    from rcpx.body.shim import BackendBody             # today's five-method backend behind the eight calls

No file in this folder names a robot, a device, a task or a benchmark: what a body is (its groups, holders,
outputs, sensors, safe states, controller) is data its owner keeps and hands to the adapter. The request and
answer vectors every adapter must pass are in tests/conformance.
"""

from rcpx.body import checks, contract, exits  # noqa: F401
from rcpx.body.contract import Body  # noqa: F401
