# SPDX-License-Identifier: AGPL-3.0-only
"""Measured bounds for positional bookkeeping, in SI units."""
from rcpx.bounds_ratios import QUANT


def of(ex):
    from rcpx import bounds
    return bounds.of(ex)


def contact_error_m(B, speed_m_s, q, u):
    return abs(speed_m_s) * B.t_det() + B.quantum(q, u)


def motion_error_m(B, fit_error_m):
    return QUANT * fit_error_m


def support_error_m(B):
    support = B.space.support()
    if support is None or support[1] is None:
        return None, "prior", "the support plane uncertainty has not been measured or stated"
    _plane, sigma, source = support
    return sigma, source, "the support plane's measured or stated uncertainty"
