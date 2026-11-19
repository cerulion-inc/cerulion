# SPDX-License-Identifier: AGPL-3.0-only
"""The closed list of measured-bound ratios in A7."""

TWO_READ = 3  # A threshold on two readings, in sigma.
ONE_READ = 5  # A threshold on one reading, in sigma.
JOINT = 4  # A joint torque threshold, in sigma.
BAND = 3  # A position band, in sigma.
QUANT = 2  # Quanta: one step can be rounding; resolution needs distinct samples.
GIVE = 1 / 2  # A give is under half the peak; the first pass brakes at half deceleration.
REL = 1 / 5  # The accepted relative error of a measured noise or fit.
DOUBLE = 2  # The search step of the self-test passes.
