# SPDX-License-Identifier: AGPL-3.0-only
"""Benchmark bridges: an outside benchmark's episode served over the same socket protocol rcpx episodes use.

Each benchmark keeps its own physics, success check and ground truth in an env host process that runs in the
benchmark's own Python (bench/ext/<benchmark>/envhost.py); the rcpx side talks to it over a unix socket with the
framing in bench/ext/envproto.py. Files the env hosts import are standard library only and valid on Python 3.10.
"""
