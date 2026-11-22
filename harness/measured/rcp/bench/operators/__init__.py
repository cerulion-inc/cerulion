# SPDX-License-Identifier: AGPL-3.0-only
"""Operators for rcpx episodes: programs that drive one episode's RCP server the way a person or a model would.

The operator brief (operator_brief.md, next to this file) is the one system text every operator kind starts from:
load it with load_brief() rather than writing another one, so every operator gets the same words.
"""
import os

BRIEF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "operator_brief.md")


def load_brief(path=BRIEF_PATH):
    """The operator brief as one stripped text."""
    with open(path, encoding="utf-8") as f:
        return f.read().strip()
