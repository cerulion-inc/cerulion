"""Execute the ```python blocks in docs/python.md in order, in one namespace.

A ```text block immediately following a ```python block is that block's expected
stdout, asserted with capsys.
"""

import re
from pathlib import Path

import pytest

DOC = Path(__file__).resolve().parents[3] / "docs" / "python.md"


def extract_blocks():
    text = DOC.read_text()
    blocks = re.findall(r"```(python|text)\n(.*?)```", text, re.DOTALL)
    return blocks


def test_docs_code_blocks(capsys):
    assert DOC.is_file(), "docs/python.md is missing - test_docs pins it"
    blocks = extract_blocks()
    assert any(lang == "python" for lang, _ in blocks), "no python blocks found"
    # Pairing contract: every `python` block is followed by exactly one
    # `text` block - its expected stdout. A bare `text` block (no
    # preceding python) or a python block with no text block fails.
    namespace = {}
    pending_python = -1
    for i, (lang, body) in enumerate(blocks):
        if lang == "python":
            if pending_python >= 0:
                pytest.fail(
                    f"python block {pending_python} is followed by python block {i} "
                    "with no intervening `text` stdout block"
                )
            pending_python = i
            capsys.readouterr()
            exec(compile(body, "docs/python.md", "exec"), namespace)
        else:
            if pending_python < 0:
                pytest.fail(f"text block {i} is not preceded by a python block")
            assert capsys.readouterr().out == body
            pending_python = -1
    if pending_python >= 0:
        pytest.fail(f"python block {pending_python} has no following `text` stdout block")
