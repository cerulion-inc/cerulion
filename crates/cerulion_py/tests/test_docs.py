"""Execute the ```python blocks in docs/python.md in order, in one namespace.

A ```text block immediately following a ```python block is that block's expected
stdout, asserted with capsys.
"""

from pathlib import Path

import pytest

DOC = Path(__file__).resolve().parents[3] / "docs" / "python.md"


def extract_blocks(text):
    """Every fenced block as ``(lang, body)`` in document order.

    A line-by-line walk rather than a regex: a fence that is never closed is
    an error here, where a non-greedy regex would silently drop it (and every
    block after it), leaving new Python examples unexecuted while the
    "some python block exists" check still passes."""
    blocks = []
    lang = None
    body = []
    opened_at = 0
    for lineno, line in enumerate(text.split("\n"), start=1):
        if lang is None:
            if line.startswith("```"):
                lang = line[3:].strip()
                body = []
                opened_at = lineno
        elif line.startswith("```"):
            blocks.append((lang, "".join(f"{row}\n" for row in body)))
            lang = None
        else:
            body.append(line)
    if lang is not None:
        pytest.fail(f"docs/python.md: the ```{lang} fence opened on line {opened_at} is never closed")
    return blocks


def test_extract_blocks_rejects_an_unterminated_fence():
    """The walk's own contract, on hand-written input."""
    assert extract_blocks("x\n```python\nprint(1)\n```\n```text\n1\n```\n") == [
        ("python", "print(1)\n"),
        ("text", "1\n"),
    ]
    with pytest.raises(pytest.fail.Exception, match="opened on line 5 is never closed"):
        extract_blocks("```python\nprint(1)\n```\n\n```python\nprint(2)\n")


def test_docs_code_blocks(capsys):
    assert DOC.is_file(), "docs/python.md is missing - test_docs pins it"
    blocks = extract_blocks(DOC.read_text())
    assert any(lang == "python" for lang, _ in blocks), "no python blocks found"
    # Pairing contract: every `python` block is followed by exactly one
    # `text` block - its expected stdout. A bare `text` block (no
    # preceding python) or a python block with no text block fails.
    namespace = {}
    pending_python = -1
    for i, (lang, body) in enumerate(blocks):
        if lang not in ("python", "text"):
            if pending_python >= 0:
                pytest.fail(
                    f"python block {pending_python} is followed by a `{lang}` block {i} "
                    "before its `text` stdout block"
                )
            continue
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
