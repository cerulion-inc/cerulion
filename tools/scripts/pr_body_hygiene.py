#!/usr/bin/env python3
"""pr_body_hygiene.py: take review-bot badge markup out of a pull request body.

WHY THIS FILE EXISTS. This repository squash-merges, and its squash commit
message is the pull request BODY. Two review apps edit that body after the
pull request opens, each appending a badge block wrapped in HTML comment
markers. Left alone, that markup lands verbatim in the history of `main`,
where it is neither readable nor removable, and it is the first thing anyone
sees in `git log`. Asking every contributor to delete it by hand before
merging is a rule people forget exactly once, and the cost of forgetting is
permanent.

So this reads a body and returns it with the bot markup gone and nothing else
touched. `.github/workflows/pr-body-hygiene.yml` is its one caller.

WHAT IT REMOVES, and only this:

  1. A DELIMITED BLOCK: everything from a known opening HTML comment marker
     through its matching closing marker, both markers included.
  2. A STANDALONE MARKER LINE: a line holding nothing but a known bot HTML
     comment that by design never gets a closing pair.
  3. A BADGE LINE: a line outside every removed block holding nothing but
     review-bot badge markup, decided by a CLOSED LIST of asset URL prefixes
     (`BOT_ASSET_PREFIXES`) and never by element name alone. A line that MIXES
     the author's prose with such markup is kept IN FULL: half-scrubbing
     somebody's sentence is worse than leaving a badge in it. This rule is the
     net for a bot that emits a badge with no markers around it; every body
     seen so far is handled by rule 1 alone.
  4. The blank lines a removal leaves behind, AT THE SEAM ONLY. Where a
     removal sat between two pieces of text, the gap closes to one blank line
     (the markdown paragraph break). Where it sat at the start or the end of
     the body, the gap closes to nothing.

QUOTED MARKUP IS NOT BOT MARKUP. A fenced code block, and a span between
single backticks, is somebody QUOTING markup in order to talk about it, which
is what any body explaining this file looks like. Markers and badges inside
one are invisible to all three rules: not removed, and not counted when
markers are paired off, so quoting a whole pair unbalances nothing. Without
this the stripper deletes the middle of somebody's own example, which is the
silent corruption it exists to avoid. An unclosed fence runs to the end of the
body, which is how the body renders, so everything after it is quoted too; the
cost is that a badge appended after an unclosed fence survives, and it is
already inside the rendered code block at that point.

WHAT IT NEVER TOUCHES: everything else, byte for byte. A body carrying no bot
markers comes back as the identical object, and the self-test asserts that on
a body full of the shapes this script knows how to find. Blank lines inside
the author's own text are never collapsed; only a gap a removal opened is
closed.

IDEMPOTENT by construction: a second run finds no markers and no badge line,
so it computes no spans and returns its input unchanged. That matters because
the workflow's own edit is one more edit to the body, and because the bots
re-insert their blocks on their next review. Removal, re-insertion, removal is
the expected steady state, not a loop to be broken.

REFUSAL. An opening marker with no closing one, a closing marker with no
opening one, or a closer that precedes its opener means the body is not a
shape this script knows. It then changes NOTHING and exits 2. Writing a
half-applied edit back over somebody's pull request body is the one outcome
worse than leaving the badge alone, so the refusal is total rather than
best-effort.

Usage:
  pr_body_hygiene.py --self-test
  pr_body_hygiene.py --from-json PR.json --out CLEANED
  pr_body_hygiene.py --in BODY --out CLEANED
  pr_body_hygiene.py --from-json PR.json --out BODY --extract-only

`--from-json` reads the `body` field of a pull request API response, which is
how the workflow gets the exact bytes: every route through a shell redirect
appends a newline the body never had, and that newline would come back as a
spurious edit. Prints `changed` or `unchanged` on stdout. Exit 0 on either,
1 on a usage or input error, 2 on a refusal.

`--extract-only` writes that field out verbatim and strips nothing. It exists
so the caller can read the body twice, before computing and again before
writing, and compare the two byte for byte: a body edited in between is a body
whose new text this run never saw, and writing a result computed from the
stale one would silently drop somebody's edit.
"""

import argparse
import contextlib
import io
import json
import os
import re
import sys
import tempfile

# Paired markers, as (opening token, closing token). Each token is matched as
# an HTML comment whose content is exactly that token, with any amount of
# surrounding whitespace, so a bot that reformats its own marker still matches.
MARKER_PAIRS = (
    ("devin-review-badge-begin", "devin-review-badge-end"),
    ("greptile_comment", "/greptile_comment"),
)

# Markers a bot emits with no closing pair, as regular expressions over the
# comment's content, because one of them carries a number. Both sit INSIDE the
# greptile block in every body seen so far, so rule 1 already carries them
# away; these entries are what keep a stray one out of a commit message.
STANDALONE_MARKERS = (r"greptile_summary", r"greptile_confidence_score:\d+")

# THE CLOSED LIST. Markup is review-bot markup only when one of these URL
# prefixes appears inside it. Nothing here is decided by element name, so an
# author's own `<img>` or `<picture>` is invisible to rule 3.
BOT_ASSET_PREFIXES = (
    "https://static.devin.ai/",
    "https://app.devin.ai/review/",
    "https://greptile-static-assets.s3.amazonaws.com/",
    "https://app.greptile.com/api/retrigger",
    "https://www.greptile.com/trex",
)

# Elements rule 3 will consider, innermost last so the fixpoint below can peel
# a wrapper off and then look at what it wrapped.
ELEMENT_PATTERNS = (
    re.compile(r"<a\b[^>]*>.*?</a\s*>", re.IGNORECASE | re.DOTALL),
    re.compile(r"<picture\b[^>]*>.*?</picture\s*>", re.IGNORECASE | re.DOTALL),
    re.compile(r"<img\b[^>]*?/?>", re.IGNORECASE),
    re.compile(r"<source\b[^>]*?/?>", re.IGNORECASE),
)

# A byte no pull request body carries. Removed spans become this, and one
# regex then closes every seam in a single pass. An input that somehow holds
# one is refused rather than silently mangled.
SENTINEL = "\x00"

# A seam: whatever whitespace ran up to a removal, the removal, and whatever
# whitespace ran away from it, with adjacent removals absorbed into one run.
SEAM = re.compile(r"[ \t\r\n]*" + SENTINEL + r"[\x00 \t\r\n]*")

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_REFUSED = 2


class Refusal(Exception):
    """The body is not a shape this script knows. Nothing is changed."""


def marker_pattern(token):
    """An HTML comment whose content is exactly the literal `token`."""
    return re.compile(r"<!--\s*" + re.escape(token) + r"\s*-->")


def standalone_pattern(fragment):
    """An HTML comment whose content matches the regular expression `fragment`."""
    return re.compile(r"<!--\s*(?:" + fragment + r")\s*-->")


STANDALONE_PATTERNS = tuple(standalone_pattern(f) for f in STANDALONE_MARKERS)


def strip_bot_markup(text):
    """Return `text` with every element holding a bot asset URL taken out.

    Used ONLY as the predicate behind rule 3: the caller keeps or drops a
    whole line and never writes this result, so a partial strip can never
    reach a body.
    """
    peeling = True
    while peeling:
        peeling = False
        for pattern in ELEMENT_PATTERNS:
            pieces = []
            last = 0
            for found in pattern.finditer(text):
                if not any(prefix in found.group(0) for prefix in BOT_ASSET_PREFIXES):
                    continue
                pieces.append(text[last : found.start()])
                last = found.end()
                peeling = True
            if pieces:
                pieces.append(text[last:])
                text = "".join(pieces)
    return text


def line_is_bot_only(line):
    """True when `line` holds review-bot markup and nothing else."""
    carries_marker = any(pattern.search(line) for pattern in STANDALONE_PATTERNS)
    carries_asset = any(prefix in line for prefix in BOT_ASSET_PREFIXES)
    if not (carries_marker or carries_asset):
        return False
    remainder = line
    for pattern in STANDALONE_PATTERNS:
        remainder = pattern.sub("", remainder)
    return strip_bot_markup(remainder).strip() == ""


def block_spans(body, quoted):
    """Spans of every delimited bot block, or raise `Refusal`.

    A marker inside `quoted` is somebody writing the marker down rather than a
    bot emitting one, so it is skipped on BOTH sides of the count: quoting a
    whole pair in an example leaves the pairing balanced.
    """
    spans = []
    for open_token, close_token in MARKER_PAIRS:
        opens = [
            found
            for found in marker_pattern(open_token).finditer(body)
            if not inside(quoted, *found.span())
        ]
        closes = [
            found
            for found in marker_pattern(close_token).finditer(body)
            if not inside(quoted, *found.span())
        ]
        if len(opens) != len(closes):
            raise Refusal(
                "marker '{}' appears {} time(s) and its pair '{}' appears {}: "
                "refusing to guess where the block ends".format(
                    open_token, len(opens), close_token, len(closes)
                )
            )
        for opened, closed in zip(opens, closes):
            if closed.start() < opened.end():
                raise Refusal(
                    "marker '{}' closes at offset {} before '{}' opens at {}: "
                    "refusing to remove an inverted block".format(
                        close_token, closed.start(), open_token, opened.start()
                    )
                )
            spans.append((opened.start(), closed.end()))
    return spans


def merge_spans(spans):
    """Sort and union overlapping or nested spans."""
    merged = []
    for start, end in sorted(spans):
        if merged and start <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], end))
        else:
            merged.append((start, end))
    return merged


# A fenced code block: three or more backticks or tildes at the start of a
# line, up to three spaces of indent, closed by at least as many of the SAME
# character on a line of its own. This is the CommonMark rule, less the info
# string, which nothing here needs to read.
FENCE = re.compile(r"^ {0,3}(`{3,}|~{3,})([^\r\n]*)$", re.MULTILINE)
# A span between single backticks, on one line: the inline way to quote a
# marker in a sentence.
INLINE_CODE = re.compile(r"`[^`\r\n]*`")


def quoted_ranges(body):
    """Spans holding quoted markup: fenced blocks first, then inline spans.

    A fence that is never closed runs to the end of the body, which is how the
    body renders.
    """
    ranges = []
    opener = None
    for fence in FENCE.finditer(body):
        marker, trailing = fence.group(1), fence.group(2)
        if opener is None:
            # An opening backtick fence's info string may not itself contain a
            # backtick, so such a line opens nothing.
            if marker[0] == "`" and "`" in trailing:
                continue
            opener = (fence.start(), marker)
            continue
        open_start, open_marker = opener
        # A CLOSING fence carries no info string: it is the fence characters
        # and nothing else. Accepting one that does would end the quotation at
        # a line like ```text sitting INSIDE the block, and expose the rest of
        # somebody's example to the rules below.
        if (
            marker[0] == open_marker[0]
            and len(marker) >= len(open_marker)
            and not trailing.strip()
        ):
            ranges.append((open_start, fence.end()))
            opener = None
    if opener is not None:
        ranges.append((opener[0], len(body)))
    inline = [
        found.span()
        for found in INLINE_CODE.finditer(body)
        if not any(start <= found.start() and found.end() <= end for start, end in ranges)
    ]
    return merge_spans(ranges + inline)


def inside(spans, start, end):
    """True when [start, end) lies wholly within one of `spans`."""
    return any(low <= start and end <= high for low, high in spans)


# Lines break at these three terminators and nowhere else. `str.splitlines`
# also breaks at form feed and at the Unicode separators, which in a pull
# request body are ordinary text somebody typed.
LINE_BREAK = re.compile(r"\r\n|\n|\r")


def badge_line_spans(body, blocks, quoted):
    """Spans of bot-only lines outside every block span and every quotation."""
    spans = []
    start = 0
    for terminator in list(LINE_BREAK.finditer(body)) + [None]:
        end = len(body) if terminator is None else terminator.start()
        content = body[start:end]
        if content.strip() and not inside(quoted, start, end) and line_is_bot_only(content):
            if not any(
                start < block_end and block_start < end
                for block_start, block_end in blocks
            ):
                spans.append((start, end))
        if terminator is None:
            break
        start = terminator.end()
    return spans


def close_seams(text, eol):
    """Replace each removal sentinel with the right amount of nothing."""

    def replace(found):
        if found.start() == 0 or found.end() == len(text):
            return ""
        return eol + eol

    return SEAM.sub(replace, text)


def clean_body(body):
    """Return `body` with bot markup removed. Raises `Refusal` unchanged."""
    if SENTINEL in body:
        raise Refusal("body holds a NUL byte: refusing to edit a body this cannot model")
    quoted = quoted_ranges(body)
    blocks = merge_spans(block_spans(body, quoted))
    spans = merge_spans(list(blocks) + badge_line_spans(body, blocks, quoted))
    if not spans:
        return body
    eol = "\r\n" if "\r\n" in body else "\n"
    pieces = []
    last = 0
    for start, end in spans:
        pieces.append(body[last:start])
        pieces.append(SENTINEL)
        last = end
    pieces.append(body[last:])
    return close_seams("".join(pieces), eol)


# ---------------------------------------------------------------------------
# Self-test. Every case names the rule it drives and compares against a hand
# written expectation, never against another run of this same code.
# ---------------------------------------------------------------------------

DEVIN_BADGE = (
    '<a href="https://app.devin.ai/review/cerulion-inc/cerulion/pull/24" '
    'target="_blank"><picture><source media="(prefers-color-scheme: dark)" '
    'srcset="https://static.devin.ai/assets/gh-devin-review-dark.svg?v=4">'
    '<img src="https://static.devin.ai/assets/gh-devin-review-light.svg?v=4" '
    'alt="Devin Review"></picture></a>'
)

GREPTILE_HEADING = (
    '<h2><a href="https://app.greptile.com/api/retrigger?id=67709577"><picture>'
    '<source media="(prefers-color-scheme: dark)" '
    'srcset="https://greptile-static-assets.s3.amazonaws.com/badges/RetriggerDark.svg?v=2">'
    '<img alt="Retrigger" '
    'src="https://greptile-static-assets.s3.amazonaws.com/badges/Retrigger.svg?v=2" '
    'align="right"></picture></a>Confidence Score: 5/5</h2>'
)

AUTHORED = (
    "## Summary\n"
    "Expose asynchronous model loading through the daemon.\n"
    "\n"
    "\n"
    "Two blank lines above are the author's own and stay.\n"
    "\n"
    "## Validation\n"
    "```sh\n"
    "cargo test -p cerulion_vizd\n"
    "```\n"
    "457 tests passed, 7 ignored."
)


def _case(name, body, expected, failures):
    try:
        got = clean_body(body)
    except Refusal as refused:
        failures.append("{}: refused unexpectedly: {}".format(name, refused))
        return
    if got != expected:
        failures.append(
            "{}:\n  expected {!r}\n  got      {!r}".format(name, expected, got)
        )
        return
    try:
        again = clean_body(got)
    except Refusal as refused:
        failures.append("{}: second run refused: {}".format(name, refused))
        return
    if again != got:
        failures.append(
            "{}: not idempotent:\n  once  {!r}\n  twice {!r}".format(name, got, again)
        )


def _json_case(name, payload, expect_exit, expect_body, failures):
    """Drive `main` over a `--from-json` file and check the exit and the bytes.

    The reading half needs its own oracle: a body that is not a string reaches
    the stripper, and a FALSY one would otherwise be written back as an empty
    body over a real one.
    """
    work = tempfile.mkdtemp(prefix="pr-body-hygiene-")
    source = os.path.join(work, "pr.json")
    target = os.path.join(work, "out.md")
    with open(source, "w", encoding="utf-8") as handle:
        handle.write(payload)
    out, err = io.StringIO(), io.StringIO()
    try:
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = main(["--from-json", source, "--out", target, "--extract-only"])
    except Exception as raised:  # noqa: BLE001 - a raise here IS the failure
        # Reading a response must fail with a reason, never with a traceback:
        # the caller is a workflow step, and a traceback says nothing about
        # which response was wrong.
        failures.append(
            "{}: raised {}: {}".format(name, type(raised).__name__, raised)
        )
        return
    if code != expect_exit:
        failures.append(
            "{}: expected exit {}, got {} ({})".format(
                name, expect_exit, code, err.getvalue().strip()
            )
        )
    elif expect_body is None:
        if os.path.exists(target):
            failures.append("{}: wrote a file for input it should have refused".format(name))
        elif not err.getvalue().strip():
            failures.append("{}: refused without saying why".format(name))
    else:
        with open(target, encoding="utf-8", newline="") as handle:
            got = handle.read()
        if got != expect_body:
            failures.append(
                "{}:\n  expected {!r}\n  got      {!r}".format(name, expect_body, got)
            )


def _refuses(name, body, failures):
    try:
        got = clean_body(body)
    except Refusal:
        return
    failures.append("{}: expected a refusal, got {!r}".format(name, got))


def self_test():
    failures = []

    # Rule 1, each marker pair on its own, in the shape a real body carries:
    # a blank line, the marker, the badge, the closing marker.
    _case(
        "devin block between paragraphs",
        "Before.\n\n<!-- devin-review-badge-begin -->\n\n---\n\n"
        + DEVIN_BADGE
        + "\n<!-- devin-review-badge-end -->\n\nAfter.",
        "Before.\n\nAfter.",
        failures,
    )
    _case(
        "greptile block between paragraphs",
        "Before.\n\n<!-- greptile_comment -->\n\n<!-- greptile_summary -->\n\n"
        + GREPTILE_HEADING
        + "\n\nSafe to merge.\n\n<!-- /greptile_comment -->\n\nAfter.",
        "Before.\n\nAfter.",
        failures,
    )

    # Rule 2: the unpaired marker on its own, outside any block. Rule 1 cannot
    # reach it, and left alone it would be one line of HTML in a commit message.
    _case(
        "standalone summary marker",
        "Before.\n\n<!-- greptile_summary -->\n\nAfter.",
        "Before.\n\nAfter.",
        failures,
    )

    # The same rule over the marker that carries a number.
    _case(
        "standalone confidence marker",
        "Before.\n\n<!-- greptile_confidence_score:4 -->\n\nAfter.",
        "Before.\n\nAfter.",
        failures,
    )

    # CONTROL: the pattern is anchored to the whole comment, so a comment that
    # merely mentions a marker is not one.
    near_miss = "Before.\n\n<!-- note: greptile_summary is theirs, not ours -->\n\nAfter."
    _case("a comment that only mentions a marker", near_miss, near_miss, failures)

    # Rule 3: a badge on a line of its own with no markers anywhere.
    _case(
        "undelimited badge line",
        "Before.\n\n" + DEVIN_BADGE + "\n\nAfter.",
        "Before.\n\nAfter.",
        failures,
    )

    # Rule 3's limit, and the reason it is a whole-line rule: prose and a badge
    # on one line is left exactly as the author wrote it.
    mixed = "See " + DEVIN_BADGE + " for the review."
    _case("prose sharing a line with a badge", mixed, mixed, failures)

    # Nothing in the closed list appears, so nothing is bot markup: an author's
    # own picture, image, anchor, and the word Retrigger in prose.
    authored_markup = (
        "Retrigger the job by hand if it stalls.\n"
        "\n"
        '<picture><source srcset="docs/media/dark.svg">'
        '<img src="docs/media/light.svg" alt="diagram"></picture>\n'
        "\n"
        '<a href="https://example.invalid/page">a link</a>\n'
    )
    _case("authored markup is not bot markup", authored_markup, authored_markup, failures)

    # THE BYTE-FOR-BYTE CASE. A body with no bot markers comes back identical,
    # blank runs, fences, trailing text and all.
    _case("body with no markers", AUTHORED, AUTHORED, failures)

    # The real shape: both blocks, one after the other, at the end of a body,
    # which is where every bot puts them. The author's own double blank line
    # survives; the seam closes to nothing because the removal ran to the end.
    _case(
        "both blocks at the end of a real body",
        AUTHORED
        + "\n\n<!-- devin-review-badge-begin -->\n\n---\n\n"
        + DEVIN_BADGE
        + "\n<!-- devin-review-badge-end -->\n\n<!-- greptile_comment -->\n\n"
        + "<!-- greptile_summary -->\n\n"
        + GREPTILE_HEADING
        + "\n\n<sub>Reviews (1)</sub>\n\n<!-- /greptile_comment -->",
        AUTHORED,
        failures,
    )

    # Nesting: one pair wholly inside the other. The union is one span, so the
    # inner markers cannot survive their container.
    _case(
        "nested marker pairs",
        "Before.\n\n<!-- greptile_comment -->\ntext\n"
        "<!-- devin-review-badge-begin -->\n"
        + DEVIN_BADGE
        + "\n<!-- devin-review-badge-end -->\nmore\n<!-- /greptile_comment -->\n\nAfter.",
        "Before.\n\nAfter.",
        failures,
    )

    # Seam arithmetic, all three positions.
    _case(
        "block at the start of the body",
        "<!-- greptile_comment -->\nx\n<!-- /greptile_comment -->\n\nAfter.",
        "After.",
        failures,
    )
    _case(
        "block at the end of the body",
        "Before.\n\n<!-- greptile_comment -->\nx\n<!-- /greptile_comment -->\n",
        "Before.",
        failures,
    )
    _case(
        "block is the whole body",
        "<!-- greptile_comment -->\nx\n<!-- /greptile_comment -->",
        "",
        failures,
    )
    _case(
        "many blank lines around a block collapse to one",
        "Before.\n\n\n\n<!-- greptile_summary -->\n\n\n\nAfter.",
        "Before.\n\nAfter.",
        failures,
    )

    # A body the web editor wrote: the seam is rebuilt in the body's own line
    # ending, not in the script's.
    _case(
        "carriage returns are preserved",
        "Before.\r\n\r\n<!-- greptile_comment -->\r\nx\r\n<!-- /greptile_comment -->\r\n\r\nAfter.",
        "Before.\r\n\r\nAfter.",
        failures,
    )

    # QUOTED MARKUP. A body explaining this file quotes the markers in a fenced
    # block, and every rule has to be blind to that: rule 3 would otherwise
    # empty a line of somebody's example, and rules 1 and 2 would delete the
    # middle of it. Found by running this stripper over the body of the pull
    # request that introduced it.
    fenced = (
        "How the markers look:\n"
        "\n"
        "```\n"
        "<!-- devin-review-badge-begin -->\n"
        + DEVIN_BADGE
        + "\n<!-- devin-review-badge-end -->\n"
        "```\n"
        "\n"
        "That is the whole shape."
    )
    _case("a fenced block quoting a whole marker pair", fenced, fenced, failures)

    fenced_greptile = (
        "The greptile pair:\n\n~~~text\n<!-- greptile_comment -->\n"
        "<!-- greptile_summary -->\n" + GREPTILE_HEADING + "\n"
        "<!-- /greptile_comment -->\n~~~\n"
    )
    _case("a tilde fence quoting the other pair", fenced_greptile, fenced_greptile, failures)

    inline = "The opener is `<!-- greptile_comment -->` and it closes with `<!-- /greptile_comment -->`."
    _case("markers quoted inline in a sentence", inline, inline, failures)

    # A fence nobody closed runs to the end of the body, which is how the body
    # renders, so what follows it is quoted too.
    unclosed = "Here:\n\n```\n<!-- greptile_comment -->\nx\n<!-- /greptile_comment -->\n"
    _case("an unclosed fence quotes everything after it", unclosed, unclosed, failures)

    # ANTI-TAUTOLOGY for the rule above: the SAME markup outside a fence is
    # still removed, so blindness to quotation is not blindness to bots.
    _case(
        "the same pair outside a fence is still removed",
        "How the markers look:\n\n<!-- devin-review-badge-begin -->\n"
        + DEVIN_BADGE
        + "\n<!-- devin-review-badge-end -->\n\nThat is the whole shape.",
        "How the markers look:\n\nThat is the whole shape.",
        failures,
    )

    # A real block BELOW a closed fence is still reached: closing the fence
    # ends the quotation.
    _case(
        "a fence that closes does not shield what follows it",
        "```\nquoted\n```\n\n<!-- greptile_comment -->\nx\n<!-- /greptile_comment -->\n\nAfter.",
        "```\nquoted\n```\n\nAfter.",
        failures,
    )

    # A line that looks like a fence but carries an info string does not CLOSE
    # a block, so the example below stays quoted to its real closing fence.
    # Accepting it would end the quotation early and leave the second marker
    # outside, which reads as an unpaired closer and refuses the whole body.
    info_string_close = (
        "Here:\n\n```\n<!-- greptile_comment -->\n```text\nx\n"
        "<!-- /greptile_comment -->\n```\n\nDone."
    )
    _case("an info string does not close a fence", info_string_close, info_string_close, failures)

    # The opening fence, though, carries an info string in every real body.
    _case(
        "an opening fence may carry an info string",
        "```sh\ncargo test\n```\n\n<!-- greptile_comment -->\nx\n"
        "<!-- /greptile_comment -->\n\nAfter.",
        "```sh\ncargo test\n```\n\nAfter.",
        failures,
    )

    # An empty body is a body.
    _case("empty body", "", "", failures)

    # REFUSALS. Each leaves the body alone; the caller writes nothing back.
    _refuses(
        "opening marker with no closing pair",
        "Before.\n\n<!-- greptile_comment -->\n" + GREPTILE_HEADING + "\n",
        failures,
    )
    _refuses(
        "closing marker with no opening pair",
        "Before.\n\nx\n<!-- /greptile_comment -->\n",
        failures,
    )
    _refuses(
        "closer before its opener",
        "<!-- devin-review-badge-end -->\nx\n<!-- devin-review-badge-begin -->\n",
        failures,
    )
    _refuses("body holding a NUL byte", "Before.\x00After.", failures)

    # A refusal really does leave the input alone: the caller has the original
    # string in hand and this asserts the exception carried nothing away.
    unbalanced = "Before.\n\n<!-- devin-review-badge-begin -->\n" + DEVIN_BADGE + "\n"
    kept = unbalanced
    try:
        clean_body(unbalanced)
    except Refusal:
        pass
    if unbalanced != kept:
        failures.append("a refusal mutated its input")

    # ANTI-TAUTOLOGY. Every case above passes trivially for a script that
    # returns its input, so one case must prove the stripping happens at all.
    loaded = "Before.\n\n<!-- greptile_comment -->\nx\n<!-- /greptile_comment -->\n\nAfter."
    if clean_body(loaded) == loaded:
        failures.append("the stripper returned a body that carried a block unchanged")

    # READING THE RESPONSE. A body is a string, or null when there is none.
    _json_case("a string body", '{"body": "text"}', EXIT_OK, "text", failures)
    _json_case("a null body", '{"body": null}', EXIT_OK, "", failures)
    # A TRUTHY non-string reaches the stripper and raises where nothing catches
    # it; a FALSY one would become the empty string and be written back over a
    # real body. Both are refused, with a reason, having written nothing.
    for literal in ("true", "0", "[]", "{}"):
        _json_case(
            "a body that is " + literal, '{"body": %s}' % literal, EXIT_ERROR, None, failures
        )
    _json_case("a response with no body field", "{}", EXIT_ERROR, None, failures)
    _json_case("a response that is not an object", "[]", EXIT_ERROR, None, failures)
    _json_case("a file that is not JSON", "not json", EXIT_ERROR, None, failures)

    for failure in failures:
        sys.stderr.write("pr-body-hygiene self-test FAILED: {}\n".format(failure))
    if failures:
        sys.stderr.write(
            "pr-body-hygiene self-test: {} case(s) failed\n".format(len(failures))
        )
        return EXIT_ERROR
    sys.stdout.write("pr-body-hygiene self-test passed\n")
    return EXIT_OK


def read_body(args):
    if args.from_json:
        with open(args.from_json, encoding="utf-8") as handle:
            payload = json.load(handle)
        if not isinstance(payload, dict) or "body" not in payload:
            raise ValueError(
                "{}: not a pull request API response (no 'body' field)".format(
                    args.from_json
                )
            )
        body = payload["body"]
        # A pull request body is a string, or null when there is none. Anything
        # else is not the response this claims to be, and the two ways of
        # carrying on are both bad: a truthy non-string reaches the stripper and
        # raises where nothing catches it, and a FALSY one (0, [], {}, false)
        # becomes the empty string, is reported as `changed`, and would have the
        # caller write an empty body over a real one.
        if body is None:
            return ""
        if not isinstance(body, str):
            raise ValueError(
                "{}: the 'body' field is {}, not a string or null".format(
                    args.from_json, type(body).__name__
                )
            )
        return body
    with open(args.input, encoding="utf-8", newline="") as handle:
        return handle.read()


def write_out(path, text):
    """Write `text` to `path` with its bytes and line endings untouched."""
    try:
        with open(path, "w", encoding="utf-8", newline="") as handle:
            handle.write(text)
    except OSError as problem:
        sys.stderr.write("pr-body-hygiene: {}\n".format(problem))
        return EXIT_ERROR
    return EXIT_OK


def main(argv):
    parser = argparse.ArgumentParser(
        description="Take review-bot badge markup out of a pull request body."
    )
    parser.add_argument("--self-test", action="store_true", help="run the oracle suite")
    parser.add_argument("--from-json", metavar="PATH", help="a pull request API response")
    parser.add_argument("--in", dest="input", metavar="PATH", help="a raw body")
    parser.add_argument("--out", metavar="PATH", help="where to write the cleaned body")
    parser.add_argument(
        "--extract-only",
        action="store_true",
        help="write the body out verbatim and strip nothing",
    )
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test()

    if bool(args.from_json) == bool(args.input):
        parser.error("give exactly one of --from-json or --in")
    if not args.out:
        parser.error("--out is required")

    try:
        body = read_body(args)
    except (OSError, ValueError, json.JSONDecodeError) as problem:
        sys.stderr.write("pr-body-hygiene: {}\n".format(problem))
        return EXIT_ERROR

    if args.extract_only:
        return write_out(args.out, body)

    try:
        cleaned = clean_body(body)
    except Refusal as refused:
        sys.stderr.write("pr-body-hygiene: refusing to edit this body: {}\n".format(refused))
        return EXIT_REFUSED

    written = write_out(args.out, cleaned)
    if written != EXIT_OK:
        return written

    sys.stdout.write("changed\n" if cleaned != body else "unchanged\n")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
