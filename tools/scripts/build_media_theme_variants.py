#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Write the fixed-palette light and dark twins of the README's theme-aware SVGs.

Why this exists. An SVG shown through an <img> element cannot see the page
around it, so a `prefers-color-scheme` rule INSIDE the file answers for the
viewer's operating system, never for the GitHub theme they picked. A viewer
whose system is light and whose GitHub theme is dark gets the light drawing
on a dark page, and the wordmark of the logo disappears. GitHub resolves the
<picture> element against its own theme, so the README names one file per
theme, and those files carry no theme rule at all.

Each source keeps its internal rule and stays the <img> fallback, which is the
right behaviour everywhere that is not GitHub. The twins are generated, never
edited: run this script after touching a source, and `--check` (the Lint job)
fails when a twin is stale, missing, or still carries a theme rule.

  tools/scripts/build_media_theme_variants.py            write the twins
  tools/scripts/build_media_theme_variants.py --check    verify, write nothing
"""
import os
import re
import sys

MEDIA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "docs", "media")
SOURCES = ["cerulion-logo.svg", "development-loop.svg", "architecture-flow.svg"]

# Every colour-scheme rule in a source must be a dark block holding exactly one
# `:root{...}` rule (a source may carry several: the architecture figure has one
# for the drawing and one for its animation layer).
DARK_BLOCK = re.compile(r"@media\s*\(\s*prefers-color-scheme\s*:\s*dark\s*\)\s*\{\s*(:root\s*\{[^{}]*\})\s*\}")


def variants(text, name):
    blocks = DARK_BLOCK.findall(text)
    rules = text.count("prefers-color-scheme")
    if not blocks:
        raise SystemExit(f"{name}: no dark `:root` block found; is this still a theme-aware source?")
    if len(blocks) != rules:
        raise SystemExit(f"{name}: {rules} colour-scheme rule(s) but only {len(blocks)} of the shape this script handles (a dark block holding one `:root` rule)")
    light = DARK_BLOCK.sub("", text)
    # Each dark rule replaces its block in place, after the light `:root` it overrides, so it wins the cascade.
    dark = DARK_BLOCK.sub(lambda m: m.group(1), text)
    for label, out in (("light", light), ("dark", dark)):
        if "prefers-color-scheme" in out:
            raise SystemExit(f"{name}: the {label} twin still carries a colour-scheme rule")
    return light, dark


def main(argv):
    check = "--check" in argv[1:]
    stale = []
    for name in SOURCES:
        path = os.path.join(MEDIA, name)
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
        light, dark = variants(text, name)
        stem = name[: -len(".svg")]
        for label, out in (("light", light), ("dark", dark)):
            target = os.path.join(MEDIA, f"{stem}-{label}.svg")
            current = None
            if os.path.exists(target):
                with open(target, encoding="utf-8") as fh:
                    current = fh.read()
            if current == out:
                continue
            if check:
                stale.append(os.path.relpath(target, os.path.join(MEDIA, "..", "..")))
            else:
                with open(target, "w", encoding="utf-8") as fh:
                    fh.write(out)
                print(f"wrote docs/media/{stem}-{label}.svg")
    if stale:
        print("build_media_theme_variants: STALE or missing (run the script without --check):")
        for s in stale:
            print(f"  {s}")
        return 1
    print(f"build_media_theme_variants: OK ({len(SOURCES)} source(s), {2 * len(SOURCES)} twin(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
