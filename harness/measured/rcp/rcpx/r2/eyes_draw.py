# SPDX-License-Identifier: AGPL-3.0-only
"""The numbered picture of the eyes package (SPEC-R5 4.1): the look's own picture with each candidate's outline and
its number beside the outline, outside it.

The picture is NEVER turned, scaled or cut: it has the look's size and, outside the drawn outlines and number
tags, the look's own pixels, so a pixel a model reads off it and sends to mark, locate or choose lands where the
model pointed. What is up in a tilted view is said in words elsewhere, not by turning the picture.

    numbered(img, rows, others=()) -> (picture, layout)

img: H x W x 3 uint8 (the look's frame). rows: the candidates of this page ({n, box_px, polygon_px}). others:
candidates of other pages, whose boxes a tag must not cover either. layout: {"size": [W, H], "line_px": the
outline's width, "tags": {n: {"tag_box": [x0, y0, x1, y1], "outline": [[u, v], ...] (closed: the first point is
not repeated), "side": where the tag stands, "free": False when no free place was left and the tag covers
something}}}. Everything a call changes lies within line_px of an outline or inside a tag_box.

Digits are drawn from the 5 x 7 table below (no font file is read), outlines with PIL's line drawing: the bytes
are the same on every computer for the same input. Standard library, numpy and PIL only.
"""

import numpy as np
from PIL import Image, ImageDraw

LINE_PX = 2
SCALE = 2                 # each cell of a digit is SCALE x SCALE pixels
PAD = 2                   # pixels round the digits inside a tag
GAP = 3                   # pixels between an outline's box and its tag
COLOURS = ((255, 64, 64), (64, 200, 255), (255, 200, 0), (120, 255, 120), (255, 120, 255), (255, 150, 60),
           (150, 150, 255), (0, 230, 190))
TAG_BACK = (0, 0, 0)
TAG_TEXT = (255, 255, 255)
DIGITS = {
    "0": ("01110", "10001", "10011", "10101", "11001", "10001", "01110"),
    "1": ("00100", "01100", "00100", "00100", "00100", "00100", "01110"),
    "2": ("01110", "10001", "00001", "00010", "00100", "01000", "11111"),
    "3": ("11110", "00001", "00001", "01110", "00001", "00001", "11110"),
    "4": ("00010", "00110", "01010", "10010", "11111", "00010", "00010"),
    "5": ("11111", "10000", "11110", "00001", "00001", "10001", "01110"),
    "6": ("00110", "01000", "10000", "11110", "10001", "10001", "01110"),
    "7": ("11111", "00001", "00010", "00100", "01000", "01000", "01000"),
    "8": ("01110", "10001", "10001", "01110", "10001", "10001", "01110"),
    "9": ("01110", "10001", "10001", "01111", "00001", "00010", "01100"),
}


def tag_size(n):
    """(width, height) in pixels of the tag that shows number n."""
    k = len(str(int(n)))
    return k * 5 * SCALE + (k - 1) * SCALE + 2 * PAD, 7 * SCALE + 2 * PAD


def _overlap(a, b):
    return max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * max(0.0, min(a[3], b[3]) - max(a[1], b[1]))


def _places(box, tw, th):
    """Tag places round a box, all outside it, nearest corners first: (side word, [x0, y0, x1, y1])."""
    x0, y0, x1, y1 = box
    g = GAP + LINE_PX
    return [("above left", [x0, y0 - g - th, x0 + tw, y0 - g]), ("above right", [x1 - tw, y0 - g - th, x1, y0 - g]),
            ("below left", [x0, y1 + g, x0 + tw, y1 + g + th]), ("below right", [x1 - tw, y1 + g, x1, y1 + g + th]),
            ("left", [x0 - g - tw, y0, x0 - g, y0 + th]), ("right", [x1 + g, y0, x1 + g + tw, y0 + th]),
            ("left low", [x0 - g - tw, y1 - th, x0 - g, y1]), ("right low", [x1 + g, y1 - th, x1 + g + tw, y1]),
            ("above", [(x0 + x1 - tw) // 2, y0 - g - th, (x0 + x1 - tw) // 2 + tw, y0 - g]),
            ("below", [(x0 + x1 - tw) // 2, y1 + g, (x0 + x1 - tw) // 2 + tw, y1 + g + th])]


def place_tags(rows, others, w, h):
    """{n: (side, tag box, free)} for the rows, in their order. A tag stands outside its own candidate's box, inside
    the frame, on no other candidate's box (of this page or of others) and on no earlier tag; when no such place
    is left, the place inside the frame that covers least is taken and free is False."""
    boxes = {}
    for r in list(rows) + list(others or ()):
        b = r.get("box_px")
        if b is not None and r.get("n") is not None:
            boxes[r["n"]] = [int(np.floor(b[0])) - LINE_PX, int(np.floor(b[1])) - LINE_PX,
                             int(np.ceil(b[2])) + LINE_PX, int(np.ceil(b[3])) + LINE_PX]
    placed, out = [], {}
    for r in rows:
        n = r["n"]
        if n not in boxes:
            continue
        tw, th = tag_size(n)
        best = None
        for side, t in _places(boxes[n], tw, th):
            t = [int(v) for v in t]
            if t[0] < 0 or t[1] < 0 or t[2] > w or t[3] > h:
                continue
            cover = sum(_overlap(t, b) for b in boxes.values()) + sum(_overlap(t, p) for p in placed)
            if cover == 0:
                best = (side, t, True)
                break
            if best is None or cover < best[3]:
                best = (side, t, False, cover)
        if best is None:      # no place inside the frame at all (a box as large as the picture): its own corner
            t = [0, 0, tw, th]
            best = ("corner", t, False)
        out[n] = (best[0], best[1], best[2])
        placed.append(best[1])
    return out


def _outline(row):
    poly = row.get("polygon_px")
    if isinstance(poly, list) and len(poly) >= 3:
        return [[float(u), float(v)] for u, v in poly]
    b = row["box_px"]
    return [[float(b[0]), float(b[1])], [float(b[2]), float(b[1])], [float(b[2]), float(b[3])],
            [float(b[0]), float(b[3])]]


def _digits(arr, n, x0, y0):
    x = x0 + PAD
    for ch in str(int(n)):
        for j, line in enumerate(DIGITS[ch]):
            for i, bit in enumerate(line):
                if bit == "1":
                    arr[y0 + PAD + j * SCALE:y0 + PAD + (j + 1) * SCALE, x + i * SCALE:x + (i + 1) * SCALE] = TAG_TEXT
        x += 5 * SCALE + SCALE


def numbered(img, rows, others=()):
    """(the numbered picture, the layout record): see the module text."""
    base = np.ascontiguousarray(np.asarray(img)[:, :, :3]).astype(np.uint8)
    h, w = base.shape[:2]
    rows = [r for r in rows or () if r.get("box_px") is not None and r.get("n") is not None]
    tags = place_tags(rows, others, w, h)
    pic = Image.fromarray(base.copy())
    draw = ImageDraw.Draw(pic)
    layout = {"size": [w, h], "line_px": LINE_PX, "tags": {}}
    for r in rows:
        colour = COLOURS[(int(r["n"]) - 1) % len(COLOURS)]
        pts = [(int(round(u)), int(round(v))) for u, v in _outline(r)]
        draw.line(pts + [pts[0]], fill=colour, width=LINE_PX)
        layout["tags"][int(r["n"])] = {"outline": [[p[0], p[1]] for p in pts]}
    arr = np.array(pic)
    for r in rows:
        n = int(r["n"])
        side, t, free = tags[n]
        colour = COLOURS[(n - 1) % len(COLOURS)]
        arr[t[1]:t[3], t[0]:t[2]] = colour
        arr[t[1] + 1:t[3] - 1, t[0] + 1:t[2] - 1] = TAG_BACK
        _digits(arr, n, t[0], t[1])
        layout["tags"][n].update(tag_box=[int(v) for v in t], side=side, free=bool(free))
    return arr, layout


__all__ = ["COLOURS", "LINE_PX", "numbered", "place_tags", "tag_size"]
