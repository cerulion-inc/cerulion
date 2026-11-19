# SPDX-License-Identifier: AGPL-3.0-only
"""Wire shapes of the robot's vision service and the geometry every backend shares. Standard library, numpy, PIL.

The service is robot equipment, declared in the installation like a camera. It answers two questions about one
picture and never reads anything but the picture, the words and the robot's own body outline:

  propose   regions that may be parts or objects, best first, for a caller to pick by number
  find      the regions that match a phrase (or each of up to 6 phrases in one request), best first, each with the
            model's confidence (or null when the model gives none)

A candidate is {"n": 1.., "point": [u, v], "box": [x1, y1, x2, y2], "polygon": [[u, v], ...], "area_px": int,
"confidence": float | null, "outline": "mask" | "box"}; a find candidate also carries "phrase", the text that
found it. Pixels: u right, v down, from the top left of the picture as sent. point is inside the region. polygon
is the outline of the region's largest piece (the bounding box corners when the backend has no mask).

A find answer says how sure it is in one word, "answer": "found" (the first candidate's confidence is at or over
the threshold the equipment file gives for those weights), "unsure" (there are candidates and the first is under
the threshold, or the model gives no confidence at all), "none" (no candidate at or over the floor the equipment
file gives; a file without a floor says none only when the model returns nothing at all). A candidate under the
floor is not returned. Threshold and floor are numbers of the installation's equipment file (T and F below):
this code holds neither. With several phrases the candidates come phrase by phrase in the order asked, best first
inside a phrase, numbered from 1 over the whole answer, "max" counts per phrase, and "per_phrase" maps each text
to its own word.

The picture of a request is image_b64 (the bytes travel in the request) or image_path. A path is honoured only
under the frames root the service was started with: the path is resolved (links and ".." followed) and must lie
under the resolved root, else the request is answered 400 and the file is never opened. With no frames root every
image_path is 400. So the service opens no file but its weights and a request's picture.

Three equipment levels, set by what the installation declares: "words" (find and propose), "segments" (propose
with a segmenting model, find answers not_equipped), "absent" (propose from colour regions only, no model at all).
"""
import base64
import io
import math
import os
import stat

import numpy as np
from PIL import Image

MAX_IMAGE_BYTES = 8 * 1024 * 1024
MAX_CANDIDATES = 16
DEFAULT_CANDIDATES = 8
MAX_POLYGON_POINTS = 48
MAX_SIDE_PX = 1280
MIN_SIDE_PX = 16
MAX_PHRASES = 6
MAX_PHRASE_CHARS = 200
OUTSIDE = "image_path is outside the frames root"
# the ranking rule's numbers per job (select below): the share of the picture over which a region is dropped, the
# smallest region kept in px, the bounding box overlap over which the weaker of two regions is dropped
RANK_PROPOSE = {"max_frac": 0.25, "min_px": 30, "nms_iou": 0.7}
RANK_FIND = {"max_frac": 1.01, "min_px": 4, "nms_iou": 0.5}


class BadRequest(ValueError):
    pass


def path_under(path, root):
    """The resolved path of a request's image_path when it lies under the frames root, else BadRequest. Links and
    ".." are followed first, so a link inside the root that points outside it is outside. A path that is not
    absolute is taken from the root. Nothing is opened here."""
    if not root:
        raise BadRequest(OUTSIDE + " (the service was started with none: send image_b64)")
    try:
        top = os.path.realpath(root)
        real = os.path.realpath(os.path.join(top, path))
        inside = real != top and os.path.commonpath([real, top]) == top
    except (ValueError, OSError) as exc:
        raise BadRequest("image_path is not a usable path") from exc
    if not inside:
        raise BadRequest(OUTSIDE)
    return real


def read_file(real):
    """Up to MAX_IMAGE_BYTES + 1 bytes of a regular file (a link, a pipe or a device is refused, never waited on)."""
    try:
        fd = os.open(real, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0))
    except OSError as exc:
        raise BadRequest(f"image_path cannot be read: {exc.strerror}") from exc
    with os.fdopen(fd, "rb") as f:
        if not stat.S_ISREG(os.fstat(f.fileno()).st_mode):
            raise BadRequest("image_path is not a file")
        return f.read(MAX_IMAGE_BYTES + 1)


def read_image(req, frames_root=None):
    """The picture of a request: image_b64 (PNG or JPEG bytes, base64), or image_path (a file under the frames
    root the service was started with; module text)."""
    if isinstance(req.get("image_b64"), str):
        try:
            raw = base64.b64decode(req["image_b64"], validate=True)
        except Exception as exc:
            raise BadRequest("image_b64 is not base64") from exc
    elif isinstance(req.get("image_path"), str):
        raw = read_file(path_under(req["image_path"], frames_root))
    else:
        raise BadRequest("give image_b64 (PNG or JPEG, base64) or image_path (a file under the frames root)")
    if len(raw) > MAX_IMAGE_BYTES:
        raise BadRequest(f"the picture is over {MAX_IMAGE_BYTES} bytes")
    try:
        img = Image.open(io.BytesIO(raw))   # reads the header only: the sides are known before any pixel is unpacked
    except Exception as exc:
        raise BadRequest("the picture is not a readable PNG or JPEG") from exc
    if max(img.size) > MAX_SIDE_PX or min(img.size) < MIN_SIDE_PX:
        raise BadRequest(f"picture sides must be from {MIN_SIDE_PX} to {MAX_SIDE_PX} px")
    try:
        img.load()
        return img.convert("RGB")
    except Exception as exc:
        raise BadRequest("the picture is not a readable PNG or JPEG") from exc


def count_arg(req, default=DEFAULT_CANDIDATES):
    n = req.get("max", default)
    if isinstance(n, bool) or not isinstance(n, int) or n < 1:
        raise BadRequest("max is a whole number of 1 or more")
    return min(n, MAX_CANDIDATES)


def _phrase(text):
    if not isinstance(text, str) or not text.strip() or len(text) > MAX_PHRASE_CHARS:
        raise BadRequest(f"phrase is a short text that names one part or thing (at most {MAX_PHRASE_CHARS} "
                         f"characters); phrases is a list of 1 to {MAX_PHRASES} such texts")
    return text.strip()


def phrases_arg(req):
    """What a find request asks for: (texts, many). "phrases" (a list of 1 to MAX_PHRASES texts) wins over
    "phrase" (one text, the first wire) when a client sends both; a text given twice is asked once."""
    many = req.get("phrases")
    if many is None:
        return [_phrase(req.get("phrase"))], False
    if not isinstance(many, list) or not 1 <= len(many) <= MAX_PHRASES:
        raise BadRequest(f"phrases is a list of 1 to {MAX_PHRASES} short texts, each naming one part or thing")
    return list(dict.fromkeys(_phrase(t) for t in many)), True


def number_or_none(value, what):
    """A threshold or a floor of the equipment file: a number from 0 to 1, or None when the file gives none."""
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0.0 <= float(value) <= 1.0:
        raise ValueError(f"equipment find {what} must be a number from 0 to 1")
    return float(value)


def over_floor(regions, floor, gives_confidence=True):
    """The raw regions at or over the floor (all of them when the file gives no floor or the model gives no
    confidence: a null confidence is never under a floor)."""
    if floor is None or not gives_confidence:
        return list(regions)
    return [r for r in regions if r["confidence"] is None or r["confidence"] >= float(floor)]


def answer_of(candidates, threshold, gives_confidence=True):
    """One word for how sure a find is, over the candidates that are returned (those at or over the floor):
    "found" when the first candidate's confidence is at or over the threshold; "unsure" when there are candidates
    and it is not (no threshold declared, or a model that gives no confidence: always unsure); "none" when there
    are no candidates."""
    if not candidates:
        return "none"
    c = candidates[0].get("confidence")
    if gives_confidence and threshold is not None and c is not None and c >= float(threshold):
        return "found"
    return "unsure"


def polygons_arg(req, key="exclude"):
    """A list of pixel polygons (the robot's own body in this picture, from its body model). Optional."""
    polys = req.get(key) or []
    out = []
    if not isinstance(polys, list):
        raise BadRequest(f"{key} is a list of polygons, each a list of [u, v]")
    for p in polys:
        try:
            pts = [(float(u), float(v)) for u, v in p]
        except Exception as exc:
            raise BadRequest(f"{key} is a list of polygons, each a list of [u, v]") from exc
        if len(pts) >= 3:
            out.append(pts)
    return out


def inside(poly, u, v):
    """Even-odd rule point in polygon."""
    hit, n = False, len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > v) != (y2 > v) and u < x1 + (v - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


def inner_point(mask):
    """A point inside the mask: the mask pixel nearest its centroid."""
    ys, xs = np.nonzero(mask)
    i = int(np.argmin((ys - ys.mean()) ** 2 + (xs - xs.mean()) ** 2))
    return [float(xs[i]), float(ys[i])]


def box_of(mask):
    ys, xs = np.nonzero(mask)
    return [float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1)]


def box_polygon(b):
    return [[b[0], b[1]], [b[2], b[1]], [b[2], b[3]], [b[0], b[3]]]


def _hull(points):
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out

    lower, upper = half(pts), half(reversed(pts))
    return lower[:-1] + upper[:-1]


def mask_polygon(mask):
    """Outline of the mask's largest piece, at most MAX_POLYGON_POINTS points. With OpenCV the true contour
    (simplified); without it the convex hull of the mask's edge pixels (a robot computer without OpenCV still
    answers, with a coarser outline)."""
    mask = np.asarray(mask, bool)
    try:
        import cv2
    except ImportError:
        cv2 = None
    if cv2 is not None:
        cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cs:
            return []
        c = max(cs, key=cv2.contourArea)
        eps = 1.0
        poly = cv2.approxPolyDP(c, eps, True)
        while len(poly) > MAX_POLYGON_POINTS:
            eps *= 1.6
            poly = cv2.approxPolyDP(c, eps, True)
        return [[float(p[0][0]), float(p[0][1])] for p in poly]
    edge = mask.copy()
    edge[1:-1, 1:-1] &= ~(mask[:-2, 1:-1] & mask[2:, 1:-1] & mask[1:-1, :-2] & mask[1:-1, 2:])
    ys, xs = np.nonzero(edge)
    hull = _hull(list(zip(xs.tolist(), ys.tolist())))
    step = max(1, int(math.ceil(len(hull) / MAX_POLYGON_POINTS)))
    return [[float(x), float(y)] for x, y in hull[::step]]


def box_iou(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def region(point, box, confidence, mask=None, area=None):
    """One raw region from a backend (before ranking): mask is a bool picture or None."""
    return {"point": [float(point[0]), float(point[1])], "box": [float(v) for v in box],
            "confidence": None if confidence is None else float(confidence), "mask": mask,
            "area_px": int(area if area is not None else (mask.sum() if mask is not None
                                                          else (box[2] - box[0]) * (box[3] - box[1])))}


def select(regions, size, n, exclude=(), max_frac=RANK_PROPOSE["max_frac"], min_px=RANK_PROPOSE["min_px"],
           nms_iou=RANK_PROPOSE["nms_iou"]):
    """The one ranking rule: best confidence first (a region without one keeps the backend's order), drop regions
    over max_frac of the picture or under min_px, drop a region whose point is inside an excluded polygon (the
    robot's own body), drop a region whose box overlaps a kept one by more than nms_iou, keep n. Returns
    (candidates in wire shape, how many were dropped on the robot's own body)."""
    w, h = size
    order = sorted(range(len(regions)), key=lambda i: (-(regions[i]["confidence"] if regions[i]["confidence"]
                                                         is not None else -1.0), i))
    kept, own = [], 0
    for i in order:
        r = regions[i]
        if r["area_px"] > max_frac * w * h or r["area_px"] < min_px:
            continue
        if any(inside(p, r["point"][0], r["point"][1]) for p in exclude):
            own += 1
            continue
        if any(box_iou(r["box"], k["box"]) > nms_iou for k in kept):
            continue
        kept.append(r)
        if len(kept) >= n:
            break
    out = []
    for k, r in enumerate(kept, start=1):
        poly = mask_polygon(r["mask"]) if r["mask"] is not None else box_polygon(r["box"])
        out.append({"n": k, "point": [round(v, 1) for v in r["point"]], "box": [round(v, 1) for v in r["box"]],
                    "polygon": [[round(u, 1), round(v, 1)] for u, v in poly], "area_px": r["area_px"],
                    "confidence": None if r["confidence"] is None else round(r["confidence"], 4),
                    "outline": "mask" if r["mask"] is not None else "box"})
    return out, own
