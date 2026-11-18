# SPDX-License-Identifier: AGPL-3.0-only
"""The client of the robot's vision service (SPEC-R5 7.2, 7.4): urllib only, never raises.

The service is robot equipment declared in the installation; its address is given by the caller (there is no
default address here). Every call returns (answer or None, reason or None, seconds): any failure (no address, a
refused connection, a timeout, an HTTP error, an answer that is not the wire shape) is None with a short reason,
so the caller can fall back to colour and geometry for that call and say so.

The picture is always sent as image_b64 (the file's own bytes): the service honours a path only under a frames
root it was started with, and this client never sends one.

    health(url)                          GET /health
    describe(url)                        GET /describe
    propose(url, image, max, exclude)    POST /propose
    find(url, image, phrase=..., phrases=[...], max, exclude)   POST /find (one phrase, or several in one request)
    find_each(url, image, phrases, max, exclude, many)          every phrase: one request when the service
                                         understands phrases (many true), else one request per phrase
    rows(answer, source)                 the answer's candidates in the package's own row shape
    inside(row, u, v)                    True when the pixel lies in the row's polygon (its box without one)

Nothing here names a model or a set of weights, and nothing reads a file but the picture the caller names.
"""

import base64
import json
import time
import urllib.error
import urllib.request

TIMEOUT_S = 3.0          # SPEC-R5 7.4: a service that has not answered by then is absent for that call
MAX_PHRASES = 6          # phrases per request (the service's own limit)
MAX_CANDIDATES = 16


def _request(url, path, body=None, timeout=TIMEOUT_S):
    """(the JSON object or None, reason or None, seconds)."""
    t0 = time.time()
    if not isinstance(url, str) or not url.strip():
        return None, "no service address", 0.0
    full = url.rstrip("/") + path
    try:
        if body is None:
            req = urllib.request.Request(full, method="GET")
        else:
            req = urllib.request.Request(full, data=json.dumps(body).encode("utf-8"),
                                         headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=float(timeout)) as resp:
            raw = resp.read()
        out = json.loads(raw)
    except urllib.error.HTTPError as exc:
        return None, f"the service answered {exc.code}", round(time.time() - t0, 3)
    except Exception as exc:      # refused, timed out, not JSON: the service is absent for this call
        return None, f"no answer ({type(exc).__name__})", round(time.time() - t0, 3)
    if not isinstance(out, dict):
        return None, "the answer is not the wire shape", round(time.time() - t0, 3)
    return out, None, round(time.time() - t0, 3)


def image_bytes(image):
    """The picture's bytes: bytes as given, or the bytes of the file a path names; None when it cannot be read."""
    if isinstance(image, (bytes, bytearray)):
        return bytes(image)
    if isinstance(image, str):
        try:
            with open(image, "rb") as f:
                return f.read()
        except OSError:
            return None
    return None


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _wire_ok(answer):
    """True when the answer holds a candidates list in the wire shape (protocol.py)."""
    cands = answer.get("candidates") if isinstance(answer, dict) else None
    if not isinstance(cands, list):
        return False
    for c in cands:
        if not isinstance(c, dict):
            return False
        pt, box = c.get("point"), c.get("box")
        if not (isinstance(pt, list) and len(pt) == 2 and all(_num(v) for v in pt)):
            return False
        if not (isinstance(box, list) and len(box) == 4 and all(_num(v) for v in box)):
            return False
    return True


def _body(image, max, exclude):
    raw = image_bytes(image)
    if raw is None:
        return None
    body = {"image_b64": base64.b64encode(raw).decode("ascii"),
            "max": int(min(MAX_CANDIDATES, max if _num(max) and max >= 1 else 8))}
    polys = [[[round(float(u), 1), round(float(v), 1)] for u, v in p] for p in (exclude or ()) if len(p) >= 3]
    if polys:
        body["exclude"] = polys
    return body


def health(url, timeout=TIMEOUT_S):
    return _request(url, "/health", None, timeout)


def describe(url, timeout=TIMEOUT_S):
    return _request(url, "/describe", None, timeout)


def propose(url, image, max=8, exclude=(), timeout=TIMEOUT_S):
    body = _body(image, max, exclude)
    if body is None:
        return None, "the picture cannot be read", 0.0
    out, why, s = _request(url, "/propose", body, timeout)
    if out is not None and not _wire_ok(out):
        return None, "the answer is not the wire shape", s
    return out, why, s


def find(url, image, phrase=None, phrases=None, max=8, exclude=(), timeout=TIMEOUT_S):
    """One find request: phrase (one text) or phrases (a list, at most MAX_PHRASES, one pass of the model)."""
    body = _body(image, max, exclude)
    if body is None:
        return None, "the picture cannot be read", 0.0
    if phrases is not None:
        body["phrases"] = [str(p) for p in list(phrases)[:MAX_PHRASES]]
    else:
        body["phrase"] = str(phrase)
    out, why, s = _request(url, "/find", body, timeout)
    if out is not None and not _wire_ok(out):
        return None, "the answer is not the wire shape", s
    return out, why, s


def find_each(url, image, phrases, max=3, exclude=(), many=None, timeout=TIMEOUT_S):
    """Every phrase asked: ({phrase: [wire candidates]}, info, reason or None, seconds, requests).
    many True: the phrases go in one request (MAX_PHRASES per request); many False: one request per phrase; many
    None (not known yet): one request with phrases is tried and, when the service refuses it as a bad request (the
    first wire, which knows one phrase), each phrase is asked alone. info: {"threshold", "floor", "many": what the
    service turned out to understand, "per_phrase": {text: its answer word}}. The first failure ends it
    (None, ..., reason): the caller falls back for the whole call."""
    phrases = list(dict.fromkeys(str(p) for p in phrases or ()))
    per, info, total, sent = {}, {"threshold": None, "floor": None, "many": many, "per_phrase": {}}, 0.0, 0

    def take(answer, texts):
        for key in ("threshold", "floor"):
            if _num(answer.get(key)):
                info[key] = float(answer[key])
        if isinstance(answer.get("per_phrase"), dict):
            info["per_phrase"].update({str(k): v for k, v in answer["per_phrase"].items()})
        for c in answer["candidates"]:
            text = c.get("phrase") if c.get("phrase") in texts else texts[0]
            per.setdefault(text, []).append(c)
        for t in texts:
            per.setdefault(t, [])

    if many is not False:
        ok = True
        for i in range(0, len(phrases), MAX_PHRASES):
            chunk = phrases[i:i + MAX_PHRASES]
            out, why, s = find(url, image, phrases=chunk, max=max, exclude=exclude, timeout=timeout)
            total, sent = total + s, sent + 1
            if out is None:
                if many is None and i == 0 and why == "the service answered 400":
                    ok = False        # the first wire: it reads one phrase per request
                    break
                return None, info, why, round(total, 3), sent
            take(out, chunk)
        if ok:
            info["many"] = True
            return per, info, None, round(total, 3), sent
        per.clear()
    info["many"] = False
    for text in phrases:
        out, why, s = find(url, image, phrase=text, max=max, exclude=exclude, timeout=timeout)
        total, sent = total + s, sent + 1
        if out is None:
            return None, info, why, round(total, 3), sent
        take(out, [text])
    return per, info, None, round(total, 3), sent


def rows(candidates, source):
    """Wire candidates as the package's rows: point_px, box_px, polygon_px, area_px, confidence, source, phrase."""
    out = []
    for c in candidates or ():
        box = [float(v) for v in c["box"]]
        poly = c.get("polygon")
        if not (isinstance(poly, list) and len(poly) >= 3 and all(
                isinstance(p, list) and len(p) == 2 and all(_num(v) for v in p) for p in poly)):
            poly = [[box[0], box[1]], [box[2], box[1]], [box[2], box[3]], [box[0], box[3]]]
        conf = c.get("confidence")
        area = c.get("area_px")
        out.append({"point_px": [float(c["point"][0]), float(c["point"][1])], "box_px": box,
                    "polygon_px": [[float(u), float(v)] for u, v in poly],
                    "area_px": int(area if _num(area) else max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])),
                    "confidence": float(conf) if _num(conf) else None, "source": source,
                    "phrase": c.get("phrase")})
    return out


def in_polygon(poly, u, v) -> bool:
    """Even-odd rule point in polygon."""
    hit, n = False, len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > v) != (y2 > v) and u < x1 + (v - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


def inside(row, u, v) -> bool:
    """True when pixel (u, v) lies in the row's polygon, or in its box when it has no polygon."""
    poly = row.get("polygon_px") or row.get("polygon")
    if isinstance(poly, list) and len(poly) >= 3:
        return in_polygon(poly, float(u), float(v))
    box = row.get("box_px") or row.get("box")
    return bool(box) and box[0] <= u <= box[2] and box[1] <= v <= box[3]


# ---------------------------------------------------------------- one session's use of the service
LEVELS = ("absent", "segments", "words")
RETRY_CALLS = 10          # calls of the package that pass before a service that failed is asked again (7.4)


def new_service(url, level_declared):
    """The service record of ex.state["eyes"]["service"]: no request is made."""
    return {"url": url, "ok": None, "seconds_last": None, "fallbacks": 0, "skip": 0, "phrases": None,
            "describe": None, "asked": False, "level_declared": bool(level_declared), "why": None}


def failed(svc, why):
    """The service did not answer: this call falls back, and so do the next RETRY_CALLS calls."""
    svc["ok"], svc["why"] = False, why
    svc["skip"] = RETRY_CALLS
    svc["fallbacks"] += 1


def service_for(st):
    """The service record when this call may ask it, else None (level absent, or inside the retry window: counted
    as a fallback). The first call of a session asks /health (the level word when the installation declared only
    an address, and whether several phrases go in one request) and /describe (kept for robot://assist)."""
    svc = st.get("service")
    if st.get("level") == "absent" or not isinstance(svc, dict):
        return None
    if svc["skip"] > 0:
        svc["skip"] -= 1
        svc["fallbacks"] += 1
        return None
    if not svc["asked"]:
        svc["asked"] = True
        answer, why, _ = health(svc["url"])
        if answer is None:
            failed(svc, why)
            return None
        svc["phrases"] = bool(answer.get("phrases") is True)
        if not svc["level_declared"] and answer.get("level") in LEVELS:
            st["level"] = answer["level"]
        svc["describe"] = describe(svc["url"])[0]
        if st["level"] == "absent":
            return None
    return svc


def limits(svc, info):
    """(threshold, floor) of the equipment: the answer's own numbers, else the find block of /describe; None for a
    number the equipment does not state. No number is typed here."""
    block = (svc.get("describe") or {}).get("find") if isinstance(svc, dict) else None
    block = block if isinstance(block, dict) else {}
    out = []
    for key in ("threshold", "floor"):
        v = (info or {}).get(key)
        v = block.get(key) if v is None else v
        out.append(float(v) if _num(v) else None)
    return out[0], out[1]


def pixel_in_find(url, image, phrase, u, v, exclude=(), first=8):
    """"in" when pixel (u, v) lies in one of the first `first` candidates the service finds for the phrase in the
    picture, "out" when it gives candidates and the pixel lies in none, "not known" otherwise. One request."""
    answer, _, _ = find(url, image, phrase=str(phrase), max=first, exclude=exclude)
    if answer is None or not answer["candidates"]:
        return "not known"
    return "in" if any(inside(r, float(u), float(v)) for r in rows(answer["candidates"][:first], "find")) else "out"


__all__ = ["LEVELS", "MAX_PHRASES", "RETRY_CALLS", "TIMEOUT_S", "describe", "failed", "find", "find_each", "health",
           "image_bytes", "in_polygon", "inside", "limits", "new_service", "pixel_in_find", "propose", "rows",
           "service_for"]
