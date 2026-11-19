# SPDX-License-Identifier: AGPL-3.0-only
"""Backends of the vision service, chosen by the installation's equipment file. No backend names a set of weights:
the equipment file gives the folder or hub id, the code only knows the INTERFACE the weights speak.

    {"propose": {"kind": "text_detection", "weights": "...", "prompt": "a photo of {phrase}", "square_pad": true},
     "find":    {"kind": "text_detection", "weights": "...", "prompt": "a photo of {phrase}", "square_pad": true,
                 "threshold": T, "floor": F},
     "outline": {"kind": "mask_generation", "weights": "...", "first": 3},
     "device": "cuda", "memory_command": ["TOOL", "ARGUMENT", ...]}
"half": true on an entry runs those weights in 16 bit floats (half the memory; for a small robot computer). Two
entries with the same kind and weights share one loaded model. Each entry states its "licence" (the text is
passed on by /describe). On the find entry, "threshold" (T) is the score at or over which a find answers "found"
and "floor" (F, at most T) the score under which a candidate is not returned at all, so that a find can answer
"none"; both are numbers from 0 to 1 that belong to those weights and are set where the weights were measured:
this code holds no value for either, and a file without them never answers found (no T) and answers none only
when the model returns nothing (no F). "memory_command" is optional (rcpx/eyes_service/fit.py).

  kind                 what the weights must speak
  colour_regions       nothing (no model): colour clusters cut into connected pieces. The level "absent".
  mask_generation      the transformers mask-generation pipeline (a promptable segmenter). As "propose" it is run
                       on a point grid; as "outline" it turns the first few candidate boxes into masks
  text_detection       transformers AutoModelForZeroShotObjectDetection (a box and a score per phrase). As
                       "propose" it needs weights whose output carries an objectness score per box. Weights
                       declared "square_pad": true give one score column per text, so several phrases are asked
                       in ONE pass of the model; other weights are asked one pass per phrase
  pointing_calls       a vision-language model with its own point(image, text) or detect(image, text) call
                       (trust_remote_code); it gives no confidence, so every answer is marked unverified

A missing entry means the robot does not have that equipment: propose falls back to colour_regions, find answers
not_equipped. Heavy imports happen inside load() so the file imports on a computer without torch.
"""
import hashlib
import json
import time

import numpy as np

from . import fit
from . import protocol as P

ROLES = ("propose", "find", "outline")
NO_LICENCE = "not stated in the equipment file"


class Backend:
    kind = ""
    gives_confidence = False

    def __init__(self, cfg, device="cpu"):
        self.cfg, self.device = dict(cfg), device

    def load(self):
        return self

    def find_many(self, img, phrases):
        """The raw regions for each phrase, in the order asked. One pass per phrase unless a backend can do better."""
        return [self.find(img, p) for p in phrases]

    def weights_mb(self):
        """The size of the loaded weights' own numbers in MB (0: no model, or not loaded)."""
        return fit.weights_mb(getattr(self, "model", None)) if getattr(self, "model", None) is not None else 0

    def describe(self, cfg=None):
        """What this equipment is, as the equipment file's entry for one role states it (cfg; default this
        backend's own entry). A backend without weights (no model) has weights null and needs no licence."""
        cfg = self.cfg if cfg is None else cfg
        has = cfg.get("weights") is not None
        return {"kind": self.kind, "gives_confidence": self.gives_confidence, "weights": cfg.get("weights"),
                "licence": (cfg.get("licence") or NO_LICENCE) if has else None,
                "half": bool(cfg.get("half")), "weights_mb": self.weights_mb()}


class ColourRegions(Backend):
    """No model. The picture at a quarter of its size, colours grouped into K clusters (k-means, fixed start),
    each cluster cut into 4-connected pieces; a piece scores by how far its colour is from the most common cluster
    and how well it fills its box. Deterministic."""
    kind = "colour_regions"
    gives_confidence = False
    K, ITERS, SHRINK, KEEP = 8, 8, 4, 64

    def propose(self, img):
        w, h = img.size
        s = self.SHRINK
        small = np.asarray(img.resize((max(8, w // s), max(8, h // s))), dtype=np.float32)
        hh, ww = small.shape[:2]
        px = small.reshape(-1, 3)
        # fixed start: K levels along the brightness order, so two runs give the same clusters
        order = np.argsort(px.sum(axis=1), kind="stable")
        cent = px[order[np.linspace(0, len(order) - 1, self.K).astype(int)]].copy()
        for _ in range(self.ITERS):
            lab = np.argmin(((px[:, None, :] - cent[None, :, :]) ** 2).sum(axis=2), axis=1)
            for k in range(self.K):
                if np.any(lab == k):
                    cent[k] = px[lab == k].mean(axis=0)
        lab = lab.reshape(hh, ww)
        bg = cent[int(np.argmax(np.bincount(lab.ravel(), minlength=self.K)))]
        seen = np.zeros((hh, ww), bool)
        out = []
        for y0 in range(hh):
            for x0 in range(ww):
                if seen[y0, x0]:
                    continue
                k = int(lab[y0, x0])
                stack, pix = [(y0, x0)], []
                seen[y0, x0] = True
                while stack:
                    y, x = stack.pop()
                    pix.append((y, x))
                    for yy, xx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                        if 0 <= yy < hh and 0 <= xx < ww and not seen[yy, xx] and lab[yy, xx] == k:
                            seen[yy, xx] = True
                            stack.append((yy, xx))
                if len(pix) < 4:
                    continue
                ys = np.array([p[0] for p in pix])
                xs = np.array([p[1] for p in pix])
                bw, bh = xs.max() - xs.min() + 1, ys.max() - ys.min() + 1
                score = float(np.linalg.norm(cent[k] - bg)) / 441.7 * (len(pix) / float(bw * bh))
                out.append((score, ys, xs))
        regs = []
        for score, ys, xs in sorted(out, key=lambda r: -r[0])[:self.KEEP]:   # full size masks for the best only
            small_m = np.zeros((hh, ww), bool)
            small_m[ys, xs] = True
            m_up = np.repeat(np.repeat(small_m, s, axis=0), s, axis=1)
            m = np.zeros((h, w), bool)
            m[:min(h, m_up.shape[0]), :min(w, m_up.shape[1])] = m_up[:h, :w]
            regs.append(P.region(P.inner_point(m), P.box_of(m), score, m))
        return regs


class MaskGeneration(Backend):
    kind = "mask_generation"
    gives_confidence = True   # the segmenter's own estimate of mask quality, not of "this is a part"

    def load(self):
        from transformers import pipeline
        kw = {}
        if self.cfg.get("half"):
            import torch
            kw["dtype"] = torch.float16
        self.pipe = pipeline("mask-generation", model=self.cfg["weights"],
                             device=0 if self.device.startswith("cuda") else -1, **kw)
        self.model = self.pipe.model
        self._boxer = None
        return self

    def propose(self, img):
        out = self.pipe(img, points_per_batch=int(self.cfg.get("points_per_batch", 64)),
                        points_per_side=int(self.cfg.get("points_per_side", 24)))
        regs = []
        for m, s in zip(out["masks"], out["scores"]):
            m = np.asarray(m.cpu() if hasattr(m, "cpu") else m, dtype=bool)
            if m.sum() == 0:
                continue
            regs.append(P.region(P.inner_point(m), P.box_of(m), float(s), m))
        return regs

    def outlines(self, img, boxes):
        """The masks of what the boxes hold, one per box (None for a box that gave none), in one pass. All None
        when the weights' processor does not take boxes."""
        import torch
        if not boxes:
            return []
        try:
            if self._boxer is None:
                from transformers import AutoProcessor
                self._boxer = AutoProcessor.from_pretrained(self.cfg["weights"])
            inp = self._boxer(images=img, input_boxes=[[[float(v) for v in b] for b in boxes]], return_tensors="pt")
            inp = inp.to(self.pipe.model.device)
            sizes = inp["original_sizes"].cpu()
            if self.cfg.get("half"):
                inp = {k: (v.half() if v.is_floating_point() else v) for k, v in inp.items()}
            with torch.no_grad():
                res = self.pipe.model(**inp, multimask_output=False)
            masks = self._boxer.post_process_masks(res.pred_masks.float().cpu(), sizes)
            m = np.asarray(masks[0]).reshape(len(boxes), -1, img.size[1], img.size[0])[:, 0] > 0
            return [x if x.sum() > 0 else None for x in m]
        except Exception:
            return [None] * len(boxes)


class TextDetection(Backend):
    kind = "text_detection"
    gives_confidence = True

    def load(self):
        import torch
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor
        self.torch = torch
        self.proc = AutoProcessor.from_pretrained(self.cfg["weights"])
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(self.cfg["weights"]).to(self.device).eval()
        if self.cfg.get("half"):
            self.model = self.model.half()
        return self

    def _inputs(self, inp):
        inp = inp.to(self.device)
        if self.cfg.get("half"):
            inp = {k: (v.half() if v.is_floating_point() else v) for k, v in inp.items()}
        return inp

    def propose(self, img):
        """Regions by the detector's own objectness (no words). Weights without that output cannot propose."""
        torch = self.torch
        w, h = img.size
        with torch.no_grad():
            inp = self._inputs(self.proc(text=[["an object"]], images=img, return_tensors="pt"))
            out = self.model(**inp)
        obj = getattr(out, "objectness_logits", None)
        if obj is None:
            raise ValueError("these weights give no objectness score: they cannot propose without words")
        sc = torch.sigmoid(obj[0]).float().cpu().numpy()
        bx = out.pred_boxes[0].float().cpu().numpy() * float(max(w, h) if self.cfg.get("square_pad") else 1.0)
        if not self.cfg.get("square_pad"):
            bx = bx * np.array([w, h, w, h], dtype=np.float32)
        return self._boxes(sc, bx, w, h, 300)

    def _boxes(self, sc, bx, w, h, keep):
        regs = []
        for i in np.argsort(-sc)[:keep]:
            cx, cy, bw, bh = (float(v) for v in bx[i])
            b = [max(0.0, cx - bw / 2), max(0.0, cy - bh / 2), min(float(w), cx + bw / 2), min(float(h), cy + bh / 2)]
            if b[2] - b[0] < 1 or b[3] - b[1] < 1:
                continue
            regs.append(P.region([(b[0] + b[2]) / 2, (b[1] + b[3]) / 2], b, float(sc[i])))
        return regs

    def _text(self, phrase):
        return self.cfg.get("prompt", "{phrase}.").format(phrase=phrase.strip().rstrip(".").lower())

    def _columns(self, img, texts):
        """ONE pass of the model for all the texts (weights that give one score column per text):
        (scores [boxes, texts], boxes [boxes, 4] as centre and size in pixels)."""
        torch = self.torch
        with torch.no_grad():
            out = self.model(**self._inputs(self.proc(text=[list(texts)], images=img, return_tensors="pt")))
        sc = torch.sigmoid(out.logits[0]).float().cpu().numpy()[:, :len(texts)]
        return sc, out.pred_boxes[0].float().cpu().numpy() * float(max(img.size))

    def _single(self, img, text):
        """One pass for one text (weights that score text tokens): (scores [boxes], boxes [boxes, 4] in pixels)."""
        torch = self.torch
        w, h = img.size
        with torch.no_grad():
            out = self.model(**self._inputs(self.proc(images=img, text=text, return_tensors="pt")))
        sc = torch.sigmoid(out.logits[0]).max(dim=-1).values.float().cpu().numpy()
        return sc, out.pred_boxes[0].float().cpu().numpy() * np.array([w, h, w, h], dtype=np.float32)

    def find_many(self, img, phrases):
        w, h = img.size
        texts = [self._text(p) for p in phrases]
        if self.cfg.get("square_pad"):
            sc, bx = self._columns(img, texts)
            return [self._boxes(sc[:, k], bx, w, h, 40) for k in range(len(texts))]
        return [self._boxes(*self._single(img, t), w, h, 40) for t in texts]

    def find(self, img, phrase):
        return self.find_many(img, [phrase])[0]


class PointingCalls(Backend):
    kind = "pointing_calls"
    gives_confidence = False

    def load(self):
        from transformers import AutoModelForCausalLM
        self.model = AutoModelForCausalLM.from_pretrained(self.cfg["weights"], trust_remote_code=True,
                                                          device_map={"": self.device})
        return self

    def find(self, img, phrase):
        w, h = img.size
        regs = []
        if self.cfg.get("call", "point") == "detect":
            for o in self.model.detect(img, phrase)["objects"]:
                b = [o["x_min"] * w, o["y_min"] * h, o["x_max"] * w, o["y_max"] * h]
                regs.append(P.region([(b[0] + b[2]) / 2, (b[1] + b[3]) / 2], b, None))
        else:
            r = float(self.cfg.get("point_box_px", 12))
            for p in self.model.point(img, phrase)["points"]:
                u, v = p["x"] * w, p["y"] * h
                regs.append(P.region([u, v], [max(0.0, u - r), max(0.0, v - r), min(float(w), u + r),
                                              min(float(h), v + r)], None))
        return regs


KINDS = {c.kind: c for c in (ColourRegions, MaskGeneration, TextDetection, PointingCalls)}


class Eyes:
    """The equipment of one robot: what it can propose and find, loaded once."""

    def __init__(self, equipment):
        self.equipment = dict(equipment or {})
        dev = self.equipment.get("device", "cpu")
        self._made = {}
        self.fallback = ColourRegions({}, "cpu")
        self.finder = self._make(self.equipment.get("find"), dev)
        self.proposer = self._make(self.equipment.get("propose"), dev)
        self.outliner = self._make(self.equipment.get("outline"), dev)
        if self.outliner is not None and not isinstance(self.outliner, MaskGeneration):
            raise ValueError("equipment outline must be of kind mask_generation")
        thr, floor = self.limits()
        if thr is not None and floor is not None and floor > thr:
            raise ValueError("equipment find floor must not be over its threshold")
        self.loaded_s = None

    def limits(self):
        """(threshold, floor) of the find role as the equipment file gives them (None: the file gives none)."""
        cfg = self.finder.cfg if self.finder is not None else {}
        return P.number_or_none(cfg.get("threshold"), "threshold"), P.number_or_none(cfg.get("floor"), "floor")

    def blocks_sha256(self):
        """sha256 of the propose, find and outline entries as this process was given them (JSON with sorted keys
        and no spaces), so a caller can tell which declared equipment is answering."""
        blocks = {r: self.equipment.get(r) for r in ROLES}
        return hashlib.sha256(json.dumps(blocks, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()

    def _make(self, cfg, dev):
        if not cfg:
            return None
        if cfg.get("kind") not in KINDS:
            raise ValueError(f"equipment kind {cfg.get('kind')!r} is not one of {sorted(KINDS)}")
        key = (cfg["kind"], cfg.get("weights"), bool(cfg.get("half")))
        if key in self._made:   # the same weights behind two jobs are loaded once (the first entry's settings)
            return self._made[key]
        self._made[key] = KINDS[cfg["kind"]](cfg, dev)
        return self._made[key]

    @property
    def level(self):
        if self.finder is not None:
            return "words"
        if self.proposer is not None and self.proposer.kind != "colour_regions":
            return "segments"
        return "absent"

    def load(self):
        t = time.time()
        for b in self._made.values():
            b.load()
        self.loaded_s = round(time.time() - t, 2)
        return self

    def warm(self):
        """One propose and one find on a blank picture, so the first real request does not pay for the first pass
        of the model. Returns the seconds it took (None when a backend failed on it: the service still starts)."""
        from PIL import Image
        t = time.time()
        try:
            img = Image.new("RGB", (640, 480), (128, 128, 128))
            self.propose(img, 1)
            if self.finder is not None:
                self.find(img, "a thing", 1)
        except Exception:
            return None
        return round(time.time() - t, 2)

    def describe(self):
        """The declared equipment, role by role, and what the process holds in memory at this call."""
        res = fit.resident_mb()
        gpu, how = fit.gpu_mb(self.equipment.get("device", "cpu"), self.equipment.get("memory_command"))
        total = int(res + (gpu or 0))
        roles = {"propose": self.proposer or self.fallback, "find": self.finder, "outline": self.outliner}
        thr, floor = self.limits()
        out = {"level": self.level, "roles": [r for r in ROLES if roles[r] is not None]}
        for r in ROLES:
            if roles[r] is None:
                out[r] = None
                continue
            out[r] = dict(roles[r].describe(self.equipment.get(r) or {}), memory_mb=total)
            if r == "find":   # the two numbers a find answers by; null: the equipment file gives none
                out[r].update(threshold=thr, floor=floor)
            same = [q for q in ROLES if q != r and roles[q] is roles[r]]
            if same:   # one loaded model behind two roles: its weights are in memory once
                out[r]["loaded_once_with"] = same
        out.update({"memory": {"resident_mb": res, "gpu_mb": gpu, "gpu_read": how,
                               "says": "read at this call; one process serves every role, so memory_mb is the "
                                       "same whole-process figure (resident plus graphics memory) in each role, "
                                       "not a share to add up; weights_mb is that role's own weights"},
                    "device": self.equipment.get("device", "cpu"), "load_seconds": self.loaded_s, "phrases": True,
                    "equipment_sha256": self.blocks_sha256(),
                    "ranking": {"propose": dict(P.RANK_PROPOSE), "find": dict(P.RANK_FIND),
                                "says": "max_frac: a region over this share of the picture is dropped; min_px: "
                                        "the smallest region kept; nms_iou: the box overlap over which the "
                                        "weaker of two regions is dropped"},
                    "limits": {"max_candidates": P.MAX_CANDIDATES, "max_phrases": P.MAX_PHRASES,
                               "max_image_bytes": P.MAX_IMAGE_BYTES, "max_side_px": P.MAX_SIDE_PX,
                               "min_side_px": P.MIN_SIDE_PX}})
        return out

    def _outline(self, img, lists):
        """Masks for the first few candidates of each list that have none (one segmenter pass for all lists)."""
        if self.outliner is None:
            return
        first = int(self.outliner.cfg.get("first", 3))
        todo = []
        for cands in lists:
            todo.extend([c for c in cands if c["outline"] == "box"][:first])
        for c, m in zip(todo, self.outliner.outlines(img, [c["box"] for c in todo])):
            if m is not None:
                c["polygon"] = [[round(u, 1), round(v, 1)] for u, v in P.mask_polygon(m)]
                c["point"] = [round(v, 1) for v in P.inner_point(m)]
                c["area_px"], c["outline"] = int(m.sum()), "mask"

    def propose(self, img, n, exclude=()):
        b = self.proposer or self.fallback
        t = time.time()
        cands, own = P.select(b.propose(img), img.size, n, exclude)
        self._outline(img, [cands])
        means = {"colour_regions": "a colour contrast score, not a probability",
                 "mask_generation": "the segmenter's estimate of outline quality, not that the region is a part",
                 "text_detection": "the detector's score that the box holds a thing of any kind"}.get(b.kind)
        return {"candidates": cands, "source": b.kind, "level": self.level, "dropped_on_own_body": own,
                "confidence_means": means, "image_size": list(img.size), "seconds": round(time.time() - t, 3)}

    def find(self, img, phrases, n, exclude=()):
        """phrases: one text (the first wire: the answer carries "phrase") or a list of texts (the answer carries
        "phrases"). n counts per phrase. Each phrase is ranked on its own, as in a request with that phrase alone
        (weights run in 16 bit floats can score a phrase a little differently in the two forms, in the third
        decimal). A candidate under the equipment file's floor is not returned; a phrase with none left answers
        "none"."""
        many = not isinstance(phrases, str)
        texts = list(phrases) if many else [phrases]
        asked = {"phrases": texts} if many else {"phrase": texts[0]}
        if self.finder is None:
            return {"found": False, "answer": "none", "candidates": [], **asked,
                    "per_phrase": {t: "none" for t in texts}, "not_equipped": True, "level": self.level,
                    "why": "this robot has no vision model that takes words",
                    "instead": "propose returns numbered regions to pick from; touch confirms a pick",
                    "image_size": list(img.size), "seconds": 0.0}
        b = self.finder
        t = time.time()
        thr, floor = self.limits()
        lists, own, per = [], 0, {}
        for text, regs in zip(texts, b.find_many(img, texts)):
            regs = P.over_floor(regs, floor, b.gives_confidence)
            cands, dropped = P.select(regs, img.size, n, exclude, **P.RANK_FIND)
            for c in cands:
                c["phrase"] = text
            per[text] = P.answer_of(cands, thr, b.gives_confidence)
            lists.append(cands)
            own += dropped
        self._outline(img, lists)
        cands = [c for cs in lists for c in cs]
        for k, c in enumerate(cands, start=1):
            c["n"] = k
        answer = "found" if "found" in per.values() else ("unsure" if cands else "none")
        return {"found": answer == "found", "answer": answer, "candidates": cands, **asked, "per_phrase": per,
                "threshold": thr, "floor": floor, "unverified": True, "gives_confidence": b.gives_confidence,
                "level": self.level, "dropped_on_own_body": own,
                "confidence_means": "the detector's score for the phrase, not a probability; the answer is found "
                                    "only when a phrase's first candidate is at or above threshold, and none when "
                                    "no candidate reaches the floor. A part that is not in the picture can still "
                                    "get candidates: confirm a pick by touch or a second view"
                if b.gives_confidence else "this model gives no confidence: every answer is unsure",
                "image_size": list(img.size), "seconds": round(time.time() - t, 3)}
