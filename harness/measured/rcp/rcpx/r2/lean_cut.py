# SPDX-License-Identifier: AGPL-3.0-only
"""The cutting rule of the lean profile (rcpx/r2/lean.py): one answer or document in, a smaller one out. Pure
functions of their input: nothing here knows a tool, a robot or a device, and nothing is decided by a tool's name.

THE RULE, the same for every tool answer and every resource document:
1. Values always stay: numbers, texts, true, false, null, and a short vector (a list of at most MAX_ITEMS plain
   values: a point, a pixel, a direction, a few names).
2. A list of more than MAX_ITEMS items is replaced by a marker.
3. Objects and lists are kept MAX_LEVELS levels below the answer (the answer's own keys are level 1); one that
   sits deeper is replaced by a marker. Inside an object the cut is member by member, so its plain values stay.
4. A list is all or nothing: when any item of it would lose a part, the whole list is replaced by ONE marker (a
   table with holes reads worse than a pointer to the table, and a marker per row costs more than it saves).
5. A key in KEEP holds the substance of its answer and is judged on its own: levels are counted again from it
   and its own length is not limited. Rules 1 to 4 still apply inside it.
6. Nothing under a key in NEVER (refused, hint, warning) is cut. In a tool answer a block that holds such a key
   anywhere inside is not replaced either (the hint is about this call); in a document it is (a refusal in a
   journal record is history). An answer that is a refusal passes whole: that is the caller's job (lean.py).
The marker is {"omitted": N, "see": WHERE}: N is the length of the block left out and WHERE says where the whole
is ("last_result" for a tool answer, the full document's URI for a resource).

FRAMES: an answer names image files and a client attaches each one, so at most `keep` of them stay as a path:
one for a tool answer, none for a document. The one kept is the most informative by its key and file name: an
annotated frame with axes, else an annotated frame, else the first. Every other path leaves its place (a key is
dropped, a list item becomes text). A tool answer lists them under "more_frames" as "KEY.PATH: FILE (not
attached)", text that no client takes for a path; KEY.PATH is what last_result(key) takes to attach that frame.
A document says how many it left out and where they are named; a whole document (robot://NAME/full, a template
entry) keeps each path where it stands, as text. When the chosen frame sat in a block that rules 2 to 4
replaced, it is put back under "frame_shown".
"""
import os

from rcpx.surface_args import IMAGE_EXT

MAX_ITEMS = 8        # a longer list is a block of data, not a value
MAX_LEVELS = 2       # nesting kept below the answer
MORE_FRAMES_MAX = 6  # frame names listed in one answer; the rest are counted
NOT_ATTACHED = " (not attached)"

# A key here is judged on its own (rule 5). Each holds what its answer is FOR: cutting it would send the operator
# straight to the whole answer and save nothing.
KEEP = {
    "image_to_base": "how a pixel offset in the frame becomes a move in the base frame: the bridge from a look to "
                     "a move, which a model cannot rebuild from the picture",
    "entries": "the world document's own content: each known point, object and device state",
    "devices": "the device document's own content: each device with its id (what verify and device_state take)",
    "parts": "what a device is made of and how each part may be approached: no stroke can be planned without it",
}
# Never cut (rule 6): what tells the operator that a call did not run, or what to do next.
NEVER = ("refused", "hint", "hints", "warning", "warnings")


def is_plain(v):
    return v is None or isinstance(v, (bool, int, float, str))


def is_vector(v):
    return isinstance(v, list) and len(v) <= MAX_ITEMS and all(is_plain(x) for x in v)


def is_marker(v):
    return isinstance(v, dict) and len(v) == 2 and "omitted" in v and "see" in v


def holds_never(v):
    """True when a NEVER key sits anywhere inside v."""
    if isinstance(v, dict):
        return any(k in NEVER or holds_never(x) for k, x in v.items())
    if isinstance(v, list):
        return any(holds_never(x) for x in v)
    return False


class _Cutter:
    def __init__(self, see, protect):
        self.see, self.protect = see, protect

    def replace(self, v, cuts, path):
        if self.protect and holds_never(v):
            return v, False
        cuts.append(path)
        return {"omitted": len(v), "see": self.see}, True

    def members(self, v, level, cuts, path):
        """A dict whose members sit at level + 1, cut member by member: (the dict, True when a part went)."""
        out, lost = {}, False
        for k, item in v.items():
            here = f"{path}.{k}" if path else str(k)
            if k in NEVER:
                out[k] = item
            elif k in KEEP:
                out[k] = self.root(item, cuts, here)   # judged on its own: its loss is not its parent's loss
            else:
                out[k], part = self.cut(item, level + 1, cuts, here)
                lost = lost or part
        return out, lost

    def cut(self, v, level, cuts, path):
        """(v cut for its level, True when v or a part of it was replaced by a marker)."""
        if is_plain(v) or is_vector(v) or is_marker(v):
            return v, False
        if level > MAX_LEVELS or (isinstance(v, list) and len(v) > MAX_ITEMS):
            return self.replace(v, cuts, path)
        if isinstance(v, dict):
            return self.members(v, level, cuts, path)
        inner, out, lost = [], [], False
        for i, item in enumerate(v):
            new, part = self.cut(item, level + 1, inner, f"{path}.{i}")
            out.append(new)
            lost = lost or part
        if lost:                 # rule 4: one marker for the list; the cuts inside it are not reported
            return self.replace(v, cuts, path)   # (in an answer, a list that holds a hint comes back whole)
        cuts.extend(inner)       # cuts inside a KEEP key of an item: the item itself lost nothing
        return out, False

    def root(self, v, cuts, path):
        """v as a block judged on its own (the answer itself, or a KEEP key): its members are level 1 and, when
        it is a list, each item is cut for itself and the list's own length is not limited."""
        if isinstance(v, dict):
            return self.members(v, 0, cuts, path)[0]
        if isinstance(v, list) and not is_vector(v):
            return [self.cut(item, 1, cuts, f"{path}.{i}")[0] for i, item in enumerate(v)]
        return v


def cut(obj, see, protect=True):
    """(obj with rules 1 to 6 applied, the key paths that were replaced). obj itself is never replaced.
    protect: a block that holds a NEVER key is not replaced (a tool answer); False for a document."""
    cuts = []
    return _Cutter(see, protect).root(obj, cuts, ""), cuts


# ---------------------------------------------------------------------------------------------------------------
# Frames


def is_image_path(x):
    return isinstance(x, str) and x.lower().endswith(IMAGE_EXT) and "\n" not in x


def frames_in(obj, attached_only=False):
    """[(key path as a tuple, the path text)] for every image path in obj, in order. attached_only leaves out
    what a client would not attach (the older key of a "supersedes" pair, as rcpx.surface_args.frame_paths)."""
    found = []

    def walk(x, path):
        if isinstance(x, dict):
            sup = x.get("supersedes") if attached_only else None
            skip = {old for new, old in sup.items() if is_image_path(x.get(new))} if isinstance(sup, dict) else ()
            for k, v in x.items():
                if k not in skip:
                    walk(v, path + (str(k),))
        elif isinstance(x, list):
            for i, v in enumerate(x):
                walk(v, path + (str(i),))
        elif is_image_path(x):
            found.append((path, x))

    walk(obj, ())
    return found


def _rank(path, value):
    text = ("/".join(path) + "/" + os.path.basename(value)).lower()
    if "annot" in text:
        return 0 if "axes" in text else 1
    return 2


def choose_frame(obj):
    """The one frame of obj to attach (see FRAMES above), or None when it names none."""
    found = frames_in(obj, attached_only=True)
    if not found:
        return None
    return min(enumerate(found), key=lambda item: (_rank(*item[1]), item[0]))[1][1]


def short_path(path, base_dir):
    """path relative to base_dir when it lies inside it (what an operator that reads the episode dir needs)."""
    if base_dir:
        base = os.path.abspath(str(base_dir)).rstrip(os.sep) + os.sep
        if os.path.isabs(path) and path.startswith(base):
            return path[len(base):]
    return path


def _strip(x, chosen, base_dir):
    """x without any image path but `chosen`: a dict key is dropped, a list item becomes text."""
    if isinstance(x, dict):
        return {k: _strip(v, chosen, base_dir) for k, v in x.items() if not (is_image_path(v) and v != chosen)}
    if isinstance(x, list):
        return [short_path(v, base_dir) + NOT_ATTACHED if is_image_path(v) and v != chosen
                else _strip(v, chosen, base_dir) for v in x]
    return x


def name_frames(x, base_dir=None):
    """x with every image path turned into text where it stands: nothing attached, nothing dropped (what a
    whole document does with the frames it names)."""
    if isinstance(x, dict):
        return {k: name_frames(v, base_dir) for k, v in x.items()}
    if isinstance(x, list):
        return [name_frames(v, base_dir) for v in x]
    return short_path(x, base_dir) + NOT_ATTACHED if is_image_path(x) else x


def one_frame(full, cut_obj, base_dir=None, keep=1, limit=MORE_FRAMES_MAX):
    """(cut_obj with at most `keep` (1 or 0) frame paths left, the texts for "more_frames", the frame chosen).
    The choice and the texts come from `full` (the answer before any cut), so a frame inside a replaced block is
    still named. limit None lists every frame."""
    chosen = choose_frame(full) if keep else None
    out = _strip(cut_obj, chosen, base_dir)
    names, seen = [], {chosen}
    for path, value in frames_in(full):
        if value not in seen:
            seen.add(value)
            names.append(f"{'.'.join(path)}: {short_path(value, base_dir)}{NOT_ATTACHED}")
    if limit is not None and len(names) > limit:
        names = names[:limit] + [f"and {len(names) - limit} more: the whole answer names each one"]
    return out, names, chosen


def lean(full, see, base_dir=None, keep=1):
    """(the lean form of a dict, what was done): the rule, then the frame rule. keep 1 is a tool answer, keep 0
    a document (no frame attached, no NEVER protection of a replaced block). done is {"cut": [key paths],
    "frames_left_out": n, "frame": the path kept or None}."""
    body, cuts = cut(full, see, protect=bool(keep))
    body, names, chosen = one_frame(full, body, base_dir, keep, MORE_FRAMES_MAX if keep else None)
    if isinstance(body, dict):
        if names and keep:
            body["more_frames"] = names
        elif names:
            body["more_frames"] = f"{len(names)} frames are not named here: {see} names each one"
        if chosen is not None and not any(v == chosen for _, v in frames_in(body)):
            body["frame_shown"] = chosen
    return body, {"cut": cuts, "frames_left_out": len({v for _, v in frames_in(full)} - {chosen}), "frame": chosen}


def dig(obj, key):
    """(True, the part of obj that `key` names) or (False, None). key is a top-level key, or a dotted path
    through objects and list positions ("look.axes_px", "frames.3.frame")."""
    if isinstance(obj, dict) and key in obj:
        return True, obj[key]
    here = obj
    for part in str(key).split("."):
        if isinstance(here, dict) and part in here:
            here = here[part]
        elif isinstance(here, list) and part.lstrip("-").isdigit() and -len(here) <= int(part) < len(here):
            here = here[int(part)]
        else:
            return False, None
    return True, here
