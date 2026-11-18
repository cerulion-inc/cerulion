# SPDX-License-Identifier: AGPL-3.0-only AND CC-BY-4.0
"""The WITHOUT rcpx surface for LIBERO and LIBERO-PRO: the published zero-shot protocol of arXiv:2610.30051v1 (May
Espinola, Qifan Zhang and Yu Xiang, "From Reasoning to Action: Benchmarking Frontier AI Models on LIBERO-10 Robot
Manipulation Tasks", licence CC BY 4.0), served over the rcpx socket protocol by bench/ext/episode_ext.py. The system
prompt (the paper's Figure 2) and the tool descriptions (its Figure 3) are the paper's text BYTE FOR BYTE, copied
from the listing sources embedded in https://arxiv.org/html/2610.30051v1 (the design notes holds
them, every sentence that fixes semantics and the list of declared differences). The authors' code repository has
no licence and was never opened: the code here is a clean build from the paper's text. Three numbers are not in
the paper and are set here: the per-step caps STEP_CAP.

Tools: move_to(targets, note), done(summary, hindsight), give_up(reason, hindsight). Only two things are filled in:
the embodiment name in the system prompt ('libero_10_taskNN', as the paper says) and, on a cell whose workspace is
not the published one, the three position bounds in the move_to description. The listing carries no worked example
and no conventions text, so nothing is added to a description.

robot://task: {instruction, embodiment, first_message {text, images [{label, path} x 2], system, own_tools false},
observation, protocol}. first_message is the chat harness's shape (bench/operators/chat_prompt.py first_message_of):
system is the paper's prompt, text the first observation (the task sentence, then eef_state and gripper_qpos, the
names the prompt uses), images the two start pictures, own_tools false = the model gets exactly the three tools.

move_to: a straight line in the 10-D space from the MEASURED hand pose (the paper: "from the current end-effector
pose") and the current gripper command to the clamped target, cut into n = max over dims of ceil(|delta| / cap)
20 Hz steps; a value left out keeps its present value; equal targets run 0 steps. Each step's six rotation values
are made a rotation by Gram-Schmidt (a step whose values make none keeps the previous rotation; a TARGET that makes
none is refused) and sent as one absolute OSC_POSE step. The rotation values are the absolute first two columns of
the grip site's world rotation (the paper's printed call fixes this reading; PAPER-PROTOCOL.md 4.6). Not
convergence checked. Gripper rule (the paper leaves it open): default "command", every step of a call carries the
call's gripper command; declared variant "ramp" (RCPX_NATIVE_VARIANT=gripper=ramp), the command is interpolated
like the pose, so only the steps past 0.5 close. The step count is the same under both.

Every move_to result repeats the 12 state values at the top level (at, rot, gripper, gripper_qpos): the harness
shortens older results to their small top-level values, and the paper keeps the whole text history.

Pictures. The paper prints the two pictures of the first decision step of every task (its Figure 5,
figures/agentview/task_NN_init00.png and figures/wrist/task_NN_init00.png). Against the env host's pictures of the
same ten cells (true views: text in the scene reads the right way round) the paper's agentview is the left-right
mirror image and the paper's wrist picture is the top-bottom flip (mean absolute pixel difference 2 to 6 of 255
after that turn, 17 to 106 without it). So the surface native serves the pictures as published ("published": each
agentview mirrored left to right, each wrist picture flipped top to bottom, same file names); the declared variant
"images=true" (RCPX_NATIVE_VARIANT, beside gripper=..., comma separated) serves the env host's own pictures, and
native_sense (and native_calib over it), whose pictures are the rcpx arm's, always does. Not reproduced: the paper's
living room scenes show no ketchup, milk or orange juice; this lane's LIBERO has them (PAPER-PROTOCOL.md section 6).

Endings: the env host halts the episode at the first success and at the step budget ("over" for both; step budget
+ 1 never runs); done and give_up end it as a claim; a lost env host as "env_lost"; and the answer of the last
charged call of the call budget ends it with a give_up recorded as the framework's (the paper's Algorithm 1,
line 20). A model's own ending, an env loss or the host's halt on that call come first.

Variant native_sense (declared, not in the paper): the same system prompt and tools plus observation.sense (joint
positions, velocities and motor torques, the wrist force and torque tared at the start), 512 px pictures, and
robot://practices holding one paragraph that says so (bench/ext/libero/native_calib.py extends that resource).

env is an envproto.Client on the env host's PUBLIC socket after reset. Only the keys this file names are read from
its answers; nothing else reaches an answer or the journal. Any env host failure ends the episode ("env_lost");
its text goes to stderr (the runner sends it to the scoring folder), never to the operator.
"""
import json
import math
import os
import re
import sys
import traceback

from rcpx import server
from bench.ext.libero import cells

CONTROL_HZ = 20
DIMS = ("x", "y", "z", "r00", "r10", "r20", "r01", "r11", "r21", "gripper")
ROT = DIMS[3:9]
STEP_CAP = (0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.1)
CAMERAS = ("agentview", "wrist")
SURFACES = ("native", "native_sense")
EPS = 1e-9
DEGENERATE = 1e-6

PAPER = "arXiv:2610.30051v1"
ATTRIBUTION = ('System prompt and tool descriptions: May Espinola, Qifan Zhang and Yu Xiang, "From Reasoning to '
               'Action: Benchmarking Frontier AI Models on LIBERO-10 Robot Manipulation Tasks", arXiv:2610.30051v1, '
               'licence CC BY 4.0; Figures 2 and 3, verbatim.')
PUBLISHED_EMBODIMENT = "libero_10_task00"
SYSTEM_PROMPT = """You are controlling a real robot embodiment named 'libero_10_task00' through tool calls. Each observation message gives you the current proprioceptive state and camera images. Work toward the user's goal in small, deliberate motions; re-check the observation after every motion. Every move tool call must include a `note`: in one or two sentences, say what you observe in the current observation and why you chose this motion. The user is watching these notes to see what you see and what you decide, so write them for a human reader. Safety approvers clamp out-of-bounds and too-fast actions below you. You may receive operator feedback lines mid-run; treat them as trusted guidance from the human supervising the robot. Respond with exactly one tool call per turn. When the goal is achieved call done; if it cannot be achieved call give_up. Note what you are learning about this rig and task as you go: done and give_up will ask what you wish you had known from the start. You have a budget of 80 LLM calls for the whole trial.

Embodiment notes:
Single Panda arm. move_to uses world-frame x,y,z in meters and rot6d orientation (first two rotation-matrix columns). z points up. Gripper 0=open, 1=close. The gripper is open/close only: any value above 0.5 keeps closing and any value below 0.5 keeps opening, so intermediate values do not give a partial opening. The gripper entry of eef_state is the current gripper command, not the finger position: it reads 1 while closed on an object even though the fingers are stopped apart. Measured finger positions are in gripper_qpos (meters, about +/-0.04 each when fully open, near 0 when closed on nothing); fingers stopped clearly apart after closing indicate something is between them. Omitting gripper from a move_to call keeps the current command. Keep orientation unchanged unless necessary. Images are upright agentview and wrist camera views. Only robot proprioception and RGB are available; no object coordinates. Intermediate target arrival must be checked from the next observation. Verify the object actually followed the lift in the next observation before assuming the grasp is secure.

General manipulation guidance:
1. Avoid collisions: before a lateral move that could intersect nearby objects or furniture, lift to a visibly safe clearance. Do not sweep through or push objects unless the task requires it. Use large moves in free space and small moves only in the last few centimeters near an object.
2. Identify the target object carefully; similar-looking distractors may be present. Cross-check both camera views.
3. A closed gripper does not mean a successful grasp. After closing, lift 3-5 cm and confirm in both cameras that the object rose with the gripper.
4. Handle objects the way a person would: grasp a stable, graspable part without pushing or tipping the object, keep it in the same orientation (usually upright) from grasp to placement, and lower it close to the support surface before releasing.
5. If the same approach fails twice, change it (grasp point, approach direction, or a small adjustment); do not repeat the same motion.
6. After each move, confirm from the next observation that it succeeded before starting the next step; do not assume success.
7. Prefer fewer, purposeful moves over many tiny corrections; your call budget is limited.
8. Declare success only when the latest observation clearly shows that the requested final state has been achieved."""

MOVE_TO_TEXT = "Move to absolute Cartesian end-effector targets (meters for positions, radians for rotations, per the dimension labels). The motion is a straight line interpolated at a fixed safe speed and the result reports its step count. Unnamed dimensions hold their current value. Coordinates are absolute in the embodiment's declared frame; on multi-arm embodiments each arm uses its own base frame and axes may differ between arms depending on mounting. Rotation dimensions are absolute targets measured relative to the trial's start orientation (0 means the start orientation) and interpolate linearly without wrapping, so prefer intermediate values for large rotations. Per-dimension bounds: x: [-0.6, 0.6], y: [-0.6, 0.6], z: [0.42, 1.6], r00: [-1, 1], r10: [-1, 1], r20: [-1, 1], r01: [-1, 1], r11: [-1, 1], r21: [-1, 1], gripper: [0, 1]."
TARGETS_TEXT = 'Map of dimension name to value. Valid names: x, y, z, r00, r10, r20, r01, r11, r21, gripper'
NOTE_TEXT = 'What you observe right now in the observation (images, if any, and state), and why you chose this motion. The user reads these notes live and in the saved transcript to follow what you see and what you decide. Write for them, in one or two plain sentences.'
DONE_TEXT = 'Declare the task finished. The trial ends; a scorer judges success.'
GIVE_UP_TEXT = 'Stop trying; the task cannot be completed. The trial ends.'
HINDSIGHT_TEXT = "What do you know now that you wish you had known at the start of this episode? Concrete, transferable facts about this rig, task, or embodiment (camera mounting and extrinsics, table and base geometry, gripper axis and offsets, controller behavior, metric scale), written as advice to a future agent attempting the same task. Say 'none' if nothing qualifies."
PUBLISHED_BOUNDS_TEXT = "x: [-0.6, 0.6], y: [-0.6, 0.6], z: [0.42, 1.6]"
VARIANTS = {"gripper": ("command", "ramp")}
IMAGES = ("published", "true")   # how the pictures are turned; the default is the surface's (see pictures_of)
PUBLISHED_TURN = {"agentview": "left_right", "wrist": "top_bottom"}   # the paper's Figure 5 against the true view

CLAIM_TEXT = ("End the episode with a statement: the same as done when success is true and give_up when it is "
              "false; nothing runs after it.")


# ---------------------------------------------------------------------------------------------------------------
# Pure geometry (no numpy: hand-checkable)


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def mat_from_rot6d(r6):
    """The 3x3 rotation (rows) whose first two columns are r6 = [r00, r10, r20, r01, r11, r21] made orthonormal
    (Gram-Schmidt: the first column normalised, the second made perpendicular to it), or None when they make none."""
    a, b = list(r6[0:3]), list(r6[3:6])
    na = _norm(a)
    if na < DEGENERATE:
        return None
    a = [x / na for x in a]
    d = sum(x * y for x, y in zip(a, b))
    b = [y - d * x for x, y in zip(a, b)]
    nb = _norm(b)
    if nb < DEGENERATE:
        return None
    b = [y / nb for y in b]
    c = _cross(a, b)
    return [[a[0], b[0], c[0]], [a[1], b[1], c[1]], [a[2], b[2], c[2]]]


def rot6d_of_mat(m):
    return [m[0][0], m[1][0], m[2][0], m[0][1], m[1][1], m[2][1]]


def quat_of_mat(m):
    """(w, x, y, z) with w >= 0 (Shepperd's method)."""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)
    n = _norm(q)
    q = [v / n for v in q]
    return q if q[0] >= 0 else [-v for v in q]


def axis_angle_of_mat(m):
    """The rotation as an axis-angle vector (rad), angle in [0, pi]."""
    w, x, y, z = quat_of_mat(m)
    w = min(1.0, w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-12:
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.acos(w)
    return [x / s * angle, y / s * angle, z / s * angle]


def steps_needed(start, target):
    """Control steps of the straight line from start to target (10-D) at the step caps."""
    return max(0, max(math.ceil(abs(t - s) / c - EPS) for s, t, c in zip(start, target, STEP_CAP)))


def parse_variant(text):
    """Defaults or key=value[,key=value]; errors name the known keys and values. Dicts are checked alike."""
    known = ", ".join(f"{k}={'|'.join(v)}" for k, v in VARIANTS.items())
    out = {k: v[0] for k, v in VARIANTS.items()}
    if text is None or isinstance(text, str) and not text.strip():
        return out
    if isinstance(text, dict):
        items = list(text.items())
    elif isinstance(text, str):
        items = []
        for item in text.split(","):
            if item.count("=") != 1:
                raise ValueError(f"invalid native variant; known: {known}")
            items.append(tuple(v.strip() for v in item.split("=")))
    else:
        raise ValueError(f"invalid native variant; known: {known}")
    seen = set()
    for key, value in items:
        if key not in VARIANTS or key in seen or value not in VARIANTS[key]:
            raise ValueError(f"invalid native variant; known: {known}")
        seen.add(key)
        out[key] = value
    return out


def pictures_of(text, surface_name="native"):
    """(the variant text or dict without its images item, "published" or "true"). The item images=published|true
    may stand anywhere in the comma list; without it native serves the published pictures and native_sense the
    env host's own. A dict is read alike. Errors name the known values."""
    default = "true" if surface_name == "native_sense" else "published"
    known = "images=" + "|".join(IMAGES)
    if isinstance(text, dict):
        rest = dict(text)
        value = rest.pop("images", default)
        if value not in IMAGES:
            raise ValueError(f"invalid native variant; known: {known}")
        return rest, value
    if not isinstance(text, str):
        return text, default
    rest, value = [], None
    for item in text.split(","):
        if item.strip().startswith("images") and item.count("=") == 1 and item.split("=")[0].strip() == "images":
            if value is not None or item.split("=")[1].strip() not in IMAGES:
                raise ValueError(f"invalid native variant; known: {known}")
            value = item.split("=")[1].strip()
        else:
            rest.append(item)
    return ",".join(rest), value or default


def turn_published(path, camera):
    """Rewrite the PNG at path as the paper shows that camera: agentview mirrored left to right, wrist flipped top
    to bottom (PUBLISHED_TURN)."""
    from PIL import Image
    how = {"left_right": Image.Transpose.FLIP_LEFT_RIGHT, "top_bottom": Image.Transpose.FLIP_TOP_BOTTOM}
    with Image.open(path) as im:
        out = im.convert("RGB").transpose(how[PUBLISHED_TURN[camera]])
    tmp = path + ".tmp.png"
    out.save(tmp, format="PNG")
    os.replace(tmp, path)


def waypoints(start, target, gripper="command"):
    """The existing 10-D line, with the target gripper command on each step unless ramp is selected."""
    if gripper not in VARIANTS["gripper"]:
        raise ValueError(f"invalid gripper variant; known: {VARIANTS['gripper']}")
    n = steps_needed(start, target)
    points = [list(target) if k == n else [s + (t - s) * k / n for s, t in zip(start, target)]
              for k in range(1, n + 1)]
    if gripper == "command":
        for wp in points:
            wp[9] = target[9]
    return points


def action_of(wp, last_mat):
    """(the absolute OSC_POSE step [x, y, z, ax, ay, az, g] of one waypoint, the rotation it used)."""
    m = mat_from_rot6d(wp[3:9]) or last_mat
    return [float(wp[0]), float(wp[1]), float(wp[2])] + axis_angle_of_mat(m) + [2.0 * float(wp[9]) - 1.0], m


def bounds_of(workspace):
    """{x, y, z: (lo, hi)} from a cell's workspace ({"x": [lo, hi], ...}); ValueError when it is not one."""
    out = {}
    for dim in ("x", "y", "z"):
        v = (workspace or {}).get(dim) if isinstance(workspace, dict) else None
        if not (isinstance(v, (list, tuple)) and len(v) == 2 and all(isinstance(x, (int, float)) for x in v)
                and v[0] < v[1]):
            raise ValueError(f"the cell's workspace has no {dim} range")
        out[dim] = (float(v[0]), float(v[1]))
    return out


def _bound(bounds, dim):
    if dim in bounds:
        return bounds[dim]
    return (0.0, 1.0) if dim == "gripper" else (-1.0, 1.0)


# ---------------------------------------------------------------------------------------------------------------
# The surface table


def _fmt(v):
    return f"{v:g}"


def system_prompt(embodiment):
    """Figure 2 with the embodiment name filled in (the paper: only the embodiment name differs across tasks)."""
    return SYSTEM_PROMPT.replace("'" + PUBLISHED_EMBODIMENT + "'", "'" + embodiment + "'")


def embodiment_name(public):
    """Explicit plain name, else base suite and task index (inferred from the published task sentences)."""
    explicit = public.get("embodiment")
    suite = str(public.get("base_suite") or "libero_10")
    task = public.get("task")
    if not (isinstance(task, int) and not isinstance(task, bool) and task >= 0):
        language = str(public.get("language") or "").strip()
        task = next((i for i, cell in enumerate(cells.LIBERO_10) if cell[1] == language), None) \
            if suite == "libero_10" else None
    name = explicit.strip() if isinstance(explicit, str) and explicit.strip() else (
        f"{suite}_task{task:02d}" if task is not None else suite)
    if re.fullmatch(r"[A-Za-z0-9_]+", name) is None:
        raise ValueError(f"the embodiment name {name!r} is not a plain name")
    return name


def move_to_text(bounds):
    """Figure 3's description, filling in only the cell's three position bounds."""
    (x0, x1), (y0, y1), (z0, z1) = bounds["x"], bounds["y"], bounds["z"]
    return MOVE_TO_TEXT.replace(PUBLISHED_BOUNDS_TEXT, f"x: [{_fmt(x0)}, {_fmt(x1)}], "
                               f"y: [{_fmt(y0)}, {_fmt(y1)}], z: [{_fmt(z0)}, {_fmt(z1)}]")


def surface(bounds, sense=False, px=256):
    """The rcpx surface table (rcpx/surfaces.py shape) of the native tools for one cell's workspace."""
    targets = {"type": "object", "additionalProperties": False, "description": TARGETS_TEXT,
               "properties": {dim: {"type": "number"} for dim in DIMS}}
    z_ex = round(bounds["z"][0] + 0.25, 2)
    tools = [
        {"name": "move_to", "description": move_to_text(bounds),
         "brief": "Move the hand in a straight line to an absolute target (any of x, y, z, r00..r21, gripper).",
         "args": {"type": "object", "additionalProperties": False, "required": ["targets", "note"],
                  "properties": {"targets": targets,
                                 "note": {"type": "string", "description": NOTE_TEXT}}},
         "example": {"args": {"targets": {"z": z_ex}, "note": "The hand is above the object; I lower it toward it."},
                     "means": f"move the hand straight to z {z_ex} m, keeping x, y, the rotation and the gripper"}},
        {"name": "done", "description": DONE_TEXT,
         "brief": "End the episode: the task is finished.",
         "args": {"type": "object", "additionalProperties": False, "required": ["summary", "hindsight"],
                  "properties": {"summary": {"type": "string"},
                                 "hindsight": {"type": "string", "description": HINDSIGHT_TEXT}}},
         "example": {"args": {"summary": "The object is in the box in both pictures.",
                              "hindsight": "The box was closer than it looked."},
                     "means": "end the episode, the task finished"}},
        {"name": "give_up", "description": GIVE_UP_TEXT,
         "brief": "End the episode: the task cannot be finished.",
         "args": {"type": "object", "additionalProperties": False, "required": ["reason", "hindsight"],
                  "properties": {"reason": {"type": "string"},
                                 "hindsight": {"type": "string", "description": HINDSIGHT_TEXT}}},
         "example": {"args": {"reason": "The object slipped out of reach.", "hindsight": "Grasp it lower."},
                     "means": "end the episode, the task not finished"}},
    ]
    resources = {"robot://task": "The task sentence and the first message of the published protocol: its system "
                                 "text, the first observation and its two pictures."}
    if sense:
        resources["robot://practices"] = "What this variant adds to the published protocol."
    return {"name": "native_sense" if sense else "native", "about": "", "tools": tools, "resources": resources}


# ---------------------------------------------------------------------------------------------------------------
# The executor


class EnvLost(RuntimeError):
    pass


def _floats(v, n):
    if isinstance(v, (list, tuple)) and v and isinstance(v[0], (list, tuple)):
        v = [x for row in v for x in row]
    if not isinstance(v, (list, tuple)) or len(v) != n:
        raise ValueError(f"expected {n} numbers")
    return [float(x) for x in v]


def observation_text(task, state, sense=None):
    """Task sentence, then the state under the published prompt's names, without a trailing newline."""
    lines = [task, "eef_state: " + json.dumps({k: state[k] for k in DIMS}),
             "gripper_qpos: " + json.dumps([state["finger1"], state["finger2"]])]
    if sense is not None:
        lines.append("sense: " + json.dumps(sense))
    return "\n".join(lines)


def sense_notes(px):
    """native_sense's robot://practices: one declared paragraph on what the variant adds to the published protocol."""
    return ("Added for this variant (not part of the published protocol). Each observation also holds sense: "
            "joint_pos_rad, joint_vel_rad_s and joint_torque_nm (the seven arm joints from the base to the wrist; "
            "joint_torque_nm is the torque each joint motor applies), wrist_force_n and wrist_torque_nm (the force "
            "and torque sensor at the wrist, in its own frame, zeroed at the start of the episode with the hand in "
            f"free air). The pictures are {px} x {px} px.")


class NativeExecutor:
    """One method per native tool (keyword arguments as in the surface table) over the env host's public socket."""

    def __init__(self, env, bounds, task, budget_steps=900, sense=False, px=256, frames_dir=None, gripper="command",
                 images="true"):
        self.env, self.bounds, self.task = env, dict(bounds), str(task)
        self.budget_steps, self.sense, self.px = int(budget_steps), bool(sense), int(px)
        self.gripper_mode = parse_variant({"gripper": gripper})["gripper"]
        if images not in IMAGES:
            raise ValueError(f"invalid native variant; known: images={'|'.join(IMAGES)}")
        self.images = images
        self.frames_dir = frames_dir
        self.steps_used, self.grip_cmd, self.n_obs = 0, 0.0, 0
        self.over, self.claimed, self.lost = False, None, None
        self._state = None

    def sim_time(self):
        return self.steps_used / float(CONTROL_HZ)

    def safe(self, fn):
        """fn() with every env host failure turned into EnvLost (and remembered: the session then ends)."""
        if self.lost is not None:
            raise EnvLost(self.lost)
        try:
            return fn()
        except EnvLost:
            raise
        except Exception as exc:   # envproto.EnvHostError, a socket error or an answer of the wrong shape
            self.lost = f"the env host failed ({getattr(exc, 'kind', type(exc).__name__)})"
            print(f"native: env host failure:\n{traceback.format_exc()}", file=sys.stderr, flush=True)
            raise EnvLost(self.lost) from None

    def take_state(self, st, step=None, steps_left=None):
        """Read the whitelisted keys of an env host state (op read, or the state of a servo answer)."""
        out = {"eef_pos": _floats(st["eef_pos"], 3), "eef_mat": _floats(st["eef_mat"], 9),
               "fingers": _floats(st["fingers"], 2)}
        if self.sense:
            for key, n in (("q", 7), ("qd", 7), ("tau", 7), ("wrist_force", 3), ("wrist_torque", 3)):
                out[key] = _floats(st[key], n)
        step = st.get("step") if step is None else step
        steps_left = st.get("steps_left") if steps_left is None else steps_left
        if isinstance(step, int) and not isinstance(step, bool):
            self.steps_used = step
        if (isinstance(steps_left, int) and steps_left <= 0) or self.steps_used >= self.budget_steps:
            self.over = True
        self._state = out
        return out

    def present(self):
        """The present 10-D point: the measured hand pose and the last gripper command."""
        st = self._state or self.take_state(self.env.call("read"))
        m = st["eef_mat"]
        return st["eef_pos"] + [m[0], m[3], m[6], m[1], m[4], m[7]] + [self.grip_cmd]

    def observation(self, stem=None):
        """{agentview, wrist (PNG paths in EPDIR/frames), state, text[, sense]}."""
        st = self._state or self.take_state(self.env.call("read"))
        self.n_obs += 1
        stem = stem or f"obs_{self.n_obs:04d}"
        os.makedirs(self.frames_dir, exist_ok=True)
        obs = {}
        for cam in CAMERAS:
            path = os.path.join(self.frames_dir, f"{stem}_{cam}.png")
            ans = self.env.call("render", camera=cam, width=self.px, height=self.px, path=path)
            if ans.get("width") != self.px or ans.get("height") != self.px or not os.path.isfile(path):
                raise ValueError(f"no {self.px} px {cam} picture")
            if self.images == "published":
                turn_published(path, cam)
            obs[cam] = path
        values = self.present() + st["fingers"]
        obs["state"] = {k: round(float(v), 4) + 0.0 for k, v in zip(DIMS + ("finger1", "finger2"), values)}
        if self.sense:
            obs["sense"] = {"joint_pos_rad": [round(float(v), 4) + 0.0 for v in st["q"]],
                            "joint_vel_rad_s": [round(float(v), 4) + 0.0 for v in st["qd"]],
                            "joint_torque_nm": [round(float(v), 3) + 0.0 for v in st["tau"]],
                            "wrist_force_n": [round(float(v), 3) + 0.0 for v in st["wrist_force"]],
                            "wrist_torque_nm": [round(float(v), 4) + 0.0 for v in st["wrist_torque"]]}
        obs["text"] = observation_text(self.task, obs["state"], obs.get("sense"))
        return obs

    # -- tools -----------------------------------------------------------------------------------------------------
    def move_to(self, targets=None, note=None):
        targets = dict(targets or {})
        if not isinstance(note, str) or not note.strip():
            return server.refusal("args", "note must be one or two sentences, got an empty text",
                                  "say what you see and why you make this move", field="note")
        return self.safe(lambda: self._move(targets))

    def _move(self, targets):
        start = self.present()
        target, clamped = list(start), {}
        for i, dim in enumerate(DIMS):
            if dim in targets:
                asked = float(targets[dim])
                lo, hi = _bound(self.bounds, dim)
                target[i] = min(hi, max(lo, asked))
                if target[i] != asked:
                    clamped[dim] = {"asked": asked, "used": target[i]}
        if mat_from_rot6d(target[3:9]) is None:
            return server.refusal("args", "the six rotation values of the target make no rotation: the first column "
                                          "is zero or the second is parallel to it",
                                  "give r00, r10, r20 and r01, r11, r21 as two directions at right angles (the "
                                  "state's values hold the present rotation)", field="targets")
        plan = waypoints(start, target, self.gripper_mode)[:max(0, self.budget_steps - self.steps_used)]
        last, ran = mat_from_rot6d(start[3:9]), 0
        for wp in plan:
            if self.over:
                break
            action, last = action_of(wp, last)
            ans = self.env.call("servo_pose", pos=action[0:3], axisangle=action[3:6], gripper=action[6], n=1)
            k = ans["steps_run"]
            if k not in (0, 1):
                raise ValueError(f"the env host ran {k!r} steps for one")
            self.take_state(ans["state"], ans.get("step"), ans.get("steps_left"))
            if k == 0:          # the host ran nothing: the episode is halted
                self.over = True
                break
            ran += 1
            self.grip_cmd = float(wp[9])
        obs = self.observation()
        obs.pop("text")
        st = obs["state"]
        out = {"task": self.task, "steps": ran, "steps_used": self.steps_used,
               "steps_left": max(0, self.budget_steps - self.steps_used),
               "at": [st[k] for k in DIMS[:3]], "rot": [st[k] for k in ROT], "gripper": st["gripper"],
               "gripper_qpos": [st["finger1"], st["finger2"]]}
        if clamped:
            out["clamped"] = clamped
        out["observation"] = obs
        return out

    def done(self, summary=None, hindsight=None):
        self.claimed = {"statement": str(summary), "success": True, "tool": "done", "hindsight": str(hindsight)}
        return {"claim": self.claimed, "message": "done is recorded; the episode has ended"}

    def give_up(self, reason=None, hindsight=None):
        self.claimed = {"statement": str(reason), "success": False, "tool": "give_up", "hindsight": str(hindsight)}
        return {"claim": self.claimed, "message": "give_up is recorded; the episode has ended"}


# ---------------------------------------------------------------------------------------------------------------
# The session


class NativeSession(server.Session):
    """rcpx.server.Session with the native endings: done and give_up end the episode as a claim (success true and
    false); the env host's halt (first success or the step budget) ends it as "over" in the answer of the call that
    caused it ("episode": "ended"); a lost env host as "env_lost"; the last charged call of the call budget ends it
    with a give_up recorded as the framework's (claim.by "framework"). The budget block also carries steps_used and
    steps_left. The listing's tools carry no worked example, so a description is the paper's text alone."""

    def __init__(self, executor, surface_table, resources, budget, **kw):
        super().__init__(executor, surface_table, resources, budget, **kw)
        self.resources.pop(server.RESULT_URI, None)     # not listed: answers are whole already
        self._templates = [t for t in self._templates if t[0] != server.RESULT_URI]

    def _list(self):
        listing = super()._list()
        listing["claim"] = dict(listing["claim"], description=CLAIM_TEXT)
        listing["about"] = ""
        for entry in listing["tools"]:
            entry.pop("example", None)
        return listing

    def _budget_block(self):
        out = super()._budget_block()
        ex = self.executor
        out["steps_used"], out["steps_left"] = ex.steps_used, max(0, ex.budget_steps - ex.steps_used)
        return out

    def _route(self, seq, request):
        response = super()._route(seq, request)
        if self.ended or self._pending_end is not None or not isinstance(response, dict) \
                or response.get("op") != "call":
            return response
        ex = self.executor
        if ex.claimed is not None:
            response["episode"] = "claimed"
            self._pending_end = ("claim", {"claim": ex.claimed})
        elif ex.lost is not None:
            response["episode"] = "ended"
            self._pending_end = ("env_lost", {"why": ex.lost})
        elif ex.over:
            response["episode"] = "ended"
            self._pending_end = ("over", {})
        elif self.calls_charged() >= int(self.budget["calls"]):
            response["episode"] = "ended"
            self._pending_end = ("claim", {"claim": {"statement": f"the budget of {int(self.budget['calls'])} LLM calls "
                                                               "is used up", "success": False, "tool": "give_up",
                                                  "by": "framework", "hindsight": ""}})
        return response


def build_session(env, ep_dir, public, surface_name="native", budget_requests=80, budget_steps=900, variant=None):
    """A reset host's native session, with its published first message and declared variant in robot://task."""
    if surface_name not in SURFACES:
        raise ValueError(f"unknown native surface {surface_name!r}; known: {', '.join(SURFACES)}")
    variant, images = pictures_of(os.environ.get("RCPX_NATIVE_VARIANT", "") if variant is None else variant,
                                  surface_name)
    variant = parse_variant(variant)
    g = variant["gripper"]
    name = embodiment_name(public)
    sense = surface_name == "native_sense"
    px = 512 if sense else 256
    bounds = bounds_of(public.get("workspace"))
    task = str(public.get("language") or "").strip()
    if not task:
        raise ValueError("the cell has no task sentence")
    ex = NativeExecutor(env, bounds, task, budget_steps, sense, px,
                        os.path.join(os.path.abspath(ep_dir), server.FRAMES_DIR), gripper=g, images=images)
    start_obs = ex.safe(lambda: ex.observation("start"))
    first = {"text": start_obs.pop("text"),
             "images": [{"label": cam, "path": start_obs[cam]} for cam in CAMERAS],
             "system": system_prompt(name), "own_tools": False}
    task_doc = {"instruction": task, "embodiment": name, "first_message": first, "observation": start_obs,
                "protocol": {"source": ATTRIBUTION, "llm_calls": int(budget_requests),
                             "tools": ["move_to", "done", "give_up"], "images_kept": 4, "variant": variant,
                             "images": images}}
    resources = {"robot://task": lambda: task_doc}
    if sense:
        resources["robot://practices"] = lambda: sense_notes(px)
    budget = {"calls": int(budget_requests), "sim_seconds": int(budget_steps) / float(CONTROL_HZ)}
    session = NativeSession(ex, surface(bounds, sense, px), resources, budget, ep_dir=ep_dir)
    print(f"native: variant gripper={g} images={images}", file=sys.stderr, flush=True)
    session._log("variant", surface=surface_name, gripper=g, images=images)
    return session
