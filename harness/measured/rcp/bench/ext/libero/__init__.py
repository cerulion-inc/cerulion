# SPDX-License-Identifier: AGPL-3.0-only
"""LIBERO and LIBERO-PRO bridge: where the benchmark lives on the machine, how to start an env host, the machine's EGL slots,
the host's argument checks, the safe init-state loader, the robot-only MJCF filter and the host's scoring-socket
answers (ScoringMixin, methods only). Standard library only at import (both Pythons import this file).

    from bench.ext import libero as L
    from bench.ext.libero import cells
    h = L.launch("base", sock="/path/env.sock", scoring_dir="/path/scoring", gl="egl")
    env = h.client()                      # public socket: the robot
    env.call("reset", cell=cells.by_id("l10_t00_i00"))
    env.call("act", action=[0, 0, 0, 0, 0, 0, -1], n=10)
    truth = h.scoring_client()            # scoring socket: never for an operator
    truth.call("status")
    h.stop()

Runtimes. "base" is LIBERO master in ~/bx/LIBERO (installed in ~/bx/libero-env); "pro" is LIBERO-PRO's own libero
package in ~/bx/LIBERO-PRO, put first on sys.path by the host, with its swap and task bddl and init files from the
Hugging Face dataset zhouxueyang/LIBERO-Pro copied into its tree. One host serves one runtime. LIBERO_BX moves the
whole layout (default ~/bx).

Renderers. "egl" on the GPU (about 0.4 GB of GPU memory per host) or "osmesa" on the CPU (LP_NUM_THREADS threads per
host, default 2). At most EGL_SLOTS (8, env LIBERO_EGL_SLOTS) EGL hosts run on the machine at once, whoever starts them:
an EGL host holds one of the slot locks in BX/egl_slots for its whole life (the kernel drops it when the process
ends). "egl" waits for a slot (--egl-wait-s, default 120 s) and fails when none frees; "auto" takes a free slot or
renders with osmesa.
"""

import hashlib
import io
import math
import os
import pickle
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
import zipfile

from bench.ext import envproto

BX = os.environ.get("LIBERO_BX") or os.path.expanduser("~/bx")
PYTHON = os.path.join(BX, "libero-env", "bin", "python")
BASE_ROOT = os.path.join(BX, "LIBERO")
PRO_ROOT = os.path.join(BX, "LIBERO-PRO")
CONFIG_DIRS = {"base": os.path.join(BX, "libero-config"), "pro": os.path.join(BX, "liberopro-config")}
RUNTIMES = ("base", "pro")
GLS = ("egl", "osmesa", "auto")
EGL_SLOTS = int(os.environ.get("LIBERO_EGL_SLOTS", "8"))
EGL_SLOT_DIR = os.path.join(BX, "egl_slots")
TRUTH_SOCK, TRUTH_LOG, HOST_LOG, READY = "truth.sock", "truth_log.jsonl", "host.log", "ready.json"
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
ROBOT_PREFIXES = ("robot0_", "gripper0_", "mount0_")
# The rcpx arm's joint servo (envhost servo_joints, robosuite JOINT_POSITION, inertia weighted PD with gravity
# compensation): the stiffness of the benchmark's own OSC_POSE controller (kp 150, critically damped) and a goal step
# of 0.2 rad a control step. robosuite's default (kp 50, 0.05 rad) caps a joint near 0.18 rad/s, about a sixth of
# the hand speed the plain arm's move_to reaches. bench/ext/libero/backend.py LAW reads the same numbers.
JOINT_SERVO = {"kp": 150.0, "damping_ratio": 1.0, "step_rad": 0.2}


def libero_package_root(runtime):
    """The folder the runtime's libero.libero package must come from."""
    return os.path.join(PRO_ROOT if runtime == "pro" else BASE_ROOT, "libero", "libero")


def host_env(runtime, gl="egl", lp_threads=2):
    """A clean environment for an env host: no PYTHONPATH from the caller, a fixed hash seed, one BLAS thread."""
    if runtime not in RUNTIMES or gl not in GLS:
        raise ValueError("runtime is base or pro; gl is egl, osmesa or auto")
    env = {k: os.environ[k] for k in ("PATH", "HOME", "USER", "LANG", "TMPDIR") if k in os.environ}
    gl = "egl" if gl == "auto" else gl   # the host settles auto before it loads a renderer
    env.update({"LIBERO_CONFIG_PATH": CONFIG_DIRS[runtime], "MUJOCO_GL": gl, "PYOPENGL_PLATFORM": gl,
                "PYTHONHASHSEED": "0", "PYTHONDONTWRITEBYTECODE": "1", "OMP_NUM_THREADS": "1",
                "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "LP_NUM_THREADS": str(int(lp_threads)),
                "LIBERO_BX": BX})
    return env


def host_argv(runtime, sock, scoring_dir, files_dir=None, gl="egl", idle_s=None):
    argv = [PYTHON, "-s", "-m", "bench.ext.libero.envhost", "--runtime", runtime, "--sock", os.path.abspath(sock),
            "--scoring-dir", os.path.abspath(scoring_dir), "--gl", gl]
    if files_dir is not None:
        argv += ["--files-dir", os.path.abspath(files_dir)]
    if idle_s is not None:
        argv += ["--idle-s", repr(float(idle_s))]
    return argv


class HostProcess:
    """One env host this process started. stop() asks it to close, then ends it by its own pid if it must."""

    def __init__(self, proc, sock, scoring_dir, log_path):
        self.proc, self.sock, self.scoring_dir, self.log_path = proc, sock, scoring_dir, log_path
        self.truth_sock = os.path.join(scoring_dir, TRUTH_SOCK)
        self.pid = proc.pid

    def client(self, timeout_s=600.0):
        return envproto.Client(self.sock, timeout_s)

    def scoring_client(self, timeout_s=600.0):
        return envproto.Client(self.truth_sock, timeout_s)

    def stop(self, wait_s=30.0):
        if self.proc.poll() is None:
            try:
                with self.client(timeout_s=wait_s) as c:
                    c.call("close")
            except envproto.EnvHostError:
                pass
            try:
                self.proc.wait(timeout=wait_s)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=wait_s)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait()
        return self.proc.returncode


def launch(runtime, sock, scoring_dir, files_dir=None, gl="egl", wait_s=240.0, log_path=None, idle_s=None,
           lp_threads=2):
    """Start an env host and wait until both sockets answer; the host's output goes to log_path (default
    SCORING_DIR/host.out). Raises RuntimeError (and ends the host) when it exits or is not ready in time."""
    scoring_dir = os.path.abspath(scoring_dir)
    os.makedirs(scoring_dir, exist_ok=True)
    ready = os.path.join(scoring_dir, READY)
    if os.path.exists(ready):
        os.unlink(ready)
    log_path = log_path or os.path.join(scoring_dir, "host.out")
    log = open(log_path, "ab")
    proc = subprocess.Popen(host_argv(runtime, sock, scoring_dir, files_dir, gl, idle_s), cwd=ROOT,
                            env=host_env(runtime, gl, lp_threads), stdin=subprocess.DEVNULL, stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True)
    log.close()
    host = HostProcess(proc, os.path.abspath(sock), scoring_dir, log_path)
    t0 = time.monotonic()
    while not os.path.exists(ready):
        if proc.poll() is not None:
            raise RuntimeError(f"the env host exited early (code {proc.returncode}); see {log_path}")
        if time.monotonic() - t0 > wait_s:
            host.stop(wait_s=10.0)
            raise RuntimeError(f"the env host was not ready after {wait_s:g} s and was stopped; see {log_path}")
        time.sleep(0.1)
    return host


def take_egl_slot(wait_s=0.0, slot_dir=None, slots=None):
    """Hold one of the machine's EGL slots: (k, open lock file) for the caller to keep open while it renders, or None
    when no slot came free within wait_s seconds."""
    import fcntl
    slot_dir, slots = slot_dir or EGL_SLOT_DIR, EGL_SLOTS if slots is None else slots
    os.makedirs(slot_dir, exist_ok=True)
    t0 = time.monotonic()
    while True:
        for k in range(slots):
            f = open(os.path.join(slot_dir, f"slot{k}.lock"), "a")
            try:
                fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return k, f
            except OSError:
                f.close()
        if time.monotonic() - t0 >= wait_s:
            return None
        time.sleep(0.5)


# -- request checks (the env host's argument errors) -----------------------------------------------------------------
class BadArgs(Exception):
    """A request the host refuses before anything runs (kind bad_args)."""


class NoEpisode(Exception):
    """A robot request before reset (kind no_episode)."""


class WrongRuntime(Exception):
    """A cell for the other runtime (kind runtime)."""


class Withheld(Exception):
    """A public answer that would name a scene item (kind withheld)."""


KINDS = {BadArgs: "bad_args", NoEpisode: "no_episode", WrongRuntime: "runtime"}


def floats_arg(value, n, what):
    if not isinstance(value, (list, tuple)) or len(value) != n:
        raise BadArgs(f"{what} needs {n} numbers")
    out = []
    for v in value:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)):
            raise BadArgs(f"{what} needs {n} finite numbers")
        out.append(float(v))
    return out


def count_arg(args, key="n", default=1):
    n = args.get(key, default)
    if isinstance(n, bool) or not isinstance(n, int) or not 1 <= n <= envproto.MAX_STEPS_PER_REQUEST:
        raise BadArgs(f"{key} must be an integer from 1 to {envproto.MAX_STEPS_PER_REQUEST}")
    return n


def gripper_arg(value, default):
    """The native gripper value: -1 opens, +1 closes, 0 holds (robosuite's PandaGripper uses only the sign)."""
    if value is None:
        return default
    if value == "open":
        return -1.0
    if value == "close":
        return 1.0
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not -1.0 <= float(value) <= 1.0:
        raise BadArgs('gripper is "open", "close" or a number from -1 (open) to 1 (close)')
    return float(value)


def frac_arg(args):
    f = args.get("torque_frac", 1.0)
    if isinstance(f, bool) or not isinstance(f, (int, float)) or not 0.0 < float(f) <= 1.0:
        raise BadArgs("torque_frac must be above 0 and at most 1")
    return float(f)


# -- init states -------------------------------------------------------------------------------------------------------
class _InitUnpickler(pickle.Unpickler):
    """Init state files are torch zip archives holding one numpy array. Only numpy's own array globals load; any
    other global, or a tensor storage, is refused, so a downloaded file cannot run code."""
    ALLOWED = {("numpy.core.multiarray", "_reconstruct"), ("numpy", "ndarray"), ("numpy", "dtype"),
               ("_codecs", "encode")}

    def find_class(self, module, name):
        if (module, name) in self.ALLOWED:
            return super().find_class(module, name)
        raise pickle.UnpicklingError(f"init file names {module}.{name}, which is not a numpy array part")

    def persistent_load(self, pid):
        raise pickle.UnpicklingError("init file holds a tensor storage, not a numpy array")


def load_init_states(path):
    if not zipfile.is_zipfile(path):
        raise ValueError(f"{path} is not a torch zip archive")
    with zipfile.ZipFile(path) as z:
        names = [n for n in z.namelist() if n.endswith("/data.pkl")]
        if len(names) != 1:
            raise ValueError(f"{path} holds {len(names)} data.pkl entries")
        return _InitUnpickler(io.BytesIO(z.read(names[0]))).load()


# -- the robot-only MJCF -----------------------------------------------------------------------------------------------
ROBOT_ROOT_BODY = "robot0_base"
_FILE_ATTRS = ("file", "fileright", "fileleft", "fileup", "filedown", "filefront", "fileback")


def robot_mjcf(scene_xml, out_dir):
    """From the compiled scene's MJCF text (mj_saveLastXML), write out_dir/robot.xml holding only the robot's body
    tree (arm, mount, gripper, wrist camera) at its scene pose, the actuators and sensors on it, and the meshes and
    materials it uses, copied to out_dir/assets. Nothing else of the scene (objects, fixtures, arena, scene cameras,
    lights, keyframes) is written. Returns {"path", "nlinks", "njoint", "nmesh"}."""
    root = ET.fromstring(scene_xml)
    wb = root.find("worldbody")
    robots = [b for b in wb if b.tag == "body" and b.get("name") == ROBOT_ROOT_BODY]
    if len(robots) != 1:
        raise ValueError(f"the scene has {len(robots)} {ROBOT_ROOT_BODY} bodies")
    for child in list(wb):
        if child is not robots[0]:
            wb.remove(child)
    tree = robots[0]
    kept = {tag: {e.get("name") for e in tree.iter(tag) if e.get("name")}
            for tag in ("body", "joint", "geom", "site", "camera")}
    kept["body"].add(ROBOT_ROOT_BODY)
    meshes = {g.get("mesh") for g in tree.iter("geom") if g.get("mesh")}
    materials = {g.get("material") for g in tree.iter("geom") if g.get("material")}
    for tag in ("keyframe", "size", "contact", "equality", "tendon"):
        for e in root.findall(tag):
            root.remove(e)
    asset = root.find("asset")
    textures = set()
    if asset is not None:
        for e in list(asset):
            if e.tag == "material" and e.get("name") in materials:
                if e.get("texture"):
                    textures.add(e.get("texture"))
        for e in list(asset):
            keep = (e.tag == "mesh" and e.get("name") in meshes) or \
                   (e.tag == "material" and e.get("name") in materials) or \
                   (e.tag == "texture" and e.get("name") in textures)
            if not keep:
                asset.remove(e)
    for section, ref in (("actuator", "joint"), ("sensor", None)):
        sec = root.find(section)
        if sec is None:
            continue
        for e in list(sec):
            refs = [e.get(a) for a in ("joint", "site", "body", "objname") if e.get(a)]
            if section == "actuator" and e.get(ref) not in kept["joint"]:
                sec.remove(e)
            elif section == "sensor" and not (refs and all(r in kept["joint"] | kept["site"] | kept["body"]
                                                            for r in refs)):
                sec.remove(e)
    adir = os.path.join(out_dir, "assets")
    os.makedirs(adir, exist_ok=True)
    nmesh = 0
    for e in (asset if asset is not None else []):
        for attr in _FILE_ATTRS:
            src = e.get(attr)
            if not src:
                continue
            if not os.path.isabs(src):
                raise ValueError(f"asset file {src} is not an absolute path")
            name = hashlib.sha1(src.encode()).hexdigest()[:8] + "_" + os.path.basename(src)
            shutil.copyfile(src, os.path.join(adir, name))
            e.set(attr, name)
            nmesh += e.tag == "mesh"
    comp = root.find("compiler")
    if comp is None:
        comp = ET.SubElement(root, "compiler")
        root.remove(comp)
        root.insert(0, comp)
    comp.set("meshdir", "assets")
    comp.set("texturedir", "assets")
    root.set("model", "robot")
    path = os.path.join(out_dir, "robot.xml")
    ET.ElementTree(root).write(path, encoding="unicode")
    return {"path": path, "nlinks": len(kept["body"]), "njoint": len(kept["joint"]), "nmesh": nmesh}


# -- the scoring socket's answers ----------------------------------------------------------------------------------
class ScoringMixin:
    """The env host's scoring-socket ops (ground truth). Methods only: they run inside the host (bench/ext/libero/
    envhost.py), on its numpy (self.np), mujoco (self.mj), model and data handles, so this file imports neither."""

    def _poses(self):
        d, out = self.d, {}
        for group in ("objects_dict", "fixtures_dict"):
            for name in getattr(self.inner, group):
                bid = self.inner.obj_body_id[name]
                out[name] = {"pos": d.xpos[bid], "quat_wxyz": d.xquat[bid], "kind": group[:-5]}
        return envproto.plain(out)

    def _state_hash(self):
        return hashlib.sha256(self.np.ascontiguousarray(self.env.get_sim_state(), dtype=self.np.float64).tobytes()
                              ).hexdigest()

    def sc_hello(self, args):
        return {"protocol": envproto.PROTOCOL, "pid": os.getpid(), "scoring": True, "ops": sorted(envproto.SCORING_OPS)}

    def sc_status(self, args):
        if self.env is None:
            return {"episode": None, "ep": self.ep}
        goals = self._goals()
        led = self.ledger
        return {"ep": self.ep, "cell": self.cell, "language": self.env.language_instruction, "bddl": self.bddl,
                "env_seed": self.seed, "init_states": self.init_len, "step": led.step, "budget": led.budget,
                "steps_left": led.steps_left, "first_success_step": led.first_success, "halted": led.halted,
                "goals_now": goals, "success_now": all(goals), "state_sha256": self._state_hash(),
                "truth_log": os.path.join(self.scoring_dir, TRUTH_LOG), "libero": self.libero_root,
                "runtime": self.runtime, "obs_images": self.obs_images}

    def sc_truth(self, args):
        self._need_env()
        m, d, mj = self.m, self.d, self.mj
        sites = {}
        for name in self.inner.object_sites_dict:
            sid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, name)
            if sid >= 0:
                sites[name] = {"pos": d.site_xpos[sid], "mat": d.site_xmat[sid].reshape(3, 3), "size": m.site_size[sid]}
        joints = {}
        for j in range(m.njnt):
            n = mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, j) or f"joint{j}"
            if not n.startswith(ROBOT_PREFIXES):
                a = m.jnt_qposadr[j]
                width = 7 if m.jnt_type[j] == mj.mjtJoint.mjJNT_FREE else 4 if m.jnt_type[j] == mj.mjtJoint.mjJNT_BALL \
                    else 1
                joints[n] = d.qpos[a:a + width]
        goals = self._goals()
        return envproto.plain({"step": self.ledger.step, "bodies": self._boxes(self._poses()), "regions": sites,
                        "joints": joints, "joint_info": self._joint_info(),
                        "goal_state": self.goal_state, "goals_now": goals, "success_now": all(goals),
                        "first_success_step": self.ledger.first_success, "halted": self.ledger.halted,
                        "eef_pos": d.site_xpos[self.robot.eef_site_id], "obj_of_interest": self.inner.obj_of_interest})

    def _boxes(self, poses):
        """Each body's world box over the collision geoms (contype or conaffinity set) of its subtree."""
        np, m, d = self.np, self.m, self.d
        for name, rec in poses.items():
            root = self.inner.obj_body_id[name]
            lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
            for g in range(m.ngeom):
                b = m.geom_bodyid[g]
                while b not in (0, root):
                    b = m.body_parentid[b]
                if b != root or not (m.geom_contype[g] or m.geom_conaffinity[g]):
                    continue
                c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
                r = np.array(d.geom_xmat[g]).reshape(3, 3)
                mid, half = d.geom_xpos[g] + r @ c, np.abs(r) @ h
                lo, hi = np.minimum(lo, mid - half), np.maximum(hi, mid + half)
            if np.all(np.isfinite(lo)):
                rec["aabb_lo"], rec["aabb_hi"] = lo.tolist(), hi.tolist()
        return poses

    def _joint_info(self):
        mj, m, d = self.mj, self.m, self.d
        out = {}
        for j in range(m.njnt):
            n = mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, j) or ""
            kind = {mj.mjtJoint.mjJNT_HINGE: "hinge", mj.mjtJoint.mjJNT_SLIDE: "slide"}.get(m.jnt_type[j])
            if kind is None or not n or n.startswith(ROBOT_PREFIXES):
                continue
            b = m.jnt_bodyid[j]
            out[n] = {"type": kind, "axis": d.xaxis[j], "anchor": d.xanchor[j], "range": m.jnt_range[j],
                      "limited": bool(m.jnt_limited[j]), "qpos": float(d.qpos[m.jnt_qposadr[j]]),
                      "body": mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, b) or "", "body_pos": d.xpos[b]}
        return out

    def sc_contacts(self, args):
        self._need_env()
        m, d, mj = self.m, self.d, self.mj
        f6 = self.np.zeros(6)
        out = []
        for k in range(d.ncon):
            c = d.contact[k]
            names = [mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, g) or "" for g in (c.geom1, c.geom2)]
            bodies = [mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "" for g in (c.geom1, c.geom2)]
            if not any(b.startswith(ROBOT_PREFIXES) for b in bodies):
                continue
            mj.mj_contactForce(m, d, k, f6)
            out.append({"geoms": names, "bodies": bodies, "normal_n": float(f6[0]), "dist": float(c.dist)})
        return {"step": self.ledger.step, "contacts": out}

    def sc_render(self, args):
        self._need_env()
        cam = args.get("camera", "frontview")
        if self.mj.mj_name2id(self.m, self.mj.mjtObj.mjOBJ_CAMERA, str(cam)) < 0:
            raise BadArgs("no camera of that name in the model")
        audit = os.path.join(self.scoring_dir, "audit_frames")
        os.makedirs(audit, exist_ok=True)
        return self._render(str(cam), str(cam), args, audit)
