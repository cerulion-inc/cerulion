# SPDX-License-Identifier: AGPL-3.0-only
"""The sim server's settings file, and the check that what it names is installed.

settings.json beside this file is the default; --settings names another file of the same keys. A relative path
in it is read against the harness folder (the parent of this folder), wherever the file itself is, and the code
and benchmark folders are resolved to their real paths: the simulator loads the benchmark by its real path, so
a folder reached through a link is named by where it really is.
"""
import json
import math
import os
import sys

from .common import DAY_S

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.dirname(HERE)
DEFAULT_SETTINGS = os.path.join(HERE, "settings.json")
DEVELOPMENT_STATES = os.path.join(HERE, "development_states.txt")

REQUIRED = ("snapshot_root", "bench_root", "body_records", "data_dir", "work_root", "states", "default_state")
OPTIONAL = ("_about", "product_name", "source_url", "python", "body_record_module", "gl", "macos_envhost", "env",
            "idle_reap", "idle_s", "pool_size", "lazy_records", "budget_requests", "budget_steps", "clock_s",
            "packages", "progress_every_s", "call_timeout_s", "keep_root", "keep_full_pictures_s",
            "keep_viewer_s", "keep_last_pictures", "sweep_every_s", "keep_work", "stop_grace_s", "public_url",
            "allowed_hosts", "allowed_origins")
IDLE_REAP = ("factor", "percentile", "gap_at_percentile_s", "gaps_counted", "source")


class SettingsError(Exception):
    """A settings file, or a choice of start state, the server does not run with. The message names the setting."""


def _number(doc, key, default, kind=float, low=0.0, allow_low=False):
    """doc[key] (or default) as kind, finite and above low (at low too when allow_low)."""
    value = doc.get(key, default)
    if value is None:
        return None
    try:
        number = kind(value)
    except (TypeError, ValueError):
        raise SettingsError(f"{key} must be a number, not {value!r}") from None
    if isinstance(value, bool) or not math.isfinite(number) or number < low or (number == low and not allow_low):
        bound = f"at least {low:g}" if allow_low else f"more than {low:g}"
        raise SettingsError(f"{key} must be a number {bound}, not {value!r}")
    return number


class Settings:
    """The settings of one server. state: the start state a connection gets when it names none (the command
    line's --state; default: the file's default_state). root: the folder relative paths are read against.
    platform, uid, env: sys.platform, this user's id and os.environ (a test passes others)."""

    def __init__(self, path=DEFAULT_SETTINGS, *, state=None, root=HARNESS, platform=sys.platform, uid=None,
                 env=None):
        try:
            with open(path, encoding="utf-8") as f:
                doc = json.load(f)
        except OSError as exc:
            raise SettingsError(f"cannot read {path}: {exc.strerror or exc}") from None
        except ValueError as exc:
            raise SettingsError(f"{path} is not JSON: {exc}") from None
        if not isinstance(doc, dict):
            raise SettingsError(f"{path} is not a JSON object")
        unknown = sorted(set(doc) - set(REQUIRED) - set(OPTIONAL))
        if unknown:
            raise SettingsError(f"not a setting: {', '.join(unknown)} (the settings are the keys of settings.json "
                                "and the ones its README section lists)")
        missing = [k for k in REQUIRED if doc.get(k) in (None, "", [], {})]
        if missing:
            raise SettingsError(f"missing {', '.join(missing)}")
        self.path, self.root, self.platform = os.path.abspath(path), root, platform
        # A path may name {uid} (this user's id) and {cerulion_home} (CERULION_HOME as it is, when it is set and
        # not empty, else ~/.cerulion: where the cerulion command keeps its own state).
        home = (os.environ if env is None else env).get("CERULION_HOME") or ""
        self.marks = {"{uid}": str(os.getuid() if uid is None else uid),
                      "{cerulion_home}": home or os.path.join(os.path.expanduser("~"), ".cerulion")}
        self.macos = platform == "darwin"

        self.snapshot_root = self._real(doc["snapshot_root"], "snapshot_root")
        self.bench_root = self._real(doc["bench_root"], "bench_root")
        self.body_records = self._real(doc["body_records"], "body_records")
        self.data_dir = self._at(doc["data_dir"], "data_dir")
        # Short on purpose: the episode's sockets live under it, and a socket path must fit 103 bytes.
        self.work_root = self._at(doc["work_root"], "work_root")
        self.python = self._at(doc["python"], "python") if doc.get("python") else sys.executable
        self.body_record_module = str(doc.get("body_record_module") or "cerulion_harness.body_record")

        # {label: {"cell": "ID BENCH SUITE TASK INIT PERTURB", "title", "note" (optional, shown beside the title)}}
        self.states = doc["states"]
        if not isinstance(self.states, dict):
            raise SettingsError("states must be an object of start states")
        for label, st in self.states.items():
            if not (isinstance(st, dict) and isinstance(st.get("cell"), str) and len(st["cell"].split()) == 6):
                raise SettingsError(f"states.{label} must state its cell as six fields: ID BENCH SUITE TASK INIT "
                                    "PERTURB")
            if not isinstance(st.get("title", ""), str) or not isinstance(st.get("note", ""), str):
                raise SettingsError(f"states.{label}: title and note are text")
        if doc["default_state"] not in self.states:
            raise SettingsError(f"default_state {doc['default_state']!r} is not one of states")
        if state is not None and state not in self.states:
            raise SettingsError(f"start state {state!r} is not offered; the start states: {', '.join(self.states)}")
        self.default_state = state if state is not None else doc["default_state"]

        # The measured runs' own episode constants (80 replies, 900 control steps, a 5,400 s clock).
        self.budget_requests = int(_number(doc, "budget_requests", 80, int))
        self.budget_steps = int(_number(doc, "budget_steps", 900, int))
        self.clock_s = _number(doc, "clock_s", 5400.0)
        self.packages = str(doc.get("packages", "all,place"))
        # The idle limit. idle_reap: the limit is factor times the stated percentile of the gaps between calls
        # (gap_at_percentile_s, read from gaps_counted gaps: source says which), until this server's own history
        # holds more gaps than that. idle_s, when stated, is the operator's own figure and wins. Neither: an idle
        # episode runs on to the clock.
        self.idle_s = _number(doc, "idle_s", None)
        reap = doc.get("idle_reap") or {}
        if not isinstance(reap, dict):
            raise SettingsError("idle_reap must be an object")
        if reap:
            lacking = [k for k in IDLE_REAP if reap.get(k) in (None, "")]
            if lacking or set(reap) - set(IDLE_REAP):
                raise SettingsError(f"idle_reap states exactly {', '.join(IDLE_REAP)} (the figure and where it was "
                                    "read)")
            _number(reap, "factor", None)
            _number(reap, "gap_at_percentile_s", None)
            _number(reap, "gaps_counted", None, int, allow_low=True)
            if _number(reap, "percentile", None) > 100:
                raise SettingsError("idle_reap.percentile is at most 100")
        self.idle_reap = dict(reap)
        pool_size = _number(doc, "pool_size", None, int)
        self.pool_size = None if pool_size is None else int(pool_size)
        # true: before an episode's simulator starts, python -m BODY_RECORD_MODULE makes sure the robot body of that
        # start state has a calibration record on this machine; one that is missing is measured then, once (one
        # to two minutes), and the episode says so. false: a start state whose body has no record refuses every
        # move.
        self.lazy_records = bool(doc.get("lazy_records", False))

        # The renderer. On macOS the simulator host starts through macos_envhost (Apple's OpenGL, offscreen): the
        # CPU renderer of the measured runs does not load there. Elsewhere it renders with gl.
        self.gl = str(doc.get("gl", "osmesa"))
        self.env = {str(k): str(v) for k, v in (doc.get("env") or {}).items()}
        self.envhost = None
        if self.macos:
            template = doc.get("macos_envhost")
            if not (isinstance(template, list) and template and all(isinstance(t, str) for t in template)):
                raise SettingsError("macos_envhost must name the macOS simulator host script and its arguments")
            self.envhost = [self._at(template[0], "macos_envhost")] + list(template[1:])
        else:
            self.env.setdefault("PYOPENGL_PLATFORM", self.gl)

        self.progress_every_s = _number(doc, "progress_every_s", 2.0)
        self.call_timeout_s = _number(doc, "call_timeout_s", 900.0)   # the bridge's own transport bound
        self.name = str(doc.get("product_name", "Cerulion"))
        self.source_url = str(doc.get("source_url") or "https://github.com/cerulion-inc/cerulion").strip()

        # What is kept of an episode for its viewer page, and for how long.
        self.keep_root = (self._at(doc["keep_root"], "keep_root") if doc.get("keep_root")
                          else os.path.join(self.data_dir, "episodes"))
        self.keep_full_pictures_s = _number(doc, "keep_full_pictures_s", DAY_S)
        self.keep_viewer_s = _number(doc, "keep_viewer_s", 30 * DAY_S)
        self.keep_last_pictures = int(_number(doc, "keep_last_pictures", 8, int, allow_low=True))
        self.sweep_every_s = _number(doc, "sweep_every_s", 600.0)
        self.keep_work = bool(doc.get("keep_work", False))   # true: leave the simulator's files (for debugging)
        self.stop_grace_s = _number(doc, "stop_grace_s", 10.0, allow_low=True)

        # HTTP only. The address links carry and the Host values /mcp answers to; unset, both follow --http ADDR.
        # A port published under another number (a container) states them.
        self.public_url = str(doc.get("public_url") or "").rstrip("/")
        self.allowed_hosts = doc.get("allowed_hosts")
        self.allowed_origins = doc.get("allowed_origins")
        for key in ("allowed_hosts", "allowed_origins"):
            value = getattr(self, key)
            if value is not None and not (isinstance(value, list) and all(isinstance(v, str) for v in value)):
                raise SettingsError(f"{key} must be a list of text")

    def _at(self, value, key):
        if not isinstance(value, str) or not value.strip():
            raise SettingsError(f"{key} must be a path")
        for mark, text in self.marks.items():
            value = value.replace(mark, text)
        return os.path.normpath(os.path.join(self.root, os.path.expanduser(value)))

    def _real(self, value, key):
        return os.path.realpath(self._at(value, key))


def missing(settings):
    """What the server needs on this machine and does not find: [(what, path)], empty when it can serve."""
    need = [("the measured code", os.path.join(settings.snapshot_root, "rcpx", "mcp_stdio.py")),
            ("the measured code", os.path.join(settings.snapshot_root, "bench", "ext", "episode_ext.py")),
            ("the simulator and its scenes", settings.bench_root),
            ("the robot body records", settings.body_records),
            ("the server's Python", settings.python)]
    if settings.envhost:
        need.append(("the macOS simulator host", settings.envhost[0]))
    return [(what, path) for what, path in need if not os.path.exists(path)]


def development_states(path=DEVELOPMENT_STATES):
    """The cell ids of development_states.txt: the measured snapshot's own development list."""
    with open(path, encoding="utf-8") as f:
        return {line.split()[0] for line in f if line.strip() and not line.startswith("#")}


def _snapshot_sealed(settings):
    if settings.snapshot_root not in sys.path:
        sys.path.insert(0, settings.snapshot_root)
    try:
        from bench.ext.libero import cells
    except ImportError as exc:
        raise SettingsError(f"the measured code at {settings.snapshot_root} has no frozen cell lists ({exc})") \
            from None
    return cells.sealed


def check_states(settings, sealed=None):
    """Every offered start state must be a development start state, one the tools were built on: its id is on
    development_states.txt, and the measured code's frozen lists do not seal it. sealed: a function of a cell id
    (default: the measured code's own); a tree without the frozen lists is checked by the file alone."""
    development = development_states()
    sealed = _snapshot_sealed(settings) if sealed is None else sealed
    for label, st in settings.states.items():
        cid = st["cell"].split()[0]
        try:
            is_sealed = bool(sealed(cid))
        except (OSError, RuntimeError):
            is_sealed = False
        if cid not in development or is_sealed:
            raise SettingsError(f"start state {label!r} ({cid}) is not a development start state, so it is not "
                                "offered: only the start states the tools were built on are served, never one of "
                                "the published results")
