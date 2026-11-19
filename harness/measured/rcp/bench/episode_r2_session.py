# SPDX-License-Identifier: AGPL-3.0-only
"""The round two session: rcpx.server.Session with think time, op_id and the claim's cannot.

SessionR2 wraps Session.dispatch; rcpx/server.py is not edited.

- THINK TIME: before each "call" request (while the episode runs) the world idles think_s simulated seconds
  through idle(seconds) (the arm holds its present command; devices, light schedules and timed interventions run).
  The session's clock is the ACTIVE clock: world time minus the think time so far, so the simulated-seconds budget
  and every journal t_s count active seconds only. Every response's budget block adds world_seconds (the think time
  so far: the seconds the world ran while the operator thought), world_clock_s (all simulated seconds since the
  session began, active plus think) and think_s_per_call. Each think is journaled as a "think" record (call index,
  seconds, world and active time after it) so a replay with the same think_s reproduces it.
- OP_ID: a request may carry "op_id". A repeat of the same request with the same op_id returns the stored first
  response unchanged; nothing runs (no think time, no motion, no budget) and an "op_replay" record is journaled. The
  same op_id on a different request is refused with check "op_id" and nothing runs.
- CLAIM: with claim=CLAIM_R2 (the vnext surface) a claim may carry cannot {reason, evidence}: the operator's
  statement that the task cannot be done and why. Other surfaces keep the round one claim. claim_check(args) runs on
  a claim that carries cannot (the person package's evidence check when that package is loaded); a refusal it
  returns refuses the claim and the episode goes on.
- CLAIM GATES: claim_gates is [(package, fn(args, session) -> None | answer dict)], in package order.
  They run on every well-formed claim, with or without cannot, BEFORE the cannot check and before anything is
  recorded. The first answer is returned as {seq, op "claim", episode "continues", result: answer, budget}: the
  claim is NOT recorded, no call is charged and the episode goes on; the journal gets a claim_answer record {req:
  the response's seq, check, hard, package}. When every gate returns None the claim is recorded as before, and
  claim_fields() (the gates' own record fields, {"gate_overridden": bool}) is merged into the claim's result and
  the end record. A gate that raises is journaled as an error and counts as None (the claim is recorded as before).
"""

import copy
import json
import traceback

from rcpx import server, surfaces
from rcpx.context import refusal

CLAIM_R2 = {
    "description": "End the episode with your own statement of what you achieved; nothing else runs after it. If "
                   "the task cannot be done here, claim success false and say why in cannot (reason and the "
                   "evidence you saw).",
    "args": surfaces._obj({
        "statement": surfaces._str("what you did and what you saw, in plain words"),
        "success": surfaces._bool("true if you believe the task is done"),
        "cannot": surfaces._obj({"reason": surfaces._str("why the task cannot be done here"),
                                 "evidence": {"anyOf": [{"type": "array", "items": {"anyOf": [
                                                  {"type": "string"}, {"type": "integer", "minimum": 0}]}},
                                                        {"type": "string"}, {"type": "integer", "minimum": 0}],
                                              "description": "the calls that show it: each response's seq (or a "
                                                             "look id such as L3), as a list"}},
                                ["reason"]),
    }, ["statement", "success"]),
    "example": {"args": {"statement": "pressed the lever four times; the lamp stayed dark and nothing moved",
                         "success": False,
                         "cannot": {"reason": "the machine does not respond: it seems to have no power",
                                    "evidence": [14, 17, 21]}},
                "means": "end the episode saying the task cannot be done, citing the calls that show it"},
}


def _key(obj):
    return json.dumps(obj, sort_keys=True, default=str)


def idle_plain(ex, seconds):
    """Think time for a round one Executor (no hook core): the world runs `seconds` with the arm holding its present
    command, exactly as rcpx.r2.core.HookMixin.idle does it."""
    seconds = max(0.0, float(seconds))
    n = int(round(seconds / ex.dt))
    if n > 0:
        if ex._q_des is not None:
            ex._command(ex._q_des)
        ex.backend.step(n)
        ex._sense()
    return {"idled_s": round(n * ex.dt, 3)}


class SessionR2(server.Session):
    """Session plus think time, op_id and the round two claim. world_clock() is the world's simulated time; idle
    is fn(seconds) that lets the world run (None: no think time)."""

    def __init__(self, executor, surface, resources, budget=None, *, journal=None, ep_dir=None, handlers=None,
                 world_clock=None, idle=None, think_s=0.0, claim=None, claim_check=None, after=None,
                 claim_gates=None, claim_fields=None):
        self.world_clock = world_clock if world_clock is not None else server._find_clock(executor)
        self.claim_check = claim_check          # fn(claim args) -> None | refusal, for a claim that carries cannot
        self.claim_gates = list(claim_gates or [])   # [(package, fn(args, session) -> None | answer)]
        self.claim_fields = claim_fields        # fn() -> dict merged into a recorded claim's result and end record
        self.claim_answers = 0                  # claims a gate answered instead of recording
        self.think_s = max(0.0, float(think_s or 0.0))
        self.think_total = 0.0
        self.thinks = 0
        self._idle = idle
        self._ops = {}
        self._after = after
        self.claim_spec = claim or surfaces.CLAIM
        self.world_t0 = float(self.world_clock())
        super().__init__(executor, surface, resources, budget, journal=journal, ep_dir=ep_dir, clock=self._active,
                         handlers=handlers)
        self._log("session_r2", think_s=self.think_s, op_id=True,
                  claim_args=sorted(self.claim_spec["args"].get("properties", {})))

    # -- clocks ----------------------------------------------------------------------------------------------------
    def _active(self):
        return float(self.world_clock()) - self.think_total

    def _end(self, reason, **info):
        info.setdefault("world_seconds", round(self.think_total, 3))
        info.setdefault("world_clock_s", round(float(self.world_clock()) - self.world_t0, 3))
        super()._end(reason, **info)

    def budget_keys(self):
        """What the round two keys of every answer's budget block mean (the list answer carries it)."""
        cap = (self.budget or {}).get("sim_seconds")
        return (f"Clocks: the {surfaces._fmt(float(cap)) + ' s ' if cap is not None else ''}time budget counts "
                "sim_seconds_used: simulated seconds of your calls' own work; world_clock_s is all simulated time "
                "since the start, world_seconds the part of it the world ran between your calls (think time, "
                "think_s_per_call before each call); time spent waiting on the world is not charged to "
                "sim_seconds_used.")

    def remaining(self):
        out = super().remaining()
        out["world_seconds"] = round(self.think_total, 3)
        out["world_clock_s"] = round(float(self.world_clock()) - self.world_t0, 3)
        out["think_s_per_call"] = self.think_s
        return out

    # -- dispatch --------------------------------------------------------------------------------------------------
    def dispatch(self, request):
        op_id = request.get("op_id") if isinstance(request, dict) else None
        key = None
        if op_id is not None:
            oid = _key(op_id)
            key = _key({k: v for k, v in request.items() if k != "op_id"})
            if oid in self._ops:
                first_key, first_seq, stored = self._ops[oid]
                if first_key == key:
                    self._log("op_replay", op_id=op_id, first_req=first_seq, request=request)
                    return copy.deepcopy(stored)
                return self._op_id_clash(request, op_id, first_seq)
        if isinstance(request, dict) and request.get("op") == "call" and not self.ended:
            self._think()
        response = super().dispatch(request)
        if op_id is not None:
            self._ops[_key(op_id)] = (key, response.get("seq"), copy.deepcopy(response))
        if self._after is not None:
            try:
                self._after()
            except Exception:
                self._log("error", where="after request", trace=traceback.format_exc())
        return response

    def _op_id_clash(self, request, op_id, first_seq):
        self.seq += 1
        seq = self.seq
        self._log("request", req=seq, t_s=self._now(), request=request)
        self.refusals += 1
        response = {"seq": seq, "op": request.get("op"),
                    "result": refusal("op_id", f"op_id {json.dumps(op_id)} was already used by request {first_seq} "
                                      "with different contents; nothing ran",
                                      "send a new op_id with a new request; repeat an op_id only to resend the "
                                      "same request"),
                    "budget": self.remaining()}
        self._log("response", req=seq, t_s=self._now(), response=response, refused="op_id")
        return response

    def _think(self):
        if self.think_s <= 0 or self._idle is None:
            return
        w0 = float(self.world_clock())
        try:
            self._idle(self.think_s)
        except Exception:
            self._log("error", where="think time", trace=traceback.format_exc())
        w1 = float(self.world_clock())
        dt = max(0.0, w1 - w0)
        self.think_total += dt
        self.thinks += 1
        self._log("think", call=self.calls_used + 1, seconds=round(dt, 6), asked_s=self.think_s,
                  world_t_s=round(w1 - self.world_t0, 6), t_s=self._now(), total_s=round(self.think_total, 6))

    # -- the claim -------------------------------------------------------------------------------------------------
    def _list(self):
        out = super()._list()
        out["about"] = (str(out.get("about") or "").rstrip() + " " + self.budget_keys()).strip()  # the compact keeps it
        c = self.claim_spec
        out["claim"] = {"description": c["description"], "args": surfaces.public_schema(c["args"]),
                        "example": {"args": c["example"]["args"], "means": c["example"]["means"],
                                    "cli": f"claim '{json.dumps(c['example']['args'])}'"}}
        return out

    def _claim(self, seq, request):
        entry = {"name": "claim", "args": self.claim_spec["args"], "example": self.claim_spec["example"]}
        args, bad = self._args(request, entry)
        if bad is None:
            problem = surfaces.check_args(entry, args)
            bad = None if problem is None else {"refused": problem}
        if bad is None:
            answered = self._claim_gate(seq, args)
            if answered is not None:
                return answered
        if bad is None and self.claim_check is not None and isinstance(args, dict) and "cannot" in args:
            try:
                bad = self.claim_check(args)
            except Exception:
                self._log("error", where="claim check", trace=traceback.format_exc())
                bad = None
        if bad is not None:
            return self._envelope(seq, "claim", None, bad)
        extra = self._claim_record_fields()
        self._pending_end = ("claim", {"claim": args, **extra})
        return {"seq": seq, "op": "claim", "episode": "claimed",
                "result": {"claim": args, "message": "claim recorded; the episode has ended", **extra},
                "budget": self.remaining()}

    def _claim_gate(self, seq, args):
        """The first loaded gate's answer to this claim as the response (the claim is not recorded, nothing is
        charged, the episode goes on), or None when every gate lets the claim be recorded."""
        for package, gate in self.claim_gates:
            try:
                answer = gate(args, self)
            except Exception:
                self._log("error", where=f"claim gate {package}", req=seq, trace=traceback.format_exc())
                continue
            if isinstance(answer, dict):
                answer = server.jsonable(answer)
                self.claim_answers += 1
                self._log("claim_answer", req=seq, check=answer.get("check"), hard=bool(answer.get("hard")),
                          package=package)
                return {"seq": seq, "op": "claim", "episode": "continues", "result": answer,
                        "budget": self.remaining()}
        return None

    def _claim_record_fields(self):
        if self.claim_fields is None:
            return {}
        try:
            out = self.claim_fields()
        except Exception:
            self._log("error", where="claim fields", trace=traceback.format_exc())
            return {}
        return {str(k): v for k, v in server.jsonable(out).items()} if isinstance(out, dict) else {}
