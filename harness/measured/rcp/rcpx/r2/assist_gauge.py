# SPDX-License-Identifier: AGPL-3.0-only
"""Streaming counts from requests and results."""

import json

try:
    from . import assist_signals as S
except ImportError:
    import assist_signals as S


class Gauge:
    def __init__(self, budget_calls=None, budget_s=None, reach_mm=None,
                 signals=None, classes=None, tools=None):
        self.budget_calls = int(budget_calls) if S.number(budget_calls) else None
        self.budget_s = budget_s if S.number(budget_s) else None
        self.signals = S.SIGNALS if signals is None else signals
        self.classes = dict(S.mapping(classes))
        self.tools = tuple(t for t in tools if isinstance(t, str)) if isinstance(tools, (list, tuple)) else None
        self.same_spot_mm, self.near_mm = S.radii(reach_mm)
        self.calls = 0
        self.calls_left = self.budget_calls
        self.events, self.signs, self.notes = [], [], {}
        self.last_point = None
        self.stage, self.phase = "no_point", "before_met"
        self.no_give, self.flags = {}, {}
        self.looks, self.marks, self.located = {}, [], {}
        self.last_located_call = 0
        self.refused_calls, self.refusals = [], {}
        self.verify_ok = self.gave_call = self.act_met_call = None
        self.feel_met, self.air_events, self.confirmed_calls = [], [], []
        self.parts, self.roles, self.arms = [], {}, {}
        self._pending_flags, self._duplicates = {}, {}
        self._looks_row = self._pointing_tests = self._open_index = 0
        self._refuse_fired = self._wind_fired = self._slow_fired = False
        self._answered = None
        self._claim_pairs = set()

    def _arm(self, arm):
        if arm not in self.arms:
            self.arms[arm] = {"act_met": None, "contact": None, "gave": None,
                              "last_act": 0, "spots": [], "repeat": 0,
                              "repeat_calls": [], "repeat_fired": False,
                              "idle_fired": False, "idle_base": 0, "pending_press": None}
        return self.arms[arm]

    def note(self, kind, **data):
        self.notes[kind] = self.notes.get(kind, 0) + 1
        if kind == "claim_answered" and data.get("success") is False:
            self._answered = (self.calls, len(self.confirmed_calls))
        return []

    def _event(self, signal, data, arm=None, call=None):
        spec = S.mapping(self.signals.get(signal))
        if not spec.get("enabled"):
            return None
        at = self.calls if call is None else call
        data = S.copy(data)
        values = dict(data, call=at)
        values.setdefault("tool", "?")
        phrase = S.evidence(signal, values, self.signals)
        if arm is not None:
            phrase += " (arm {})".format(arm)
        return {"signal": signal, "skill": spec.get("skill"), "call": at,
                "arm": arm, "evidence": phrase, "data": data}

    def _raise(self, signal, data, arm=None, call=None):
        event = self._event(signal, data, arm, call)
        if event is None:
            self.note(signal)
        else:
            self.events.append(event)
        return event

    def _gave(self, state):
        # a part was felt to give: in a guarded act, or under a plain move that pushed it
        if state["gave"] is None:
            state["gave"] = self.calls
        if self.gave_call is None:
            self.gave_call = self.calls
        self.signs.append({"sign": "gave", "call": self.calls})
        self._rise("gave")

    def _rise(self, stage):
        if S.STAGES.index(stage) > S.STAGES.index(self.stage):
            self.stage = stage

    def _budget(self, response, preserve=False):
        budget = S.mapping(response.get("budget"))
        left = budget.get("calls_left")
        if S.number(left):
            self.calls_left = left
        elif not preserve:
            self.calls_left = self.budget_calls - self.calls if self.budget_calls is not None else None
        return budget

    def _meta(self, request, extra):
        supplied = extra.get("meta")
        meta = supplied if isinstance(supplied, dict) else S.mapping(request.get("meta"))
        for key in ("no_call_replies", "malformed_json"):
            value = meta.get(key)
            if S.number(value) and value > 0:
                name = "meta_" + key
                self.notes[name] = self.notes.get(name, 0) + int(value)

    def _store_look(self, result):
        look = result.get("look_id")
        pos = S.vector(S.mapping(result.get("camera_pose")).get("position_mm"))
        if isinstance(look, str) and pos is not None and look not in self.looks:
            self.looks[look] = list(pos)

    def _nearest(self, point):
        choices = []
        if S.vector(point) is not None:
            for entry in sorted(self.located):
                dist = S.distance(point, self.located[entry]["point_mm"])
                if dist is not None:
                    choices.append((dist, entry))
        return min(choices) if choices else (None, None)

    def _toward(self, args, point):
        target = args.get("target") or args.get("on")
        if isinstance(target, str):
            if target in self.located:
                return target, True
            word = S.label(target)
            choices = [(self.located[e]["call"], e) for e in sorted(self.located)
                       if self.located[e]["label"] == word]
            if choices:
                latest = max(c for c, e in choices)
                return min(e for c, e in choices if c == latest), True
        return self._nearest(point)[1], False

    def _confirmed_now(self, result, extra):
        explicit = extra.get("confirmed")
        routine = extra.get("by_robot") and extra.get("unconfirmed")
        return explicit is True or explicit is None and result.get("device_outcome") == "expected" and not routine

    def _confirm(self):
        self.signs.append({"sign": "confirmed", "call": self.calls})
        self.confirmed_calls.append(self.calls)
        self._open_index = len(self.events)
        for arm in sorted(self.arms, key=lambda a: (a is not None, str(a))):
            state = self.arms[arm]
            state.update(repeat=0, repeat_calls=[], spots=[], repeat_fired=False,
                         idle_fired=False, idle_base=self.calls, pending_press=None)
        self._rise("confirmed")

    def _point(self, tool, args, result, extra, classes, read, arm):
        px = S.pixel(args, result)
        word = px[2] if px else S.word(args, result)
        body = extra.get("on_body")
        if body in ("static", "moving"):
            self._raise("body" if body == "static" else "body_moving", {"tool": tool}, arm)
            if body == "static" and word is not None:
                self._pending_flags.setdefault(word, {})["body"] = True
        if read["refused"]:
            return
        wrong = extra.get("mislabel")
        if isinstance(wrong, dict):
            data = {k: wrong.get(k) for k in ("label", "colour", "other", "state")}
            data.update(tool=tool, look=px[0] if px else None, px=px[1] if px else None)
            self._raise("mislabel", data, arm)
            pending = S.label(wrong.get("label")) or word
            if pending is not None:
                self._pending_flags.setdefault(pending, {})["mislabel"] = True
        if "point" not in classes:
            return
        if px:
            look, coords, word = px
            self.last_point = {"look": look, "px": list(coords), "label": word,
                               "call": self.calls, "tool": tool, "is_part": word in self.parts}
        self._pointing_tests += 1
        if px is None or "locate" in classes or not word:
            return
        look, (u, v), word = px
        for old_look, old_u, old_v, old_word, old_call in self.marks:
            if old_word != word or old_look == look or abs(u - old_u) > S.SAMEPX_PX or abs(v - old_v) > S.SAMEPX_PX:
                continue
            cam = S.distance(self.looks.get(look), self.looks.get(old_look))
            if cam is not None and cam > S.SAMEPX_CAM_MM:
                self._raise("samepx", {"label": word, "u": u, "v": v, "look": look,
                                      "first_look": old_look, "first_call": old_call, "cam_mm": cam}, arm)
                break
        self.marks.append((look, u, v, word, self.calls))

    def _locate(self, args, result, arm):
        entry = result.get("id") if isinstance(result.get("id"), str) else result.get("located")
        if not isinstance(entry, str):
            return
        word = S.word(args, result)
        point = S.vector(result.get("point_mm"))
        self.located[entry] = {"point_mm": list(point) if point is not None else None,
                               "label": word, "call": self.calls}
        if point is not None:
            self.last_located_call = self.calls
            self._rise("located")
        flags = self.flags.setdefault(entry, {"mislabel": False, "body": False, "air": False,
                                              "no_give": 0, "pressed_no_change": False})
        chosen = result.get("chosen") if isinstance(result.get("chosen"), dict) else {}
        # a pending flag belongs to the operator's own marks under that label; a point made from a numbered
        # candidate the robot proposed is not one of those marks and does not take it over
        pending = {} if chosen.get("n") is not None else self._pending_flags.get(word, {})
        for key in ("mislabel", "body"):
            if pending.get(key):
                flags[key] = True
        error = result.get("error_mm")
        disagree = "depth" in result and "did not agree" in str(result.get("said"))
        if S.number(error) and error > S.RAYS_ERR_MM or disagree:
            window = round(S.EARLY_SHARE * self.budget_calls) if self.budget_calls is not None else S.EARLY_CALLS
            if self.calls <= window:
                self._raise("rays15", {"label": word, "error_mm": error if S.number(error) else None, "id": entry}, arm)
            else:
                self.note("ray_miss_late")

    def _act(self, tool, args, result, extra, classes, read, arm):
        state = self._arm(arm)
        pending = state["pending_press"]
        if pending is not None:
            self.flags[pending]["pressed_no_change"] = True
        state["pending_press"] = None
        confirmed = self._confirmed_now(result, extra)
        if confirmed:
            self._confirm()
        toward, named = self._toward(args, read["point_mm"])
        to_point, _ = self._nearest(read["tip_mm"])
        near_dist, near_entry = self._nearest(read["point_mm"])
        near = near_entry if near_dist is not None and near_dist <= self.near_mm else None
        if read["free"]:
            if toward is not None:
                self.flags[toward]["air"] = True
            if state["act_met"] is None:
                event = self._raise("air", {"tool": tool, "ran_mm": read["ran_mm"],
                                           "to_point_mm": to_point, "entry": toward}, arm)
                if event is not None:
                    self.air_events.append(event)
                if self.stage == "located":
                    self._rise("ran_free")
        if read["met"]:
            if state["act_met"] is None:
                state["act_met"] = self.calls
            if self.act_met_call is None:
                self.act_met_call = self.calls
            self.phase = "after_met"
            if state["contact"] is None:
                state["contact"] = self.calls
            if "feel" in classes:
                self.feel_met.append(self.calls)
            if near is not None:
                self.signs.append({"sign": "surface_met", "call": self.calls})
                self.flags[near]["air"] = False
        routine = bool(extra.get("by_robot") and extra.get("unconfirmed"))
        if read["gave"]:
            if routine:
                self.note("routine_gave")
            else:
                self._gave(state)
        if "feel" not in classes and read["no_give"] and near is not None and self.located[near]["label"] in self.parts:
            self.note("no_give")
            self.no_give[near] = self.no_give.get(near, 0) + 1
            self.flags[near]["no_give"] = self.no_give[near]
        if read["met"]:
            self._rise("met")
            if "feel" not in classes and not read["gave"]:
                if read["back"]:
                    self._rise("moved_back")
                elif read["no_give"]:
                    self._rise("no_give")
        tip = read["tip_mm"]
        if tip is not None:
            raw_force = args.get("force_n")
            force = float(raw_force) if S.number(raw_force) else 0.0
            prev = [f for pos, name, f in state["spots"]
                    if name == tool and S.distance(pos, tip) <= self.same_spot_mm]
            if prev and not confirmed and force <= max(prev):
                state["repeat"] += 1
                state["repeat_calls"].append(self.calls)
                if state["repeat"] >= S.REPEATS and not state["repeat_fired"]:
                    self._raise("repeat", {"count": state["repeat"], "radius_mm": self.same_spot_mm,
                                           "calls": state["repeat_calls"]}, arm)
                    state["repeat_fired"] = True
            if not confirmed:
                state["spots"].append((list(tip), tool, force))
        state["last_act"] = self.calls
        if "feel" not in classes and read["met"] and (near is not None or named):
            state["pending_press"] = toward if named else near
        if confirmed:
            state["pending_press"] = None
        return confirmed

    def feed(self, request, response, extra=None):
        request, response, extra = S.mapping(request), S.mapping(response), S.mapping(extra)
        before = len(self.events)
        op = request.get("op")
        if op == "call":
            self.calls += 1
        self._meta(request, extra)
        if op not in ("call", "claim"):
            return []
        budget = self._budget(response, preserve=op == "claim")
        args, result = S.mapping(request.get("args")), S.mapping(response.get("result"))
        if op == "claim":
            for event in self.claim_checks(args.get("success")):
                pair = (event["signal"], event["call"])
                if pair not in self._claim_pairs:
                    self._claim_pairs.add(pair)
                    self.events.append(event)
            return S.copy(self.events[before:])
        tool = request.get("name")
        tool = tool if isinstance(tool, str) else "?"
        arm = args.get("arm", result.get("arm"))
        if not isinstance(arm, (str, int, float)):
            arm = None
        if isinstance(arm, float):
            arm = round(arm, 1) if S.number(arm) else None
        state = self._arm(arm)
        if "parts" in extra and isinstance(extra["parts"], (list, tuple)):
            self.parts = sorted(x for x in (S.label(p) for p in extra["parts"]) if x is not None)
        if "roles" in extra:
            self.roles = dict(S.mapping(extra["roles"]))
        self._store_look(result)
        self._store_look(S.mapping(result.get("look")))
        read = S.read(result)
        if not read["refused"]:
            if extra.get("met") is True:
                read["met"] = True
            if extra.get("gave") is True:
                read.update(gave=True, met=True)
            if extra.get("ran_free") is True:
                read.update(free=True, met=False)
        classes = S.classify(tool, args, result, self.classes)
        try:
            signature = (tool, json.dumps(args, sort_keys=True, default=str))
        except (TypeError, ValueError):
            signature = (tool, repr(args))
        if signature in self._duplicates and "look" not in classes:
            self.note("duplicate")
        self._duplicates[signature] = self.calls
        self._looks_row = self._looks_row + 1 if "look" in classes else 0
        if self._looks_row == S.LOOKS_ROW:
            self.note("looks_row")
            if self.calls <= S.LOOKS3_WITHIN:
                self._raise("looks3", {"tool": tool}, arm)
        if read["refused"]:
            check = read["check"]
            check = check if isinstance(check, str) else "?"
            if check in S.MALFORMED_CHECKS:
                self.note("malformed")
            key = (tool, check)
            previous = self.refusals.setdefault(key, [])
            if previous:
                self.note("same_refusal")
            previous.append(self.calls)
            self.refused_calls.append(self.calls)
            if not self._refuse_fired:
                if len(self.refused_calls) >= S.REFUSALS:
                    self._raise("refuse", {"why": "total", "count": len(self.refused_calls), "calls": self.refused_calls}, arm)
                    self._refuse_fired = True
                elif len(previous) >= S.SAME_REFUSAL:
                    self._raise("refuse", {"why": "same", "tool": tool, "check": check,
                                           "count": len(previous), "calls": previous}, arm)
                    self._refuse_fired = True
        elif "act" not in classes:
            if read["bump"]:
                self.note("bump")
                if state["contact"] is None:
                    state["contact"] = self.calls
            if read["gave"]:
                self._gave(state)
        self._point(tool, args, result, extra, classes, read, arm)
        if not read["refused"]:
            if "locate" in classes:
                self._locate(args, result, arm)
            confirmed = False
            if "act" in classes:
                confirmed = self._act(tool, args, result, extra, classes, read, arm)
            elif self._confirmed_now(result, extra):
                self._confirm()
                confirmed = True
            if confirmed:
                self._open_index = len(self.events)
            if extra.get("check") == "no_change" and self.stage == "gave":
                self._rise("gave_unchanged")
            if "verify" in classes and result.get("accepted") is True and self.verify_ok is None:
                self.verify_ok = self.calls
        if not self._wind_fired and "end" in classes and not read["refused"] and self.verify_ok is None and self.gave_call is None and not self.confirmed_calls:
            if self.budget_calls is None or self.calls_left >= S.PLENTY_SHARE * self.budget_calls:
                self._raise("wind_down", {"tool": tool, "calls_left": self.calls_left if self.budget_calls is not None else None}, arm)
                self._wind_fired = True
        for key in sorted(self.arms, key=lambda a: (a is not None, str(a))):
            current = self.arms[key]
            since = max(current["last_act"], self.last_located_call, current["idle_base"])
            gap = self.calls - since
            if current["contact"] is not None and current["gave"] is None and current["last_act"] > 0 and gap >= S.IDLE_CALLS and not current["idle_fired"]:
                self._raise("idle", {"idle_calls": gap, "since": since, "contact_call": current["contact"]}, key)
                current["idle_fired"] = True
        used = budget.get("sim_seconds_used")
        if not self._slow_fired and self.budget_s is not None and S.number(used) and used > S.SLOW_TIME_SHARE * self.budget_s and self.last_located_call == 0:
            self._raise("slow_start", {"used_s": used, "tool": tool}, arm)
            self._slow_fired = True
        sense = extra.get("sense")
        for finding in sense if isinstance(sense, list) else []:
            finding = S.mapping(finding)
            verdict = finding.get("verdict")
            if verdict == "in":
                self.note("mislabel_find_in")
            elif verdict == "out":
                self.note("mislabel_find_out")
                at = finding.get("call")
                self._raise("mislabel_find", {"tool": tool}, arm, int(at) if S.number(at) else self.calls)
            else:
                self.note("mislabel_find_unknown")
        return S.copy(self.events[before:])

    def claim_checks(self, success):
        """The claim events that hold for a claim sent now; records nothing. success: True, False or the args."""
        if isinstance(success, dict):
            success = success.get("success")
        checks = []
        def add(name, **data):
            event = self._event(name, data)
            if event is not None:
                checks.append(event)
        if success is False:
            if self.calls_left >= S.PLENTY_SHARE * self.budget_calls if self.budget_calls is not None else self.act_met_call is None:
                add("stop", calls_left=self.calls_left if self.budget_calls is not None else None)
        elif success is True:
            if self.act_met_call is None and self.gave_call is None:
                add("no_contact")
            if self._answered is not None and self._answered[1] == len(self.confirmed_calls):
                add("flip", answered_call=self._answered[0])
            if self.air_events:
                air = self.air_events[0]
                if not any(call > air["call"] for call in self.feel_met):
                    add("touch_contradicts", air_call=air["call"], air_tool=air["data"]["tool"], ran_mm=air["data"]["ran_mm"])
            if self.verify_ok is None and self.gave_call is None:
                add("no_check")
        return checks

    def _open(self):
        out = {}
        for skill in S.SKILLS:
            events = [e for e in self.events[self._open_index:] if e["skill"] == skill]
            newest = max((e["call"] for e in events), default=None)
            out[skill] = events if newest is not None and self.calls - newest <= S.DECAY_CALLS else []
        return out

    def standing(self):
        return {skill: len(events) for skill, events in self._open().items()}

    def snap(self):
        opened = self._open()
        count = lambda name: sum(e["signal"] == name for e in self.events)
        sign = lambda name: sum(e["sign"] == name for e in self.signs)
        candidates = {name: self.notes.get(name, 0) + count(name) for name in sorted(self.signals)
                      if self.signals[name].get("kind") == "candidate"}
        off = []
        if self.budget_s is None:
            off.append({"signal": "slow_start", "why": "no time budget"})
        if self.tools is not None:
            loaded = [S.may_be(name, self.classes) for name in sorted(self.tools)]
            for name in sorted(self.signals):
                spec = self.signals[name]
                if spec.get("enabled") and spec.get("kind") == "event":
                    for kind in sorted(spec.get("classes", ())):
                        if not any(kind in kinds for kinds in loaded):
                            off.append({"signal": name, "why": "no loaded tool of class " + kind})
        phrases = []
        for skill in S.SKILLS:
            events = opened[skill]
            phrase = skill + ": " + str(len(events))
            if events:
                newest = max(enumerate(events), key=lambda item: (item[1]["call"], item[0]))[1]
                phrase += " open event" + ("s" if len(events) > 1 else "") + " (" + newest["evidence"] + ")"
            phrases.append(phrase)
        return S.copy({"calls": self.calls, "standing": self.standing(), "events": self.events,
                       "signs": self.signs, "notes": self.notes, "candidates": candidates,
                       "pointing": {"tests": self._pointing_tests, "on_own_body": count("body"),
                                    "mislabelled": count("mislabel"), "same_pixel": count("samepx"),
                                    "surface_met": sign("surface_met"), "ran_free": count("air"), "confirmed": sign("confirmed")},
                       "stage": self.stage, "phase": self.phase, "flags": self.flags,
                       "says": "; ".join(phrases), "caveat": S.CAVEAT.format(n=len(self.events)),
                       "off": sorted(off, key=lambda item: (item["signal"], item["why"]))})


def feed_journal(gauge, records, extra_for=None):
    events, pending = [], None
    for record in records:
        record = S.mapping(record)
        kind = record.get("kind")
        if kind == "request":
            if pending is not None:
                events.extend(gauge.feed(pending, {}))
                pending = None
            request = S.mapping(record.get("request"))
            if request.get("op") == "claim":
                pending = request
        elif kind == "response" and S.mapping(record.get("response")).get("op") == "claim":
            if pending is not None:
                events.extend(gauge.feed(pending, record["response"]))
                pending = None
        elif kind == "call":
            events.extend(gauge.feed({"op": "call", "name": record.get("name"), "args": record.get("args")},
                                     {"result": record.get("result"), "budget": record.get("budget")},
                                     extra_for(record) if extra_for is not None else None))
    if pending is not None:
        events.extend(gauge.feed(pending, {}))
    return events


def journal_budget(records):
    for record in records:
        record = S.mapping(record)
        if record.get("kind") == "session" and record.get("event") == "start":
            budget = S.mapping(record.get("budget"))
            calls, seconds = budget.get("calls"), budget.get("sim_seconds")
            return (int(calls) if S.number(calls) else None,
                    float(seconds) if S.number(seconds) else None)
    return None, None
