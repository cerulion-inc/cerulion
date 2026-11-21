# SPDX-License-Identifier: AGPL-3.0-only
"""The chat operator's answers to strict endpoints and to models that write calls as text (a mixin of
bench/operators/chat_operator.ChatOperator; the rules are in that module's docstring and in portable.py and
text_calls.py). Uses the operator's own state: schemas, listing, phase, turn, counts, _log, _build_tools and the
request settings. No clock, no I/O beyond the operator's journal.
"""
import json

from bench.operators.chat_prompt import CLAIM_TOOL, positional_names
from bench.operators.portable import schema_refusal, unknown_body_fields
from bench.operators.text_calls import find_text_call

MAX_HTTP_400_FREE = 8      # free retries (a field dropped or renamed, plain schemas) per episode, a bound on loops


class StrictEndpointMixin:
    def _body(self, messages, tools):
        """One request body: the loop's own keys, --extra-body, then what a strict endpoint refused taken out."""
        body = {"model": self.model, "messages": messages, "tools": tools, "tool_choice": "auto",
                "temperature": self.temperature, "max_tokens": self.max_tokens}
        if self.seed is not None:
            body["seed"] = self.seed
        for key, value in self.body_extra.items():
            if value is None:
                body.pop(key, None)
            else:
                body[key] = value
        for key in self.body_dropped:
            body.pop(key, None)
        for old, new in self.body_renamed.items():
            if old in body:
                body[new] = body.pop(old)
        return body

    def _strict_400(self, exc, body, attempt):
        """True when a 400 was a strict endpoint refusing a body field or a schema keyword and the request is worth
        one more try with the field dropped (or renamed) or with plain schemas; such a try is not a failure."""
        if exc.status != 400 or self.free_retries >= MAX_HTTP_400_FREE:
            return False
        drop, rename = unknown_body_fields(exc.why, body)
        if drop or rename:
            self.body_dropped += [k for k in drop if k not in self.body_dropped]
            self.body_renamed.update(rename)
            self.counts["body_fields_dropped"] += len(drop) + len(rename)
            what = {"dropped": drop, "renamed": rename}
        elif schema_refusal(exc.why) and not self.plain:
            self.plain = True
            self._build_tools()
            self.counts["plain_schema_switches"] += 1
            what = {"plain_schema": True}
        else:
            return False
        self.free_retries += 1
        self._log("strict_endpoint", attempt=attempt, status=exc.status, why=exc.why, **what)
        return True

    def _text_call(self, text):
        """The first call the reply wrote as text, as tool calls with ids, or None (bench/operators/text_calls.py).
        Only claim counts in the claim phase; the text after the call is journaled and dropped."""
        known = [n for n in self.schemas if self.phase == "work" or n == CLAIM_TOOL]
        found = find_text_call(text, known, lambda n: positional_names(self.schemas.get(n)))
        if found is None or (self.phase != "work" and any(n != CLAIM_TOOL for n, _ in found["calls"])):
            return None
        calls = [{"id": f"x{self.turn:05d}{i:03d}", "type": "function",
                  "function": {"name": name, "arguments": json.dumps(args)}}
                 for i, (name, args) in enumerate(found["calls"])]
        self.counts["tool_calls_in_text"] += len(calls)
        tail = found["after"].strip()
        self.counts["text_tails_dropped"] += bool(tail)
        self._log("text_call", form=found["form"], calls=[c["function"] for c in calls], before=found["before"],
                  dropped_after=tail)
        return {"tool_calls": calls, "before": found["before"]}
