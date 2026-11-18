# SPDX-License-Identifier: AGPL-3.0-only
"""Chat operator: any OpenAI-compatible chat model with tool calling and image input operates one rcpx episode.

    ROOT/venv/bin/python -m bench.operators.chat_operator --ep EPDIR --base-url URL --model NAME
        [--max-turns 120] [--keep-images 6] [--temperature 0.2] [--seed N] [--context-tokens 32000]
        [--max-tokens 4096] [--http-timeout 600] [--socket-timeout 900] [--retry-wait 8] [--api-key-env NAME]
        [--extra-body JSON] [--plain-schema] [--one-call-per-reply] [--reply-budget N]

Run from ROOT. URL is the endpoint's API root, for example http://127.0.0.1:8000/v1 (vLLM); /chat/completions is
added. The episode server must already answer on EPDIR/rcp.sock (rcpx/server.py; same protocol as rcpx/cli.py).

What the model gets, and nothing else: the operator brief (operator_brief.md, the same text for every operator
kind) as the system message; then one user message with the task sentence (robot://task), the practices text the
episode server sends with its first answer, and the conventions and budget of the server's list answer. Tools: every
tool of the list answer (name, description plus its worked example, argument schema), read_resource(uri) and
claim(statement, success). After each call the tool message holds the result as JSON text (long arrays trimmed,
refusals whole) and the budget left; frames the answer names go in the next user message as JPEG images, longest
side at most 640 px, each frame once per episode. Only the newest --keep-images images stay in the history; older
tool results are shortened when the estimated context passes the limit taken from --context-tokens. Every request
sends parallel_tool_calls false plus --extra-body; without --one-call-per-reply, several calls in one reply run
in order.

A surface may open the conversation itself: the robot://task content may carry first_message {text, images
[{label, path} or a path, files inside EPDIR], state {...}, system, own_tools}. The first user message is then the
paragraphs above (task, practices, conventions, budget), the surface's text verbatim, a line "state: {...}", and
per picture a line "LABEL: FILE (W x H px)" and the picture; the pictures are the oldest of the --keep-images
window and are never attached twice. system replaces
the brief as the system message. own_tools false offers exactly the listed tools: no read_resource and no claim, no
notice names them, and when the budget or the turns are spent the loop claims failure itself instead of asking. A
malformed first_message or a picture that cannot be attached ends the run with exit 3 before any model request
(never a silent start without them). A surface without first_message is unchanged.

--one-call-per-reply: only the first tool call of a reply runs (also one written as text); the others are answered
"not run: one call per reply" and counted (calls_not_run). --reply-budget N: the episode has N model replies and each
counts whatever it calls (a read, a claim, a refused call, no call); every reply, the Nth too, is offered every tool
(no claim-only last turn), and after reply N the loop claims failure for the model. Both rules are said in the
first user message, and with a reply budget every tool result says how many replies are left.

Strict endpoints (bench/operators/portable.py): --plain-schema sends every argument schema with the keywords a strict
endpoint may refuse (minimum, maximum, minItems, pattern, default, ...) said in the argument's description instead,
and anyOf or oneOf flattened where a branch is plain. Without the flag the loop switches to it by itself after an
HTTP 400 about the tool schemas. A 400 that names a top-level body field as unknown or unsupported (seed,
parallel_tool_calls, temperature, ...) drops that field for the rest of the episode (or renames it when the error
says "use X instead") and retries. Neither retry counts as an endpoint failure; each is journaled.

A reply with no tool call is read for a call written as TEXT (bench/operators/text_calls.py: [name(arg=value)],
a JSON {"name", "arguments"} object, a <tool_call> block): the first such block (a bracket may hold several calls)
runs as tool calls (counted in tool_calls_in_text), the text after it is dropped from the history (never taken for a
result) and the
model is told so. A reply with no call at all gets a nudge ("call a tool or claim"; a reply cut off at --max-tokens
is told so); a second in a row gets one message restating the exact call format with a worked example from the tool
list; a third ends the episode with the claim {"statement": "operator stopped calling tools", "success": false} (or
one naming the cut-off when the last was cut off too). Arguments that are not JSON are answered
with the parse error and sent nowhere; arguments that parse are sent as they are, so the server's refusal teaches.
When the budget is spent the model is asked to claim (only claim is offered, two turns); the last turn of
--max-turns without a reply budget asks the same. Five endpoint failures in a row stop the loop.

EPDIR/operator.jsonl: every request summary, reply, tool call, refusal and the counters; positions are turns and
the server's call count, never wall-clock time. Exit codes: 0 the episode ended by a claim (the model's own, or
the loop's after the model stopped calling tools, ran out of turns or did not claim after the budget: the end
record's claim_by says which); 2 the episode ended without a claim (an e-stop, or it had already ended); 3 no
episode server answers at the start, or its list, task or first_message cannot be read; 4 the endpoint failed five
times in a row (the loop claims failure for the model); 5 the episode server stopped answering mid-episode, or the
loop failed; 64 a malformed command line.
"""
import argparse
import json
import os
import sys
import time
import traceback

from bench.operators import load_brief
from bench.operators.chat_prompt import (BUDGET_NOTICE, BUDGET_SPENT_STOP, CLAIM_NOW, CLAIM_TOOL, CUT_OFF_NOTE,
                                         LAST_TURN_NOTICE, NO_CLAIM_AFTER_BUDGET, NOT_RUN, NUDGE, NUDGE_LISTED,
                                         PRACTICES_URI, READ_TOOL, REPLIES_CUT_OFF, REPLY_BUDGET_SPENT,
                                         STOPPED_CALLING, TASK_URI, TEXT_CALL_NOTE, TURN_LIMIT, argument_problem,
                                         build_tools, first_message_of, first_message_text, first_user_text,
                                         format_note, parse_arguments, task_sentence, text_of, tool_calls_of)
from bench.operators.chat_support import (ChatEndpoint, EndpointError, EpisodeClient, History, OperatorJournal,
                                          compact_text, encode_frame, has_refusal, inside, parse_extra_body,
                                          result_text, trim_value)
from bench.operators.chat_strict import StrictEndpointMixin

JOURNAL_NAME = "operator.jsonl"
MAX_HTTP_FAILURES = 5
CLAIM_PHASE_TURNS = 2
MAX_SIDE_PX = 640
REPLY_RESERVE = 512        # tokens kept free beyond --max-tokens when the context limit is computed
SHRINK_ON_400 = 0.6        # an HTTP 400 (often an over-long context) shrinks the limit by this factor

ENDPOINT_FAILED = f"operator stopped: the model endpoint failed {MAX_HTTP_FAILURES} times in a row"

EXIT_CLAIM, EXIT_ENDED, EXIT_NO_EPISODE, EXIT_ENDPOINT, EXIT_LOST, EXIT_USAGE = 0, 2, 3, 4, 5, 64
# Sent with every request unless --extra-body overrides them. parallel_tool_calls false: one call per reply, so
# every action is chosen after the result and frames of the one before it (an endpoint without the flag may still
# send several calls; without --one-call-per-reply each runs in order and is answered).
DEFAULT_BODY = {"parallel_tool_calls": False}


# ---------------------------------------------------------------------------------------------------------------
# The loop

FORCED_CLAIM_RESENDS = 8   # resends of the harness's own closing claim while a claim gate answers it


class ChatOperator(StrictEndpointMixin):
    def __init__(self, ep_dir, endpoint, model, *, max_turns=120, keep_images=6, temperature=0.2, seed=None,
                 context_tokens=32000, max_tokens=4096, retry_wait_s=8.0, socket_timeout_s=900.0, sleep=time.sleep,
                 run_info=None, extra_body=None, plain_schema=False, one_call=False, reply_budget=None):
        self.ep_dir = os.path.abspath(ep_dir)
        self.body_extra = {**DEFAULT_BODY, **dict(extra_body or {})}  # a None value drops the key from the body
        self.plain = bool(plain_schema)
        self.own_tools = True
        self.body_dropped, self.body_renamed, self.free_retries = [], {}, 0  # what strict endpoints refused
        self.listing, self.no_call_streak = None, 0
        self.shown_frames = {}  # frame path -> the turn it was attached in; a frame is attached once
        self.episode = EpisodeClient(self.ep_dir, socket_timeout_s)
        self.endpoint, self.model = endpoint, model
        self.reply_budget, self.one_call = reply_budget, bool(one_call)
        self.max_turns = int(max_turns if reply_budget is None else reply_budget)
        self.keep_images = int(keep_images)
        self.temperature, self.seed, self.max_tokens = temperature, seed, int(max_tokens)
        self.limit_tokens = max(int(context_tokens) // 4, int(context_tokens) - self.max_tokens - REPLY_RESERVE)
        self.retry_wait_s, self.sleep = float(retry_wait_s), sleep
        self.run_info = dict(run_info or {})
        self.journal = OperatorJournal(os.path.join(self.ep_dir, JOURNAL_NAME))
        self.turn, self.phase, self.claim_turns = 0, "work", 0
        self.budget, self.budget_refused, self.consecutive_failures, self.last_error = {}, False, 0, None
        self.claim_by, self.claim, self.end = None, None, None
        self.history = self.tools = self.claim_tools = self.schemas = None
        self.counts = {k: 0 for k in ("turns", "http_requests", "http_failures", "tool_calls", "server_calls",
                                      "reads", "claims_sent", "refusals", "malformed_json", "malformed_args",
                                      "unknown_tools", "nudges", "format_notes", "frames_attached",
                                      "frames_not_attached", "frames_repeated", "tool_calls_in_text",
                                      "text_tails_dropped", "length_cutoffs", "plain_schema_switches",
                                      "body_fields_dropped", "calls_not_run", "multi_call_replies")}

    def _log(self, kind, **fields):
        self.journal.append(kind, turn=self.turn, calls_used=self.budget.get("calls_used"),
                            sim_seconds_used=self.budget.get("sim_seconds_used"), **fields)

    def run(self):
        try:
            code = self._run()
        except Exception:  # a loop bug: journaled, and the episode is left for its runner to stop
            self._log("error", trace=traceback.format_exc())
            code = self._finish("operator_failed", EXIT_LOST)
        counts = {**self.counts, **(self.history.counts if self.history is not None else {})}
        if self.end is None:
            self._finish("operator_failed", EXIT_LOST)
            code = EXIT_LOST
        self._log("end", **self.end, claim_by=self.claim_by, claim=self.claim, counts=counts)
        self.journal.close()
        return code

    def _finish(self, reason, code, **info):
        self.end = {"reason": reason, "exit_code": code, **info}
        return code

    def _note_budget(self, budget):
        if isinstance(budget, dict):
            self.budget = budget

    def _budget_spent(self):
        b = self.budget
        return self.budget_refused or (b.get("calls_left") is not None and b["calls_left"] <= 0) or (
            b.get("sim_seconds_left") is not None and b["sim_seconds_left"] <= 0)

    # -- start ---------------------------------------------------------------------------------------------------

    def _run(self):
        resp, why = self.episode.request({"op": "list", "full": True})
        if resp is None:
            return self._finish("no_server", EXIT_NO_EPISODE, why=why)
        if resp.get("episode") == "ended":
            return self._finish("episode_ended", EXIT_ENDED)
        listing = resp.get("result")
        if not isinstance(listing, dict) or not isinstance(listing.get("tools"), list):
            return self._finish("list_unreadable", EXIT_NO_EPISODE, response=resp)
        self._note_budget(resp.get("budget"))
        practices, source = resp.get("practices"), "first answer"
        uris = [r.get("uri") for r in listing.get("resources", []) if isinstance(r, dict)]
        if practices is None and PRACTICES_URI in uris:
            presp, _ = self.episode.request({"op": "read", "name": PRACTICES_URI})
            presult = presp.get("result") if isinstance(presp, dict) else None
            content = presult.get("content") if isinstance(presult, dict) else None
            practices, source = (content, "read") if isinstance(content, str) else (None, "none")
        elif practices is None:
            source = "none"
        tresp, why = self.episode.request({"op": "read", "name": TASK_URI})
        result = (tresp or {}).get("result")
        if not isinstance(result, dict) or "content" not in result:
            return self._finish("task_unreadable", EXIT_NO_EPISODE, why=why, response=tresp)
        task = task_sentence(result["content"])
        spec, why = first_message_of(result["content"])
        if why is not None:
            return self._finish("first_message_unreadable", EXIT_NO_EPISODE, why=why)
        self.own_tools = spec["own_tools"] if spec is not None else True
        self.listing = listing
        taken = self._build_tools()
        system = spec["system"] if spec is not None and spec["system"] is not None else load_brief()
        first = first_user_text(task, practices, listing, self.reply_budget, self.one_call)
        first_message = None
        if spec is not None:
            first += "\n\n" + first_message_text(spec)
            parts = [{"type": "text", "text": first}]
            first_message = {"system": spec["system"] is not None, "state": spec["state"] is not None,
                             "own_tools": self.own_tools, "images": []}
            for path, label in spec["images"]:
                full = path if os.path.isabs(path) else os.path.join(self.ep_dir, path)
                attached, sent = self._frame_parts(full, label)
                if attached is None:
                    return self._finish("first_message_unreadable", EXIT_NO_EPISODE,
                                        why=f"the image {path} could not be attached: {sent}")
                parts.extend(attached)
                first_message["images"].append({"label": label, **sent})
            content = parts
        else:
            content = first
        self.history = History(system, content, self.keep_images)
        self._log("start", run=self.run_info, surface=listing.get("surface"), task=task, practices_source=source,
                  tools=[t["function"]["name"] for t in self.tools], tools_taken_by_operator=taken,
                  system=system, first_user=first, first_message=first_message, own_tools=self.own_tools,
                  limit_tokens=self.limit_tokens, plain_schema=self.plain, reply_budget=self.reply_budget,
                  one_call_per_reply=self.one_call)
        if self._budget_spent():
            self._enter_claim_phase()
        return self._loop()

    def _build_tools(self):
        """(Re)build the tool definitions from the listing, plain or full; the names the operator's own tools took."""
        self.tools, self.schemas, taken = build_tools(self.listing, plain=self.plain, own_tools=self.own_tools)
        self.claim_tools = [t for t in self.tools if self.own_tools and t["function"]["name"] == CLAIM_TOOL]
        return taken

    def _enter_claim_phase(self):
        self.phase, self.claim_turns = "claim", 0
        if self.claim_tools:
            self.history.add_user_text(BUDGET_NOTICE)
            self._log("notice", phase="claim", text=BUDGET_NOTICE)

    # -- turns ---------------------------------------------------------------------------------------------------

    def _loop(self):
        while True:
            if self.turn >= self.max_turns:
                if self.reply_budget is not None:
                    return self._forced_claim(REPLY_BUDGET_SPENT, "reply_budget")
                return self._forced_claim(TURN_LIMIT, "max_turns")
            if self.phase == "claim" and not self.claim_tools:
                return self._forced_claim(BUDGET_SPENT_STOP, "budget_spent")
            if self.reply_budget is None and self.claim_tools and self.phase == "work" \
                    and self.turn == self.max_turns - 1:
                self.phase = "last"
                self.history.add_user_text(LAST_TURN_NOTICE)
                self._log("notice", phase="last", text=LAST_TURN_NOTICE)
            reply = self._complete()
            if reply is None:
                code = self._forced_claim(ENDPOINT_FAILED, "endpoint_failed")
                self.end.update(exit_code=EXIT_ENDPOINT, last_error=self.last_error, claim_exit=code)
                return EXIT_ENDPOINT
            self.turn += 1
            self.counts["turns"] += 1
            outcome = self._take_reply(reply)
            if outcome is not None:
                return outcome

    def _take_reply(self, reply):
        """Act on one reply; an exit code when the episode is over, else None."""
        message = reply["message"]
        text = text_of(message.get("content"))
        calls = tool_calls_of(message, self.turn)
        reasoning = message.get("reasoning_content") or message.get("reasoning")
        self._log("reply", finish_reason=reply.get("finish_reason"), content=text, usage=reply.get("usage"),
                  reasoning=reasoning if isinstance(reasoning, str) else None,
                  tool_calls=[{"id": c["id"], "name": c["function"]["name"], "arguments": c["function"]["arguments"]}
                              for c in calls])
        assistant = {"role": "assistant", "content": text}
        written = None
        if not calls:
            written = self._text_call(text)
            if written is None:
                return self._no_tool_call(assistant, text, reply.get("finish_reason"))
            calls, assistant["content"] = written["tool_calls"], written["before"]
        self.counts["multi_call_replies"] += int(len(calls) > 1)
        assistant["tool_calls"] = calls
        self.no_call_streak = 0
        turn_msgs, frames, outcome = [assistant], [], None
        for i, call in enumerate(calls):
            if self.one_call and i > 0:
                self.counts["tool_calls"] += 1
                self.counts["calls_not_run"] += 1
                self._log("tool_call", id=call["id"], name=call["function"]["name"],
                          arguments=call["function"]["arguments"], sent=False, not_run=True, text=NOT_RUN)
                msg, call_frames = self._tool_message(call["id"], NOT_RUN), []
            else:
                msg, call_frames, outcome = self._do_call(call)
            turn_msgs.append(msg)
            frames.extend(call_frames)
            if outcome is not None:
                break
        if frames:
            turn_msgs.append(self._frames_message(frames))
        self.history.add_turn(turn_msgs)
        if written is not None and outcome is None:
            self.history.add_user_text(TEXT_CALL_NOTE.format(name=calls[0]["function"]["name"]))
        if outcome is not None:
            return self._finish(*outcome)
        if self.phase == "work" and self._budget_spent():
            self._enter_claim_phase()
        elif self.phase == "claim":
            self.claim_turns += 1
            if self.claim_turns >= CLAIM_PHASE_TURNS:
                return self._forced_claim(NO_CLAIM_AFTER_BUDGET, "budget_spent_no_claim")
        return None

    def _no_tool_call(self, assistant, text, finish_reason):
        self.history.add_turn([assistant])
        cut = finish_reason == "length"
        if cut:
            self.counts["length_cutoffs"] += 1
        if self.phase == "work":
            self.no_call_streak += 1
            if self.no_call_streak >= 3:
                if cut:
                    return self._forced_claim(REPLIES_CUT_OFF, "replies_cut_off")
                return self._forced_claim(STOPPED_CALLING, "stopped_calling_tools")
            if self.no_call_streak == 2:   # before giving up: the exact call format, with a worked example
                note = format_note(self.listing, own_tools=self.own_tools)
                if cut:
                    note = CUT_OFF_NOTE.format(n=self.max_tokens, then="call a tool") + " " + note
                self.counts["format_notes"] += 1
                self.history.add_user_text(note)
                self._log("format_note", text=note, cut_off=cut)
                return None
            self.counts["nudges"] += 1
            nudge = NUDGE if self.own_tools else NUDGE_LISTED
            nudge = CUT_OFF_NOTE.format(n=self.max_tokens, then=nudge) if cut else nudge
            self.history.add_user_text(nudge)
            self._log("nudge", text=nudge, cut_off=cut)
            return None
        if self.phase == "claim":
            self.claim_turns += 1
            if self.claim_turns >= CLAIM_PHASE_TURNS:
                return self._forced_claim(NO_CLAIM_AFTER_BUDGET, "budget_spent_no_claim")
            nudge = CUT_OFF_NOTE.format(n=self.max_tokens, then=CLAIM_NOW) if cut else CLAIM_NOW
            self.history.add_user_text(nudge)
            self._log("nudge", text=nudge, cut_off=cut)
        return None  # the last turn: the loop's turn check claims next

    def _complete(self):
        """One completion with up to MAX_HTTP_FAILURES tries in a row; None when they all failed. A 400 that a strict
        endpoint gave for a body field or a schema keyword is retried at once and is not a failure."""
        attempt = 0
        while True:
            attempt += 1
            tools = self.tools if self.phase == "work" else self.claim_tools
            tools_tokens = (len(json.dumps(tools)) + 2) // 3
            messages, trimmed, est = self.history.build(self.limit_tokens, tools_tokens)
            body = self._body(messages, tools)
            self.counts["http_requests"] += 1
            self._log("request", for_turn=self.turn + 1, attempt=attempt, phase=self.phase, messages=len(messages),
                      images=self.history.images(), est_tokens=est, limit_tokens=self.limit_tokens,
                      tools=len(tools), trimmed=trimmed, plain_schema=self.plain,
                      body_keys=sorted(k for k in body if k not in ("messages", "tools")))
            try:
                reply = self.endpoint.complete(body)
            except EndpointError as exc:
                if self._strict_400(exc, body, attempt):
                    continue
                self.consecutive_failures += 1
                self.counts["http_failures"] += 1
                self.last_error = exc.why
                if exc.status == 400:
                    self.limit_tokens = max(1000, int(self.limit_tokens * SHRINK_ON_400))
                self._log("http_error", attempt=attempt, error_kind=exc.kind, status=exc.status, why=exc.why,
                          in_a_row=self.consecutive_failures, limit_tokens=self.limit_tokens)
                if self.consecutive_failures >= MAX_HTTP_FAILURES:
                    return None
                if self.retry_wait_s > 0:
                    self.sleep(self.retry_wait_s * 2 ** (self.consecutive_failures - 1))
                continue
            self.consecutive_failures = 0
            return reply

    # -- one tool call -------------------------------------------------------------------------------------------

    def _tool_message(self, call_id, text, compact=None):
        return {"role": "tool", "tool_call_id": call_id, "content": text, "_compact": compact or text}

    def _do_call(self, call):
        """(tool message, [(frame path, label)], outcome or None). The outcome is (reason, exit code)."""
        self.counts["tool_calls"] += 1
        name, raw = call["function"]["name"], call["function"]["arguments"]
        args, err = parse_arguments(raw)
        if err is not None:
            self.counts["malformed_json"] += 1
            schema = self.schemas.get(name)
            text = (f"the arguments are not valid JSON ({err}); nothing was sent. Send the arguments as one JSON "
                    f"object" + (" with the keys of the tool's schema" if schema else "") + ".")
            self._log("tool_call", id=call["id"], name=name, arguments=raw, malformed="json", sent=False, text=text)
            return self._tool_message(call["id"], text), [], None
        problem = argument_problem(self.schemas, name, args)
        if problem == "unknown_tool":
            self.counts["unknown_tools"] += 1
        elif problem is not None:
            self.counts["malformed_args"] += 1
        if self.own_tools and name == CLAIM_TOOL:
            req = {"op": "claim", "args": args}
            self.counts["claims_sent"] += 1
        elif self.own_tools and name == READ_TOOL:
            req = {"op": "read", "name": args.get("uri") if isinstance(args, dict) else None}
            self.counts["reads"] += 1
        else:
            req = {"op": "call", "name": name, "args": args}
            self.counts["server_calls"] += 1
        resp, why = self.episode.request(req)
        if resp is None:
            ended = self.episode.ended_on_disk()
            text = "the robot's server did not answer" + ("; the episode has ended" if ended else f": {why}")
            self._log("tool_call", id=call["id"], name=name, arguments=raw, malformed=problem, sent=True, op=req["op"],
                      lost=why, text=text)
            return (self._tool_message(call["id"], text), [],
                    ("episode_ended", EXIT_ENDED) if ended else ("episode_lost", EXIT_LOST))
        self._note_budget(resp.get("budget"))
        episode = resp.get("episode")
        result = resp.get("result")
        refused = result.get("refused") if has_refusal(result) else None
        if refused is not None:
            self.counts["refusals"] += 1
            if refused.get("check") == "budget":
                self.budget_refused = True
        outcome = None
        if episode == "ended":
            text, outcome = "the episode has ended", ("episode_ended", EXIT_ENDED)
        else:
            replies = None if self.reply_budget is None else (self.max_turns - self.turn, self.max_turns)
            text = result_text(result, resp.get("budget"), stopped=episode == "stopped", replies=replies)
            if episode == "claimed":
                self.claim_by, self.claim = "model", (result or {}).get("claim")
                outcome = ("claim", EXIT_CLAIM)
            elif episode == "stopped":
                outcome = ("estop", EXIT_ENDED)
        frames = [(p, name) for p in resp.get("frames", []) if isinstance(p, str)]
        self._log("tool_call", id=call["id"], name=name, arguments=raw, malformed=problem, sent=True, op=req["op"],
                  episode=episode, refused=refused, result=trim_value(result), frames=[p for p, _ in frames])
        return self._tool_message(call["id"], text, compact_text(result)), frames, outcome

    def _frame_parts(self, path, lead):
        """(text and image parts, sent record), or (None, why) when a frame cannot be attached."""
        if not inside(path, self.ep_dir) or not os.path.isfile(path):
            return None, "it is not a file in the episode directory"
        try:
            url, size, source = encode_frame(path, MAX_SIDE_PX)
        except Exception as exc:
            return None, f"{type(exc).__name__}: {exc}"
        shown = f"{size[0]}x{size[1]} px" + ("" if size == source else f"; the file is {source[0]}x{source[1]} px")
        parts = [{"type": "text", "text": f"{lead}: {os.path.basename(path)} ({shown})"},
                 {"type": "image_url", "image_url": {"url": url}, "_path": path}]
        self.counts["frames_attached"] += 1
        self.shown_frames[path] = self.turn
        return parts, {"path": path, "sent_px": list(size), "file_px": list(source)}

    def _frames_message(self, frames):
        """The user message with this turn's frames. A frame already attached (a read of robot://journal or
        robot://world names every earlier frame again) is listed once by name, not attached again, so it cannot
        push the newest frames out of the --keep-images window."""
        parts, sent, again = [], [], []
        for path, label in frames:
            base = os.path.basename(path)
            if path in self.shown_frames:
                if base not in again:
                    again.append(base)
                continue
            attached, record = self._frame_parts(path, f"frame from {label}")
            if attached is not None:
                parts.extend(attached)
                sent.append(record)
                continue
            why = record
            parts.append({"type": "text", "text": f"frame from {label}: {base} could not be attached ({why})"})
            self.counts["frames_not_attached"] += 1
        if again:
            self.counts["frames_repeated"] += len(again)
            names = ", ".join(again[:8]) + (f" and {len(again) - 8} more" if len(again) > 8 else "")
            parts.append({"type": "text", "text": f"{len(again)} frame(s) named here were attached earlier and are "
                                                  f"not attached again: {names}"})
        self._log("frames", attached=sent, repeated=again, parts=len(parts))
        return {"role": "user", "content": parts}

    def _forced_claim(self, statement, reason):
        args = {"statement": statement, "success": False}
        resp, why = self.episode.request({"op": "claim", "args": args})
        self.counts["claims_sent"] += 1
        self._log("forced_claim", reason=reason, args=args, response=resp, lost=why)
        resends = 0
        # a claim gate may answer instead of recording ("continues"): no turn is left, so send it again
        while resp is not None and resp.get("episode") == "continues" and resends < FORCED_CLAIM_RESENDS:
            resends += 1
            resp, why = self.episode.request({"op": "claim", "args": args})
            self._log("forced_claim", reason=reason, args=args, response=resp, lost=why, resend=resends)
        if resp is not None and resp.get("episode") == "claimed":
            self.claim_by, self.claim = "operator", args
            return self._finish(reason, EXIT_CLAIM)
        if (resp is not None and resp.get("episode") == "ended") or (resp is None and self.episode.ended_on_disk()):
            return self._finish(reason, EXIT_ENDED)
        return self._finish(reason, EXIT_LOST, claim_response=resp)


# ---------------------------------------------------------------------------------------------------------------
# Command line


class _Parser(argparse.ArgumentParser):
    def error(self, message):
        self.print_usage(sys.stderr)
        sys.stderr.write(f"{self.prog}: error: {message}\n")
        raise SystemExit(EXIT_USAGE)


def build_parser():
    p = _Parser(prog="python -m bench.operators.chat_operator",
                description="Let an OpenAI-compatible chat model operate one rcpx episode.")
    p.add_argument("--ep", required=True, help="the episode directory (EPDIR/rcp.sock must answer)")
    p.add_argument("--base-url", required=True, help="the endpoint's API root, for example http://127.0.0.1:8000/v1")
    p.add_argument("--model", required=True, help="the model name the endpoint serves")
    p.add_argument("--max-turns", type=int, default=None, help="most model replies (default 120)")
    p.add_argument("--reply-budget", type=int, default=None,
                   help="N model replies in all, each counted whatever it calls and each offered every tool; after "
                        "reply N the loop claims failure; said in the first message and in every tool result "
                        "(sets --max-turns; giving both with different numbers is an error)")
    p.add_argument("--one-call-per-reply", action="store_true",
                   help="run only the first tool call of each reply; the others are answered as not run")
    p.add_argument("--keep-images", type=int, default=6, help="images kept in the history (default 6)")
    p.add_argument("--temperature", type=float, default=0.2)
    p.add_argument("--seed", type=int, default=None, help="sampling seed sent with every request")
    p.add_argument("--context-tokens", type=int, default=32000, help="the model's context length (default 32000)")
    p.add_argument("--max-tokens", type=int, default=4096,
                   help="most tokens per reply, reasoning included (default 4096)")
    p.add_argument("--http-timeout", type=float, default=600.0, help="seconds to wait for one reply")
    p.add_argument("--socket-timeout", type=float, default=900.0, help="seconds to wait for the episode server")
    p.add_argument("--retry-wait", type=float, default=8.0,
                   help="seconds before the first retry after an endpoint failure, doubling, so the default "
                        "rides out about two minutes of a restarting server (0: no wait)")
    p.add_argument("--api-key-env", default=None,
                   help="name of an environment variable holding a bearer key (the key is never written down)")
    p.add_argument("--extra-body", default=None,
                   help="a JSON object merged into every request body, for example '{\"reasoning_effort\": \"low\", "
                        "\"top_k\": 20}'; a null value drops a key (default sends parallel_tool_calls false; "
                        "'{\"parallel_tool_calls\": null}' leaves it to the endpoint)")
    p.add_argument("--plain-schema", action="store_true",
                   help="send argument schemas without the keywords a strict endpoint may refuse (bounds said in "
                        "the descriptions); without it the loop switches after a 400 about the schemas")
    return p


def main(argv=None):
    ns = build_parser().parse_args(argv)
    max_turns = ns.reply_budget if ns.reply_budget is not None else (120 if ns.max_turns is None else ns.max_turns)
    problems = [text for bad, text in (
        (ns.reply_budget is not None and ns.reply_budget < 1, "--reply-budget must be at least 1"),
        (ns.reply_budget is not None and ns.max_turns is not None and ns.max_turns != ns.reply_budget,
         "--max-turns and --reply-budget differ (give one, or the same number)"),
        (ns.max_turns is not None and ns.max_turns < 1, "--max-turns must be at least 1"),
        (ns.keep_images < 0, "--keep-images must be >= 0"),
        (ns.context_tokens < 1000, "--context-tokens must be at least 1000"),
        (not 1 <= ns.max_tokens < ns.context_tokens, "--max-tokens must be from 1 to below --context-tokens"),
        (ns.retry_wait < 0, "--retry-wait must be >= 0"), (not os.path.isdir(ns.ep), "--ep must be a directory"))
        if bad]
    extra_body, why = parse_extra_body(ns.extra_body)
    if why:
        problems.append(why)
    api_key = None
    if ns.api_key_env:
        api_key = os.environ.get(ns.api_key_env)
        if not api_key:
            problems.append(f"the environment variable {ns.api_key_env} is not set")
    try:
        endpoint = ChatEndpoint(ns.base_url, ns.http_timeout, api_key)
    except ValueError as exc:
        problems.append(str(exc))
    if problems:
        sys.stderr.write("".join(f"chat_operator: {p}\n" for p in problems))
        return EXIT_USAGE
    info = {"model": ns.model, "base_url": ns.base_url, "max_turns": max_turns, "keep_images": ns.keep_images,
            "temperature": ns.temperature, "seed": ns.seed, "context_tokens": ns.context_tokens,
            "max_tokens": ns.max_tokens, "api_key_env": ns.api_key_env,
            "body_extra": {**DEFAULT_BODY, **extra_body}, "plain_schema": ns.plain_schema,
            "reply_budget": ns.reply_budget, "one_call_per_reply": ns.one_call_per_reply}
    op = ChatOperator(ns.ep, endpoint, ns.model, max_turns=max_turns, keep_images=ns.keep_images,
                      temperature=ns.temperature, seed=ns.seed, context_tokens=ns.context_tokens,
                      max_tokens=ns.max_tokens, retry_wait_s=ns.retry_wait, socket_timeout_s=ns.socket_timeout,
                      run_info=info, extra_body=extra_body, plain_schema=ns.plain_schema,
                      reply_budget=ns.reply_budget, one_call=ns.one_call_per_reply)
    return op.run()


if __name__ == "__main__":
    raise SystemExit(main())
