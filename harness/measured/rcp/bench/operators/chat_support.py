# SPDX-License-Identifier: AGPL-3.0-only
"""Parts of the chat operator (bench/operators/chat_operator.py): the episode socket client, the OpenAI-compatible
chat endpoint over urllib, frame encoding, result trimming and the conversation history with its image cap and
context trim.

Standard library only, except pillow for frames (imported when a frame is attached). Nothing here reads a clock:
every decision depends on the conversation alone, so two runs against the same replies are identical.
"""
import base64
import io
import ipaddress
import json
import os
import socket
import urllib.error
import urllib.parse
import urllib.request

from rcpx.server import END_NAME, SOCK_NAME

DROPPED = "[frame dropped to save context]"
IMAGE_MODES = ("swap", "append")   # swap: the published rule; append: older turns are never edited for a picture
PREFIX_MARK = {"type": "ephemeral"}  # the cache mark on the first user message (--cache-prefix-mark)
SHORTENED = "[older result shortened to save context] "
CUT_REPLY = " [rest of this older reply trimmed to save context]"
MAX_ITEMS = 24           # a list longer than this is cut to its first KEEP_ITEMS items and a note
KEEP_ITEMS = 12
MAX_STR = 1500           # characters kept of one long string in a result
MAX_RESULT_CHARS = 16000  # characters of one tool message (a result with a refusal is never cut)
COMPACT_CHARS = 400      # characters of an older result's short form (its refusal is kept whole)
REPLY_KEEP_CHARS = 400   # characters kept of an older reply's text when the context is tight
IMAGE_TOKENS = 800       # estimated tokens for one attached frame (640 px on the long side)
MESSAGE_TOKENS = 8       # estimated overhead per message
CHARS_PER_TOKEN = 3      # conservative for JSON-heavy text
PROTECT_TURNS = 2        # the newest turns are never shortened or dropped
OWN_KEYS = ("model", "messages", "tools", "tool_choice", "temperature", "seed", "max_tokens")  # not in --extra-body


def parse_extra_body(text):
    """(dict, None) or (None, why) for --extra-body."""
    if text is None:
        return {}, None
    try:
        value = json.loads(text)
    except ValueError as exc:
        return None, f"--extra-body is not JSON ({exc})"
    if not isinstance(value, dict):
        return None, "--extra-body must be a JSON object"
    own = sorted(set(value) & set(OWN_KEYS))
    if own:
        return None, f"--extra-body may not set {', '.join(own)} (the loop or its own flags set them)"
    return value, None


# ---------------------------------------------------------------------------------------------------------------
# The episode socket (the same protocol as rcpx/cli.py: one JSON line per connection, one JSON line back)


class OperatorJournal:
    """operator.jsonl: one JSON object per line {"seq", "kind", ...}, flushed per line, appended."""

    def __init__(self, path):
        self.seq = 0
        self._f = open(path, "a", encoding="utf-8")

    def append(self, kind, **fields):
        record = {"seq": self.seq, "kind": kind, **fields}
        self._f.write(json.dumps(record, sort_keys=True, ensure_ascii=False, default=str) + "\n")
        self._f.flush()
        self.seq += 1

    def close(self):
        self._f.close()


class EpisodeClient:
    def __init__(self, ep_dir, timeout_s=900.0):
        self.ep_dir = os.path.abspath(ep_dir)
        self.sock_path = os.path.join(self.ep_dir, SOCK_NAME)
        self.timeout_s = float(timeout_s)

    def ended_on_disk(self):
        """True when the episode server has written its end record."""
        return os.path.exists(os.path.join(self.ep_dir, END_NAME))

    def _open(self):
        return socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

    def request(self, req):
        """(response dict, None) when the server answers, else (None, why)."""
        s = self._open()
        s.settimeout(self.timeout_s)
        buf = b""
        try:
            s.connect(self.sock_path)
            s.sendall((json.dumps(req) + "\n").encode("utf-8"))
            try:
                s.shutdown(socket.SHUT_WR)
            except OSError:  # the server answered an episode-ending request and closed first; the answer is
                pass         # already in the receive buffer, so read it rather than drop it
            while b"\n" not in buf:
                chunk = s.recv(65536)
                if not chunk:
                    break
                buf += chunk
        except OSError as exc:
            return None, f"{type(exc).__name__}: {exc}"
        finally:
            s.close()
        line = buf.split(b"\n", 1)[0]
        if not line.strip():
            return None, "the episode server closed the connection without an answer"
        try:
            response = json.loads(line.decode("utf-8", "replace"))
        except ValueError as exc:
            return None, f"the episode server's answer is not JSON ({exc})"
        if not isinstance(response, dict):
            return None, "the episode server's answer is not a JSON object"
        return response, None


# ---------------------------------------------------------------------------------------------------------------
# The chat endpoint


class EndpointError(Exception):
    """One failed chat completion: kind is unreachable, http_status, bad_json or bad_reply."""

    def __init__(self, kind, why, status=None):
        super().__init__(why)
        self.kind, self.why, self.status = kind, why, status


def _loopback(host):
    if host in ("localhost",):
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def completions_url(base_url):
    """base_url (for example http://127.0.0.1:8000/v1) plus /chat/completions unless it already ends so."""
    url = str(base_url).rstrip("/")
    return url if url.endswith("/chat/completions") else url + "/chat/completions"


class ChatEndpoint:
    """POST /chat/completions with urllib. A loopback endpoint bypasses any proxy settings."""

    def __init__(self, base_url, timeout_s=600.0, api_key=None):
        self.url = completions_url(base_url)
        parts = urllib.parse.urlparse(self.url)
        if parts.scheme not in ("http", "https") or not parts.hostname:
            raise ValueError(f"the base URL must be http or https with a host, got {base_url!r}")
        handlers = [urllib.request.ProxyHandler({})] if _loopback(parts.hostname) else []
        self._opener = urllib.request.build_opener(*handlers)
        self.timeout_s = float(timeout_s)
        self._api_key = api_key  # sent as a bearer header, never journaled

    def complete(self, body):
        """The first choice of one completion: {"message", "finish_reason", "usage"}. Raises EndpointError."""
        headers = {"Content-Type": "application/json", "Accept": "application/json"}
        if self._api_key:
            headers["Authorization"] = "Bearer " + self._api_key
        req = urllib.request.Request(self.url, data=json.dumps(body).encode("utf-8"), headers=headers,
                                     method="POST")
        try:
            with self._opener.open(req, timeout=self.timeout_s) as resp:
                raw = resp.read()
        except urllib.error.HTTPError as exc:
            try:
                excerpt = exc.read(2000).decode("utf-8", "replace")
            except Exception:  # the error body is a courtesy; its absence is not a second failure
                excerpt = ""
            raise EndpointError("http_status", f"HTTP {exc.code}: {excerpt.strip()[:500]}", exc.code) from None
        except (urllib.error.URLError, OSError) as exc:
            reason = getattr(exc, "reason", exc)
            raise EndpointError("unreachable", f"{type(exc).__name__}: {reason}") from None
        try:
            data = json.loads(raw.decode("utf-8", "replace"))
        except ValueError as exc:
            raise EndpointError("bad_json", f"the endpoint's answer is not JSON ({exc})") from None
        if not isinstance(data, dict):
            raise EndpointError("bad_reply", "the endpoint's answer is not a JSON object")
        if data.get("error"):
            raise EndpointError("bad_reply", "the endpoint answered with an error: " + json.dumps(data["error"])[:500])
        choices = data.get("choices")
        if not isinstance(choices, list) or not choices or not isinstance(choices[0], dict) \
                or not isinstance(choices[0].get("message"), dict):
            raise EndpointError("bad_reply", "the endpoint's answer has no choices[0].message")
        first = choices[0]
        return {"message": first["message"], "finish_reason": first.get("finish_reason"),
                "usage": data.get("usage") if isinstance(data.get("usage"), dict) else None}


# ---------------------------------------------------------------------------------------------------------------
# Frames


def inside(path, root):
    """True when path (symlinks resolved) lies inside the directory root."""
    real, base = os.path.realpath(path), os.path.realpath(root)
    return real == base or real.startswith(base.rstrip(os.sep) + os.sep)


def encode_frame(path, max_side=640, quality=85):
    """(data URL of a JPEG whose longest side is at most max_side, (w, h) sent, (w, h) of the file)."""
    from PIL import Image
    with Image.open(path) as im:
        im.load()
        source = im.size
        img = im.convert("RGB")
    w, h = img.size
    if max(w, h) > max_side:
        if w >= h:
            size = (max_side, max(1, round(h * max_side / w)))
        else:
            size = (max(1, round(w * max_side / h)), max_side)
        img = img.resize(size, Image.Resampling.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode("ascii"), img.size, source


# ---------------------------------------------------------------------------------------------------------------
# Results as text


def trim_value(obj, in_refusal=False):
    """A copy with very long lists and strings cut, each cut marked; nothing under a "refused" key is cut."""
    if isinstance(obj, dict):
        return {k: trim_value(v, in_refusal or k == "refused") for k, v in obj.items()}
    if isinstance(obj, list):
        items = [trim_value(v, in_refusal) for v in obj]
        if not in_refusal and len(items) > MAX_ITEMS:
            items = items[:KEEP_ITEMS] + [f"... {len(items) - KEEP_ITEMS} more items trimmed"]
        return items
    if isinstance(obj, str) and not in_refusal and len(obj) > MAX_STR:
        return obj[:MAX_STR] + f"... ({len(obj) - MAX_STR} more characters trimmed)"
    return obj


def has_refusal(result):
    return isinstance(result, dict) and isinstance(result.get("refused"), dict)


def budget_line(budget, replies=None):
    if replies is not None:
        left, total = replies
        line = f"budget: {left} of {total} replies left"
        if isinstance(budget, dict) and "sim_seconds_left" in budget:
            line += f" and {budget['sim_seconds_left']} s of simulated time left"
        return line
    if not isinstance(budget, dict) or "calls_left" not in budget:
        return None
    return (f"budget: {budget.get('calls_left')} calls and {budget.get('sim_seconds_left')} s of simulated time "
            f"left")


def result_text(result, budget=None, stopped=False, replies=None):
    """The tool message for one server answer: the result as JSON (trimmed), then the budget left."""
    text = json.dumps(trim_value(result), ensure_ascii=False)
    if len(text) > MAX_RESULT_CHARS and not has_refusal(result):
        text = text[:MAX_RESULT_CHARS] + f" ... (result trimmed: {len(text) - MAX_RESULT_CHARS} more characters)"
    lines = [text]
    line = budget_line(budget, replies)
    if line:
        lines.append(line)
    if stopped:
        lines.append("the episode has stopped: no more calls will run")
    return "\n".join(lines)


def _small(value):
    if value is None or isinstance(value, (bool, int, float)):
        return True
    if isinstance(value, str):
        return len(value) <= 80
    if isinstance(value, list):
        return len(value) <= 6 and all(v is None or isinstance(v, (bool, int, float)) for v in value)
    return False


def compact_text(result):
    """The short form of an older result: its refusal whole, else its small top-level fields."""
    if has_refusal(result):
        return SHORTENED + json.dumps({"refused": result["refused"]}, ensure_ascii=False)
    if isinstance(result, dict):
        keep = {k: v for k, v in result.items() if _small(v)}
    else:
        keep = result if _small(result) else "(a large value)"
    text = json.dumps(keep, ensure_ascii=False)
    if len(text) > COMPACT_CHARS:
        text = text[:COMPACT_CHARS] + " ..."
    return SHORTENED + text


# ---------------------------------------------------------------------------------------------------------------
# The conversation history


def _tokens_of_text(text):
    return (len(text) + CHARS_PER_TOKEN - 1) // CHARS_PER_TOKEN


def message_tokens(m):
    """Estimated tokens of one message."""
    n = MESSAGE_TOKENS
    content = m.get("content")
    if isinstance(content, str):
        n += _tokens_of_text(content)
    elif isinstance(content, list):
        for part in content:
            if part.get("type") == "image_url":
                n += IMAGE_TOKENS
            else:
                n += _tokens_of_text(str(part.get("text", "")))
    if m.get("tool_calls"):
        n += _tokens_of_text(json.dumps(m["tool_calls"]))
    return n


def _clean(m):
    out = {k: v for k, v in m.items() if not k.startswith("_")}
    if isinstance(out.get("content"), list):
        out["content"] = [{k: v for k, v in p.items() if not k.startswith("_")} for p in out["content"]]
    return out


def _image_parts(m):
    content = m.get("content")
    if not isinstance(content, list):
        return []
    return [i for i, p in enumerate(content) if p.get("type") == "image_url"]


class History:
    """The system text and first user message (the prefix), then one list of messages per model turn.

    image_mode "swap" (the published rule): on every build each image beyond the newest keep_images becomes
    DROPPED inside its older turn. That edit changes the request's prefix, so an endpoint with a prefix cache
    writes the whole request again on every turn that brings a picture. image_mode "append": a build never edits
    an older turn for a picture; the request keeps its prefix byte for byte and only grows at its end. Pictures
    then go only in an epoch: when more than image_high_water are attached (0: no count limit), or when the
    estimate passes the token limit, every picture beyond the newest keep_images goes in that one build.

    Every trim is permanent and oldest first, so the conversation only ever loses its oldest detail: images
    beyond keep_images become DROPPED (swap; append: in epochs); over the token limit, older tool results take their short form ("_compact"),
    then older replies are cut, then images outside the newest turns go, then whole old turns go (a note in the
    first user message counts them). The newest PROTECT_TURNS turns are never shortened or dropped.
    """

    def __init__(self, system_text, first_user, keep_images, image_mode="swap", image_high_water=0,
                 mark_prefix=False):
        self.prefix = [{"role": "system", "content": system_text}, {"role": "user", "content": first_user}]
        self.turns = []
        self.keep_images = int(keep_images)
        if image_mode not in IMAGE_MODES:
            raise ValueError(f"image_mode must be one of {', '.join(IMAGE_MODES)}")
        self.image_mode = image_mode
        self.image_high_water = int(image_high_water)
        self.mark_prefix = bool(mark_prefix)
        self.counts = {"frames_dropped": 0, "results_shortened": 0, "replies_shortened": 0, "turns_dropped": 0,
                       "image_epochs": 0}

    def add_turn(self, messages):
        self.turns.append(list(messages))

    def add_user_text(self, text):
        """Add text to the newest message if it is a user message, else as a new user message."""
        target = self.turns[-1] if self.turns else self.prefix
        last = target[-1]
        if last["role"] != "user":
            target.append({"role": "user", "content": text})
        elif isinstance(last["content"], list):
            last["content"].append({"type": "text", "text": text})
        else:
            last["content"] = f"{last['content']}\n\n{text}"

    def images(self):
        return sum(len(_image_parts(m)) for turn in [self.prefix] + self.turns for m in turn)

    def _drop_images(self, keep, turns):
        """Replace every image beyond the newest keep in turns with DROPPED; returns how many went."""
        seen = dropped = 0
        for turn in reversed(turns):
            for m in reversed(turn):
                for i in reversed(_image_parts(m)):
                    seen += 1
                    if seen > keep:
                        m["content"][i] = {"type": "text", "text": DROPPED}
                        dropped += 1
        self.counts["frames_dropped"] += dropped
        return dropped

    def _image_epoch(self, acts):
        """Append mode's only picture edit: every picture beyond the newest keep_images goes in ONE build, so the
        request prefix changes once (one cache rewrite) and then stays byte for byte until the next epoch."""
        dropped = self._drop_images(self.keep_images, [self.prefix] + self.turns)
        if dropped:
            self.counts["image_epochs"] += 1
            acts["image_epochs"] = acts.get("image_epochs", 0) + 1
        return dropped

    def _prefix_messages(self):
        """The system text and the first user message. With mark_prefix both are sent as parts and carry a cache
        mark: one on the system text (the tools and the system text are the same in every episode of an arm, so
        a later episode can read them) and one on the last part of the first user message's fixed text (every
        request of this episode can then read the tools, the system text and the first message whatever was
        edited after them). The dropped-turns note goes after the mark, so the marked bytes never change."""
        dropped = self.counts["turns_dropped"]
        if not dropped and not self.mark_prefix:
            return list(self.prefix)
        note = f"[{dropped} earlier turns dropped to save context]" if dropped else None
        first = dict(self.prefix[1])
        if self.mark_prefix:
            parts = list(first["content"]) if isinstance(first["content"], list) \
                else [{"type": "text", "text": first["content"]}]
            parts[-1] = dict(parts[-1], cache_control=dict(PREFIX_MARK))
            first["content"] = parts + ([{"type": "text", "text": note}] if note else [])
        elif isinstance(first["content"], list):
            first["content"] = first["content"] + [{"type": "text", "text": note}]
        else:
            first["content"] = f"{first['content']}\n\n{note}"
        system = self.prefix[0]
        if self.mark_prefix and isinstance(system["content"], str):   # the tools and this text: every episode's
            system = dict(system, content=[{"type": "text", "text": system["content"],
                                            "cache_control": dict(PREFIX_MARK)}])
        return [system, first] + self.prefix[2:]

    def estimate(self, extra_tokens=0):
        msgs = self._prefix_messages() + [m for turn in self.turns for m in turn]
        return extra_tokens + sum(message_tokens(m) for m in msgs)

    def build(self, limit_tokens, extra_tokens=0):
        """(messages to send, what was trimmed now, estimated tokens). extra_tokens is the estimate for the tool
        definitions sent with the messages."""
        acts = {"frames_dropped": 0}
        if self.image_mode == "swap":
            acts["frames_dropped"] = self._drop_images(self.keep_images, [self.prefix] + self.turns)
        elif self.image_high_water > 0 and self.images() > self.image_high_water:
            acts["frames_dropped"] = self._image_epoch(acts)   # append: one edit for many pictures, not one each
        est = self.estimate(extra_tokens)
        if self.image_mode == "append" and est > limit_tokens and self.images() > self.keep_images:
            acts["frames_dropped"] += self._image_epoch(acts)  # the context is tight: pictures go first, at once
            est = self.estimate(extra_tokens)
        old = self.turns[:-PROTECT_TURNS] if len(self.turns) > PROTECT_TURNS else []
        if est > limit_tokens:
            for turn in old:
                for m in turn:
                    if est <= limit_tokens:
                        break
                    if m["role"] == "tool" and not m.get("_short") and "_compact" in m:
                        before = message_tokens(m)
                        m["content"], m["_short"] = m["_compact"], True
                        est += message_tokens(m) - before
                        self.counts["results_shortened"] += 1
                        acts["results_shortened"] = acts.get("results_shortened", 0) + 1
        if est > limit_tokens:
            for turn in old:
                for m in turn:
                    text = m.get("content")
                    if est <= limit_tokens:
                        break
                    if m["role"] == "assistant" and isinstance(text, str) and len(text) > REPLY_KEEP_CHARS \
                            and not text.endswith(CUT_REPLY):
                        before = message_tokens(m)
                        m["content"] = text[:REPLY_KEEP_CHARS] + CUT_REPLY
                        est += message_tokens(m) - before
                        self.counts["replies_shortened"] += 1
                        acts["replies_shortened"] = acts.get("replies_shortened", 0) + 1
        if est > limit_tokens and old:
            acts["frames_dropped"] += self._drop_images(0, [self.prefix] + old)
            est = self.estimate(extra_tokens)
        while est > limit_tokens and len(self.turns) > PROTECT_TURNS:
            self.turns.pop(0)
            self.counts["turns_dropped"] += 1
            acts["turns_dropped"] = acts.get("turns_dropped", 0) + 1
            est = self.estimate(extra_tokens)
        acts = {k: v for k, v in acts.items() if v}
        messages = [_clean(m) for m in self._prefix_messages() + [m for turn in self.turns for m in turn]]
        return messages, acts, est
