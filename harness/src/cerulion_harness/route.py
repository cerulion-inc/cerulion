# SPDX-License-Identifier: AGPL-3.0-only
"""What run-cell sends to the model endpoint on each route, and the loopback relay the episode talks to. Standard
library only.

Every request of the measured runs had {"reasoning_effort": "high"} merged into its body, and its history carried
the model's thought signature back with each tool call it came with: an endpoint that returns the signature in
tool_calls[].extra_content gets it back unchanged, and reads the earlier reasoning in again. OpenRouter returns
the same signature in another field, message.reasoning_details: a list of blocks, one of which names the tool
call it belongs to. When those blocks are not sent back, the model starts its reasoning over on every turn, and a
reply carries several times the reasoning tokens a reply of the measured runs carried. That was measured with
Gemini models, whose measured route returned the signature in extra_content. With the blocks sent back, an episode
through OpenRouter comes close to the measured condition, not to the token: once a reply's tool call differs from
the measured one, the episodes part. A model whose measured route returned no signature (the measured GPT runs went
through a route that returns none) had no earlier reasoning in its history there; through OpenRouter the relay
sends back whatever reasoning_details OpenRouter returns for it, which those runs did not have.

So every request of an episode passes through a relay on the loopback address inside run-cell:

    - a reply that carries reasoning_details is remembered by the ids of its tool calls;
    - a later request whose history holds an assistant message with one of those tool calls gets the blocks back
      on that message, unchanged;
    - the body of every other request and of every answer goes on as it came. Of the headers, only Content-Type,
      Accept and Authorization go on to the endpoint, and only Content-Type and Retry-After come back. An
      endpoint that returns no reasoning_details (Google's endpoint returns its signature in extra_content)
      receives the very bytes the operator sent;
    - a request that names no Content-Length, names one that is not a count, or sends its body in chunks is
      answered 411 and not sent on; a fault of the relay itself is answered 502 and counted (relay_errors);
    - every request goes to the URL given and to no other, with the key the operator sent: the relay follows no
      redirect (redirects.py), and passes a redirect answer back with its status and body but without its
      Location header, so the operator has no other address to send the key to.

The blocks are added after the operator has estimated the size of its history, which decides when it shortens old
turns (above 167,488 estimated tokens with the measured flags). That estimate counts a signature in extra_content
but not the blocks, so through OpenRouter it stays well below what the same history counts on Google's endpoint,
and the operator shortens old turns later than it would there, or not at all. Counting the blocks would not give
the measured point either: OpenRouter's blocks for a Gemini model are larger than the signature Google's endpoint
returns (they also hold a summary of the reasoning as text), so with them counted the operator would shorten old
turns earlier than it would there. The start line and summary.json state this with the route.

The JSON object merged into every request body (--extra-body) is {"reasoning_effort": "high"} on every route, and
through OpenRouter a model id that starts with google/ also gets {"provider": {"only": ["google-vertex"]}}:
OpenRouter's default routing sends such a model to Google AI Studio as well, which answers the same request
differently from Google Vertex, the backend the measured runs used. A --extra-body given on the command line is
sent as given.
"""
import http.client
import http.server
import ipaddress
import json
import os
import selectors
import signal
import threading
import urllib.error
import urllib.parse
import urllib.request

from cerulion_harness import redirects

DEFAULT_EXTRA_BODY = {"reasoning_effort": "high"}
GOOGLE_ON_OPENROUTER = {"only": ["google-vertex"]}
# The operator's own --http-timeout. The relay waits as long for the endpoint, so the operator's timeout decides.
HTTP_TIMEOUT_S = 600
# Headers that go on to the endpoint, and back to the operator; every other one (Location among them) is the
# relay's own.
REQUEST_HEADERS = ("content-type", "accept", "authorization")
ANSWER_HEADERS = ("content-type", "retry-after")
BLOCKS_NOTE = ("sent back on the assistant message whose tool calls they came with, outside the operator's context "
               "estimate")


def host_of(base_url):
    """The lower-case host name of a URL, or "" when it has none."""
    try:
        host = urllib.parse.urlparse(str(base_url)).hostname
    except ValueError:
        return ""
    return (host or "").lower()


def is_openrouter(host):
    return host == "openrouter.ai" or host.endswith(".openrouter.ai")


def extra_body_for(base_url, model, given=None):
    """The JSON text merged into every request body: the --extra-body given as it is, else this route's default."""
    if given is not None:
        return given
    body = dict(DEFAULT_EXTRA_BODY)
    if is_openrouter(host_of(base_url)) and str(model).startswith("google/"):
        body["provider"] = {"only": list(GOOGLE_ON_OPENROUTER["only"])}
    return json.dumps(body)


def shown_url(base_url):
    """base_url as summary.json records it: without a user name, password, query or fragment, any of which can
    hold a key."""
    try:
        parts = urllib.parse.urlsplit(str(base_url))
        netloc = parts.hostname or ""
        if ":" in netloc:
            netloc = f"[{netloc}]"
        if parts.port is not None:
            netloc += f":{parts.port}"
    except ValueError:
        return ""
    return urllib.parse.urlunsplit((parts.scheme, netloc, parts.path, "", ""))


def condition(base_url, extra_text, given):
    """The route condition the start line prints and summary.json records. extra_text must be a JSON object."""
    extra = json.loads(extra_text)
    provider = extra.get("provider")
    return {
        "base_url": shown_url(base_url),
        "host": host_of(base_url),
        "extra_body": extra,
        "extra_body_from": "--extra-body" if given is not None else "the default for this route",
        "provider_only": provider.get("only") if isinstance(provider, dict) else None,
        "reasoning_effort": extra.get("reasoning_effort"),
        "reasoning_blocks": BLOCKS_NOTE,
    }


def condition_line(cond):
    """One line: the host, the object merged into every body, and what happens to reasoning blocks."""
    return (f"route {cond['host'] or '(no host)'}: every request body adds {json.dumps(cond['extra_body'])} "
            f"({cond['extra_body_from']}); reasoning blocks a reply carries are {BLOCKS_NOTE}")


def completions_url(base_url):
    """base_url plus /chat/completions unless it already ends so (the operator's own rule)."""
    url = str(base_url).rstrip("/")
    return url if url.endswith("/chat/completions") else url + "/chat/completions"


def loopback(host):
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def blocks_of(answer):
    """{tool call id: reasoning_details} of one endpoint answer (bytes), or {} when it carries none."""
    try:
        doc = json.loads(answer)
    except (ValueError, RecursionError):   # not JSON, or nested deeper than the parser goes
        return {}
    choices = doc.get("choices") if isinstance(doc, dict) else None
    if not isinstance(choices, list) or not choices or not isinstance(choices[0], dict):
        return {}
    message = choices[0].get("message")
    if not isinstance(message, dict):
        return {}
    details, calls = message.get("reasoning_details"), message.get("tool_calls")
    if not isinstance(details, list) or not details or not isinstance(calls, list):
        return {}
    ids = [c.get("id") for c in calls if isinstance(c, dict)]
    return {i: details for i in ids if isinstance(i, str) and i}


def with_blocks(body, blocks):
    """(request bytes, how many assistant messages got their blocks back). An assistant message gets the blocks of
    the first of its tool calls that has some, unless it carries reasoning_details already. Nothing to add: the
    bytes as they came. Something added: the body as json.dumps writes it, which is how the operator wrote it, so
    the added field is the only change."""
    if not blocks:
        return body, 0
    try:
        doc = json.loads(body)
    except (ValueError, RecursionError):   # not JSON, or nested deeper than the parser goes
        return body, 0
    messages = doc.get("messages") if isinstance(doc, dict) else None
    if not isinstance(messages, list):
        return body, 0
    given = 0
    for message in messages:
        if not isinstance(message, dict) or message.get("role") != "assistant" or "reasoning_details" in message:
            continue
        calls = message.get("tool_calls")
        for call in calls if isinstance(calls, list) else []:
            cid = call.get("id") if isinstance(call, dict) else None
            if isinstance(cid, str) and cid in blocks:
                message["reasoning_details"] = blocks[cid]
                given += 1
                break
    if not given:
        return body, 0
    try:
        return json.dumps(doc).encode("utf-8"), given
    except RecursionError:
        return body, 0


def content_length(headers):
    """The length of a request's body, or None when it names no Content-Length, names one that is not a count,
    or sends its body in chunks."""
    text = headers.get("Content-Length")
    if headers.get("Transfer-Encoding") is not None or text is None or not text.strip().isdigit():
        return None
    return int(text)


class _QuietServer(http.server.ThreadingHTTPServer):
    def handle_error(self, request, client_address):   # a request that broke off prints nothing to the terminal
        pass


class Relay:
    """The loopback endpoint an episode's operator talks to; see the module text. Use as a context manager: it
    serves from entering to leaving. on_request() is called as each request arrives, before it goes on. Every
    request goes to completions_url(base_url), whatever path or host the request line names."""

    def __init__(self, base_url, timeout_s=HTTP_TIMEOUT_S, on_request=None):
        self.url = completions_url(base_url)
        parts = urllib.parse.urlparse(self.url)
        if parts.scheme not in ("http", "https") or not parts.hostname:
            raise ValueError(f"the base URL must be http or https with a host, got {base_url!r}")
        handlers = [redirects.Refuse()] + ([urllib.request.ProxyHandler({})] if loopback(parts.hostname) else [])
        self._opener = urllib.request.build_opener(*handlers)
        self.timeout_s = float(timeout_s)
        self.on_request = on_request
        self._blocks = {}
        self._lock = threading.Lock()
        self.counts = {"requests": 0, "not_reached": 0, "replies_with_reasoning_blocks": 0,
                       "requests_carrying_blocks": 0, "relay_errors": 0}
        relay = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_POST(self):
                length = content_length(self.headers)
                if length is None:   # its body cannot be read whole: nothing is sent on
                    self._answer(411, {"Content-Type": "text/plain"}, b"the relay takes a body with a "
                                                                      b"Content-Length only\n")
                    return
                data = self.rfile.read(length) if length > 0 else b""
                if not self.path.split("?")[0].rstrip("/").endswith("/chat/completions"):
                    self._answer(404, {"Content-Type": "text/plain"}, b"not found\n")
                    return
                try:
                    answer = relay.exchange(data, dict(self.headers.items()))
                except Exception as exc:   # a fault of the relay itself: answered and counted, never a silent close
                    with relay._lock:
                        relay.counts["relay_errors"] += 1
                    self._answer(502, {"Content-Type": "text/plain"},
                                 f"the relay could not pass this request on ({type(exc).__name__})\n".encode())
                    return
                if answer is not None:   # else the connection closes unanswered, as the endpoint's would have
                    self._answer(*answer)

            def _answer(self, status, headers, body):
                try:
                    self.send_response(status)
                    for name, value in headers.items():
                        self.send_header(name, value)
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except OSError:   # the operator stopped waiting
                    pass

            def log_message(self, *args):   # the terminal is the user's
                pass

        self._server = _QuietServer(("127.0.0.1", 0), Handler)
        self._server.daemon_threads = True
        # handle_request() waits this long for a connection the outer select saw: one that went away meanwhile
        # must not keep _serve() from seeing close().
        self._server.timeout = 0
        self._wake = os.pipe()
        self._thread = None

    @property
    def base_url(self):
        return f"http://127.0.0.1:{self._server.server_address[1]}/v1"

    def __enter__(self):
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def _serve(self):
        # This thread and the request threads it starts block the signals run-cell stops on, so the kernel hands
        # those to the main thread, which waits for the episode driver and acts on them.
        if hasattr(signal, "pthread_sigmask"):
            signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT, signal.SIGTERM, signal.SIGHUP})
        with selectors.DefaultSelector() as sel:
            sel.register(self._server, selectors.EVENT_READ)
            sel.register(self._wake[0], selectors.EVENT_READ)
            while True:   # wakes for a connection or for close(), never on a timer
                ready = [key.fileobj for key, _ in sel.select()]
                if self._wake[0] in ready:
                    return
                self._server.handle_request()

    def close(self):
        if self._thread is not None:
            os.write(self._wake[1], b"x")
            self._thread.join()
            self._thread = None
        if self._wake is not None:
            for fd in self._wake:
                os.close(fd)
            self._wake = None
            self._server.server_close()

    def exchange(self, data, headers):
        """(status, headers, body) of the endpoint's answer to one request, or None when it could not be reached."""
        if self.on_request is not None:
            try:
                self.on_request()
            except Exception:   # a progress line that cannot be written never costs the request
                pass
        with self._lock:
            blocks = dict(self._blocks)
            self.counts["requests"] += 1
        data, given = with_blocks(data, blocks)
        send = {k: v for k, v in headers.items() if k.lower() in REQUEST_HEADERS}
        req = urllib.request.Request(self.url, data=data, headers=send, method="POST")
        try:
            with self._opener.open(req, timeout=self.timeout_s) as resp:
                status, got, body = resp.status, resp.headers, resp.read()
        except urllib.error.HTTPError as exc:
            status, got = exc.code, exc.headers
            try:
                body = exc.read()
            except (OSError, http.client.HTTPException):
                body = b""
        except (urllib.error.URLError, OSError, http.client.HTTPException, ValueError):
            with self._lock:
                self.counts["not_reached"] += 1
            return None
        found = blocks_of(body) if status == 200 else {}
        with self._lock:
            self._blocks.update(found)
            self.counts["replies_with_reasoning_blocks"] += int(bool(found))
            self.counts["requests_carrying_blocks"] += int(bool(given))
        keep = {k: v for k, v in (got.items() if got is not None else []) if k.lower() in ANSWER_HEADERS}
        return status, keep, body
