# SPDX-License-Identifier: AGPL-3.0-only
"""Shared parts of the run-cell tests: a stand-in model endpoint on the loopback address and the replies it gives.

The endpoint answers like Google's endpoint (the thought signature in tool_calls[].extra_content, no reasoning
text) or like OpenRouter (the signature as message.reasoning_details, a cost in usage). It picks the reply by the
number of assistant messages in the request, so every episode gets the same two replies in the same order. Set
as http_proxy it also stands in for a named host: a request to http://openrouter.ai/... reaches it with that
absolute URL as its path.
"""
import http.server
import json
import pathlib
import signal
import threading
import time

TESTS = pathlib.Path(__file__).resolve().parent
FIXTURES = TESTS / "fixtures"
KEY = "test-key-0123456789"
MODEL = "google/gemini-3.8-flash"

SIGNATURES = [{"google": {"thought_signature": "c2lnLTE="}}, {"google": {"thought_signature": "c2lnLTI="}}]
BLOCKS = [
    [{"type": "reasoning.text", "text": "Moving above the bowl.", "format": "google-gemini-v1", "index": 0},
     {"type": "reasoning.encrypted", "data": "QUJDRA==", "format": "google-gemini-v1", "id": "call_1", "index": 1}],
    [{"type": "reasoning.encrypted", "data": "RUZHSA==", "format": "google-gemini-v1", "id": "call_2", "index": 0}],
]
USAGE = [
    {"prompt_tokens": 1000, "completion_tokens": 200, "completion_tokens_details": {"reasoning_tokens": 150}},
    {"prompt_tokens": 2000, "completion_tokens": 100, "completion_tokens_details": {"reasoning_tokens": 50},
     "prompt_tokens_details": {"cached_tokens": 1000}},
]
COSTS = [0.0016, 0.0012]


def reply(kind, n):
    """The endpoint's answer to the request that holds n assistant messages (n is 0 or 1)."""
    call = {"id": f"call_{n + 1}", "type": "function",
            "function": {"name": "move_to", "arguments": json.dumps({"x": 0.1 * (n + 1)})}}
    message = {"role": "assistant", "content": None, "tool_calls": [call]}
    usage = dict(USAGE[n])
    if kind == "google":
        call["extra_content"] = SIGNATURES[n]
    else:
        message["reasoning"] = "Moving above the bowl."
        message["reasoning_details"] = BLOCKS[n]
        usage["cost"] = COSTS[n]
    return {"id": f"r{n}", "choices": [{"index": 0, "message": message, "finish_reason": "tool_calls"}],
            "usage": usage}


def block_signals():
    """In a helper thread: leave SIGINT, SIGTERM and SIGHUP to the main thread, as run-cell's own threads do."""
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT, signal.SIGTERM, signal.SIGHUP})


def _do_nothing(signum, frame):
    pass


def unignore_signals(test):
    """For the length of a test: each of SIGINT, SIGTERM and SIGHUP that this process ignores (the tests were
    started under nohup, or as a background job of a script) gets a handler that does nothing, so run-cell's guard
    takes it, as it does in a terminal, and is ignored again after the test."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        if signal.getsignal(sig) is signal.SIG_IGN:
            signal.signal(sig, _do_nothing)
            test.addCleanup(signal.signal, sig, signal.SIG_IGN)


class FakeEndpoint:
    """kind "google" or "openrouter"; seen holds (path, Authorization, raw body) of every request, and headers
    the (name, value) pairs of its headers as they arrived. answer, when set, is (status, headers, body) sent
    instead of a reply; delay is seconds to wait before answering; gate, when set, is a threading.Event each
    request waits for (up to 30 s) before it is answered."""

    def __init__(self, kind):
        self.kind, self.seen, self.headers, self.answer, self.delay, self.gate = kind, [], [], None, 0.0, None
        test = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_POST(self):
                body = self.rfile.read(int(self.headers.get("Content-Length") or 0))
                test.seen.append((self.path, self.headers.get("Authorization"), body))
                test.headers.append(list(self.headers.items()))
                if test.gate is not None:
                    test.gate.wait(30)
                time.sleep(test.delay)
                if test.answer is not None:
                    status, headers, text = test.answer
                else:
                    n = sum(1 for m in json.loads(body)["messages"] if m.get("role") == "assistant")
                    status, headers, text = 200, {"Content-Type": "application/json"}, \
                        json.dumps(reply(test.kind, n)).encode("utf-8")
                self.send_response(status)
                for name, value in headers.items():
                    self.send_header(name, value)
                self.send_header("Content-Length", str(len(text)))
                self.end_headers()
                self.wfile.write(text)

            def log_message(self, *a):
                pass

        self.server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.daemon_threads = True

        def serve():
            block_signals()
            self.server.serve_forever(poll_interval=0.05)

        self.thread = threading.Thread(target=serve, daemon=True)
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
