# SPDX-License-Identifier: AGPL-3.0-only
"""What run-cell sends on each route: the object merged into every request body, the operator flags, and the
relay that carries reasoning blocks back. Standard library only: the endpoints are servers on the loopback
address inside the test. Every expected body is written out by hand below, never produced by the code under
test."""
import contextlib
import http.client
import io
import json
import os
import pathlib
import socket
import sys
import threading
import time
import unittest
import urllib.error
import urllib.parse
import urllib.request
from unittest import mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from cerulion_harness import cli, route  # noqa: E402
from run_cell_support import BLOCKS, KEY, MODEL, FakeEndpoint, keep_redirects, reply  # noqa: E402

GOOGLE = "https://aiplatform.googleapis.com/v1/projects/p/locations/global/endpoints/openapi"
OPENROUTER = "https://openrouter.ai/api/v1"
MEASURED_BODY = '{"reasoning_effort": "high"}'
PINNED_BODY = '{"reasoning_effort": "high", "provider": {"only": ["google-vertex"]}}'

# What the operator sends at turn 2 through OpenRouter (it carries extra_content only, and OpenRouter sends none),
# and what the endpoint must receive: the same bytes with the first reply's blocks on its assistant message.
OPERATOR_TURN_2 = (
    r'{"model": "google/gemini-3.8-flash", "messages": [{"role": "user", "content": "Task: pick the bowl."}, '
    r'{"role": "assistant", "content": "", "tool_calls": [{"id": "call_1", "type": "function", "function": '
    r'{"name": "move_to", "arguments": "{\"x\": 0.1}"}}]}, {"role": "tool", "tool_call_id": "call_1", '
    r'"content": "moved"}], "reasoning_effort": "high", "provider": {"only": ["google-vertex"]}}').encode()
ENDPOINT_TURN_2 = (
    r'{"model": "google/gemini-3.8-flash", "messages": [{"role": "user", "content": "Task: pick the bowl."}, '
    r'{"role": "assistant", "content": "", "tool_calls": [{"id": "call_1", "type": "function", "function": '
    r'{"name": "move_to", "arguments": "{\"x\": 0.1}"}}], "reasoning_details": [{"type": "reasoning.text", '
    r'"text": "Moving above the bowl.", "format": "google-gemini-v1", "index": 0}, {"type": '
    r'"reasoning.encrypted", "data": "QUJDRA==", "format": "google-gemini-v1", "id": "call_1", "index": 1}]}, '
    r'{"role": "tool", "tool_call_id": "call_1", "content": "moved"}], "reasoning_effort": "high", "provider": '
    r'{"only": ["google-vertex"]}}').encode()
# Turn 2 on Google's endpoint: the signature rides in extra_content; the relay must change nothing.
GOOGLE_TURN_2 = (
    r'{"model": "google/gemini-3.8-flash", "messages": [{"role": "user", "content": "Task: pick the bowl."}, '
    r'{"role": "assistant", "content": "", "tool_calls": [{"id": "call_1", "type": "function", "function": '
    r'{"name": "move_to", "arguments": "{\"x\": 0.1}"}, "extra_content": {"google": {"thought_signature": '
    r'"c2lnLTE="}}}]}, {"role": "tool", "tool_call_id": "call_1", "content": "moved"}], "reasoning_effort": '
    r'"high"}').encode()
TURN_1 = b'{"model": "google/gemini-3.8-flash", "messages": [{"role": "user", "content": "Task: pick the bowl."}]}'


class ExtraBody(unittest.TestCase):
    """The object merged into every request body, by route."""

    def test_google_endpoint_gets_the_measured_body(self):
        self.assertEqual(route.extra_body_for(GOOGLE, MODEL), MEASURED_BODY)

    def test_a_loopback_relay_and_any_other_host_get_the_measured_body(self):
        for url in ("http://127.0.0.1:18790/vertex", "http://localhost:8000/v1", "https://api.openai.com/v1"):
            self.assertEqual(route.extra_body_for(url, MODEL), MEASURED_BODY, url)

    def test_openrouter_pins_a_google_model_to_vertex(self):
        self.assertEqual(route.extra_body_for(OPENROUTER, MODEL), PINNED_BODY)

    def test_an_openrouter_subdomain_or_capitals_are_openrouter(self):
        for url in ("https://eu.openrouter.ai/api/v1", "https://OpenRouter.AI/api/v1/"):
            self.assertEqual(route.extra_body_for(url, MODEL), PINNED_BODY, url)

    def test_openrouter_leaves_a_model_of_another_maker_unpinned(self):
        for model in ("anthropic/claude-sonnet-5.5", "gemini-3.8-flash", "Google/gemini-3.8-flash", ""):
            self.assertEqual(route.extra_body_for(OPENROUTER, model), MEASURED_BODY, model)

    def test_a_host_that_only_looks_like_openrouter_is_not_openrouter(self):
        for url in ("https://openrouter.ai.example.com/v1", "https://notopenrouter.ai/v1",
                    "https://example.com/openrouter.ai/v1", "https://openrouter.ai@example.com/v1"):
            self.assertEqual(route.extra_body_for(url, MODEL), MEASURED_BODY, url)

    def test_a_given_extra_body_is_sent_as_given_on_every_route(self):
        for url in (OPENROUTER, GOOGLE):
            self.assertEqual(route.extra_body_for(url, MODEL, '{"reasoning_effort": "low"}'),
                             '{"reasoning_effort": "low"}')
            self.assertEqual(route.extra_body_for(url, MODEL, "{}"), "{}")

    def test_a_url_with_no_host_gets_the_measured_body(self):
        for url in ("", "not a url", "http://[::1", None):
            self.assertEqual(route.host_of(url), "", url)
            self.assertEqual(route.extra_body_for(url, MODEL), MEASURED_BODY, url)

    def test_the_same_route_gives_the_same_text_and_leaves_the_defaults_alone(self):
        first, second = route.extra_body_for(OPENROUTER, MODEL), route.extra_body_for(OPENROUTER, MODEL)
        self.assertEqual(first, second)
        self.assertEqual(route.DEFAULT_EXTRA_BODY, {"reasoning_effort": "high"})
        self.assertEqual(route.GOOGLE_ON_OPENROUTER, {"only": ["google-vertex"]})


class OperatorFlags(unittest.TestCase):
    """The --op-args string the episode driver hands the operator, by route."""

    def test_google_endpoint_gets_the_flags_of_the_measured_runs(self):
        self.assertEqual(cli.op_args(route.extra_body_for(GOOGLE, MODEL)),
                         "--context-tokens 200000 --max-tokens 32000 --http-timeout 600 --temperature 1.0 "
                         "--retry-wait 20 --seed 1 --extra-body '{\"reasoning_effort\": \"high\"}'")

    def test_openrouter_adds_only_the_provider_pin(self):
        self.assertEqual(cli.op_args(route.extra_body_for(OPENROUTER, MODEL)),
                         "--context-tokens 200000 --max-tokens 32000 --http-timeout 600 --temperature 1.0 "
                         "--retry-wait 20 --seed 1 --extra-body '{\"reasoning_effort\": \"high\", \"provider\": "
                         "{\"only\": [\"google-vertex\"]}}'")


class Condition(unittest.TestCase):
    """What the start line prints and summary.json records about the route."""

    def test_openrouter_default(self):
        self.assertEqual(route.condition(OPENROUTER, PINNED_BODY, None), {
            "base_url": OPENROUTER, "host": "openrouter.ai",
            "extra_body": {"reasoning_effort": "high", "provider": {"only": ["google-vertex"]}},
            "extra_body_from": "the default for this route", "provider_only": ["google-vertex"],
            "reasoning_effort": "high",
            "reasoning_blocks": "sent back on the assistant message whose tool calls they came with, outside the "
                                "operator's context estimate"})

    def test_a_given_body_without_a_pin(self):
        cond = route.condition(GOOGLE, '{"reasoning": {"effort": "medium"}, "provider": "x"}', "given")
        self.assertEqual((cond["host"], cond["extra_body_from"], cond["provider_only"], cond["reasoning_effort"]),
                         ("aiplatform.googleapis.com", "--extra-body", None, None))

    def test_the_recorded_url_holds_no_user_name_password_query_or_fragment(self):
        for url, shown in (("https://user:secret@openrouter.ai/api/v1?key=secret#x", "https://openrouter.ai/api/v1"),
                           ("http://[::1]:8080/v1?api-version=1", "http://[::1]:8080/v1"),
                           ("http://127.0.0.1:9/v1", "http://127.0.0.1:9/v1"),
                           ("http://h:notaport/v1", "")):
            self.assertEqual(route.condition(url, MEASURED_BODY, None)["base_url"], shown, url)

    def test_the_line(self):
        self.assertEqual(route.condition_line(route.condition(GOOGLE, MEASURED_BODY, None)),
                         'route aiplatform.googleapis.com: every request body adds {"reasoning_effort": "high"} '
                         '(the default for this route); reasoning blocks a reply carries are sent back on the '
                         "assistant message whose tool calls they came with, outside the operator's context "
                         "estimate")


def answer(kind, n):
    return json.dumps(reply(kind, n)).encode()


class BlocksOf(unittest.TestCase):
    """Which blocks a reply carries, by tool call id."""

    def test_an_openrouter_reply_maps_its_call_to_its_blocks(self):
        self.assertEqual(route.blocks_of(answer("openrouter", 0)), {"call_1": BLOCKS[0]})

    def test_every_tool_call_of_the_reply_maps_to_the_same_blocks(self):
        doc = reply("openrouter", 0)
        doc["choices"][0]["message"]["tool_calls"].append({"id": "call_9", "type": "function"})
        self.assertEqual(route.blocks_of(json.dumps(doc).encode()), {"call_1": BLOCKS[0], "call_9": BLOCKS[0]})

    def test_a_google_reply_carries_none(self):
        self.assertEqual(route.blocks_of(answer("google", 0)), {})

    def test_a_reply_without_calls_or_blocks_or_ids_carries_none(self):
        for change in ({"reasoning_details": []}, {"tool_calls": []}, {"tool_calls": None},
                       {"tool_calls": [{"id": ""}, {"id": 7}, {"type": "function"}, "x"]}):
            doc = reply("openrouter", 0)
            doc["choices"][0]["message"].update(change)
            self.assertEqual(route.blocks_of(json.dumps(doc).encode()), {}, change)

    def test_an_answer_that_is_not_a_reply_carries_none(self):
        for text in (b"not json", b"\xff\xfe", b"[]", b'{"choices": "x"}', b'{"choices": []}', b'{"choices": [1]}',
                     b'{"choices": [{"message": "x"}]}', b'{"choices": [{"message": {"reasoning_details": "x", '
                                                         b'"tool_calls": [{"id": "a"}]}}]}'):
            self.assertEqual(route.blocks_of(text), {}, text)


class WithBlocks(unittest.TestCase):
    """Putting blocks back on the history."""

    def test_the_blocks_go_back_on_their_assistant_message(self):
        self.assertEqual(route.with_blocks(OPERATOR_TURN_2, {"call_1": BLOCKS[0]}), (ENDPOINT_TURN_2, 1))

    def test_with_nothing_remembered_the_bytes_are_the_same_object(self):
        out, given = route.with_blocks(GOOGLE_TURN_2, {})
        self.assertIs(out, GOOGLE_TURN_2)
        self.assertEqual(given, 0)

    def test_with_no_matching_call_the_bytes_are_unchanged(self):
        self.assertEqual(route.with_blocks(GOOGLE_TURN_2, {"call_7": BLOCKS[0]}), (GOOGLE_TURN_2, 0))

    def test_a_message_that_has_blocks_or_is_not_the_assistant_is_left_alone(self):
        body = {"messages": [{"role": "assistant", "content": "", "tool_calls": [{"id": "call_1"}],
                              "reasoning_details": ["mine"]},
                             {"role": "tool", "tool_call_id": "call_1", "content": "moved"},
                             {"role": "user", "content": "x", "tool_calls": [{"id": "call_1"}]},
                             {"role": "assistant", "content": "text only"}]}
        data = json.dumps(body).encode()
        self.assertEqual(route.with_blocks(data, {"call_1": BLOCKS[0]}), (data, 0))

    def test_the_first_call_with_blocks_decides_and_each_message_is_counted_once(self):
        first = {"role": "assistant", "tool_calls": [{"id": "call_0"}, {"id": "call_2"}, {"id": "call_1"}]}
        body = {"messages": [first, {"role": "assistant", "tool_calls": [{"id": "call_1"}]}]}
        out, given = route.with_blocks(json.dumps(body).encode(), {"call_1": BLOCKS[0], "call_2": BLOCKS[1]})
        doc = json.loads(out)
        self.assertEqual(given, 2)
        self.assertEqual(doc["messages"][0]["reasoning_details"], BLOCKS[1])
        self.assertEqual(doc["messages"][1]["reasoning_details"], BLOCKS[0])

    def test_a_body_of_the_wrong_shape_goes_on_as_it_came(self):
        blocks = {"call_1": BLOCKS[0]}
        for data in (b"not json", b"[1]", b'{"messages": "x"}', b'{"messages": [1, "x", null]}',
                     b'{"messages": [{"role": "assistant", "tool_calls": "x"}]}',
                     b'{"messages": [{"role": "assistant", "tool_calls": [1, {"id": 5}]}]}'):
            self.assertEqual(route.with_blocks(data, blocks), (data, 0), data)

    def test_the_same_history_gives_the_same_bytes(self):
        blocks = {"call_1": BLOCKS[0]}
        self.assertEqual(route.with_blocks(OPERATOR_TURN_2, blocks), route.with_blocks(OPERATOR_TURN_2, blocks))


def post(url, body, opener=None):
    """One request as the measured operator sends it: its three headers (bench/operators/chat_support.py)."""
    req = urllib.request.Request(url + "/chat/completions", data=body, method="POST",
                                 headers={"Content-Type": "application/json", "Accept": "application/json",
                                          "Authorization": "Bearer " + KEY})
    with (opener or urllib.request.build_opener(urllib.request.ProxyHandler({}))).open(req, timeout=30) as resp:
        return resp.status, resp.read()


def raw_endpoint(test, answer):
    """A server on the loopback address that answers one request with the bytes answer and closes: an answer no
    HTTP server would send. Returns its base URL; what it was sent is test.raw_seen."""
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    test.addCleanup(listener.close)
    test.raw_seen = []

    def serve():
        conn, _ = listener.accept()
        with conn:
            conn.settimeout(10)
            test.raw_seen.append(conn.recv(65536))
            conn.sendall(answer)

    t = threading.Thread(target=serve, daemon=True)
    t.start()
    test.addCleanup(t.join, 5)
    return f"http://127.0.0.1:{listener.getsockname()[1]}/v1"


class RelayExchange(unittest.TestCase):
    """The relay between the operator and the endpoint."""

    def setUp(self):
        self.env = mock.patch.dict(os.environ, {}, clear=True)
        self.env.start()

    def tearDown(self):
        self.env.stop()

    def endpoint(self, kind):
        end = FakeEndpoint(kind)
        self.addCleanup(end.close)
        return end

    def assert_operator_headers(self, end, bodies):
        """Each request reaches the endpoint with the three headers the operator set and the five urllib adds to
        every request, and no other: the headers the measured operator sent the endpoint itself."""
        host = urllib.parse.urlparse(end.url).netloc
        agent = "Python-urllib/%d.%d" % sys.version_info[:2]
        want = [sorted([("accept", "application/json"), ("accept-encoding", "identity"),
                        ("authorization", "Bearer " + KEY), ("connection", "close"),
                        ("content-length", str(len(body))), ("content-type", "application/json"), ("host", host),
                        ("user-agent", agent)]) for body in bodies]
        self.assertEqual([sorted((k.lower(), v) for k, v in got) for got in end.headers], want)

    def test_on_google_endpoint_every_byte_reaches_the_endpoint_and_comes_back_as_it_was(self):
        end = self.endpoint("google")
        with route.Relay(end.url + "/v1") as relay:
            status, first = post(relay.base_url, TURN_1)
            post(relay.base_url, GOOGLE_TURN_2)
        self.assertEqual(status, 200)
        self.assertEqual([s[2] for s in end.seen], [TURN_1, GOOGLE_TURN_2])
        self.assertEqual([s[:2] for s in end.seen], [("/v1/chat/completions", "Bearer " + KEY)] * 2)
        self.assert_operator_headers(end, [TURN_1, GOOGLE_TURN_2])
        self.assertEqual(first, answer("google", 0))
        self.assertEqual(relay.counts, {"requests": 2, "not_reached": 0, "replies_with_reasoning_blocks": 0,
                                        "requests_carrying_blocks": 0, "relay_errors": 0})

    def test_on_openrouter_the_first_replys_blocks_go_back_at_the_next_turn(self):
        end = self.endpoint("openrouter")
        with route.Relay(end.url + "/v1") as relay:
            post(relay.base_url, TURN_1)
            post(relay.base_url, OPERATOR_TURN_2)
        self.assertEqual([s[2] for s in end.seen], [TURN_1, ENDPOINT_TURN_2])
        self.assert_operator_headers(end, [TURN_1, ENDPOINT_TURN_2])
        self.assertEqual(relay.counts, {"requests": 2, "not_reached": 0, "replies_with_reasoning_blocks": 2,
                                        "requests_carrying_blocks": 1, "relay_errors": 0})

    def test_an_http_error_comes_back_as_it_came(self):
        end = self.endpoint("openrouter")
        end.answer = (429, {"Content-Type": "application/json", "Retry-After": "7", "X-Other": "no"},
                      b'{"error": "slow down"}')
        with route.Relay(end.url + "/v1") as relay:
            with self.assertRaises(urllib.error.HTTPError) as caught:
                post(relay.base_url, TURN_1)
        self.assertEqual(caught.exception.code, 429)
        self.assertEqual(caught.exception.read(), b'{"error": "slow down"}')
        self.assertEqual(caught.exception.headers.get("Retry-After"), "7")
        self.assertIsNone(caught.exception.headers.get("X-Other"))
        self.assertEqual(relay.counts["replies_with_reasoning_blocks"], 0)

    def test_an_endpoint_that_cannot_be_reached_leaves_the_connection_unanswered(self):
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
        sock.close()
        with route.Relay(f"http://127.0.0.1:{port}/v1") as relay:
            with self.assertRaises(OSError) as caught:   # the class the operator reads as "unreachable"
                post(relay.base_url, TURN_1)
        self.assertNotIsInstance(caught.exception, urllib.error.HTTPError)
        self.assertEqual(relay.counts["not_reached"], 1)

    def test_another_path_is_not_found_and_not_sent(self):
        end = self.endpoint("google")
        with route.Relay(end.url + "/v1") as relay:
            req = urllib.request.Request(relay.base_url + "/models", data=b"{}", method="POST")
            with self.assertRaises(urllib.error.HTTPError) as caught:
                urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req, timeout=30)
        self.assertEqual(caught.exception.code, 404)
        self.assertEqual(end.seen, [])

    def test_a_request_goes_on_when_the_progress_callback_fails(self):
        end = self.endpoint("google")
        calls = []

        def broken():
            calls.append(1)
            raise RuntimeError("no terminal")

        with route.Relay(end.url + "/v1", on_request=broken) as relay:
            status, _ = post(relay.base_url, TURN_1)
        self.assertEqual((status, len(calls), len(end.seen)), (200, 1, 1))

    def test_a_loopback_endpoint_is_reached_directly_past_a_proxy(self):
        end = self.endpoint("google")
        os.environ.update(http_proxy="http://127.0.0.1:9", HTTP_PROXY="http://127.0.0.1:9")
        with route.Relay(end.url + "/v1") as relay:
            status, _ = post(relay.base_url, TURN_1)
        self.assertEqual(status, 200)

    def test_a_named_host_is_sent_its_own_url_through_the_proxy(self):
        end = self.endpoint("openrouter")
        os.environ.update(http_proxy=end.url, HTTP_PROXY=end.url)
        with route.Relay("http://openrouter.ai/api/v1") as relay:
            post(relay.base_url, TURN_1)
        self.assertEqual(end.seen[0][:2], ("http://openrouter.ai/api/v1/chat/completions", "Bearer " + KEY))

    def test_the_relay_keeps_no_key_and_writes_nothing(self):
        end = self.endpoint("openrouter")
        err = io.StringIO()
        with contextlib.redirect_stderr(err), route.Relay(end.url + "/v1") as relay:
            post(relay.base_url, TURN_1)
            kept = json.dumps([relay.counts, relay._blocks, relay.url])
        self.assertNotIn(KEY, kept)
        self.assertEqual(err.getvalue(), "")

    def test_an_operator_that_stopped_waiting_costs_the_relay_nothing(self):
        end = self.endpoint("google")
        end.delay = 1.0
        with route.Relay(end.url + "/v1") as relay:
            req = urllib.request.Request(relay.base_url + "/chat/completions", data=TURN_1, method="POST")
            with self.assertRaises(OSError):   # the operator's own timeout
                urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req, timeout=0.2)
            end.delay = 0.0
            time.sleep(1.2)   # the late answer finds the connection closed
            status, _ = post(relay.base_url, TURN_1)
        self.assertEqual(status, 200)
        self.assertEqual(len(end.seen), 2)

    def test_a_request_that_broke_off_prints_nothing(self):
        err = io.StringIO()
        with contextlib.redirect_stderr(err), route.Relay("http://127.0.0.1:9/v1") as relay:
            try:
                raise ConnectionResetError("the operator went away")
            except ConnectionResetError:
                relay._server.handle_error(None, ("127.0.0.1", 0))
        self.assertEqual(err.getvalue(), "")

    def test_after_close_nothing_listens(self):
        end = self.endpoint("google")
        with route.Relay(end.url + "/v1") as relay:
            url = relay.base_url
        with self.assertRaises(OSError):
            post(url, TURN_1)

    def test_the_same_exchange_twice_sends_the_same_bytes(self):
        sent = []
        for _ in range(2):
            end = self.endpoint("openrouter")
            with route.Relay(end.url + "/v1") as relay:
                post(relay.base_url, TURN_1)
                post(relay.base_url, OPERATOR_TURN_2)
            sent.append([s[2] for s in end.seen])
        self.assertEqual(sent[0], sent[1])

    def test_a_redirect_is_not_followed_and_comes_back_without_its_address(self):
        # A second server stands for another host. urllib's own redirect step is in place here, as in a process
        # where run-cell has not replaced it: the relay follows no redirect by itself. The operator's opener in
        # post() would follow a Location; it gets none.
        keep_redirects(self)
        elsewhere = self.endpoint("openrouter")
        for code in (301, 302, 303, 307, 308):
            for location in (elsewhere.url + "/v1/chat/completions", "/v2/chat/completions"):
                with self.subTest(code=code, location=location):
                    end = self.endpoint("openrouter")
                    end.answer = (code, {"Location": location, "Content-Type": "application/json"}, b'{"moved": 1}')
                    with route.Relay(end.url + "/v1") as relay:
                        with self.assertRaises(urllib.error.HTTPError) as caught:
                            post(relay.base_url, TURN_1)
                    self.assertEqual(caught.exception.code, code)
                    self.assertEqual(caught.exception.read(), b'{"moved": 1}')
                    self.assertIsNone(caught.exception.headers.get("Location"))
                    self.assertEqual(caught.exception.headers.get("Content-Type"), "application/json")
                    self.assertEqual(end.seen, [("/v1/chat/completions", "Bearer " + KEY, TURN_1)])
                    self.assertEqual(relay.counts, {"requests": 1, "not_reached": 0,
                                                    "replies_with_reasoning_blocks": 0,
                                                    "requests_carrying_blocks": 0, "relay_errors": 0})
        self.assertEqual(elsewhere.seen, [])

    def test_a_request_line_that_names_another_host_goes_to_the_given_url_only(self):
        end, elsewhere = self.endpoint("google"), self.endpoint("google")
        other = urllib.parse.urlparse(elsewhere.url).netloc
        with route.Relay(end.url + "/v1") as relay:
            conn = http.client.HTTPConnection(urllib.parse.urlparse(relay.base_url).netloc, timeout=30)
            self.addCleanup(conn.close)
            conn.request("POST", elsewhere.url + "/v1/chat/completions", body=TURN_1,
                         headers={"Host": other, "Content-Type": "application/json",
                                  "Authorization": "Bearer " + KEY})
            status = conn.getresponse().status
        self.assertEqual(status, 200)
        self.assertEqual(end.seen, [("/v1/chat/completions", "Bearer " + KEY, TURN_1)])
        self.assertEqual(elsewhere.seen, [])

    def test_an_error_answer_that_carries_blocks_is_not_remembered(self):
        end = self.endpoint("openrouter")
        end.answer = (500, {"Content-Type": "application/json"}, answer("openrouter", 0))
        with route.Relay(end.url + "/v1") as relay:
            with self.assertRaises(urllib.error.HTTPError) as caught:
                post(relay.base_url, TURN_1)
            self.assertEqual(caught.exception.code, 500)
            end.answer = None
            post(relay.base_url, OPERATOR_TURN_2)
        self.assertEqual(end.seen[1][2], OPERATOR_TURN_2)
        self.assertEqual((relay.counts["replies_with_reasoning_blocks"], relay.counts["requests_carrying_blocks"]),
                         (1, 0))   # the second answer's own blocks; none went back

    def test_an_error_answer_whose_body_breaks_off_comes_back_with_its_status_and_no_body(self):
        url = raw_endpoint(self, b"HTTP/1.1 500 Internal Server Error\r\nContent-Type: application/json\r\n"
                                 b"Content-Length: 100\r\n\r\n{\"err")
        with route.Relay(url) as relay:
            with self.assertRaises(urllib.error.HTTPError) as caught:
                post(relay.base_url, TURN_1)
        self.assertEqual((caught.exception.code, caught.exception.read()), (500, b""))
        self.assertEqual((relay.counts["not_reached"], relay.counts["relay_errors"]), (0, 0))

    def test_a_request_whose_body_length_is_not_named_is_answered_411_and_not_sent(self):
        end = self.endpoint("google")
        with route.Relay(end.url + "/v1") as relay:
            for name, headers, body in (("no Content-Length", {}, None),
                                        ("one that is not a count", {"Content-Length": "12x"}, None),
                                        ("a digit that is not 0 to 9", {"Content-Length": "\u00b2"}, None),
                                        ("a chunked body", {"Transfer-Encoding": "chunked"}, TURN_1),
                                        ("a chunked body that also names a length",
                                         {"Content-Length": str(len(TURN_1)), "Transfer-Encoding": "chunked"},
                                         TURN_1)):
                with self.subTest(name):
                    conn = http.client.HTTPConnection(urllib.parse.urlparse(relay.base_url).netloc, timeout=30)
                    conn.putrequest("POST", "/v1/chat/completions", skip_accept_encoding=True)
                    for key, value in dict(headers, Authorization="Bearer " + KEY).items():
                        conn.putheader(key, value.encode("latin-1"))
                    conn.endheaders()
                    if body is not None:
                        conn.send(b"%x\r\n%s\r\n0\r\n\r\n" % (len(TURN_1), TURN_1))
                    got = conn.getresponse()
                    self.assertEqual((got.status, got.read()), (411, b"the relay takes a body with a Content-Length "
                                                                     b"only\n"))
                    conn.close()
        self.assertEqual(end.seen, [])
        self.assertEqual(relay.counts["requests"], 0)

    def test_an_answer_that_is_not_http_counts_as_not_reached(self):
        url = raw_endpoint(self, b"not an HTTP answer\r\n\r\n")
        with route.Relay(url) as relay:
            with self.assertRaises(OSError):   # the connection closes unanswered
                post(relay.base_url, TURN_1)
        self.assertEqual(len(self.raw_seen), 1)
        self.assertEqual((relay.counts["not_reached"], relay.counts["relay_errors"]), (1, 0))

    def test_a_url_urllib_cannot_send_to_counts_as_not_reached(self):
        with route.Relay("http://127.0.0.1:notaport/v1") as relay:
            with self.assertRaises(OSError):
                post(relay.base_url, TURN_1)
        self.assertEqual((relay.counts["not_reached"], relay.counts["relay_errors"]), (1, 0))

    def test_an_endpoint_slower_than_the_relays_timeout_counts_as_not_reached(self):
        end = self.endpoint("google")
        end.gate = threading.Event()   # answered only once the test is over
        self.addCleanup(end.gate.set)
        t0 = time.monotonic()
        with route.Relay(end.url + "/v1", timeout_s=0.5) as relay:
            with self.assertRaises(OSError):
                post(relay.base_url, TURN_1)
        self.assertLess(time.monotonic() - t0, 10)
        self.assertEqual(relay.counts["not_reached"], 1)

    def test_no_header_but_the_operators_three_goes_on(self):
        end = self.endpoint("google")
        with route.Relay(end.url + "/v1") as relay:
            req = urllib.request.Request(relay.base_url + "/chat/completions", data=TURN_1, method="POST",
                                         headers={"Content-Type": "application/json", "Accept": "application/json",
                                                  "Authorization": "Bearer " + KEY, "Cookie": "session=1",
                                                  "X-Forwarded-For": "10.0.0.1", "Proxy-Authorization": "Basic eA=="})
            urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req, timeout=30).close()
        self.assert_operator_headers(end, [TURN_1])

    def test_a_fault_of_the_relay_itself_is_answered_502_and_counted(self):
        end = self.endpoint("google")
        with route.Relay(end.url + "/v1") as relay, \
                mock.patch.object(route.Relay, "exchange", side_effect=RuntimeError("a bug")):
            with self.assertRaises(urllib.error.HTTPError) as caught:
                post(relay.base_url, TURN_1)
        self.assertEqual((caught.exception.code, caught.exception.read()),
                         (502, b"the relay could not pass this request on (RuntimeError)\n"))
        self.assertEqual(relay.counts["relay_errors"], 1)

    def test_json_nested_deeper_than_the_parser_goes_goes_on_as_it_came_both_ways(self):
        end = self.endpoint("openrouter")
        deep = b"[" * 100_000 + b"]" * 100_000
        with route.Relay(end.url + "/v1") as relay:
            post(relay.base_url, TURN_1)   # a reply with blocks: the next request is read for its history
            end.answer = (200, {"Content-Type": "application/json"}, deep)
            status, body = post(relay.base_url, deep)
        self.assertEqual((status, body, end.seen[1][2]), (200, deep, deep))
        self.assertEqual(relay.counts, {"requests": 2, "not_reached": 0, "replies_with_reasoning_blocks": 1,
                                        "requests_carrying_blocks": 0, "relay_errors": 0})

    def test_a_base_url_that_is_not_http_with_a_host_is_refused(self):
        for url in ("ftp://example.com/v1", "http://", "example.com/v1"):
            with self.assertRaises(ValueError, msg=url):
                route.Relay(url)

    def test_a_closed_answer_from_the_endpoint_counts_as_not_reached(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        self.addCleanup(listener.close)
        port = listener.getsockname()[1]
        with route.Relay(f"http://127.0.0.1:{port}/v1", timeout_s=5) as relay:
            def close_one():
                conn, _ = listener.accept()
                conn.recv(65536)
                conn.close()
            t = threading.Thread(target=close_one, daemon=True)
            t.start()
            with self.assertRaises(OSError):
                post(relay.base_url, TURN_1)
            t.join(5)
        self.assertEqual(relay.counts["not_reached"], 1)
        self.assertTrue(issubclass(http.client.RemoteDisconnected, OSError))


if __name__ == "__main__":
    unittest.main()
