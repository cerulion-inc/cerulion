#!/usr/bin/env python3
"""Unit tests for crosshost.py: `python3 -m unittest test_crosshost` from
this directory. Standard library only."""

import os
import signal
import struct
import subprocess
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import crosshost as xh
from compile_csv import percentile

NS = "{http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles}"


class GraphRendering(unittest.TestCase):
    def test_ping_graph_dials_egresses_ping_and_ingresses_echo(self):
        g = xh.render_ping_graph("tcp", "10.0.0.2", 7690, 1024)
        self.assertIn("  connect:\n  - tcp/10.0.0.2:7690\n", g)
        self.assertIn(f"  egress:\n  - {xh.PING_TOPIC}\n", g)
        self.assertIn(f"  ingress:\n  - {xh.ECHO_TOPIC}\n", g)
        self.assertIn(f"    source: {xh.ECHO_TOPIC}\n", g)
        self.assertIn(f"    max_slice_len: {1024 + 65536}\n", g)
        self.assertNotIn("listen:", g)
        self.assertNotIn("tls:", g)

    def test_pong_graph_listens_ingresses_ping_and_egresses_echo(self):
        g = xh.render_pong_graph("tcp", "10.0.0.2", 7690, 64)
        self.assertIn("  listen:\n  - tcp/10.0.0.2:7690\n", g)
        self.assertIn(f"  egress:\n  - {xh.ECHO_TOPIC}\n", g)
        self.assertIn(f"  ingress:\n  - {xh.PING_TOPIC}\n", g)
        self.assertIn(f"    source: {xh.PING_TOPIC}\n", g)
        self.assertNotIn("connect:", g)

    def test_topics_follow_the_prefix_node_output_rule(self):
        # The graph loader names an output /{prefix}/{node}/{output}; the two
        # graphs only meet if these constants match that rule.
        self.assertEqual(xh.PING_TOPIC, f"/{xh.PREFIX}/ping/ping_out")
        self.assertEqual(xh.ECHO_TOPIC, f"/{xh.PREFIX}/pong/echo_out")

    def test_quic_sides_carry_the_tls_file_each_side_reads(self):
        ping = xh.render_ping_graph("quic", "10.0.0.2", 7690, 64, root_ca="/c/peer.pem")
        self.assertIn("  - quic/10.0.0.2:7690\n", ping)
        self.assertIn("  tls:\n    root_ca: /c/peer.pem\n", ping)
        self.assertNotIn("certificate:", ping)
        pong = xh.render_pong_graph("quic", "10.0.0.2", 7690, 64, cert="/c/l.pem", key="/c/l.key")
        self.assertIn("  tls:\n    certificate: /c/l.pem\n    private_key: /c/l.key\n", pong)
        self.assertNotIn("root_ca:", pong)

    def test_quic_without_its_tls_files_is_refused(self):
        with self.assertRaises(ValueError):
            xh.render_ping_graph("quic", "10.0.0.2", 7690, 64)
        with self.assertRaises(ValueError):
            xh.render_pong_graph("quic", "10.0.0.2", 7690, 64, cert="/c/l.pem")


class FastDdsProfiles(unittest.TestCase):
    def test_listener_has_only_a_whitelisted_tcp_listener(self):
        root = ET.fromstring(xh.render_fastdds_listener("10.0.0.2", 7690))
        descs = root.findall(f"{NS}transport_descriptors/{NS}transport_descriptor")
        self.assertEqual([d.findtext(f"{NS}type") for d in descs], ["TCPv4"])
        self.assertEqual(descs[0].findtext(f"{NS}listening_ports/{NS}port"), "7690")
        self.assertEqual(descs[0].findtext(f"{NS}interfaceWhiteList/{NS}address"), "10.0.0.2")
        self.assertEqual(descs[0].findtext(f"{NS}enable_tcp_nodelay"), "true")
        part = root.find(f"{NS}participant")
        self.assertEqual(part.get("is_default_profile"), "true")
        self.assertEqual(part.findtext(f"{NS}rtps/{NS}useBuiltinTransports"), "false")

    def test_dialer_opens_tcp_and_keeps_udp_on_loopback(self):
        root = ET.fromstring(xh.render_fastdds_dialer("10.0.0.2", 7690))
        descs = {d.findtext(f"{NS}type"): d
                 for d in root.findall(f"{NS}transport_descriptors/{NS}transport_descriptor")}
        self.assertEqual(sorted(descs), ["TCPv4", "UDPv4"])
        self.assertIsNone(descs["TCPv4"].find(f"{NS}listening_ports"))
        self.assertEqual(descs["TCPv4"].findtext(f"{NS}enable_tcp_nodelay"), "true")
        self.assertEqual(descs["UDPv4"].findtext(f"{NS}interfaceWhiteList/{NS}address"),
                         "127.0.0.1")
        peers = root.find(f"{NS}participant/{NS}rtps/{NS}builtin/{NS}initialPeersList")
        tcp = peers.find(f"{NS}locator/{NS}tcpv4")
        self.assertEqual(tcp.findtext(f"{NS}address"), "10.0.0.2")
        self.assertEqual(tcp.findtext(f"{NS}physical_port"), "7690")
        self.assertEqual(peers.findtext(f"{NS}locator/{NS}udpv4/{NS}address"), "127.0.0.1")
        rtps = root.find(f"{NS}participant/{NS}rtps")
        self.assertEqual(rtps.findtext(f"{NS}useBuiltinTransports"), "false")
        used = [t.text for t in rtps.findall(f"{NS}userTransports/{NS}transport_id")]
        ids = [d.findtext(f"{NS}transport_id") for d in descs.values()]
        self.assertEqual(sorted(used), sorted(ids))


class Samples(unittest.TestCase):
    def write_bin(self, d: Path, name: str, values):
        p = d / name
        p.write_bytes(struct.pack(f"<{len(values)}Q", *values))
        return p

    def test_summary_uses_the_suite_quantile(self):
        vals = [5_000, 1_000, 3_000, 2_000, 4_000, 100_000]
        s = xh.summarize_samples(vals)
        srt = sorted(vals)
        self.assertEqual(s["n"], 6)
        self.assertEqual(s["min"], 1_000)
        self.assertEqual(s["max"], 100_000)
        self.assertEqual(s["p50"], percentile(srt, 0.5))
        self.assertEqual(s["p99"], percentile(srt, 0.99))

    def test_bin_names_parse_to_stem_and_size(self):
        self.assertEqual(xh.parse_bin_name("xhost_cerulion_tcp_1024.bin"),
                         ("xhost_cerulion_tcp", 1024))
        self.assertIsNone(xh.parse_bin_name("xhost_cerulion_tcp_1024.rate"))
        self.assertIsNone(xh.parse_bin_name("notes.bin"))
        self.assertEqual(xh.raw_name("fastdds-tcp"), "xhost_fastdds_tcp")

    def test_accepted_list_adds_once_and_drops(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            xh.set_accepted(d, "a_64.bin", True)
            xh.set_accepted(d, "a_64.bin", True)
            xh.set_accepted(d, "b_64.bin", True)
            self.assertEqual(xh.accepted_names(d), ["a_64.bin", "b_64.bin"])
            xh.set_accepted(d, "a_64.bin", False)
            self.assertEqual(xh.accepted_names(d), ["b_64.bin"])

    def test_a_torn_bin_is_refused(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "x_64.bin"
            p.write_bytes(b"\x00" * 12)
            with self.assertRaises(ValueError):
                xh.read_bin(p)

    def test_table_sorts_by_size_then_stack_in_microseconds(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            self.write_bin(d, "xhost_fastdds_tcp_64.bin", [2_000_000] * 10)
            self.write_bin(d, "xhost_cerulion_tcp_1024.bin", [3_000_000] * 10)
            self.write_bin(d, "xhost_cerulion_quic_64.bin", [1_500_000] * 10)
            self.write_bin(d, "xhost_fastdds_tcp_1024.bin", [9_000_000] * 10)  # not accepted
            (d / "ignored.txt").write_text("x")
            for n in ("xhost_fastdds_tcp_64.bin", "xhost_cerulion_tcp_1024.bin",
                      "xhost_cerulion_quic_64.bin"):
                xh.set_accepted(d, n, True)
            rows = xh.summary_table(d).splitlines()[2:]
            self.assertEqual(len(rows), 3)
            self.assertTrue(rows[0].startswith("| 64 | cerulion-quic | 10 | 1500.0 | 1500.0 |"))
            self.assertTrue(rows[1].startswith("| 64 | fastdds-tcp |"))
            self.assertTrue(rows[2].startswith("| 1024 | cerulion-tcp |"))


class Receipts(unittest.TestCase):
    def test_both_receipt_spellings_parse(self):
        text = (
            "noise\n"
            "RTT_DELIVERY role=ping leg=xhost-ping published=2150 slots_skipped=0\n"
            "[INFO] DELIVERY role=latency received=2100 kicks_sent=2101 unstamped=0 nonpositive_rtt=0\n"
            "DELIVERY no role here\n"
        )
        r = xh.parse_receipts(text)
        self.assertEqual([x["role"] for x in r], ["ping", "latency"])
        self.assertEqual(r[0]["published"], "2150")
        self.assertEqual(r[1]["kicks_sent"], "2101")


class Peers(unittest.TestCase):
    def test_connected_peers_lists_established_remote_ends_on_the_port(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            head = "  sl  local_address rem_address   st\n"
            (d / "tcp").write_text(head
                + "   0: 0200000A:1E0A 00000000:0000 0A\n"     # listener
                + "   1: 0200000A:1E0A 0300000A:C350 01\n"     # 10.0.0.3:50000
                + "   2: 0200000A:1F90 0400000A:C351 01\n")    # other port
            (d / "udp").write_text(head + "   0: 0200000A:1E0A 0500000A:D431 01\n")
            self.assertEqual(xh.connected_peers(7690, d),
                             ["tcp:10.0.0.3:50000", "udp:10.0.0.5:54321"])


def cer_receipts(published=2106, skipped=0, received=2100):
    return [{"role": "ping", "published": str(published), "slots_skipped": str(skipped)},
            {"role": "latency", "received": str(received), "measured": str(received)}]


def dds_receipts(published=2100, received=2100, unstamped=0, nonpositive=0):
    return [{"role": "ping", "published": str(published)},
            {"role": "latency", "received": str(received), "kicks_sent": str(published),
             "unstamped": str(unstamped), "nonpositive_rtt": str(nonpositive)}]


TEL = "live loop delivery telemetry node_id=latency input=echo_in fires=2100 drop_oldest=0\n"


class Verdicts(unittest.TestCase):
    def test_a_cell_that_ran_as_requested_passes(self):
        self.assertEqual(xh.delivery_verdict("cerulion-tcp", cer_receipts(), 2100, TEL), "")
        self.assertEqual(xh.delivery_verdict("fastdds-tcp", dds_receipts(), 2100), "")

    def test_cerulion_cell_fails_past_one_percent_skipped_slots(self):
        self.assertEqual(
            xh.delivery_verdict("cerulion-tcp", cer_receipts(skipped=21), 2100, TEL), "")
        self.assertIn("skipped 22",
                      xh.delivery_verdict("cerulion-quic", cer_receipts(skipped=22), 2100))

    def test_unanswered_pings_beyond_the_allowance_fail(self):
        lost = 2100 + xh.MAX_UNANSWERED + 1
        self.assertIn("ping sent", xh.delivery_verdict("cerulion-tcp",
                                                       cer_receipts(published=lost), 2100))
        self.assertIn("ping sent", xh.delivery_verdict("fastdds-tcp",
                                                       dds_receipts(published=4200), 2100))

    def test_too_few_echoes_fail(self):
        self.assertIn("received 2099", xh.delivery_verdict("cerulion-tcp",
                                                           cer_receipts(received=2099), 2100))

    def test_missing_receipts_or_counters_fail(self):
        self.assertIn("missing", xh.delivery_verdict("cerulion-tcp", [], 2100))
        partial = dds_receipts()
        del partial[1]["unstamped"]
        self.assertIn("no unstamped", xh.delivery_verdict("fastdds-tcp", partial, 2100))

    def test_a_cerulion_log_without_telemetry_fails(self):
        self.assertIn("no drop_oldest",
                      xh.delivery_verdict("cerulion-tcp", cer_receipts(), 2100, "no lines"))

    def test_fastdds_kicks_that_never_became_pings_fail(self):
        r = dds_receipts()
        r[1]["kicks_sent"] = "5000"
        self.assertIn("5000 kicks", xh.delivery_verdict("fastdds-tcp", r, 2100))

    def test_serve_verdict_checks_the_pong_receipt_and_evictions(self):
        tel = "telemetry node_id=pong drop_oldest=0\n"
        pong = [{"role": "pong", "forwarded": "2100"}]
        self.assertEqual(xh.serve_verdict("cerulion-tcp", pong, 2100, tel), "")
        self.assertIn("2100 of 2101", xh.serve_verdict("cerulion-tcp", pong, 2101, tel))
        self.assertIn("drop_oldest=2", xh.serve_verdict(
            "cerulion-quic", pong, 2100, tel.replace("=0", "=2")))
        self.assertIn("no drop_oldest", xh.serve_verdict("cerulion-tcp", pong, 2100, ""))
        self.assertIn("no echoed", xh.serve_verdict("fastdds-tcp", pong, 0, ""))
        self.assertEqual(xh.serve_verdict("fastdds-tcp",
                                          [{"role": "pong", "echoed": "2100"}], 2100, ""), "")

    def test_an_eviction_in_the_cerulion_log_fails(self):
        log = ("\x1b[2mT\x1b[0m INFO live loop delivery telemetry \x1b[3mnode_id\x1b[0m"
               "\x1b[2m=\x1b[0mlatency \x1b[3mdrop_oldest\x1b[0m\x1b[2m=\x1b[0m3\n")
        self.assertIn("drop_oldest=3", xh.delivery_verdict("cerulion-tcp", cer_receipts(), 2100, log))
        clean = log.replace("m3\n", "m0\n")
        self.assertEqual(xh.delivery_verdict("cerulion-tcp", cer_receipts(), 2100, clean), "")

    def test_fastdds_cell_fails_on_an_unusable_echo(self):
        self.assertIn("nonpositive_rtt=3",
                      xh.delivery_verdict("fastdds-tcp", dds_receipts(nonpositive=3), 2100))


class Iox2ConfigRestore(unittest.TestCase):
    def with_workspace(self, d: Path):
        old = xh.WORKSPACE_DIR
        xh.WORKSPACE_DIR = d
        self.addCleanup(setattr, xh, "WORKSPACE_DIR", old)

    def test_a_missing_config_is_removed_again(self):
        with tempfile.TemporaryDirectory() as d:
            self.with_workspace(Path(d))
            with xh.Iox2Config("ping"):
                text = (Path(d) / "config" / "iceoryx2.toml").read_text()
                self.assertIn(f'prefix = "{xh.iox2_prefix("ping")}"', text)
            self.assertFalse((Path(d) / "config").exists())

    def test_an_existing_config_is_restored(self):
        with tempfile.TemporaryDirectory() as d:
            self.with_workspace(Path(d))
            cfg = Path(d) / "config" / "iceoryx2.toml"
            cfg.parent.mkdir()
            cfg.write_text("[global]\nprefix = \"mine_\"\n")
            with xh.Iox2Config("pong"):
                self.assertNotIn("mine_", cfg.read_text())
            self.assertEqual(cfg.read_text(), "[global]\nprefix = \"mine_\"\n")

    def test_a_failed_backup_read_leaves_the_original_alone(self):
        with tempfile.TemporaryDirectory() as d:
            self.with_workspace(Path(d))
            cfg = Path(d) / "config" / "iceoryx2.toml"
            cfg.parent.mkdir()
            cfg.write_bytes(b"\xff\xfe not utf-8")
            cm = xh.Iox2Config("ping")
            with self.assertRaises(UnicodeDecodeError):
                cm.__enter__()
            cm.__exit__(None, None, None)
            self.assertEqual(cfg.read_bytes(), b"\xff\xfe not utf-8")

    def test_exit_without_enter_leaves_the_file_alone(self):
        with tempfile.TemporaryDirectory() as d:
            self.with_workspace(Path(d))
            cfg = Path(d) / "config" / "iceoryx2.toml"
            cfg.parent.mkdir()
            cfg.write_text("keep")
            xh.Iox2Config("ping").__exit__(None, None, None)
            self.assertEqual(cfg.read_text(), "keep")


class Stop(unittest.TestCase):
    def test_stop_kills_a_descendant_that_outlives_the_leader(self):
        with tempfile.TemporaryDirectory() as d:
            pidfile = Path(d) / "child.pid"
            # The leader exits on TERM; its background child ignores TERM.
            script = (f"(trap '' TERM; echo $BASHPID > {pidfile}; exec sleep 60) & "
                      "trap 'exit 0' TERM; wait")
            proc = subprocess.Popen(["bash", "-c", script], start_new_session=True)
            deadline = time.monotonic() + 5
            while not pidfile.exists() and time.monotonic() < deadline:
                time.sleep(0.05)
            child = int(pidfile.read_text())
            xh.stop(proc, signal.SIGTERM, grace=1.0)
            time.sleep(0.2)
            with self.assertRaises(ProcessLookupError):
                os.kill(child, 0)


class StopAfterLeaderExit(unittest.TestCase):
    def test_stop_reaps_descendants_of_a_leader_that_already_exited(self):
        with tempfile.TemporaryDirectory() as d:
            pidfile = Path(d) / "child.pid"
            script = f"(trap '' TERM; echo $BASHPID > {pidfile}; exec sleep 60) & exit 0"
            proc = subprocess.Popen(["bash", "-c", script], start_new_session=True)
            proc.wait()
            deadline = time.monotonic() + 5
            while not pidfile.exists() and time.monotonic() < deadline:
                time.sleep(0.05)
            child = int(pidfile.read_text())
            xh.stop(proc, signal.SIGTERM, grace=0.5)
            with self.assertRaises(ProcessLookupError):
                os.kill(child, 0)


class PortProbe(unittest.TestCase):
    HEAD = "  sl  local_address rem_address   st tx_queue rx_queue\n"

    def fake(self, d: Path, tcp="", udp=""):
        (d / "tcp").write_text(self.HEAD + tcp)
        (d / "udp").write_text(self.HEAD + udp)

    def test_tcp_counts_only_a_listening_socket(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            # 0x1E0A = 7690. State 01 (ESTABLISHED) is not a listener.
            self.fake(d, tcp="   0: 0200000A:1E0A 0300000A:C350 01 0:0\n")
            self.assertFalse(xh.port_is_bound(7690, d))
            self.fake(d, tcp="   0: 0200000A:1E0A 00000000:0000 0A 0:0\n")
            self.assertTrue(xh.port_is_bound(7690, d))
            self.assertFalse(xh.port_is_bound(7691, d))

    def test_udp_counts_any_bound_socket(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            self.fake(d, udp="   0: 0200000A:1E0A 00000000:0000 07 0:0\n")
            self.assertTrue(xh.port_is_bound(7690, d))


if __name__ == "__main__":
    unittest.main()
