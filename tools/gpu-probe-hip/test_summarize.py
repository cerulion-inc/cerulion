#!/usr/bin/env python3
"""Unit tests for summarize.py on a synthetic raw.log (test fixture rows, not measurements).

    python3 -I test_summarize.py
"""
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import summarize  # noqa: E402


def lat(rep, mode, size, p50, p99, mm=0, frames=2100):
    return ('RESULT {"rep":%d,"align":"pad","kind":"latency","mode":"%s","size":%d,"iters":2000,"warmup":100,'
            '"p50_us":%s,"p99_us":%s,"gpu_mismatch":%d,"cpu_mismatch":0,"frames":%d,"gpu_mhz_mid":2100,'
            '"smi_sclk_post":"sclk 2100Mhz"}' % (rep, mode, size, p50, p99, mm, frames))


FIXTURE = "\n".join([
    'RESULT {"rep":0,"align":"pad","kind":"probe","device":"fixture","gcn_arch":"gfx942"}',
    lat(1, "copy", 4096, 100.0, 120.0), lat(2, "copy", 4096, 110.0, 130.0), lat(3, "copy", 4096, 90.0, 200.0),
    lat(1, "unified-event", 4096, 50.0, 60.0), lat(2, "unified-event", 4096, 50.0, 70.0),
    lat(1, "cpu", 4096, 1.25, 1.5),
    lat(1, "copy", 1 << 20, 400.0, 500.0),
    'RESULT {"rep":1,"align":"pad","kind":"onetime","side":"writer","mode":"unified-event","size":4096,"segment":24576,'
    '"register_us":300.0,"register_us_per_mib":12800.0,"flags":3,"reg_path":"cerulion_device","map":"rw-s /dev/shm/x",'
    '"dptr_eq_host":true,"reg_tried":""}',
    'RESULT {"rep":1,"align":"pad","kind":"onetime","side":"reader","mode":"device-vmm-event","size":4096,"block":2097152,'
    '"import_us":70.0,"map_access_us":170.0,"import_arg":"dual (pointer and value)"}',
    'RESULT {"rep":1,"align":"pad","kind":"onetime","side":"reader","mode":"device-vmm-event","size":4096,"ipc_event_open_us":210.0}',
    'RESULT {"rep":1,"align":"pad","kind":"skip","side":"reader","mode":"device-ipcmem-event","size":4096,"reason":"hipIpcOpenMemHandle -> 1 (hipErrorInvalidValue)"}',
    'RESULT {"rep":1,"align":"pad","kind":"failed","mode":"device-dmabuf-event","rc":139,"reason":"unit exited 139"}',
    'WALL {"rep":1,"unit":"copy","secs":12.5,"rc":0}',
    'WALL {"rep":2,"unit":"copy","secs":13.5,"rc":0}',
]) + "\n"


class SummarizeTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = pathlib.Path(self.tmp.name)
        (self.dir / "env.txt").write_text("host=fixture\n")

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, text):
        (self.dir / "raw.log").write_text(text)

    def test_cells_are_the_median_across_reps(self):
        self.write(FIXTURE)
        out = summarize.summary(self.dir)
        self.assertIn("| 4 KiB | - | 100 / 130 | - | 50 / 65 | - | - | - | - | - |", out)
        self.assertIn("| 1 MiB |", out)
        self.assertIn("| 4 KiB | 1.2 / 1.5 |", out)  # under 10 us keeps one decimal

    def test_speedup_table_divides_copy_by_mode(self):
        self.write(FIXTURE)
        out = summarize.summary(self.dir)
        self.assertIn("Speedup over the copy path", out)
        self.assertIn("2.00x", out)  # copy 100 / unified-event 50

    def test_mismatch_totals_and_onetime_and_skips(self):
        self.write(FIXTURE)
        out = summarize.summary(self.dir)
        self.assertIn("Total mismatches: 0 over 14700 frames", out)
        self.assertIn("| unified-event | writer | 4 KiB | 0.02 | 300.0 | 12800.0 | 3 | cerulion_device | True |", out)
        self.assertIn("dual (pointer and value)", out)
        self.assertIn("IPC event open (hipIpcOpenEventHandle, us, median): device-vmm-event 210.0", out)
        self.assertIn("**skip** device-ipcmem-event (reader) sizes 4 KiB: hipIpcOpenMemHandle", out)
        self.assertIn("**failed** device-dmabuf-event: unit exited 139", out)
        self.assertIn("| copy | 2 | 13.0 | 26.0 |", out)
        self.assertIn("Device probe: device=fixture, gcn_arch=gfx942", out)

    def test_check_passes_clean_and_fails_on_mismatch_or_missing_rows(self):
        self.write(FIXTURE)
        ok, msgs = summarize.check(self.dir)
        self.assertTrue(ok, msgs)
        self.write(FIXTURE + lat(1, "unified-event", 65536, 1.0, 1.0, mm=3) + "\n")
        ok, msgs = summarize.check(self.dir)
        self.assertFalse(ok)
        self.assertIn("FAIL: mismatches are not 0", msgs)
        self.write("\n".join(l for l in FIXTURE.splitlines() if '"mode":"cpu"' not in l) + "\n")
        ok, msgs = summarize.check(self.dir)
        self.assertFalse(ok)
        self.assertIn("FAIL: no latency row for cpu", msgs)

    def test_unparsable_lines_fail_the_check_and_are_listed(self):
        self.write(FIXTURE + "RESULT {not json\n")
        ok, msgs = summarize.check(self.dir)
        self.assertFalse(ok)
        self.assertIn("Unparsable lines", summarize.summary(self.dir))

    def test_empty_log_summarizes_without_error(self):
        self.write("")
        out = summarize.summary(self.dir)
        self.assertIn("Total mismatches: 0 over 0 frames", out)
        self.assertFalse(summarize.check(self.dir)[0])


if __name__ == "__main__":
    unittest.main()
