# SPDX-License-Identifier: AGPL-3.0-only
"""The sim server's settings: the shipped file, every refusal, the paths, the renderer by platform and the
development start state check. Standard library only; no simulator."""
import json
import os
import unittest

from playground_support import FIXTURE, HARNESS, SHIPPED, Folder, write_settings

from playground.settings import (DEFAULT_SETTINGS, Settings, SettingsError, check_states, development_states,
                                 missing)


class ShippedSettings(unittest.TestCase):
    def test_the_shipped_file_loads_and_reads_its_paths_against_the_harness_folder(self):
        s = Settings(DEFAULT_SETTINGS, platform="linux", uid=501, env={"CERULION_HOME": "/home/u/.c"})
        self.assertEqual(s.snapshot_root, os.path.realpath(os.path.join(HARNESS, "measured", "rcp")))
        self.assertEqual(s.bench_root, os.path.realpath(os.path.join(HARNESS, "bx")))
        self.assertEqual(s.body_records, os.path.realpath(os.path.join(HARNESS, "body_records")))
        self.assertEqual(s.data_dir, "/home/u/.c/sim")
        self.assertEqual(s.keep_root, "/home/u/.c/sim/episodes")
        self.assertEqual(s.work_root, "/tmp/pg501")
        self.assertEqual(s.default_state, "bowl-on-plate")
        self.assertEqual(list(s.states), ["bowl-on-plate", "open-drawer", "soup-in-basket"])
        self.assertEqual((s.budget_requests, s.budget_steps, s.clock_s), (80, 900, 5400.0))
        self.assertEqual(s.idle_reap["gap_at_percentile_s"], 43.02)
        self.assertEqual(s.body_record_module, "cerulion_harness.body_record")
        self.assertEqual(s.source_url, "https://github.com/cerulion-inc/cerulion")
        self.assertTrue(s.lazy_records)

    def test_every_shipped_start_state_is_on_the_development_list(self):
        s = Settings(DEFAULT_SETTINGS, platform="linux")
        self.assertEqual(len(development_states()), 35)
        check_states(s, sealed=lambda cid: False)

    def test_the_data_folder_follows_cerulion_home_and_falls_back_to_the_home_folder(self):
        for env in ({}, {"CERULION_HOME": ""}):
            self.assertEqual(Settings(DEFAULT_SETTINGS, platform="linux", env=env).data_dir,
                             os.path.join(os.path.expanduser("~"), ".cerulion", "sim"))
        self.assertEqual(Settings(DEFAULT_SETTINGS, platform="linux", env={"CERULION_HOME": "/srv/c"}).data_dir,
                         "/srv/c/sim")

    def test_linux_renders_with_the_cpu_renderer_and_names_no_macos_host(self):
        s = Settings(DEFAULT_SETTINGS, platform="linux")
        self.assertIsNone(s.envhost)
        self.assertEqual(s.env["PYOPENGL_PLATFORM"], "osmesa")
        self.assertEqual(s.env["OMP_NUM_THREADS"], "1")

    def test_macos_starts_the_simulator_host_through_the_macos_script(self):
        s = Settings(DEFAULT_SETTINGS, platform="darwin")
        self.assertEqual(s.envhost, [os.path.join(HARNESS, "macos", "envhost_macos.sh"), "{runtime}", "{env_sock}",
                                     "{scoring}", "{gl}"])
        self.assertNotIn("PYOPENGL_PLATFORM", s.env)

    def test_two_loads_of_one_file_are_identical(self):
        a, b = Settings(DEFAULT_SETTINGS, platform="linux"), Settings(DEFAULT_SETTINGS, platform="linux")
        self.assertEqual(vars(a), vars(b))


class Refusals(unittest.TestCase):
    def setUp(self):
        self.folder = Folder()
        self.addCleanup(self.folder.remove)

    def refused(self, needle, **changes):
        path = write_settings(self.folder, **changes)
        with self.assertRaises(SettingsError) as caught:
            Settings(path, platform="linux")
        self.assertIn(needle, str(caught.exception))

    def test_an_unknown_key_is_refused_by_name(self):
        self.refused("not a setting: signin", signin={"verifier": "account"})

    def test_a_missing_required_key_is_refused_by_name(self):
        self.refused("missing bench_root", bench_root=None)
        self.refused("missing states", states={})

    def test_a_default_state_outside_the_states_is_refused(self):
        self.refused("default_state 'nowhere' is not one of states", default_state="nowhere")

    def test_a_state_without_six_cell_fields_is_refused(self):
        self.refused("states.x must state its cell as six fields", default_state="x", states={"x": {"cell": "a b"}})

    def test_idle_reap_must_state_the_figure_and_its_source(self):
        self.refused("idle_reap states exactly", idle_reap={"factor": 10, "percentile": 99})
        reap = dict(SHIPPED["idle_reap"], percentile=101)
        self.refused("idle_reap.percentile is at most 100", idle_reap=reap)

    def test_numbers_must_be_positive_and_finite(self):
        self.refused("clock_s must be a number more than 0", clock_s=0)
        self.refused("clock_s must be a number", clock_s="soon")
        self.refused("pool_size must be a number", pool_size=True)
        self.refused("idle_s must be a number more than 0", idle_s=-1)

    def with_null(self, *keys):
        """A settings file that states keys as JSON null (write_settings drops a key it is given None for)."""
        path = write_settings(self.folder)
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
        doc.update(dict.fromkeys(keys))
        with open(path, "w", encoding="utf-8") as f:
            json.dump(doc, f)
        return path

    def test_null_is_refused_for_a_number_setting_and_reads_as_not_set_where_not_set_is_the_default(self):
        for key in ("budget_requests", "budget_steps", "clock_s", "progress_every_s", "call_timeout_s",
                    "keep_full_pictures_s", "keep_viewer_s", "keep_last_pictures", "sweep_every_s", "stop_grace_s"):
            with self.assertRaises(SettingsError, msg=key) as caught:
                Settings(self.with_null(key), platform="linux")
            self.assertEqual(str(caught.exception), f"{key} must be a number, not null")
        s = Settings(self.with_null("idle_s", "pool_size"), platform="linux")
        self.assertEqual((s.idle_s, s.pool_size), (None, None))

    def test_macos_needs_the_host_script(self):
        path = write_settings(self.folder, macos_envhost=None)
        Settings(path, platform="linux")
        with self.assertRaises(SettingsError) as caught:
            Settings(path, platform="darwin")
        self.assertIn("macos_envhost", str(caught.exception))

    def test_a_file_that_is_not_json_or_missing_is_refused(self):
        bad = self.folder.at("bad.json")
        with open(bad, "w", encoding="utf-8") as f:
            f.write("{")
        with self.assertRaises(SettingsError):
            Settings(bad)
        with self.assertRaises(SettingsError):
            Settings(self.folder.at("absent.json"))

    def test_an_unoffered_state_choice_is_refused_with_the_offered_ones(self):
        with self.assertRaises(SettingsError) as caught:
            Settings(DEFAULT_SETTINGS, state="kitchen", platform="linux")
        self.assertIn("'kitchen' is not offered; the start states: bowl-on-plate, open-drawer, soup-in-basket",
                      str(caught.exception))
        self.assertEqual(Settings(DEFAULT_SETTINGS, state="open-drawer", platform="linux").default_state,
                         "open-drawer")


class StatesAndInstall(unittest.TestCase):
    def setUp(self):
        self.folder = Folder()
        self.addCleanup(self.folder.remove)

    def test_a_state_off_the_development_list_is_not_offered(self):
        states = {"x": {"cell": "pro_goal_swap_t09_i02 libero_pro libero_goal_swap 9 2 swap", "title": "x"}}
        s = Settings(write_settings(self.folder, states=states, default_state="x"), platform="linux")
        with self.assertRaises(SettingsError) as caught:
            check_states(s, sealed=lambda cid: False)
        self.assertIn("'x' (pro_goal_swap_t09_i02) is not a development start state", str(caught.exception))

    def test_a_sealed_state_is_not_offered_even_on_the_list(self):
        s = Settings(write_settings(self.folder), platform="linux")
        with self.assertRaises(SettingsError):
            check_states(s, sealed=lambda cid: cid == "pro_object_swap_t00_i00")

    def test_a_seal_that_cannot_be_read_leaves_the_list_check(self):
        s = Settings(write_settings(self.folder), platform="linux")

        def unreadable(cid):
            raise OSError("no lists")
        check_states(s, sealed=unreadable)

    def test_the_measured_code_seal_is_read_from_the_snapshot(self):
        s = Settings(write_settings(self.folder), platform="linux")
        os.environ["FAKE_SEALED"] = "pro_goal_swap_t00_i00"
        self.addCleanup(os.environ.pop, "FAKE_SEALED", None)
        with self.assertRaises(SettingsError) as caught:
            check_states(s)
        self.assertIn("'open-drawer'", str(caught.exception))

    def test_missing_names_every_absent_piece_and_nothing_else(self):
        s = Settings(write_settings(self.folder), platform="linux")
        self.assertEqual(missing(s), [])
        s = Settings(write_settings(self.folder, bench_root=self.folder.at("nobx")), platform="linux")
        self.assertEqual(missing(s), [("the simulator and its scenes", os.path.realpath(self.folder.at("nobx")))])

    def test_the_fixture_snapshot_counts_as_the_measured_code(self):
        self.assertTrue(os.path.isfile(os.path.join(FIXTURE, "rcpx", "mcp_stdio.py")))


if __name__ == "__main__":
    unittest.main()
