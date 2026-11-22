# SPDX-License-Identifier: AGPL-3.0-only
"""Cell ids: the start state run-cell runs, and the scene a robot body's record is measured in. Standard library
only; the expected lines are written by hand."""
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from cerulion_harness import body_record, cli  # noqa: E402


class CellLines(unittest.TestCase):
    def test_a_libero_pro_cell_names_its_suite_perturbation_task_and_start_state(self):
        self.assertEqual(cli.cell_line("pro_spatial_swap_t00_i00"),
                         "pro_spatial_swap_t00_i00 libero_pro libero_spatial_swap 0 0 swap")
        self.assertEqual(cli.cell_line("pro_goal_task_t03_i07"),
                         "pro_goal_task_t03_i07 libero_pro libero_goal_task 3 7 task")

    def test_a_libero_10_cell_has_no_perturbation(self):
        self.assertEqual(cli.cell_line("l10_t09_i12"), "l10_t09_i12 libero libero_10 9 12 none")

    def test_the_default_cell_is_a_cell(self):
        self.assertEqual(cli.cell_line(cli.DEFAULT_CELL).split()[0], cli.DEFAULT_CELL)
        for cid, _ in cli.DEVELOPMENT_CELLS:
            self.assertEqual(cli.cell_line(cid).split()[0], cid)

    def test_anything_else_is_refused_with_the_forms_it_takes(self):
        for bad in ("", "pro_spatial_swap_t00", "pro_spatial_shift_t00_i00", "PRO_spatial_swap_t00_i00",
                    "l10_t00_i00 ; rm -rf /", "../pro_spatial_swap_t00_i00", "l10_tx_i00", "pro__swap_t00_i00"):
            with self.assertRaises(ValueError) as ctx:
                cli.cell_line(bad)
            self.assertIn("pro_SUITE_swap_tNN_iNN", str(ctx.exception))


class BodyScenes(unittest.TestCase):
    """Which scene the self-test measures a start state's robot body in (the shipped record's scene)."""

    def test_the_spatial_and_goal_suites_and_libero_10_task_9_share_one_body(self):
        for cid in ("pro_spatial_swap_t00_i00", "pro_goal_task_t04_i02", "l10_t09_i00", "l10_t9_i3"):
            self.assertEqual(body_record.scene_of(cid), "l10_t02_i00", cid)

    def test_libero_10_tasks_0_and_6_share_another(self):
        for cid in ("l10_t00_i05", "l10_t06_i01", "l10_t0_i0"):
            self.assertEqual(body_record.scene_of(cid), "l10_t00_i00", cid)

    def test_the_object_suite_has_its_own(self):
        self.assertEqual(body_record.scene_of("pro_object_swap_t07_i03"), "pro_object_swap_t00_i00")

    def test_any_other_start_state_is_measured_in_its_own_scene(self):
        for cid in ("l10_t19_i00", "l10_t01_i00", "pro_10_swap_t00_i00"):
            self.assertEqual(body_record.scene_of(cid), cid, cid)

    def test_each_shipped_body_scene_maps_to_itself(self):
        for scene, _, _ in body_record.BODIES:
            self.assertEqual(body_record.scene_of(scene), scene)


if __name__ == "__main__":
    unittest.main()
