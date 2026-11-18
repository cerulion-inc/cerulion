# SPDX-License-Identifier: AGPL-3.0-only
"""Full grid and calibration cells of the LIBERO and LIBERO-PRO study (the design notes section 4). Standard library only.

The grid uses all ten inits per LIBERO-10 task; day_one remains the 75-cell oracle set.

A cell is a plain dict, the reset argument of the env host:
    id            "l10_t03_i02" (LIBERO-10 task 3, init state 2) or "pro_goal_swap_t03_i00"
    bench         "libero" | "libero_pro"
    runtime       "base" (LIBERO master) | "pro" (LIBERO-PRO's package and files)
    suite         the runtime's benchmark suite: libero_10, or libero_goal_swap, libero_spatial_task, ...
    base_suite    libero_10 | libero_goal | libero_spatial | libero_object
    perturb       "none" | "swap" (LIBERO-PRO position) | "task" (LIBERO-PRO task)
    task, init    task index in the suite's native order; stored init state index
    task_name     the task's BDDL file name (harness side only, never shown to an operator)
    language      THE task sentence: the BDDL file's (:language) text, what env.language_instruction returns
    language_filename  the sentence LIBERO's benchmark derives from the file name (Task.language)
    max_steps_ref the reference evaluations' max_steps for the suite (openpi examples/libero; LIBERO-PRO README)
    budget_steps  900 control steps (20 Hz), the published LLM-operator budget
    env_seed      142 + init (the published LLM-operator harness)
    scene         the LIBERO problem class; surface_z the support surface height (m, world) under the work area
    workspace     end-effector clamp for the plain interface: the published one (x, y within 0.6, z 0.42 to 1.6)
                  on LIBERO-10; on LIBERO-PRO the same box moved with the scene's support surface (declared
                  extension: z from surface_z - 0.017 to surface_z + 1.163, the published offsets from the
                  living-room table top at 0.437)

Why the BDDL sentence: on LIBERO-PRO's task perturbation the file name keeps the ORIGINAL task while the BDDL
file's language and goal predicate are the NEW task (task_name "pick_up_the_alphabet_soup_and_place_it_in_the_basket"
holds goal (in cream_cheese_1 basket_1_contain_region) and language "pick the cream cheese and place it in the
basket"); the file-name sentence would ask for the wrong task. On LIBERO-10 the two sentences are identical.

surface_z was measured by downward ray casts over the work area (hits on the table or floor bodies):
living room 0.437, kitchen 0.900, study 0.885, tabletop 0.900, floor 0.000.
"""

import json
import os
import re

BUDGET_STEPS = 900
ENV_SEED_BASE = 142
MAX_STEPS_REF = {"libero_10": 520, "libero_goal": 300, "libero_spatial": 220, "libero_object": 280}
SURFACE_Z = {"Libero_Living_Room_Tabletop_Manipulation": 0.437, "Libero_Kitchen_Tabletop_Manipulation": 0.900,
             "Libero_Study_Tabletop_Manipulation": 0.885, "Libero_Tabletop_Manipulation": 0.900,
             "Libero_Floor_Manipulation": 0.000}
PUBLISHED_WORKSPACE = {"x": [-0.6, 0.6], "y": [-0.6, 0.6], "z": [0.42, 1.6]}
LIVING, KITCHEN, STUDY = ("Libero_Living_Room_Tabletop_Manipulation", "Libero_Kitchen_Tabletop_Manipulation",
                          "Libero_Study_Tabletop_Manipulation")
TABLETOP, FLOOR = "Libero_Tabletop_Manipulation", "Libero_Floor_Manipulation"

# LIBERO-10 in native order: (task_name, sentence, scene). The sentence is both the BDDL and the file-name one.
LIBERO_10 = (
    ("LIVING_ROOM_SCENE2_put_both_the_alphabet_soup_and_the_tomato_sauce_in_the_basket",
     "put both the alphabet soup and the tomato sauce in the basket", LIVING),
    ("LIVING_ROOM_SCENE2_put_both_the_cream_cheese_box_and_the_butter_in_the_basket",
     "put both the cream cheese box and the butter in the basket", LIVING),
    ("KITCHEN_SCENE3_turn_on_the_stove_and_put_the_moka_pot_on_it",
     "turn on the stove and put the moka pot on it", KITCHEN),
    ("KITCHEN_SCENE4_put_the_black_bowl_in_the_bottom_drawer_of_the_cabinet_and_close_it",
     "put the black bowl in the bottom drawer of the cabinet and close it", KITCHEN),
    ("LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate_and_put_the_yellow_and_white_mug_on_the_right_plate",
     "put the white mug on the left plate and put the yellow and white mug on the right plate", LIVING),
    ("STUDY_SCENE1_pick_up_the_book_and_place_it_in_the_back_compartment_of_the_caddy",
     "pick up the book and place it in the back compartment of the caddy", STUDY),
    ("LIVING_ROOM_SCENE6_put_the_white_mug_on_the_plate_and_put_the_chocolate_pudding_to_the_right_of_the_plate",
     "put the white mug on the plate and put the chocolate pudding to the right of the plate", LIVING),
    ("LIVING_ROOM_SCENE1_put_both_the_alphabet_soup_and_the_cream_cheese_box_in_the_basket",
     "put both the alphabet soup and the cream cheese box in the basket", LIVING),
    ("KITCHEN_SCENE8_put_both_moka_pots_on_the_stove", "put both moka pots on the stove", KITCHEN),
    ("KITCHEN_SCENE6_put_the_yellow_and_white_mug_in_the_microwave_and_close_it",
     "put the yellow and white mug in the microwave and close it", KITCHEN),
)

GOAL_TASKS = ("open_the_middle_drawer_of_the_cabinet", "put_the_bowl_on_the_stove",
              "put_the_wine_bottle_on_top_of_the_cabinet", "open_the_top_drawer_and_put_the_bowl_inside",
              "put_the_bowl_on_top_of_the_cabinet", "push_the_plate_to_the_front_of_the_stove",
              "put_the_cream_cheese_in_the_bowl", "turn_on_the_stove", "put_the_bowl_on_the_plate",
              "put_the_wine_bottle_on_the_rack")
SPATIAL_TASKS = ("pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate",
                 "pick_up_the_black_bowl_next_to_the_ramekin_and_place_it_on_the_plate",
                 "pick_up_the_black_bowl_from_table_center_and_place_it_on_the_plate")
OBJECT_TASKS = ("pick_up_the_alphabet_soup_and_place_it_in_the_basket",
                "pick_up_the_cream_cheese_and_place_it_in_the_basket",
                "pick_up_the_salad_dressing_and_place_it_in_the_basket")

# LIBERO-PRO BDDL sentences (read from each downloaded file's (:language)), per (base_suite, perturb).
PRO_LANGUAGE = {
    ("libero_goal", "swap"): ("open the middle layer of the drawer", "put the bowl on the stove",
                              "put the wine bottle on the top of the drawer",
                              "open the top layer of the drawer and put the bowl inside",
                              "put the bowl on the top of the drawer", "push the plate to the front of the stove",
                              "put the cream cheese on the bowl", "turn on the stove", "put the bowl on the plate",
                              "put the wine bottle on the rack"),
    ("libero_goal", "task"): ("open the bottom drawer of the cabinet", "put the plate on the stove",
                              "put the wine bottle in the bowl"),
    ("libero_spatial", "swap"): ("pick the akita black bowl between the plate and the ramekin and place it on the "
                                 "plate", "pick the akita black bowl next to the ramekin and place it on the plate",
                                 "pick the akita black bowl from table center and place it on the plate"),
    ("libero_spatial", "task"): ("pick the akita black bowl not between the plate and the ramekin and place it on the "
                                 "plate", "pick the akita black bowl next to the cookie box and place it on the plate",
                                 "pick the akita black bowl next to the plate and place it on the plate"),
    ("libero_object", "swap"): ("pick the alphabet soup and place it in the basket",
                                "pick the cream cheese and place it in the basket",
                                "pick the salad dressing and place it in the basket"),
    ("libero_object", "task"): ("pick the cream cheese and place it in the basket",
                                "pick the alphabet soup and place it in the basket",
                                "pick the tomato sauce and place it in the basket"),
}
PRO_TASKS = {"libero_goal": GOAL_TASKS, "libero_spatial": SPATIAL_TASKS, "libero_object": OBJECT_TASKS}
PRO_SCENE = {"libero_goal": TABLETOP, "libero_spatial": TABLETOP, "libero_object": FLOOR}


def _filename_sentence(task_name):
    """LIBERO's grab_language_from_filename: the words after SCENEn_ for LIBERO-10 names, else the whole name."""
    if task_name[0].isupper():
        cut = task_name.find("SCENE")
        return " ".join(task_name[cut + (8 if "SCENE10" in task_name else 7):].split("_"))
    return " ".join(task_name.split("_"))


def _cell(cid, bench, runtime, suite, base_suite, perturb, task, init, task_name, language, scene, workspace):
    return {"id": cid, "bench": bench, "runtime": runtime, "suite": suite, "base_suite": base_suite,
            "perturb": perturb, "task": task, "init": init, "task_name": task_name, "language": language,
            "language_filename": _filename_sentence(task_name), "max_steps_ref": MAX_STEPS_REF[base_suite],
            "budget_steps": BUDGET_STEPS, "env_seed": ENV_SEED_BASE + init, "scene": scene,
            "surface_z": SURFACE_Z[scene], "workspace": workspace}


def scene_workspace(scene):
    """The published clamp moved with the scene's support surface (the LIBERO-PRO extension)."""
    z = SURFACE_Z[scene]
    return {"x": [-0.6, 0.6], "y": [-0.6, 0.6], "z": [round(z - 0.017, 3), round(z + 1.163, 3)]}


def libero10(inits=range(4), tasks=range(10)):
    """LIBERO-10 cells; legacy default inits 0 to 3, grid and published set 0 to 9."""
    out = []
    for t in tasks:
        name, sentence, scene = LIBERO_10[t]
        for i in inits:
            out.append(_cell(f"l10_t{t:02d}_i{i:02d}", "libero", "base", "libero_10", "libero_10", "none", t, i,
                             name, sentence, scene, dict(PUBLISHED_WORKSPACE)))
    return out


def pro_cell(base_suite, perturb, task, init):
    name = PRO_TASKS[base_suite][task]
    short = base_suite.split("_", 1)[1]
    scene = PRO_SCENE[base_suite]
    return _cell(f"pro_{short}_{perturb}_t{task:02d}_i{init:02d}", "libero_pro", "pro", f"{base_suite}_{perturb}",
                 base_suite, perturb, task, init, name, PRO_LANGUAGE[(base_suite, perturb)][task], scene,
                 scene_workspace(scene))


def pro_set_a(inits=(0, 1)):
    """Set A: LIBERO-Goal x position swap, tasks 0 to 9, inits 0 and 1 (20 cells)."""
    return [pro_cell("libero_goal", "swap", t, i) for t in range(10) for i in inits]


def pro_set_b(skip_set_a=True):
    """Set B: {Spatial, Object, Goal} x {swap, task} x tasks 0 to 2, init 0 (18 cells, 15 once set A's three are
    left out)."""
    out = []
    for base in ("libero_spatial", "libero_object", "libero_goal"):
        for perturb in ("swap", "task"):
            if skip_set_a and (base, perturb) == ("libero_goal", "swap"):
                continue
            out.extend(pro_cell(base, perturb, t, 0) for t in range(3))
    return out


def day_one():
    """The 75 day-one cells: LIBERO-10 inits 0 to 3 (40), LIBERO-PRO set A (20) and set B less set A (15)."""
    return libero10() + pro_set_a() + pro_set_b()


# The frozen cell lists (bench/cells/outside_v1, the design rulings): every headline and every reserve list of every
# benchmark named in its benchmarks.json is sealed at every scripted door, whatever the benchmark.
LISTS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "cells",
                         "outside_v1")
SEALED_ROLES = ("headline", "reserve")
_SEALED = {}


def sealed_lists(lists_dir=None):
    """{cell id: (benchmark key, role)} over every headline and reserve list of lists_dir/benchmarks.json (default
    LISTS_DIR). A cell line's first field is its id; a line that starts with '#' is a comment. A missing or unreadable
    benchmarks.json or list file raises: a door that cannot read the seal builds nothing."""
    return _seal(lists_dir)[0]


def _seal(lists_dir=None):
    """(sealed_lists, [(id regex, development ids)] of every benchmark that has a headline list)."""
    d = os.path.abspath(lists_dir or LISTS_DIR)
    if d in _SEALED:
        return _SEALED[d]
    with open(os.path.join(d, "benchmarks.json")) as f:
        benches = json.load(f)["benchmarks"]
    out, pools = {}, []
    for key, b in benches.items():
        lists = b.get("lists") or {}
        for role in SEALED_ROLES:
            if lists.get(role):
                out.update((cid, (key, role)) for cid in _list_ids(os.path.join(d, b["dir"], lists[role]))
                           if cid not in out)
        if lists.get("headline") and b.get("id_regex"):
            dev = _list_ids(os.path.join(d, b["dir"], lists["development"])) if lists.get("development") else []
            pools.append((key, re.compile(b["id_regex"]), frozenset(dev)))
    if not out:
        raise RuntimeError(f"no sealed cell read from {d}: the seal cannot be read")
    _SEALED[d] = (out, pools)
    return _SEALED[d]


def _list_ids(path):
    with open(path) as f:
        return [line.split()[0] for line in (x.strip() for x in f) if line and not line.startswith("#")]


def _l10_late_init(cid):
    parts = str(cid).split("_")
    return parts[0] == "l10" and parts[-1][:1] == "i" and parts[-1][1:].isdigit() and int(parts[-1][1:]) >= 4


def _unlisted(cid, lists_dir=None):
    """The benchmark key when cid is a cell of a benchmark that has a headline list (its id regex) and is on none of
    its development list, else None."""
    return next((key for key, rx, dev in _seal(lists_dir)[1] if rx.fullmatch(str(cid)) and str(cid) not in dev),
                None)


def sealed(cid, lists_dir=None):
    """True for a cell never run and never rendered by the gates' instruments, the tests or the oracles' development
    doors (bench/ext/oracle_rcpx.py, oracle_ext.py): a cell of any headline or reserve list of bench/cells/outside_v1
    (every benchmark of its benchmarks.json), a cell of a benchmark with a headline list that is on none of its
    development list (LIBERO-PRO's initial states the lists leave out), and every LIBERO-10 cell of an initial state
    of 4 or more."""
    return _l10_late_init(cid) or str(cid) in sealed_lists(lists_dir) or _unlisted(cid, lists_dir) is not None


def refuse_sealed(cid, lists_dir=None):
    """Raise before anything is built for a sealed cell."""
    if sealed(cid, lists_dir):
        where = sealed_lists(lists_dir).get(str(cid))
        other = _unlisted(cid, lists_dir)
        what = (f"on the {where[1]} list of {where[0]} (bench/cells/outside_v1)" if where else
                f"a cell of {other} on no development list (bench/cells/outside_v1)" if other else
                "a LIBERO-10 initial state of 4 or more")
        raise ValueError(f"refused: {cid} is a sealed cell, {what}: a scripted door never builds it")


def by_id(cid):
    """The cell with that id, from any list this file makes (LIBERO-10 inits 0 to 49, LIBERO-PRO any init)."""
    parts = cid.split("_")
    try:
        task, init = int(parts[-2][1:]), int(parts[-1][1:])
        if parts[0] == "l10" and len(parts) == 3:
            return libero10(inits=[init], tasks=[task])[0]
        if parts[0] == "pro" and len(parts) == 5:
            return pro_cell(f"libero_{parts[1]}", parts[2], task, init)
    except (ValueError, IndexError, KeyError):
        pass
    raise KeyError(f"no cell {cid!r}")


def libero10_full():
    """All 100 published cells, in task then init order."""
    return libero10(inits=range(10))


def calib():
    """Calibration: ten tasks x inits 0 and 1, 20 cells."""
    return libero10(inits=range(2))


SETS = {"libero10": libero10_full, "calib": calib, "pro_set_a": pro_set_a, "pro_set_b": pro_set_b,
        "day_one": day_one}
