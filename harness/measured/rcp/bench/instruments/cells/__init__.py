# SPDX-License-Identifier: AGPL-3.0-only
"""The cell instruments: the frozen lists of outside benchmark cells, the cell log and the walker (ROADMAP A13).

What is here, all standard library only, so any machine and any benchmark's own Python can run it:

    registry.py        the benchmarks file, the arm words, the list files, their hashes and the freeze manifest
    scan.py            reads journals that exist (result files, oracle rows, status files, logs) and lists which
                       cell was run by which arm: the record of what was touched before the cell log existed
    celllog.py         the cell log: one line per run of a cell (time, who, cell, arm, scripted or paid, the
                       snapshot's hash); the guard a launcher asks before it starts; the one confirmation of a
                       headline list and the promotion of a reserve
    walk.py            the walker: fails any RCP-arm run on a headline or reserve cell before that list's one
                       confirmation run, and prints the counts per list
    packages_equal.py  the check that the shipped arm's package list is byte-equal on every bridge
    snapshot.py        the hash of a code tree, the same for every builder
    freeze.py          writes the freeze manifest of a lists folder once; checks it later
    test_cells.py      the tests: venv/bin/python bench/instruments/cells/test_cells.py
    __main__.py        python -m bench.instruments.cells VERB ...

The lists themselves, the script that drew them and the words for an arm are data: bench/cells/outside_v1/.
No file here names a benchmark, a task or a robot, with one exception: the last test of test_cells.py pins the
counts of the real frozen lists by their benchmark keys.
"""

VERSION = "1"
