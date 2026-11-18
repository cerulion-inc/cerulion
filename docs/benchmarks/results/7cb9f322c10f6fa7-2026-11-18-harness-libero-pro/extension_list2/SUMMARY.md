# Summary of list 2: 400 further start states, 2 models, both arms

Computed by ../recompute.py from this folder's CSV files as read at 2026-11-18T16:16:30Z. Every start state of this list has ended in both arms of both models.

Read these three lines before any number here:

- These are new start states of task rows Cerulion was developed on at other start states. The tasks themselves are not new to it.
- This 400-start-state list was added after the first rows of the 100-start-state grid had been scored and seen. The 100-start-state test declared beforehand remains the main result.
- The reading rule for this list was fixed before its first row. It has a family of twelve tests: two models, this list and a second list on another suite, three measures. Holm's step-down at 0.05 over the twelve; the strictest threshold is 0.05 / 12 = 0.00417. The second list is not in this package, so no test here carries a family verdict. A p at or under the strictest threshold passes whatever the other tests show.

## Every model, both arms

| model | arm | finished | solved | Wilson 95, percent | clean | Wilson 95, percent | false claims | per unsolved episode | unscored | not finished |
|---|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | without Cerulion | 400 | 101 | 21.2 to 29.7 | 52 | 10.1 to 16.7 | 147 | 0.49 | 0 | 0 |
| Gemini 3.8 Flash | with Cerulion | 400 | 296 | 69.5 to 78.1 | 274 | 63.8 to 72.9 | 52 | 0.50 | 0 | 0 |
| GPT-6 Astra | without Cerulion | 400 | 289 | 67.7 to 76.4 | 160 | 35.3 to 44.9 | 89 | 0.80 | 0 | 0 |
| GPT-6 Astra | with Cerulion | 400 | 294 | 69.0 to 77.6 | 259 | 59.9 to 69.3 | 57 | 0.54 | 0 | 0 |

## The six tests on this list

Exact two-sided sign test on the discordant start states, paired by start state. For FALSE CLAIM the favourable side is fewer.

| model | measure | with | without | with only | without only | difference, points (Newcombe 95) | exact p | at or under 0.05 / 12 | scored pairs only: with only, without only, p | task rows: with ahead, behind, tied, p |
|---|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | SOLVED | 296 of 400 | 101 of 400 | 211 | 16 | +48.8 (42.8 to 54.1) | 1.4e-44 | yes | 211, 16, 1.4e-44 | 21, 3, 1, 0.00028 |
| Gemini 3.8 Flash | CLEAN | 274 of 400 | 52 of 400 | 236 | 14 | +55.5 (49.7 to 60.7) | 3.5e-53 | yes | 236, 14, 3.5e-53 | 20, 2, 3, 0.00012 |
| Gemini 3.8 Flash | FALSE CLAIM | 52 of 400 | 147 of 400 | 26 | 121 | -23.8 (-29.1 to -18.2) | 7.5e-16 | yes | 26, 121, 7.5e-16 | 3, 19, 3, 0.00086 |
| GPT-6 Astra | SOLVED | 294 of 400 | 289 of 400 | 57 | 52 | +1.2 (-3.9 to 6.4) | 0.702 | no | 57, 52, 0.702 | 15, 6, 4, 0.078 |
| GPT-6 Astra | CLEAN | 259 of 400 | 160 of 400 | 123 | 24 | +24.8 (19.2 to 30.0) | 3.2e-17 | yes | 123, 24, 3.2e-17 | 19, 4, 2, 0.003 |
| GPT-6 Astra | FALSE CLAIM | 57 of 400 | 89 of 400 | 26 | 58 | -8.0 (-12.5 to -3.6) | 0.00063 | yes | 26, 58, 0.00063 | 3, 15, 7, 0.008 |

The task row columns count the 25 task rows where the arm with Cerulion has more of the measure, fewer, and the same; start states of one task row are not independent, so read the row counts beside the start-state test. Every row is in task_rows.csv.

## False claims per unsolved episode

| model | arm | false claims | unsolved episodes | per unsolved episode |
|---|---|---|---|---|
| Gemini 3.8 Flash | without Cerulion | 147 | 299 | 0.49 |
| Gemini 3.8 Flash | with Cerulion | 52 | 104 | 0.50 |
| GPT-6 Astra | without Cerulion | 89 | 111 | 0.80 |
| GPT-6 Astra | with Cerulion | 57 | 106 | 0.54 |

## By suite and perturbation (counts and intervals only, no test)

| model | suite, perturbation | start states | with: solved (Wilson 95) | without: solved (Wilson 95) |
|---|---|---|---|---|
| Gemini 3.8 Flash | libero_goal, swap | 160 | 109 (60.6 to 74.8) | 46 (22.3 to 36.2) |
| Gemini 3.8 Flash | libero_spatial, swap | 48 | 44 (80.4 to 96.7) | 4 (3.3 to 19.6) |
| Gemini 3.8 Flash | libero_spatial, task | 48 | 43 (77.8 to 95.5) | 6 (5.9 to 24.7) |
| Gemini 3.8 Flash | libero_object, swap | 48 | 40 (70.4 to 91.3) | 17 (23.4 to 49.6) |
| Gemini 3.8 Flash | libero_object, task | 48 | 43 (77.8 to 95.5) | 22 (32.6 to 59.7) |
| Gemini 3.8 Flash | libero_goal, task | 48 | 17 (23.4 to 49.6) | 6 (5.9 to 24.7) |
| GPT-6 Astra | libero_goal, swap | 160 | 114 (63.8 to 77.7) | 118 (66.4 to 80.0) |
| GPT-6 Astra | libero_spatial, swap | 48 | 46 (86.0 to 98.8) | 36 (61.2 to 85.1) |
| GPT-6 Astra | libero_spatial, task | 48 | 42 (75.3 to 94.1) | 36 (61.2 to 85.1) |
| GPT-6 Astra | libero_object, swap | 48 | 35 (59.0 to 83.4) | 34 (56.8 to 81.8) |
| GPT-6 Astra | libero_object, task | 48 | 36 (61.2 to 85.1) | 33 (54.7 to 80.1) |
| GPT-6 Astra | libero_goal, task | 48 | 21 (30.7 to 57.7) | 32 (52.5 to 78.3) |

## The one cross-model pair named beforehand (outside the family): Gemini 3.8 Flash with Cerulion against GPT-6 Astra without

- SOLVED: 296 of 400 against 289 of 400; first only 76, second only 69, p = 0.618 (no difference shown).
- CLEAN: 274 of 400 against 160 of 400; first only 143, second only 29, p = 2.6e-19 (differs; outside the corrected family).

## Means per scored episode

| model | arm | scored episodes | replies | control steps | operator seconds | USD, list price | tokens in | of which cached | tokens out | USD per solved start state |
|---|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | without Cerulion | 400 | 54.7 | 521.0 | 776.9 | 1.607 | 4,175,695 | 2,677,081 | 75,292 | 6.36 |
| Gemini 3.8 Flash | with Cerulion | 400 | 26.9 | 242.7 | 255.7 | 0.392 | 1,605,501 | 1,306,987 | 18,641 | 0.53 |
| GPT-6 Astra | without Cerulion | 400 | 34.2 | 339.2 | 818.6 | 5.387 | 548,963 | 60,647 | 8,860 | 7.46 |
| GPT-6 Astra | with Cerulion | 400 | 15.5 | 257.1 | 235.1 | 2.133 | 626,533 | 468,705 | 1,718 | 2.90 |

## The two lists together (an estimate, not a test)

A pool of two lists: 100 start states whose first rows had been seen when the other 400 were added; 25 task rows, 20 start states each, six suite by perturbation columns; not the published 10 tasks x 50 of one column.

| model | measure | arm | grid 100 (Wilson 95) | list 2 (Wilson 95) | together (Wilson 95) | with only, without only | difference, points (Newcombe 95) | pooled p (contains the main test's 100 start states; not an independent test; not in any family) |
|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | SOLVED | with Cerulion | 73 of 100 (63.6 to 80.7) | 296 of 400 (69.5 to 78.1) | 369 of 500 (69.8 to 77.5) | 263, 20 | +48.6 (43.3 to 53.4) | 3.1e-55 |
| Gemini 3.8 Flash | SOLVED | without Cerulion | 25 of 100 (17.5 to 34.3) | 101 of 400 (21.2 to 29.7) | 126 of 500 (21.6 to 29.2) |  | |  |
| Gemini 3.8 Flash | CLEAN | with Cerulion | 68 of 100 (58.3 to 76.3) | 274 of 400 (63.8 to 72.9) | 342 of 500 (64.2 to 72.3) | 296, 16 | +56.0 (50.9 to 60.6) | 6.6e-68 |
| Gemini 3.8 Flash | CLEAN | without Cerulion | 10 of 100 (5.5 to 17.4) | 52 of 400 (10.1 to 16.7) | 62 of 500 (9.8 to 15.6) |  | |  |
| Gemini 3.8 Flash | FALSE CLAIM | with Cerulion | 16 of 100 (10.1 to 24.4) | 52 of 400 (10.1 to 16.7) | 68 of 500 (10.9 to 16.9) | 32, 141 | -21.8 (-26.5 to -17.0) | 1.6e-17 |
| Gemini 3.8 Flash | FALSE CLAIM | without Cerulion | 30 of 100 (21.9 to 39.6) | 147 of 400 (32.2 to 41.6) | 177 of 500 (31.3 to 39.7) |  | |  |
| GPT-6 Astra | SOLVED | with Cerulion | 70 of 100 (60.4 to 78.1) | 294 of 400 (69.0 to 77.6) | 364 of 500 (68.7 to 76.5) | 70, 66 | +0.8 (-3.8 to 5.4) | 0.797 |
| GPT-6 Astra | SOLVED | without Cerulion | 71 of 100 (61.5 to 79.0) | 289 of 400 (67.7 to 76.4) | 360 of 500 (67.9 to 75.8) |  | |  |
| GPT-6 Astra | CLEAN | with Cerulion | 61 of 100 (51.2 to 70.0) | 259 of 400 (59.9 to 69.3) | 320 of 500 (59.7 to 68.1) | 149, 33 | +23.2 (18.2 to 28.0) | 8.3e-19 |
| GPT-6 Astra | CLEAN | without Cerulion | 44 of 100 (34.7 to 53.8) | 160 of 400 (35.3 to 44.9) | 204 of 500 (36.6 to 45.2) |  | |  |
| GPT-6 Astra | FALSE CLAIM | with Cerulion | 14 of 100 (8.5 to 22.1) | 57 of 400 (11.2 to 18.0) | 71 of 500 (11.4 to 17.5) | 34, 76 | -8.4 (-12.5 to -4.3) | 7.7e-05 |
| GPT-6 Astra | FALSE CLAIM | without Cerulion | 24 of 100 (16.7 to 33.2) | 89 of 400 (18.4 to 26.6) | 113 of 500 (19.2 to 26.5) |  | |  |

## Episodes that ended without a score

None.
