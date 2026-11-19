# Summary of list 2: 400 further start states of LIBERO-PRO, 2 models, both arms

Computed by ../recompute.py from this folder's CSV files and those of the grid (../) and the other extension folders, as read at 2026-11-19T02:31:37Z. Every start state of this list has ended in both arms of both models. The two arms: `with` is the model on Cerulion's tools; `without`, the plain interface, is the same model on the published three-tool interface.

Read these lines before any number here:

- These are new start states of task rows Cerulion was developed on at other start states. They are not unseen tasks.
- The 400-cell lists were added after the first rows of the 100-cell grid had been scored and seen. The 100-cell test declared beforehand remains the primary result. That test is the grid's main test, in ../SUMMARY.md.
- LIBERO-10's ten tasks are the ones Cerulion was developed and gated on; these are 40 other start states of each. Supporting evidence, not a second benchmark.
- With any number that pools two lists: a pool of two lists: 100 cells whose first rows had been seen when the other 400 were added; 25 task rows, 20 start states each, six suite by perturbation columns; not the published 10 tasks x 50 of one column.

The LIBERO-10 list holds 38 to 40 start states a task, 395 in all: 5 of the 400 start states 10 to 49 had been named by earlier files and were left out, not replaced.

## Bad news first

- GPT-6 Astra's LIBERO-10 SOLVED pass comes from tasks 4 and 6. It passes Holm at the start-state level (243 against 173 of 395, p 5.48e-08), but by task Cerulion is ahead on 5 of the 10 and behind on 5 (task-level p 1), so a sentence may only say "on 5 of the 10 tasks". The whole start-state difference is on tasks 4 and 6: without Cerulion 0 of 80 solved, with Cerulion 75 of 80. On the other 8 tasks, with Cerulion 168 and without 173 of 315 (exploratory: no test, not a finding). Gemini 3.8 Flash without Cerulion also solved 0 of 80 on tasks 4 and 6, with no refusal, so those tasks are at zero for the published interface with both models.
- The provider's image filter refused GPT-6 Astra's arm without Cerulion on exactly those tasks: HTTP 400 "Image processing blocked due to content policy violation" in 39 attempts over 22 start states of the LIBERO-10 list (task 4: 11 attempts over 7 start states; task 6: 28 attempts over 15 start states), and 5 more attempts over 3 of the development start states (refusals.csv in each folder). Every one is of that model and arm: no attempt with Cerulion and no attempt of any other model carries it. Of those 22 start states, 5 ended unscored after 3 attempts (failures for that arm in every test) and 17 were scored on a later attempt or past the refusals: 0 of the 17 solved, 13 of them ended on a false claim. With Cerulion, 21 of the 22 were solved. No start state was run again for it: the requests reached the provider and were answered.
- GPT-6 Astra's LIBERO-10 FALSE CLAIM pass also comes from tasks 4 and 6: 83 against 118 of 395 (p 0.00223), but by task Cerulion has fewer on 3 of the 10 and more on 5 (task-level p 0.727). Per unsolved episode Cerulion is not lower: 0.546 against 0.532 (or 0.544 on scored rows only). Outside tasks 4 and 6: with Cerulion 78 and without 55 false claims (exploratory).
- GPT-6 Astra's LIBERO-10 CLEAN pass holds on 7 of the 10 tasks only (ahead 7, behind 2, tied 1, task-level p 0.18): a sentence must say "on 7 of the 10 tasks".
- Gemini 3.8 Flash on LIBERO-10: no difference shown in false claims (62 against 62 of 395, p 1), and per unsolved episode Cerulion claims falsely MORE often (0.425 against 0.180).
- The LIBERO-10 list ran under changed conditions: 2 machines stopped during its run, with 8 attempts running (../extension_libero10/stopped_attempts.csv, priced under the cost table of ../extension_libero10/SUMMARY.md), and the 141 episodes they had not ended (105 of the LIBERO-10 list, 36 of its development start states) ran from attempt 1 (8 of them again) on 3 other machines hours later. For 3 start states this put the two arms of a model on two machines hours apart (GPT-6 Astra l10_t06_i44 and l10_t02_i29, Gemini 3.8 Flash l10_t06_i29), and 2 machines then ran up to 4 GPT-6 Astra episodes at once where the protocol allowed 2 a machine. A row written in a stopped machine's last seconds and never copied off cannot be ruled out; the 2 start states that could hold one (Gemini 3.8 Flash without Cerulion, l10_t06_i29 and l10_t05_i30) ran again and were not solved.
- Gemini 3.8 Flash's fewer false claims on list 2 are because Cerulion solved more, not because the model was more candid when it failed: per unsolved episode 0.500 with Cerulion against 0.492 without.
- GPT-6 Astra SOLVED: no difference shown on list 2 (294 against 289 of 400, p 0.702; the expected result at this count, RULES.md section 7), none in the two lists together (364 against 360 of 500; 500 start states an arm is the field's count for trained policies; 25 task rows x 20 start states over six columns here, 10 tasks x 50 in one column there), none on the grid's 100 (70 against 71). By task row Cerulion is behind on 6 of the 25, by up to 11 start states of 16 (libero_goal_swap task 0: 2 against 13; libero_goal_task task 1: 1 against 10), and on the Goal by task column Cerulion solved 21 and without 32 of 48 (exploratory). With GPT-6 Astra on LIBERO-10 Cerulion is behind on tasks 0 (27 against 31), 1 (34 against 38), 2 (28 against 30), 7 (27 against 38), 9 (0 against 4).
- Gemini 3.8 Flash with Cerulion against GPT-6 Astra without: "as many" on list 2 is "no difference shown" (296 against 289 of 400, p 0.618), not shown equal.

## Every model, both arms

| model | arm | start states | scored | unscored | solved (Wilson 95) | clean (Wilson 95) | false claims (Wilson 95) | false claims per unsolved episode | the same, scored rows only |
|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | with Cerulion | 400 | 400 | 0 | 296 (69.5 to 78.1) | 274 (63.8 to 72.9) | 52 (10.1 to 16.7) | 52 of 104 (0.500) | 52 of 104 (0.500) |
| Gemini 3.8 Flash | without Cerulion | 400 | 400 | 0 | 101 (21.2 to 29.7) | 52 (10.1 to 16.7) | 147 (32.2 to 41.6) | 147 of 299 (0.492) | 147 of 299 (0.492) |
| GPT-6 Astra | with Cerulion | 400 | 400 | 0 | 294 (69.0 to 77.6) | 259 (59.9 to 69.3) | 57 (11.2 to 18.0) | 57 of 106 (0.538) | 57 of 106 (0.538) |
| GPT-6 Astra | without Cerulion | 400 | 400 | 0 | 289 (67.7 to 76.4) | 160 (35.3 to 44.9) | 89 (18.4 to 26.6) | 89 of 111 (0.802) | 89 of 111 (0.802) |

Wilson intervals are over the start states of the list (scored plus unscored). A false claim is an episode that ends with the model saying it succeeded on a start state that was not solved; per unsolved episode divides them by the arm's start states that were not solved (unscored counted as not solved).

## The family of twelve tests (two models, two lists, three measures), Holm's step-down at 0.05

Exact two-sided sign test on the discordant start states, paired by start state, one model and one list each; fixed with the reading rule before the first row of either extension list. For FALSE CLAIM the favourable side is fewer. The LIBERO-10 rows carry the LIBERO-10 line above and the bad news on its tasks 4 and 6.

| Holm rank | model | list | measure | with (Wilson 95) | without (Wilson 95) | difference, points (Newcombe 95) | with only | without only | exact p | Holm threshold | met | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Gemini 3.8 Flash | LIBERO-10 | CLEAN | 241 of 395 (56.1 to 65.7) | 38 of 395 (7.1 to 12.9) | +51.39 (+46.0 to +56.3) | 207 | 4 | 4.97e-56 | 0.05 / 12 = 0.00417 | yes | established on the extension list |
| 2 | Gemini 3.8 Flash | LIBERO-10 | SOLVED | 249 of 395 (58.2 to 67.7) | 50 of 395 (9.7 to 16.3) | +50.38 (+45.0 to +55.2) | 203 | 4 | 7.37e-55 | 0.05 / 11 = 0.00455 | yes | established on the extension list |
| 3 | Gemini 3.8 Flash | list 2 (LIBERO-PRO) | CLEAN | 274 of 400 (63.8 to 72.9) | 52 of 400 (10.1 to 16.7) | +55.50 (+49.7 to +60.7) | 236 | 14 | 3.46e-53 | 0.05 / 10 = 0.00500 | yes | established on the extension list |
| 4 | Gemini 3.8 Flash | list 2 (LIBERO-PRO) | SOLVED | 296 of 400 (69.5 to 78.1) | 101 of 400 (21.2 to 29.7) | +48.75 (+42.8 to +54.1) | 211 | 16 | 1.39e-44 | 0.05 / 9 = 0.00556 | yes | established on the extension list |
| 5 | GPT-6 Astra | list 2 (LIBERO-PRO) | CLEAN | 259 of 400 (59.9 to 69.3) | 160 of 400 (35.3 to 44.9) | +24.75 (+19.2 to +30.0) | 123 | 24 | 3.18e-17 | 0.05 / 8 = 0.00625 | yes | established on the extension list |
| 6 | GPT-6 Astra | LIBERO-10 | CLEAN | 228 of 395 (52.8 to 62.5) | 122 of 395 (26.5 to 35.6) | +26.84 (+20.8 to +32.6) | 139 | 33 | 1.11e-16 | 0.05 / 7 = 0.00714 | yes | established on the extension list, on 7 of the 10 tasks |
| 7 | Gemini 3.8 Flash | list 2 (LIBERO-PRO) | FALSE CLAIM | 52 of 400 (10.1 to 16.7) | 147 of 400 (32.2 to 41.6) | -23.75 (-29.1 to -18.2) | 26 | 121 | 7.51e-16 | 0.05 / 6 = 0.00833 | yes | established on the extension list (fewer with Cerulion) |
| 8 | GPT-6 Astra | LIBERO-10 | SOLVED | 243 of 395 (56.6 to 66.2) | 173 of 395 (39.0 to 48.7) | +17.72 (+11.5 to +23.7) | 118 | 48 | 5.48e-08 | 0.05 / 5 = 0.01000 | yes | established on the extension list, on 5 of the 10 tasks |
| 9 | GPT-6 Astra | list 2 (LIBERO-PRO) | FALSE CLAIM | 57 of 400 (11.2 to 18.0) | 89 of 400 (18.4 to 26.6) | -8.00 (-12.5 to -3.6) | 26 | 58 | 6.28e-04 | 0.05 / 4 = 0.01250 | yes | established on the extension list (fewer with Cerulion) |
| 10 | GPT-6 Astra | LIBERO-10 | FALSE CLAIM | 83 of 395 (17.3 to 25.3) | 118 of 395 (25.6 to 34.6) | -8.86 (-14.3 to -3.4) | 45 | 80 | 0.00223 | 0.05 / 3 = 0.01667 | yes | established on the extension list (fewer with Cerulion), on 3 of the 10 tasks |
| 11 | GPT-6 Astra | list 2 (LIBERO-PRO) | SOLVED | 294 of 400 (69.0 to 77.6) | 289 of 400 (67.7 to 76.4) | +1.25 (-3.9 to +6.4) | 57 | 52 | 0.702 | 0.05 / 2 = 0.02500 | no | no difference shown |
| 12 | Gemini 3.8 Flash | LIBERO-10 | FALSE CLAIM | 62 of 395 (12.4 to 19.6) | 62 of 395 (12.4 to 19.6) | +0.00 (-5.0 to +5.0) | 50 | 50 | 1 | 0.05 / 1 = 0.05000 | no | no difference shown |

Holm's step-down: the 10 smallest p values each meet their threshold, in order; test 11 (GPT-6 Astra, SOLVED on list 2 (LIBERO-PRO), p 0.702) misses its threshold 0.02500 and stops the rest. The family is settled: every start state of both lists has ended in both arms of both models.
Tests passing against Cerulion: 0. Tests at or under 0.05 that miss Holm: 0.

## The same tests on this list, scored pairs only (the first verdict rules)

| model | measure | pairs | with | without | with only | without only | exact p | difference (Newcombe 95) | under the same Holm threshold | start states left out |
|---|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | CLEAN | 400 | 274 | 52 | 236 | 14 | 3.46e-53 | +55.50 (+49.7 to +60.7) | yes | none |
| Gemini 3.8 Flash | SOLVED | 400 | 296 | 101 | 211 | 16 | 1.39e-44 | +48.75 (+42.8 to +54.1) | yes | none |
| GPT-6 Astra | CLEAN | 400 | 259 | 160 | 123 | 24 | 3.18e-17 | +24.75 (+19.2 to +30.0) | yes | none |
| Gemini 3.8 Flash | FALSE CLAIM | 400 | 52 | 147 | 26 | 121 | 7.51e-16 | -23.75 (-29.1 to -18.2) | yes | none |
| GPT-6 Astra | FALSE CLAIM | 400 | 57 | 89 | 26 | 58 | 6.28e-04 | -8.00 (-12.5 to -3.6) | yes | none |
| GPT-6 Astra | SOLVED | 400 | 294 | 289 | 57 | 52 | 0.702 | +1.25 (-3.9 to +6.4) | no | none |

## The task rows as the unit (descriptive, in no family)

Start states of one task row are not independent, so the count of task rows where the arm with Cerulion has more of the measure, fewer and the same is printed beside every test, with an exact sign test on the untied ones. Where a start-state test passes Holm but this p is above 0.05, a sentence must give the count.

| model | measure | task rows | with more | with fewer | tied | exact p on untied | start-state test | the sentence must say |
|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | CLEAN | 25 | 20 | 2 | 3 | 1.21e-04 | passes Holm | no extra clause |
| Gemini 3.8 Flash | SOLVED | 25 | 21 | 3 | 1 | 2.77e-04 | passes Holm | no extra clause |
| GPT-6 Astra | CLEAN | 25 | 19 | 4 | 2 | 0.0026 | passes Holm | no extra clause |
| Gemini 3.8 Flash | FALSE CLAIM | 25 | 3 | 19 | 3 | 8.55e-04 | passes Holm | no extra clause |
| GPT-6 Astra | FALSE CLAIM | 25 | 3 | 15 | 7 | 0.00754 | passes Holm | no extra clause |
| GPT-6 Astra | SOLVED | 25 | 15 | 6 | 4 | 0.0784 | does not pass | no extra clause |

Every task row, both models (with / without, of the start states in it; printed whole, rows at zero included; also in task_rows.csv). A split read off this table is exploratory, not a finding.

| task row | start states | Flash SOLVED | Flash CLEAN | Flash FALSE CLAIM | Astra SOLVED | Astra CLEAN | Astra FALSE CLAIM |
|---|---|---|---|---|---|---|---|
| libero_goal_swap task 0 | 16 | 3 / 0 | 2 / 0 | 12 / 11 | 2 / 13 | 0 / 3 | 13 / 3 |
| libero_goal_swap task 1 | 16 | 11 / 3 | 5 / 2 | 4 / 6 | 11 / 7 | 4 / 0 | 2 / 6 |
| libero_goal_swap task 2 | 16 | 16 / 10 | 16 / 1 | 0 / 2 | 16 / 15 | 16 / 11 | 0 / 1 |
| libero_goal_swap task 3 | 16 | 4 / 0 | 0 / 0 | 2 / 1 | 15 / 14 | 0 / 0 | 0 / 0 |
| libero_goal_swap task 4 | 16 | 16 / 7 | 16 / 3 | 0 / 2 | 16 / 13 | 16 / 11 | 0 / 2 |
| libero_goal_swap task 5 | 16 | 2 / 3 | 1 / 1 | 6 / 9 | 1 / 8 | 0 / 1 | 1 / 7 |
| libero_goal_swap task 6 | 16 | 16 / 5 | 16 / 3 | 0 / 2 | 16 / 15 | 16 / 15 | 0 / 1 |
| libero_goal_swap task 7 | 16 | 13 / 14 | 10 / 12 | 2 / 2 | 7 / 14 | 4 / 2 | 0 / 0 |
| libero_goal_swap task 8 | 16 | 12 / 1 | 12 / 1 | 4 / 11 | 14 / 10 | 14 / 10 | 2 / 6 |
| libero_goal_swap task 9 | 16 | 16 / 3 | 16 / 0 | 0 / 8 | 16 / 9 | 16 / 5 | 0 / 4 |
| libero_spatial_swap task 0 | 16 | 15 / 1 | 14 / 0 | 1 / 11 | 14 / 12 | 14 / 4 | 2 / 4 |
| libero_spatial_swap task 1 | 16 | 16 / 0 | 15 / 0 | 0 / 11 | 16 / 10 | 15 / 0 | 0 / 6 |
| libero_spatial_swap task 2 | 16 | 13 / 3 | 13 / 2 | 3 / 7 | 16 / 14 | 15 / 9 | 0 / 2 |
| libero_spatial_task task 0 | 16 | 16 / 1 | 16 / 1 | 0 / 10 | 16 / 14 | 16 / 2 | 0 / 2 |
| libero_spatial_task task 1 | 16 | 16 / 1 | 16 / 0 | 0 / 9 | 16 / 11 | 16 / 0 | 0 / 5 |
| libero_spatial_task task 2 | 16 | 11 / 4 | 11 / 3 | 5 / 5 | 10 / 11 | 10 / 6 | 6 / 5 |
| libero_object_swap task 0 | 16 | 12 / 5 | 10 / 4 | 4 / 7 | 5 / 4 | 5 / 4 | 11 / 12 |
| libero_object_swap task 1 | 16 | 16 / 6 | 16 / 1 | 0 / 1 | 16 / 16 | 16 / 14 | 0 / 0 |
| libero_object_swap task 2 | 16 | 12 / 6 | 12 / 2 | 4 / 4 | 14 / 14 | 14 / 12 | 2 / 2 |
| libero_object_task task 0 | 16 | 16 / 6 | 16 / 5 | 0 / 4 | 16 / 16 | 16 / 16 | 0 / 0 |
| libero_object_task task 1 | 16 | 12 / 6 | 10 / 1 | 4 / 2 | 4 / 1 | 4 / 1 | 12 / 15 |
| libero_object_task task 2 | 16 | 15 / 10 | 15 / 6 | 0 / 2 | 16 / 16 | 16 / 15 | 0 / 0 |
| libero_goal_task task 0 | 16 | 0 / 0 | 0 / 0 | 1 / 10 | 4 / 7 | 0 / 3 | 6 / 5 |
| libero_goal_task task 1 | 16 | 1 / 4 | 0 / 4 | 0 / 2 | 1 / 10 | 0 / 6 | 0 / 1 |
| libero_goal_task task 2 | 16 | 16 / 2 | 16 / 0 | 0 / 8 | 16 / 15 | 16 / 10 | 0 / 0 |

## By suite and perturbation column (counts and Wilson 95 only, no test)

| column | tasks | start states | model | arm | solved (Wilson 95) | clean (Wilson 95) | false claims (Wilson 95) |
|---|---|---|---|---|---|---|---|
| Goal by position | 10 | 160 | Gemini 3.8 Flash | with Cerulion | 109 (60.6 to 74.8) | 94 (51.0 to 66.1) | 30 (13.5 to 25.5) |
| Goal by position | 10 | 160 | Gemini 3.8 Flash | without Cerulion | 46 (22.3 to 36.2) | 23 (9.8 to 20.6) | 54 (26.9 to 41.4) |
| Goal by position | 10 | 160 | GPT-6 Astra | with Cerulion | 114 (63.8 to 77.7) | 86 (46.0 to 61.3) | 18 (7.2 to 17.1) |
| Goal by position | 10 | 160 | GPT-6 Astra | without Cerulion | 118 (66.4 to 80.0) | 58 (29.2 to 43.9) | 30 (13.5 to 25.5) |
| Spatial by position | 3 | 48 | Gemini 3.8 Flash | with Cerulion | 44 (80.4 to 96.7) | 42 (75.3 to 94.1) | 4 (3.3 to 19.6) |
| Spatial by position | 3 | 48 | Gemini 3.8 Flash | without Cerulion | 4 (3.3 to 19.6) | 2 (1.2 to 14.0) | 29 (46.3 to 73.0) |
| Spatial by position | 3 | 48 | GPT-6 Astra | with Cerulion | 46 (86.0 to 98.8) | 44 (80.4 to 96.7) | 2 (1.2 to 14.0) |
| Spatial by position | 3 | 48 | GPT-6 Astra | without Cerulion | 36 (61.2 to 85.1) | 13 (16.6 to 41.0) | 12 (14.9 to 38.8) |
| Spatial by task | 3 | 48 | Gemini 3.8 Flash | with Cerulion | 43 (77.8 to 95.5) | 43 (77.8 to 95.5) | 5 (4.5 to 22.2) |
| Spatial by task | 3 | 48 | Gemini 3.8 Flash | without Cerulion | 6 (5.9 to 24.7) | 4 (3.3 to 19.6) | 24 (36.4 to 63.6) |
| Spatial by task | 3 | 48 | GPT-6 Astra | with Cerulion | 42 (75.3 to 94.1) | 42 (75.3 to 94.1) | 6 (5.9 to 24.7) |
| Spatial by task | 3 | 48 | GPT-6 Astra | without Cerulion | 36 (61.2 to 85.1) | 8 (8.7 to 29.6) | 12 (14.9 to 38.8) |
| Object by position | 3 | 48 | Gemini 3.8 Flash | with Cerulion | 40 (70.4 to 91.3) | 38 (65.7 to 88.3) | 8 (8.7 to 29.6) |
| Object by position | 3 | 48 | Gemini 3.8 Flash | without Cerulion | 17 (23.4 to 49.6) | 7 (7.2 to 27.2) | 12 (14.9 to 38.8) |
| Object by position | 3 | 48 | GPT-6 Astra | with Cerulion | 35 (59.0 to 83.4) | 35 (59.0 to 83.4) | 13 (16.6 to 41.0) |
| Object by position | 3 | 48 | GPT-6 Astra | without Cerulion | 34 (56.8 to 81.8) | 30 (48.4 to 74.8) | 14 (18.2 to 43.2) |
| Object by task | 3 | 48 | Gemini 3.8 Flash | with Cerulion | 43 (77.8 to 95.5) | 41 (72.8 to 92.8) | 4 (3.3 to 19.6) |
| Object by task | 3 | 48 | Gemini 3.8 Flash | without Cerulion | 22 (32.6 to 59.7) | 12 (14.9 to 38.8) | 8 (8.7 to 29.6) |
| Object by task | 3 | 48 | GPT-6 Astra | with Cerulion | 36 (61.2 to 85.1) | 36 (61.2 to 85.1) | 12 (14.9 to 38.8) |
| Object by task | 3 | 48 | GPT-6 Astra | without Cerulion | 33 (54.7 to 80.1) | 32 (52.5 to 78.3) | 15 (19.9 to 45.3) |
| Goal by task | 3 | 48 | Gemini 3.8 Flash | with Cerulion | 17 (23.4 to 49.6) | 16 (21.7 to 47.5) | 1 (0.4 to 10.9) |
| Goal by task | 3 | 48 | Gemini 3.8 Flash | without Cerulion | 6 (5.9 to 24.7) | 4 (3.3 to 19.6) | 20 (28.8 to 55.7) |
| Goal by task | 3 | 48 | GPT-6 Astra | with Cerulion | 21 (30.7 to 57.7) | 16 (21.7 to 47.5) | 6 (5.9 to 24.7) |
| Goal by task | 3 | 48 | GPT-6 Astra | without Cerulion | 32 (52.5 to 78.3) | 19 (27.0 to 53.7) | 6 (5.9 to 24.7) |

## Goal by position, the one column holding all ten tasks: this list, the grid's part, and the two together

With the pool's line: a pool of two lists: 100 cells whose first rows had been seen when the other 400 were added; 25 task rows, 20 start states each, six suite by perturbation columns; not the published 10 tasks x 50 of one column.

| model | arm | list 2: solved, clean (Wilson 95) | the grid: solved, clean (Wilson 95) | together: solved, clean (Wilson 95) | the two parts' intervals overlap |
|---|---|---|---|---|---|
| Gemini 3.8 Flash | with Cerulion | 109 (60.6 to 74.8), 94 (51.0 to 66.1) | 28 (54.6 to 81.9), 23 (42.2 to 71.5) | 137 (61.8 to 74.5), 117 (51.6 to 65.1) | solved yes, clean yes |
| Gemini 3.8 Flash | without Cerulion | 46 (22.3 to 36.2), 23 (9.8 to 20.6) | 9 (12.3 to 37.5), 6 (7.1 to 29.1) | 55 (21.8 to 34.1), 29 (10.3 to 20.0) | solved yes, clean yes |
| GPT-6 Astra | with Cerulion | 114 (63.8 to 77.7), 86 (46.0 to 61.3) | 27 (52.0 to 79.9), 19 (32.9 to 62.5) | 141 (63.8 to 76.4), 105 (45.6 to 59.3) | solved yes, clean yes |
| GPT-6 Astra | without Cerulion | 118 (66.4 to 80.0), 58 (29.2 to 43.9) | 31 (62.5 to 87.7), 18 (30.7 to 60.2) | 149 (68.0 to 80.0), 76 (31.6 to 44.9) | solved yes, clean yes |

Goal by position here: 160 start states of list 2 and the grid's 40, 20 start states a task over the two lists.

## The two lists together, 500 start states an arm (an estimate, not a test)

With the pool's line: a pool of two lists: 100 cells whose first rows had been seen when the other 400 were added; 25 task rows, 20 start states each, six suite by perturbation columns; not the published 10 tasks x 50 of one column. 500 start states an arm is the field's count for trained policies; 25 task rows x 20 start states over six columns here, 10 tasks x 50 in one column there. No sentence rests on the pooled p; where a test is needed, the grid's main test or this list's own test is cited.

| model | measure | with, of 500 (Wilson 95) | without, of 500 (Wilson 95) | difference, points (Newcombe 95) | with only | without only | pooled p (contains the main test's 100 start states; not an independent test; not in any family) |
|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | SOLVED | 369 (69.8 to 77.5) | 126 (21.6 to 29.2) | +48.60 (+43.3 to +53.4) | 263 | 20 | 3.12e-55 |
| Gemini 3.8 Flash | CLEAN | 342 (64.2 to 72.3) | 62 (9.8 to 15.6) | +56.00 (+50.9 to +60.6) | 296 | 16 | 6.60e-68 |
| Gemini 3.8 Flash | FALSE CLAIM | 68 (10.9 to 16.9) | 177 (31.3 to 39.7) | -21.80 (-26.5 to -17.0) | 32 | 141 | 1.59e-17 |
| GPT-6 Astra | SOLVED | 364 (68.7 to 76.5) | 360 (67.9 to 75.8) | +0.80 (-3.8 to +5.4) | 70 | 66 | 0.797 |
| GPT-6 Astra | CLEAN | 320 (59.7 to 68.1) | 204 (36.6 to 45.2) | +23.20 (+18.2 to +28.0) | 149 | 33 | 8.35e-19 |
| GPT-6 Astra | FALSE CLAIM | 71 (11.4 to 17.5) | 113 (19.2 to 26.5) | -8.40 (-12.5 to -4.3) | 34 | 76 | 7.69e-05 |

The two parts apart:

| model | measure | the grid: with (Wilson 95) | the grid: without (Wilson 95) | list 2: with (Wilson 95) | list 2: without (Wilson 95) | the parts' intervals overlap: with, without |
|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | SOLVED | 73 (63.6 to 80.7) | 25 (17.5 to 34.3) | 296 (69.5 to 78.1) | 101 (21.2 to 29.7) | yes, yes |
| Gemini 3.8 Flash | CLEAN | 68 (58.3 to 76.3) | 10 (5.5 to 17.4) | 274 (63.8 to 72.9) | 52 (10.1 to 16.7) | yes, yes |
| Gemini 3.8 Flash | FALSE CLAIM | 16 (10.1 to 24.4) | 30 (21.9 to 39.6) | 52 (10.1 to 16.7) | 147 (32.2 to 41.6) | yes, yes |
| GPT-6 Astra | SOLVED | 70 (60.4 to 78.1) | 71 (61.5 to 79.0) | 294 (69.0 to 77.6) | 289 (67.7 to 76.4) | yes, yes |
| GPT-6 Astra | CLEAN | 61 (51.2 to 70.0) | 44 (34.7 to 53.8) | 259 (59.9 to 69.3) | 160 (35.3 to 44.9) | yes, yes |
| GPT-6 Astra | FALSE CLAIM | 14 (8.5 to 22.1) | 24 (16.7 to 33.2) | 57 (11.2 to 18.0) | 89 (18.4 to 26.6) | yes, yes |

## How list 2 relates to the grid's tests (no statement counted twice)

The grid's tests are read on the grid's 100 start states only, by the grid's rule; no list 2 start state enters them, and no grid start state enters the family above.

| question | the grid's test, grid 100 | list 2's test (Holm) | what may be said |
|---|---|---|---|
| Gemini 3.8 Flash SOLVED | the main test: 73 against 25 of 100 (with only 52, without only 4), p 1.10e-11: passes at 0.05 | passes | the statement, then "and it held on 400 further start states (a list added after the first grid rows were seen)" |
| GPT-6 Astra CLEAN | declared secondary: 61 against 44 (26 against 9), p 0.00599: passes at 0.05 | passes | the statement, then "and it held on 400 further start states (a list added after the first grid rows were seen)" |
| GPT-6 Astra SOLVED | in the per-model family: 70 against 71 (13 against 14), p 1: cannot pass at any threshold | does not pass | no statement; both printed |
| Gemini 3.8 Flash CLEAN | in the grid's CLEAN family, settled; 68 against 10 (60 against 2), p 8.47e-16, under 0.05 / 15, the strictest threshold that family can set | passes | the statement, then "and it held on 400 further start states (a list added after the first grid rows were seen)" |
| FALSE CLAIM, a model | the grid's test pools every model and is settled; the grid's per-model counts (Gemini 3.8 Flash 16 against 30; GPT-6 Astra 14 against 24) are not a grid test | passes for both models | this list's own sentences only |

"Replicated" may be said only where both columns read passes: Gemini 3.8 Flash SOLVED and GPT-6 Astra CLEAN and Gemini 3.8 Flash CLEAN. Not of Gemini 3.8 Flash FALSE CLAIM, GPT-6 Astra FALSE CLAIM from this list. One effect is one statement: the grid's test and this list's test on the same model and measure are one finding, their p values are never combined, and nothing is confirmed twice.

## The one cross-model pair named beforehand: Gemini 3.8 Flash with Cerulion against GPT-6 Astra without (descriptive, outside the corrected family)

| measure | Gemini 3.8 Flash with Cerulion (Wilson 95) | GPT-6 Astra without (Wilson 95) | difference, points (Newcombe 95) | first only | second only | exact p | wording |
|---|---|---|---|---|---|---|---|
| SOLVED | 296 of 400 (69.5 to 78.1) | 289 of 400 (67.7 to 76.4) | +1.75 (-4.1 to +7.6) | 76 | 69 | 0.618 | no difference shown |
| CLEAN | 274 of 400 (63.8 to 72.9) | 160 of 400 (35.3 to 44.9) | +28.50 (+22.6 to +34.1) | 143 | 29 | 2.61e-19 | differs (p 2.61e-19; a pair named beforehand, outside the corrected family) |

## Requests, steps, tokens and cost (means over scored rows; USD at the list prices of ../prices.csv)

| model | arm | mean requests | mean control steps | requests answered | failed requests, each sent again | tokens in (cached share) | tokens out | USD, scored rows | attempts in no row, USD | USD a start state | USD a solved start state | mean seconds an episode |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | with Cerulion | 26.9 | 242.7 | 10746 | 2 | 642,200,474 (81.4 percent) | 7,456,507 | 156.73 | 0, 0.00 | 0.39 | 0.53 | 255.7 |
| Gemini 3.8 Flash | without Cerulion | 54.7 | 521.0 | 21890 | 1 | 1,670,278,157 (64.1 percent) | 30,116,941 | 642.84 | 0, 0.00 | 1.61 | 6.36 | 776.9 |
| GPT-6 Astra | with Cerulion | 15.5 | 257.1 | 6208 | 580 | 250,613,350 (74.8 percent) | 687,296 | 853.16 | 0, 0.00 | 2.13 | 2.90 | 235.1 |
| GPT-6 Astra | without Cerulion | 34.2 | 339.2 | 13686 | 1391 | 219,585,114 (11.0 percent) | 3,544,034 | 2154.72 | 1, 2.32 | 5.39 | 7.46 | 818.6 |

USD a start state and USD a solved start state count the scored rows and the attempts in no row (lost_attempts.csv) over all start states of the arm, or over its solved ones. USD is list price only, not a billed amount; part of the gap between the arms is caching (the cached share of input tokens), not only fewer requests. Seconds are wall clock from the episode's start to its end and include the time an endpoint stalled or refused requests; a failed request was sent again inside its episode.

## Episodes that ended without a score

None.

## What these numbers allow a sentence to say

Each sentence after the lines at the top of this file, with the LIBERO-10 line wherever a LIBERO-10 number is in it, and with the bad news beside the numbers it qualifies. LIBERO-10 sentences carry 395, the size of that list.

- Gemini 3.8 Flash SOLVED, list 2: "With Gemini 3.8 Flash, on 400 further start states, Cerulion solved 296 of 400 and the plain interface 101 of 400 (exact paired test p 1.39e-44; Cerulion only 211, plain only 16)." The main test passed: "The 100-cell result held on 400 further start states."
- The two lists together, Gemini 3.8 Flash SOLVED: "Over 500 start states an arm (two lists), Cerulion solved 369 of 500 (69.8 to 77.5 percent) and the plain interface 126 of 500 (21.6 to 29.2 percent)." No p in this sentence; with the pool's line, "a pool of two lists: 100 cells whose first rows had been seen when the other 400 were added; 25 task rows, 20 start states each, six suite by perturbation columns; not the published 10 tasks x 50 of one column.", and "500 start states an arm is the field's count for trained policies; 25 task rows x 20 start states over six columns here, 10 tasks x 50 in one column there."
- GPT-6 Astra CLEAN, list 2, with its SOLVED counts: "With GPT-6 Astra, more of Cerulion's solves came inside the published step limit: 259 against 160 of 400 (p 3.18e-17); it solved 294 against 289 of 400 (p 0.702)."
- GPT-6 Astra SOLVED, list 2: "With GPT-6 Astra, Cerulion did not solve more start states (294 against 289 of 400, p 0.702). A difference this small could not be shown at this count."
- Gemini 3.8 Flash FALSE CLAIM, list 2: "Fewer episodes ended with the model wrongly saying it had succeeded: 52 against 147 of 400 (p 7.51e-16), because it solved more, not because the model was more candid when it failed." Beside it: 52 of 104 unsolved (0.500) against 147 of 299 (0.492).
- GPT-6 Astra FALSE CLAIM, list 2: "Fewer episodes ended with the model wrongly saying it had succeeded: 57 against 89 of 400 (p 6.28e-04)." Beside it: 57 of 106 unsolved (0.538) against 89 of 111 (0.802).
- The cross-model pair, list 2: "Gemini 3.8 Flash with Cerulion solved 296 of 400; GPT-6 Astra without it solved 289 of 400 (no difference shown, p 0.618)."
- Task-row reads, list 2: Gemini 3.8 Flash SOLVED: "Cerulion ahead on 21 of 25 task rows, behind on 3, tied on 1"; Gemini 3.8 Flash CLEAN: "Cerulion ahead on 20 of 25 task rows, behind on 2, tied on 3"; Gemini 3.8 Flash FALSE CLAIM (fewer): "Cerulion ahead on 19 of 25 task rows, behind on 3, tied on 3"; GPT-6 Astra SOLVED: "Cerulion ahead on 15 of 25 task rows, behind on 6, tied on 4"; GPT-6 Astra CLEAN: "Cerulion ahead on 19 of 25 task rows, behind on 4, tied on 2"; GPT-6 Astra FALSE CLAIM (fewer): "Cerulion ahead on 15 of 25 task rows, behind on 3, tied on 7".

Not said, whatever these rows show: that the tasks are new, held back or never developed on; "the full LIBERO-PRO benchmark", "500 a suite", "the full LIBERO-10 suite" or "LIBERO-10 at 500"; that the extension's existence or size was declared before the grid's rows (its reading rule was fixed before its own rows; its decision came after the first grid rows were seen); "confirmed twice" or any combined p; anything resting on the pooled p; any comparison verb, multiple or rank against a published system; "Cerulion helps strong models solve more" (GPT-6 Astra SOLVED did not pass on LIBERO-PRO, its LIBERO-10 pass is on 5 of the 10 tasks, and LIBERO-10 alone may not carry it); "Cerulion stops false claims" or "makes the model honest"; "LIBERO-10 confirms it on a second benchmark"; any development number in a headline, chart of results, pool or test; anything about another model from these lists; any split of these rows worded as a finding, the tasks 4 and 6 reading included.
