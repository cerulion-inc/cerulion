# Summary of the LIBERO-10 list: 395 start states, 2 models, both arms

Computed by ../recompute.py from this folder's CSV files and those of the grid (../) and the other extension folders, as read at 2026-11-19T02:31:46Z. Every start state of this list has ended in both arms of both models. The two arms: `with` is the model on Cerulion's tools; `without`, the plain interface, is the same model on the published three-tool interface.

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
| Gemini 3.8 Flash | with Cerulion | 395 | 395 | 0 | 249 (58.2 to 67.7) | 241 (56.1 to 65.7) | 62 (12.4 to 19.6) | 62 of 146 (0.425) | 62 of 146 (0.425) |
| Gemini 3.8 Flash | without Cerulion | 395 | 395 | 0 | 50 (9.7 to 16.3) | 38 (7.1 to 12.9) | 62 (12.4 to 19.6) | 62 of 345 (0.180) | 62 of 345 (0.180) |
| GPT-6 Astra | with Cerulion | 395 | 395 | 0 | 243 (56.6 to 66.2) | 228 (52.8 to 62.5) | 83 (17.3 to 25.3) | 83 of 152 (0.546) | 83 of 152 (0.546) |
| GPT-6 Astra | without Cerulion | 395 | 390 | 5 | 173 (39.0 to 48.7) | 122 (26.5 to 35.6) | 118 (25.6 to 34.6) | 118 of 222 (0.532) | 118 of 217 (0.544) |

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
| Gemini 3.8 Flash | CLEAN | 395 | 241 | 38 | 207 | 4 | 4.97e-56 | +51.39 (+46.0 to +56.3) | yes | none |
| Gemini 3.8 Flash | SOLVED | 395 | 249 | 50 | 203 | 4 | 7.37e-55 | +50.38 (+45.0 to +55.2) | yes | none |
| GPT-6 Astra | CLEAN | 390 | 223 | 122 | 134 | 33 | 1.22e-15 | +25.90 (+19.8 to +31.7) | yes | l10_t06_i21, l10_t06_i26, l10_t06_i31, l10_t06_i42, l10_t04_i45 |
| GPT-6 Astra | SOLVED | 390 | 238 | 173 | 113 | 48 | 3.19e-07 | +16.67 (+10.4 to +22.7) | yes | l10_t06_i21, l10_t06_i26, l10_t06_i31, l10_t06_i42, l10_t04_i45 |
| GPT-6 Astra | FALSE CLAIM | 390 | 83 | 118 | 45 | 80 | 0.00223 | -8.97 (-14.5 to -3.4) | yes | l10_t06_i21, l10_t06_i26, l10_t06_i31, l10_t06_i42, l10_t04_i45 |
| Gemini 3.8 Flash | FALSE CLAIM | 395 | 62 | 62 | 50 | 50 | 1 | +0.00 (-5.0 to +5.0) | no | none |

## The tasks as the unit (descriptive, in no family)

Start states of one task are not independent, so the count of tasks where the arm with Cerulion has more of the measure, fewer and the same is printed beside every test, with an exact sign test on the untied ones. Where a start-state test passes Holm but this p is above 0.05, a sentence must give the count.

| model | measure | tasks | with more | with fewer | tied | exact p on untied | start-state test | the sentence must say |
|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | CLEAN | 10 | 9 | 0 | 1 | 0.00391 | passes Holm | no extra clause |
| Gemini 3.8 Flash | SOLVED | 10 | 10 | 0 | 0 | 0.00195 | passes Holm | no extra clause |
| GPT-6 Astra | CLEAN | 10 | 7 | 2 | 1 | 0.18 | passes Holm | on 7 of the 10 tasks (with more 7, fewer 2, tied 1); not "across the suite" |
| GPT-6 Astra | SOLVED | 10 | 5 | 5 | 0 | 1 | passes Holm | on 5 of the 10 tasks (with more 5, fewer 5, tied 0); not "across the suite" |
| GPT-6 Astra | FALSE CLAIM | 10 | 5 | 3 | 2 | 0.727 | passes Holm | on 3 of the 10 tasks (with more 5, fewer 3, tied 2); not "across the suite" |
| Gemini 3.8 Flash | FALSE CLAIM | 10 | 3 | 5 | 2 | 0.727 | does not pass | no extra clause |

Every task, both models (with / without, of the start states in it; printed whole, rows at zero included; also in task_rows.csv). A split read off this table is exploratory, not a finding.

| task | start states | Flash SOLVED | Flash CLEAN | Flash FALSE CLAIM | Astra SOLVED | Astra CLEAN | Astra FALSE CLAIM |
|---|---|---|---|---|---|---|---|
| libero_10 task 0 | 38 | 36 / 8 | 36 / 5 | 2 / 2 | 27 / 31 | 26 / 16 | 10 / 7 |
| libero_10 task 1 | 40 | 39 / 21 | 38 / 18 | 1 / 5 | 34 / 38 | 34 / 31 | 6 / 0 |
| libero_10 task 2 | 40 | 19 / 15 | 16 / 9 | 16 / 4 | 28 / 30 | 22 / 23 | 0 / 0 |
| libero_10 task 3 | 39 | 1 / 0 | 1 / 0 | 4 / 15 | 15 / 9 | 8 / 2 | 10 / 6 |
| libero_10 task 4 | 40 | 39 / 0 | 39 / 0 | 1 / 9 | 39 / 0 | 39 / 0 | 1 / 30 |
| libero_10 task 5 | 39 | 38 / 2 | 38 / 2 | 1 / 5 | 1 / 0 | 1 / 0 | 37 / 34 |
| libero_10 task 6 | 40 | 32 / 0 | 32 / 0 | 8 / 8 | 36 / 0 | 36 / 0 | 4 / 33 |
| libero_10 task 7 | 40 | 34 / 4 | 33 / 4 | 6 / 1 | 27 / 38 | 27 / 31 | 13 / 2 |
| libero_10 task 8 | 40 | 10 / 0 | 8 / 0 | 21 / 8 | 36 / 23 | 35 / 19 | 2 / 6 |
| libero_10 task 9 | 39 | 1 / 0 | 0 / 0 | 2 / 5 | 0 / 4 | 0 / 0 | 0 / 0 |

## The one cross-model pair named beforehand: Gemini 3.8 Flash with Cerulion against GPT-6 Astra without (descriptive, outside the corrected family)

| measure | Gemini 3.8 Flash with Cerulion (Wilson 95) | GPT-6 Astra without (Wilson 95) | difference, points (Newcombe 95) | first only | second only | exact p | wording |
|---|---|---|---|---|---|---|---|
| SOLVED | 249 of 395 (58.2 to 67.7) | 173 of 395 (39.0 to 48.7) | +19.24 (+12.7 to +25.5) | 129 | 53 | 1.69e-08 | differs (p 1.69e-08; a pair named beforehand, outside the corrected family) |
| CLEAN | 241 of 395 (56.1 to 65.7) | 122 of 395 (26.5 to 35.6) | +30.13 (+23.7 to +36.2) | 157 | 38 | 2.21e-18 | differs (p 2.21e-18; a pair named beforehand, outside the corrected family) |

## Requests, steps, tokens and cost (means over scored rows; USD at the list prices of ../prices.csv)

| model | arm | mean requests | mean control steps | requests answered | failed requests, each sent again | tokens in (cached share) | tokens out | USD, scored rows | attempts in no row, USD | USD a start state | USD a solved start state | mean seconds an episode |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | with Cerulion | 32.1 | 369.1 | 12694 | 3 | 725,039,091 (78.7 percent) | 8,420,121 | 190.15 | 0, 0.00 | 0.48 | 0.76 | 294.5 |
| Gemini 3.8 Flash | without Cerulion | 58.7 | 638.5 | 23180 | 1 | 1,589,369,811 (63.5 percent) | 27,761,071 | 614.84 | 0, 0.00 | 1.56 | 12.30 | 757.0 |
| GPT-6 Astra | with Cerulion | 19.0 | 407.9 | 7517 | 67 | 303,064,799 (70.3 percent) | 678,162 | 1147.76 | 0, 0.00 | 2.91 | 4.72 | 227.9 |
| GPT-6 Astra | without Cerulion | 49.9 | 577.9 | 19470 | 357 | 374,500,170 (9.0 percent) | 4,568,684 | 3670.27 | 37, 142.82 | 9.65 | 22.04 | 927.2 |

USD a start state and USD a solved start state count the scored rows and the attempts in no row (lost_attempts.csv) over all start states of the arm, or over its solved ones. USD is list price only, not a billed amount; part of the gap between the arms is caching (the cached share of input tokens), not only fewer requests. Seconds are wall clock from the episode's start to its end and include the time an endpoint stalled or refused requests; a failed request was sent again inside its episode.

8 attempts were running when their machine stopped and left no token count on any copy (stopped_attempts.csv, with the longest each can have run). They are in no row and in no total above, and each start state ran again from attempt 1 on another machine (8 of the 8 were then scored, all on that attempt 1). Priced at the mean spend a second of their model and arm on this list (the USD of its scored rows over their seconds), they come to about 23 USD; at the highest spend a second of any scored row of their model and arm, about 65 USD. No row bounds them.

## Episodes that ended without a score

| model | arm | start state | attempts | reason | USD of its attempts |
|---|---|---|---|---|---|
| GPT-6 Astra | without Cerulion | l10_t06_i21 | 3 | the model endpoint refused every attempt: HTTP 400, Image processing blocked due to content policy violation | 6.07 |
| GPT-6 Astra | without Cerulion | l10_t06_i26 | 3 | the model endpoint refused every attempt: HTTP 400, Image processing blocked due to content policy violation | 0.28 |
| GPT-6 Astra | without Cerulion | l10_t06_i31 | 3 | the model endpoint refused every attempt: HTTP 400, Image processing blocked due to content policy violation | 0.88 |
| GPT-6 Astra | without Cerulion | l10_t06_i42 | 3 | the model endpoint refused every attempt: HTTP 400, Image processing blocked due to content policy violation | 12.24 |
| GPT-6 Astra | without Cerulion | l10_t04_i45 | 3 | the model endpoint refused every attempt: HTTP 400, Image processing blocked due to content policy violation | 9.89 |

Each one is a failure for its arm: not solved, not clean, no false claim. It stays in the count (395), enters the paired tests as a failure, and the scored-pairs table prints every test without it; the first verdict rules. The most unscored in one arm is 5 of 395 (1.3 percent), under the 5 percent at which a column's heading would carry the count.

## What these numbers allow a sentence to say

Each sentence after the lines at the top of this file, with the LIBERO-10 line wherever a LIBERO-10 number is in it, and with the bad news beside the numbers it qualifies. LIBERO-10 sentences carry 395, the size of that list.

- Gemini 3.8 Flash SOLVED, LIBERO-10, with the LIBERO-10 line: "On LIBERO-10, the same direction on 395 other start states of its ten tasks: 249 against 50 (p 7.37e-55)."
- Gemini 3.8 Flash CLEAN, LIBERO-10, with the LIBERO-10 line: "On LIBERO-10, the same direction on 395 other start states of its ten tasks: 241 against 38 (p 4.97e-56)."
- Gemini 3.8 Flash FALSE CLAIM, LIBERO-10, with the LIBERO-10 line: "On LIBERO-10, no difference shown for Gemini 3.8 Flash (62 against 62 of 395, p 1)."
- GPT-6 Astra SOLVED, LIBERO-10, with the LIBERO-10 line: "On LIBERO-10, the same direction on 395 other start states of its ten tasks, on 5 of the 10 tasks (Cerulion ahead on 5, behind on 5, tied on 0): 243 against 173 (p 5.48e-08)." Never without, in the same place, the bad news on tasks 4 and 6 and the image filter, and the LIBERO-PRO null beside it: "With GPT-6 Astra, Cerulion did not solve more start states (294 against 289 of 400, p 0.702)."
- GPT-6 Astra CLEAN, LIBERO-10, with the LIBERO-10 line: "On LIBERO-10, the same direction on 395 other start states of its ten tasks, on 7 of the 10 tasks (Cerulion ahead on 7, behind on 2, tied on 1): 228 against 122 (p 1.11e-16)."
- GPT-6 Astra FALSE CLAIM, LIBERO-10, with the LIBERO-10 line: "On LIBERO-10, the same direction on 395 other start states of its ten tasks: fewer episodes ended with the model wrongly saying it had succeeded, on 3 of the 10 tasks (Cerulion fewer on 3, more on 5, tied on 2): 83 against 118 (p 0.00223), because it solved more, not because the model was more candid when it failed." Beside it: 83 of 152 unsolved (0.546) against 118 of 222 (0.532), and the bad news on tasks 4 and 6.
- The cross-model pair, LIBERO-10, with the LIBERO-10 line: SOLVED 249 against 173 of 395, differs (p 1.69e-08; a pair named beforehand, outside the corrected family); CLEAN 241 against 122 of 395, differs (p 2.21e-18; a pair named beforehand, outside the corrected family).
- Task reads, LIBERO-10, with the LIBERO-10 line: Gemini 3.8 Flash SOLVED: "Cerulion ahead on 10 of 10 tasks, behind on 0, tied on 0"; Gemini 3.8 Flash CLEAN: "Cerulion ahead on 9 of 10 tasks, behind on 0, tied on 1"; Gemini 3.8 Flash FALSE CLAIM (fewer): "Cerulion ahead on 5 of 10 tasks, behind on 3, tied on 2"; GPT-6 Astra SOLVED: "Cerulion ahead on 5 of 10 tasks, behind on 5, tied on 0"; GPT-6 Astra CLEAN: "Cerulion ahead on 7 of 10 tasks, behind on 2, tied on 1"; GPT-6 Astra FALSE CLAIM (fewer): "Cerulion ahead on 3 of 10 tasks, behind on 5, tied on 2".

Not said, whatever these rows show: that the tasks are new, held back or never developed on; "the full LIBERO-PRO benchmark", "500 a suite", "the full LIBERO-10 suite" or "LIBERO-10 at 500"; that the extension's existence or size was declared before the grid's rows (its reading rule was fixed before its own rows; its decision came after the first grid rows were seen); "confirmed twice" or any combined p; anything resting on the pooled p; any comparison verb, multiple or rank against a published system; "Cerulion helps strong models solve more" (GPT-6 Astra SOLVED did not pass on LIBERO-PRO, its LIBERO-10 pass is on 5 of the 10 tasks, and LIBERO-10 alone may not carry it); "Cerulion stops false claims" or "makes the model honest"; "LIBERO-10 confirms it on a second benchmark"; any development number in a headline, chart of results, pool or test; anything about another model from these lists; any split of these rows worded as a finding, the tasks 4 and 6 reading included.
