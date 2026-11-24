# Summary of the grid: 100 start states, 16 models, both arms

Computed by recompute.py from episodes.csv, unscored.csv and not_finished.csv as read at 2026-11-21T09:40:40Z. recompute.py computes this file again from the CSV files; with --check it writes nothing and says whether this file still matches them.

Said before any number (RULES.md section 2):

1. They are new start states of 25 task rows that the Cerulion harness was developed on at other start states. The tasks themselves are not new to it. Task rows that the Cerulion harness never met exist in the benchmark and are on no list here.
2. The result is on one benchmark family, the one the Cerulion harness was built and checked on.
3. The arm with Cerulion reads a record of the robot's own calibration that the other arm does not have. The comparison is the whole package against the published interface; it cannot be split into tools, record and documents.
4. One episode a start state at temperature 1.0. Provider model versions are known only by the model's name.
5. Other groups publish higher rates on this benchmark under their own protocols. Nothing here supports a ranking against any of them.

With them: the arm without Cerulion is the benchmark's published minimal interface, not the best harness there is. The calibration records were made by the robot's own calibration episodes on development start states of LIBERO-10 and LIBERO-PRO, and which record each episode read is not recorded. The arm without Cerulion reads the gripper in one of the two ways the published paper allows, the one that scored lower when both ran in an earlier check on LIBERO-10 (not in this package); the other way was not run here, so a small gain over this arm is not shown to hold against it. CLEAN was defined before the first episode with Cerulion on the earlier lists, and became a claim only after the first of those lists had shown a gain on it; for this grid it was declared before the first episode.

Every model has finished all 100 start states in both arms, so every verdict below is settled.

Arms: `without` is the same model on a published three-tool interface; `with` is the same model on Cerulion's tools. Same start states, same budgets (80 replies, one tool call a reply, 900 control steps), one episode a start state an arm. An episode that ended without a score counts as a failure for its arm and stays in the count.

## Read these first

- With Cerulion, 4 of the 16 models solved fewer start states than without it, the 4 with the highest rates without it: Gemini 4 Neon (90 against 93 of 100; with only 7, without only 10; p = 0.629); Gemini 4 Carbon (76 against 82 of 100; with only 13, without only 19; p = 0.377); GPT-6.2 Astra (77 against 81 of 100; with only 10, without only 14; p = 0.541); GPT-6 Astra (70 against 71 of 100; with only 13, without only 14; p = 1). None of them is significant at 0.05, so each reads "no difference shown" (RULES.md section 5 (3)).
- On CLEAN, 1 of the 16 models had fewer with Cerulion: Gemini 4 Carbon (58 against 71 of 100; with only 17, without only 30; p = 0.0789). It is not significant at 0.05, so it reads "no difference shown" (RULES.md section 5 (3)).
- With Cerulion ahead, printed whole: on SOLVED in 12 of 16, and on CLEAN in 15 of 16 models.
- Established after Holm's correction over the 16 models (RULES.md section 3): on SOLVED for 9 of 16, and on CLEAN for 13 of 16. Ahead with Cerulion but not established: GPT-6.2 Sol SOLVED (73 against 59 of 100; with only 26, without only 12; p = 0.0336 against its Holm threshold 0.0083): points up, not established; GPT-6.1 Sol SOLVED (68 against 59 of 100; with only 22, without only 13; p = 0.175 against its Holm threshold 0.0100): no difference shown; GPT-6.1 Terra SOLVED (74 against 57 of 100; with only 27, without only 10; p = 0.00763 against its Holm threshold 0.0071): points up, not established; GPT-6.2 Astra CLEAN (58 against 54 of 100; with only 15, without only 11; p = 0.557 against its Holm threshold 0.0500): no difference shown; Gemini 4 Neon CLEAN (70 against 59 of 100; with only 25, without only 14; p = 0.108 against its Holm threshold 0.0250): no difference shown. Only a model established on SOLVED may be read as "Cerulion lifts" it.
- The provider's content filter stopped replies (finish reason content_filter) in 4 of the 32 arms: Claude Fable 5.2 without Cerulion (337 of its 1,101 replies in scored episodes, in 94 of its 100 scored episodes, 86 of which ended on filtered replies); Gemini 4 Argon without Cerulion (1 of the 155 replies of its 3 unscored episodes, in 1 of them); Gemini 4 Argon with Cerulion (1 of its 1,647 replies in scored episodes, in 1 of its 100 scored episodes, which did not end on it); Gemini 4 Hydrogen without Cerulion (1 of its 5,990 replies in scored episodes, in 1 of its 100 scored episodes, which did not end on it). No other arm had one. Every arm's replies by finish reason are in the table under "How the replies ended".
- The pooled false-claim test (the second secondary test below) passes: fewer episodes ended with the model wrongly saying it had succeeded. Per unsolved episode the rate is higher with Cerulion: 186 false claims in 501 unsolved episodes (0.371) against 255 in 905 (0.282) without it, over all 16 models; as high or higher with Cerulion for 7 of the 16 (with against without): Gemini 3.8 Flash 0.593 against 0.400; Claude Opus 5.5 0.385 against 0.235; Claude Sonnet 5.5 0.510 against 0.190; Claude Haiku 5.5 0.339 against 0.042; Claude Fable 5.2 0.500 against 0.044; GPT-6 Luna 0.197 against 0.120; Grok 4.7 0.319 against 0.062.
- With Cerulion the model made more false claims in all for 5 of the 16 models: Claude Haiku 5.5 (19 against 4; with only 18, without only 3; p = 0.00149); Claude Sonnet 5.5 (25 against 16; with only 22, without only 13; p = 0.175); Grok 4.7 (15 against 6; with only 15, without only 6; p = 0.0784); Claude Fable 5.2 (8 against 4; with only 7, without only 3; p = 0.344); GPT-6 Luna (14 against 12; with only 14, without only 12; p = 0.845). Each per-model count is descriptive; the declared test is the pooled one.
- Read with the content filter (descriptive; no rule declared this reading): Claude Fable 5.2 without Cerulion ended 86 of its 90 unsolved episodes on filtered replies, and no claim from the model reached any of them: every reply that ended them came back content_filter, with no text and no tool call. Its false claims per unsolved episode: 4 of 90 (0.044) with them counted, 4 of the 4 that did not end that way (1.000); with Cerulion, 8 of 16 (0.500). With Cerulion against without, Claude Fable 5.2 made 8 against 4 false claims in all, and per unsolved episode 0.500 against 0.044 with them counted, 0.500 against 1.000 with them left out. The two readings point opposite ways, so the comparison measures the filter as much as the model and is read neither way.
- The pooled rate per unsolved episode, over all 16 models: 186 in 501 (0.371) with Cerulion against 255 in 905 (0.282) without; with Claude Fable 5.2 left out (both arms, so each pool holds the same 15 models): 178 in 485 (0.367) with Cerulion against 251 in 815 (0.308) without (descriptive; no rule declared this reading). With each arm's unsolved episodes that ended on filtered replies left out of its count, the arm with Cerulion claims falsely as often or more for 6 of the 16 models (descriptive; no rule declared this reading), against 7 with them counted; not most, either way.
- Claude Fable 5.2 without Cerulion ended 86 of its 100 episodes on 3 replies in a row with no tool call, which end an episode (ended_by stopped_calling_tools), 44 of them before any control step. All 86 ended on content_filter replies: in every one, the 3 replies with no tool call that ended it were all ones the provider's content filter stopped. 86 of the 90 start states it did not solve without Cerulion ended that way, so its gain on SOLVED (84 against 10) is mostly against an arm that the content filter stopped.
- 74 scored episodes ran no control step: Claude Fable 5.2 without Cerulion 44, GPT-6 Luna with Cerulion 28, Grok 4.7 without Cerulion 1, Grok 4.7 with Cerulion 1. Each carries a problem row and none was solved or is a false claim, so no count of solved, clean or false claims changes with them.
- With Cerulion, 2 of the 16 models did not use fewer model replies and control steps per scored episode than without it: Claude Fable 5.2 (14.9 against 11.0 replies, 257.9 against 76.2 control steps); Gemini 4 Carbon (18.3 against 16.0 replies, 247.7 against 200.4 control steps).
- With Claude Fable 5.2, an episode with Cerulion cost more at list price (1.82 against 1.50 USD an episode) and took longer (485.8 against 250.4 wall seconds); a success cost less (2.15 against 15.03 USD). These means are over scored episodes, and 1 of its episodes with Cerulion and 0 without ended without a score. Counted too (an episode the 90 minute clock ended at its 5400 seconds), with each arm's attempts in lost_attempts.csv at their cost: 535.0 against 250.4 wall seconds, 2.00 against 1.50 USD an episode and 2.38 against 15.03 USD a success.
- GPT-6.1 Terra was added after the grid's first rows had been seen. It does not pass SOLVED at p = 0.00763 against its Holm threshold 0.00714.
- Gemini 4 Neon was added after every other column had finished and been read. It does not pass SOLVED at p = 0.629 against its Holm threshold 0.025.
- With the task row as the unit (the start states of each of the 25 task rows counted together in each arm; descriptive, no test is declared on it), 2 of the passes below do not reach 0.05: Gemini 4 Argon SOLVED (11 task rows more, 3 fewer, 11 the same, p = 0.0574); GPT-6 Astra CLEAN (12 task rows more, 4 fewer, 9 the same, p = 0.0768).
- The cross-model pairs named before the grid (RULES.md section 3), SOLVED: Gemini 3.8 Flash with Cerulion against GPT-6 Astra without, 73 against 71 of 100 (first only 18, second only 16, p = 0.864): no difference shown; Gemini 3.8 Flash with Cerulion against GPT-6.2 Astra without, 73 against 81 of 100 (first only 8, second only 16, p = 0.152): no difference shown, -8.0 points. Any other such pair found in the grid is exploratory.
- 28 episodes ended without a score (22 without Cerulion, 6 with), each counted as a failure for its arm: 8 because the model endpoint answered rate limit on every attempt; 20 because the 90 minute episode clock ran out. On scored pairs only (start states scored in both arms) the same models pass Holm on SOLVED and on CLEAN.
- Each start state ran once in each arm. How much the outcome of one start state varies from run to run was not measured.
- How the run went, and where it differed from the protocol (the pause of the Claude models, the other models' start states whose two arms started more than an hour apart, two machines that stopped and a third whose waiting episodes were moved, a network outage, one model answered from two provider deployments, the column that ran on one machine on 2026-11-21, the machines shared with other work), is in README.md under "Things to know before quoting a number".

## The main test, as declared before the first episode

Declared: Gemini 3.8 Flash, SOLVED within the 900-step budget, with Cerulion against without, paired by start state on the grid's 100 start states, exact two-sided sign test on the discordant start states, alpha 0.05, no correction. One test.

Result (complete): with Cerulion 73 of 100 (63.6 to 80.7 percent), without 25 of 100 (17.5 to 34.3 percent). Solved with only: 52. Solved without only: 4. Both: 21. Neither: 23. Exact two-sided sign test p = 1.10e-11. Difference 48.0 points (Newcombe 95 for paired rates: 35.7 to 57.9).

The test passes at 0.05.

Read with it: the statement it carries (RULES.md section 3) needs the main test to pass, the rate with Cerulion to overlap the earlier 61.5 to 81.9 percent and the rate without to overlap 10.1 to 27.6 percent (here 63.6 to 80.7 and 17.5 to 34.3). The test passes and both overlap, so the statement stands, with these counts: 73 against 25 of 100, 2.9 times as many. With the task row as the unit (descriptive): 21 task rows more, 1 fewer, 3 the same, p = 1.10e-05.

## The two secondary tests, declared with it (alpha 0.05 each, apart from the per-model family)

1. GPT-6 Astra, CLEAN (solved at or before the suite's published step limit, with no protocol break, no problem row and no false claim): with Cerulion 61 of 100 (51.2 to 70.0), without 44 of 100 (34.7 to 53.8); with only 26, without only 9, p = 0.00599. Its SOLVED counts, to be read with it: 70 against 71 (with only 13, without only 14, p = 1).

   The test passes at 0.05, on 100 start states (the rule asks for at least 63). What it supports: with GPT-6 Astra, Cerulion did not solve more start states (70 against 71 of 100); more of its solves came inside the published step limit (61 against 44, p = 0.00599). With the task row as the unit (descriptive): 12 task rows more, 4 fewer, 9 the same, p = 0.0768.

2. Fewer false claims, pooled over every model that ran both arms, one sign test over all discordant pairs of (model, start state): a false claim without Cerulion only on 209 pairs, with Cerulion only on 140 pairs, p = 2.61e-04. The rate per unsolved episode is in the table below for each model and arm.

   The test passes at 0.05. False claims in all: 255 of 1600 episodes without Cerulion, 186 of 1600 with it. On scored pairs only: without only 209, with only 137, p = 1.28e-04. What it supports: fewer episodes ended with the model wrongly saying it had succeeded; not that Cerulion stops false claims. The rule narrows the statement to "fewer because it solves more" if, per unsolved episode, the arm with Cerulion claims falsely as often or more for most models: it does for 7 of the 16, which is not most, so the statement stands as written. The rates are under "Read these first" and in the table below.

## Every model, both arms

| model | arm | finished | solved | Wilson 95, percent | clean | Wilson 95, percent | false claims | per unsolved episode | unscored | not finished |
|---|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | without Cerulion | 100 | 25 | 17.5 to 34.3 | 10 | 5.5 to 17.4 | 30 | 0.40 | 0 | 0 |
| Gemini 3.8 Flash | with Cerulion | 100 | 73 | 63.6 to 80.7 | 68 | 58.3 to 76.3 | 16 | 0.59 | 0 | 0 |
| GPT-6 Astra | without Cerulion | 100 | 71 | 61.5 to 79.0 | 44 | 34.7 to 53.8 | 24 | 0.83 | 0 | 0 |
| GPT-6 Astra | with Cerulion | 100 | 70 | 60.4 to 78.1 | 61 | 51.2 to 70.0 | 14 | 0.47 | 0 | 0 |
| Claude Opus 5.5 | without Cerulion | 100 | 32 | 23.7 to 41.7 | 11 | 6.3 to 18.6 | 16 | 0.24 | 8 | 0 |
| Claude Opus 5.5 | with Cerulion | 100 | 74 | 64.6 to 81.6 | 68 | 58.3 to 76.3 | 10 | 0.38 | 0 | 0 |
| Claude Sonnet 5.5 | without Cerulion | 100 | 16 | 10.1 to 24.4 | 7 | 3.4 to 13.7 | 16 | 0.19 | 0 | 0 |
| Claude Sonnet 5.5 | with Cerulion | 100 | 51 | 41.3 to 60.6 | 36 | 27.3 to 45.8 | 25 | 0.51 | 0 | 0 |
| Claude Haiku 5.5 | without Cerulion | 100 | 4 | 1.6 to 9.8 | 1 | 0.2 to 5.4 | 4 | 0.04 | 0 | 0 |
| Claude Haiku 5.5 | with Cerulion | 100 | 44 | 34.7 to 53.8 | 33 | 24.6 to 42.7 | 19 | 0.34 | 0 | 0 |
| Claude Fable 5.2 | without Cerulion | 100 | 10 | 5.5 to 17.4 | 6 | 2.8 to 12.5 | 4 | 0.04 | 0 | 0 |
| Claude Fable 5.2 | with Cerulion | 100 | 84 | 75.6 to 89.9 | 66 | 56.3 to 74.5 | 8 | 0.50 | 1 | 0 |
| Gemini 4 Carbon | without Cerulion | 100 | 82 | 73.3 to 88.3 | 71 | 61.5 to 79.0 | 7 | 0.39 | 10 | 0 |
| Gemini 4 Carbon | with Cerulion | 100 | 76 | 66.8 to 83.3 | 58 | 48.2 to 67.2 | 6 | 0.25 | 5 | 0 |
| Gemini 4 Argon | without Cerulion | 100 | 63 | 53.2 to 71.8 | 25 | 17.5 to 34.3 | 26 | 0.70 | 3 | 0 |
| Gemini 4 Argon | with Cerulion | 100 | 84 | 75.6 to 89.9 | 68 | 58.3 to 76.3 | 7 | 0.44 | 0 | 0 |
| Gemini 4 Hydrogen | without Cerulion | 100 | 39 | 30.0 to 48.8 | 8 | 4.1 to 15.0 | 23 | 0.38 | 0 | 0 |
| Gemini 4 Hydrogen | with Cerulion | 100 | 79 | 70.0 to 85.8 | 55 | 45.2 to 64.4 | 6 | 0.29 | 0 | 0 |
| GPT-6.2 Astra | without Cerulion | 100 | 81 | 72.2 to 87.5 | 54 | 44.3 to 63.4 | 12 | 0.63 | 0 | 0 |
| GPT-6.2 Astra | with Cerulion | 100 | 77 | 67.8 to 84.2 | 58 | 48.2 to 67.2 | 10 | 0.43 | 0 | 0 |
| GPT-6.2 Sol | without Cerulion | 100 | 59 | 49.2 to 68.1 | 32 | 23.7 to 41.7 | 17 | 0.41 | 0 | 0 |
| GPT-6.2 Sol | with Cerulion | 100 | 73 | 63.6 to 80.7 | 60 | 50.2 to 69.1 | 6 | 0.22 | 0 | 0 |
| GPT-6.1 Sol | without Cerulion | 100 | 59 | 49.2 to 68.1 | 25 | 17.5 to 34.3 | 30 | 0.73 | 0 | 0 |
| GPT-6.1 Sol | with Cerulion | 100 | 68 | 58.3 to 76.3 | 61 | 51.2 to 70.0 | 16 | 0.50 | 0 | 0 |
| GPT-6 Luna | without Cerulion | 100 | 0 | 0.0 to 3.7 | 0 | 0.0 to 3.7 | 12 | 0.12 | 0 | 0 |
| GPT-6 Luna | with Cerulion | 100 | 29 | 21.0 to 38.5 | 25 | 17.5 to 34.3 | 14 | 0.20 | 0 | 0 |
| Grok 4.7 | without Cerulion | 100 | 4 | 1.6 to 9.8 | 1 | 0.2 to 5.4 | 6 | 0.06 | 1 | 0 |
| Grok 4.7 | with Cerulion | 100 | 53 | 43.3 to 62.5 | 45 | 35.6 to 54.8 | 15 | 0.32 | 0 | 0 |
| GPT-6.1 Terra | without Cerulion | 100 | 57 | 47.2 to 66.3 | 22 | 15.0 to 31.1 | 21 | 0.49 | 0 | 0 |
| GPT-6.1 Terra | with Cerulion | 100 | 74 | 64.6 to 81.6 | 62 | 52.2 to 70.9 | 12 | 0.46 | 0 | 0 |
| Gemini 4 Neon | without Cerulion | 100 | 93 | 86.3 to 96.6 | 59 | 49.2 to 68.1 | 7 | 1.00 | 0 | 0 |
| Gemini 4 Neon | with Cerulion | 100 | 90 | 82.6 to 94.5 | 70 | 60.4 to 78.1 | 2 | 0.20 | 0 | 0 |

Wilson intervals are over the finished start states of the arm (scored plus unscored). A false claim is an episode that ends with the model saying it succeeded on a start state that was not solved; per unsolved episode divides them by the arm's finished episodes that were not solved.

## Paired by start state, and the per-model family (16 models, Holm's step-down at 0.05)

The family is every model that ran, the main test's model included, however each came out; the main test above is read alone at 0.05 and does not depend on this table. Thresholds are computed from the p values of all the models; every column is complete, so they are final.

### SOLVED

| model | paired start states | with only | without only | both | neither | difference, points (Newcombe 95) | exact p | Holm threshold | verdict |
|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | 100 | 52 | 4 | 21 | 23 | +48.0 (35.7 to 57.9) | 1.10e-11 | 0.0036 | established |
| GPT-6 Astra | 100 | 13 | 14 | 57 | 16 | -1.0 (-11.1 to 9.1) | 1 | 0.0500 | no difference shown |
| Claude Opus 5.5 | 100 | 45 | 3 | 29 | 23 | +42.0 (30.4 to 51.7) | 1.31e-10 | 0.0038 | established |
| Claude Sonnet 5.5 | 100 | 42 | 7 | 9 | 42 | +35.0 (22.5 to 46.0) | 3.62e-07 | 0.0056 | established |
| Claude Haiku 5.5 | 100 | 43 | 3 | 1 | 53 | +40.0 (28.6 to 50.3) | 4.62e-10 | 0.0042 | established |
| Claude Fable 5.2 | 100 | 74 | 0 | 10 | 16 | +74.0 (63.6 to 80.9) | 1.06e-22 | 0.0031 | established |
| Gemini 4 Carbon | 100 | 13 | 19 | 63 | 5 | -6.0 (-17.0 to 5.1) | 0.377 | 0.0125 | no difference shown |
| Gemini 4 Argon | 100 | 29 | 8 | 55 | 8 | +21.0 (9.5 to 31.8) | 7.53e-04 | 0.0063 | established |
| Gemini 4 Hydrogen | 100 | 48 | 8 | 31 | 13 | +40.0 (26.8 to 51.2) | 4.69e-08 | 0.0050 | established |
| GPT-6.2 Astra | 100 | 10 | 14 | 67 | 9 | -4.0 (-13.6 to 5.6) | 0.541 | 0.0167 | no difference shown |
| GPT-6.2 Sol | 100 | 26 | 12 | 47 | 15 | +14.0 (2.1 to 25.3) | 0.0336 | 0.0083 | points up, not established |
| GPT-6.1 Sol | 100 | 22 | 13 | 46 | 19 | +9.0 (-2.5 to 20.1) | 0.175 | 0.0100 | no difference shown |
| GPT-6 Luna | 100 | 29 | 0 | 0 | 71 | +29.0 (20.2 to 38.5) | 3.73e-09 | 0.0045 | established |
| Grok 4.7 | 100 | 51 | 2 | 2 | 45 | +49.0 (37.6 to 58.8) | 3.18e-13 | 0.0033 | established |
| GPT-6.1 Terra | 100 | 27 | 10 | 47 | 16 | +17.0 (5.4 to 28.0) | 0.00763 | 0.0071 | points up, not established (column added after the grid's first rows had been seen) |
| Gemini 4 Neon | 100 | 7 | 10 | 83 | 0 | -3.0 (-11.5 to 5.4) | 0.629 | 0.0250 | no difference shown (column added after every other column had finished and been read) |

### CLEAN

| model | paired start states | with only | without only | both | neither | difference, points (Newcombe 95) | exact p | Holm threshold | verdict |
|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | 100 | 60 | 2 | 8 | 30 | +58.0 (46.3 to 67.1) | 8.47e-16 | 0.0033 | established |
| GPT-6 Astra | 100 | 26 | 9 | 35 | 30 | +17.0 (5.7 to 27.5) | 0.00599 | 0.0125 | established |
| Claude Opus 5.5 | 100 | 59 | 2 | 9 | 30 | +57.0 (45.3 to 66.1) | 1.64e-15 | 0.0036 | established |
| Claude Sonnet 5.5 | 100 | 32 | 3 | 4 | 61 | +29.0 (18.6 to 39.0) | 4.18e-07 | 0.0083 | established |
| Claude Haiku 5.5 | 100 | 33 | 1 | 0 | 66 | +32.0 (22.2 to 41.8) | 4.07e-09 | 0.0050 | established |
| Claude Fable 5.2 | 100 | 61 | 1 | 5 | 33 | +60.0 (48.8 to 68.8) | 2.73e-17 | 0.0031 | established |
| Gemini 4 Carbon | 100 | 17 | 30 | 41 | 12 | -13.0 (-25.7 to 0.3) | 0.0789 | 0.0167 | no difference shown |
| Gemini 4 Argon | 100 | 50 | 7 | 18 | 25 | +43.0 (29.9 to 53.9) | 4.24e-09 | 0.0056 | established |
| Gemini 4 Hydrogen | 100 | 49 | 2 | 6 | 43 | +47.0 (35.7 to 56.7) | 1.18e-12 | 0.0038 | established |
| GPT-6.2 Astra | 100 | 15 | 11 | 43 | 31 | +4.0 (-5.9 to 13.7) | 0.557 | 0.0500 | no difference shown |
| GPT-6.2 Sol | 100 | 37 | 9 | 23 | 31 | +28.0 (15.4 to 39.2) | 4.06e-05 | 0.0100 | established |
| GPT-6.1 Sol | 100 | 42 | 6 | 19 | 33 | +36.0 (23.7 to 46.6) | 1.01e-07 | 0.0071 | established |
| GPT-6 Luna | 100 | 25 | 0 | 0 | 75 | +25.0 (16.7 to 34.3) | 5.96e-08 | 0.0063 | established |
| Grok 4.7 | 100 | 45 | 1 | 0 | 54 | +44.0 (33.3 to 53.9) | 1.34e-12 | 0.0042 | established |
| GPT-6.1 Terra | 100 | 42 | 2 | 20 | 36 | +40.0 (29.0 to 49.4) | 1.13e-10 | 0.0045 | established (column added after the grid's first rows had been seen) |
| Gemini 4 Neon | 100 | 25 | 14 | 45 | 16 | +11.0 (-1.1 to 22.6) | 0.108 | 0.0250 | no difference shown (column added after every other column had finished and been read) |

Tally over the 16 complete columns, printed whole: with Cerulion solved more start states in 12, fewer in 4, the same number in 0. Fewer: GPT-6 Astra (70 against 71, p = 1); Gemini 4 Carbon (76 against 82, p = 0.377); GPT-6.2 Astra (77 against 81, p = 0.541); Gemini 4 Neon (90 against 93, p = 0.629). On CLEAN, with Cerulion more in 15, fewer in 1, the same number in 0. Fewer: Gemini 4 Carbon (58 against 71, p = 0.0789).

### FALSE CLAIM (descriptive for each model; the declared test is the pooled one above)

| model | paired start states | false claim without only | with only | exact p |
|---|---|---|---|---|
| Gemini 3.8 Flash | 100 | 20 | 6 | 0.00936 |
| GPT-6 Astra | 100 | 18 | 8 | 0.0755 |
| Claude Opus 5.5 | 100 | 13 | 7 | 0.263 |
| Claude Sonnet 5.5 | 100 | 13 | 22 | 0.175 |
| Claude Haiku 5.5 | 100 | 3 | 18 | 0.00149 |
| Claude Fable 5.2 | 100 | 3 | 7 | 0.344 |
| Gemini 4 Carbon | 100 | 6 | 5 | 1 |
| Gemini 4 Argon | 100 | 23 | 4 | 3.11e-04 |
| Gemini 4 Hydrogen | 100 | 20 | 3 | 4.88e-04 |
| GPT-6.2 Astra | 100 | 9 | 7 | 0.804 |
| GPT-6.2 Sol | 100 | 17 | 6 | 0.0347 |
| GPT-6.1 Sol | 100 | 20 | 6 | 0.00936 |
| GPT-6 Luna | 100 | 12 | 14 | 0.845 |
| Grok 4.7 | 100 | 6 | 15 | 0.0784 |
| GPT-6.1 Terra | 100 | 19 | 10 | 0.136 |
| Gemini 4 Neon | 100 | 7 | 2 | 0.18 |

## The cross-model pairs named before the grid: Gemini 3.8 Flash with Cerulion against GPT-6 Astra without and against GPT-6.2 Astra without (descriptive, outside the corrected family)

| measure | against | Gemini 3.8 Flash with Cerulion (Wilson 95, percent) | the other model without Cerulion (Wilson 95, percent) | difference, points (Newcombe 95) | first only | second only | exact p | wording |
|---|---|---|---|---|---|---|---|---|
| SOLVED | GPT-6 Astra without Cerulion | 73 of 100 (63.6 to 80.7) | 71 of 100 (61.5 to 79.0) | +2.0 (-9.4 to 13.3) | 18 | 16 | 0.864 | no difference shown |
| SOLVED | GPT-6.2 Astra without Cerulion | 73 of 100 (63.6 to 80.7) | 81 of 100 (72.2 to 87.5) | -8.0 (-17.5 to 1.5) | 8 | 16 | 0.152 | no difference shown |
| CLEAN | GPT-6 Astra without Cerulion | 68 of 100 (58.3 to 76.3) | 44 of 100 (34.7 to 53.8) | +24.0 (12.2 to 34.8) | 32 | 8 | 1.82e-04 | differs (p = 1.82e-04; a pair named beforehand, outside the corrected family) |
| CLEAN | GPT-6.2 Astra without Cerulion | 68 of 100 (58.3 to 76.3) | 54 of 100 (44.3 to 63.4) | +14.0 (3.9 to 23.6) | 21 | 7 | 0.0125 | differs (p = 0.0125; a pair named beforehand, outside the corrected family) |

RULES.md section 3 names these two pairs, and only these, before the grid was read: any other pair of a cheaper model with Cerulion against a dearer model without it found in these rows is exploratory. "No difference shown" does not mean the two were shown equal.

## Means per scored episode

| model | arm | scored episodes | replies | control steps | wall seconds | failed requests in all, each sent again | USD, list price | tokens in | of which cached | cached share, percent | tokens out | USD per solved start state |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | without Cerulion | 100 | 55.9 | 538.8 | 846.6 | 366 | 1.666 | 4,147,296 | 2,556,882 | 61.7 | 75,171 | 6.67 |
| Gemini 3.8 Flash | with Cerulion | 100 | 25.6 | 224.8 | 258.7 | 126 | 0.348 | 1,434,994 | 1,179,094 | 82.2 | 18,020 | 0.48 |
| GPT-6 Astra | without Cerulion | 100 | 29.9 | 305.0 | 787.9 | 648 | 4.183 | 423,452 | 49,626 | 11.7 | 7,896 | 5.89 |
| GPT-6 Astra | with Cerulion | 100 | 16.4 | 269.2 | 289.4 | 310 | 2.387 | 672,567 | 492,974 | 73.3 | 1,956 | 3.41 |
| Claude Opus 5.5 | without Cerulion | 92 | 59.9 | 497.3 | 902.1 | 282 | 8.235 | 1,429,377 | 11,128 | 0.8 | 57,088 | 23.68 |
| Claude Opus 5.5 | with Cerulion | 100 | 15.8 | 266.9 | 242.9 | 1 | 0.614 | 976,012 | 954,877 | 97.8 | 15,889 | 0.83 |
| Claude Sonnet 5.5 | without Cerulion | 100 | 56.2 | 609.6 | 533.2 | 144 | 3.509 | 1,232,207 | 3,411 | 0.3 | 43,649 | 21.93 |
| Claude Sonnet 5.5 | with Cerulion | 100 | 19.0 | 343.2 | 233.5 | 2 | 0.366 | 1,194,716 | 1,169,704 | 97.9 | 18,660 | 0.72 |
| Claude Haiku 5.5 | without Cerulion | 100 | 65.4 | 647.4 | 1,102.7 | 2,164 | 0.256 | 1,791,573 | 4,379 | 0.2 | 64,749 | 6.40 |
| Claude Haiku 5.5 | with Cerulion | 100 | 17.1 | 303.7 | 190.0 | 133 | 0.023 | 947,695 | 922,776 | 97.4 | 17,241 | 0.05 |
| Claude Fable 5.2 | without Cerulion | 100 | 11.0 | 76.2 | 250.4 | 60 | 1.503 | 105,983 | 21,397 | 20.2 | 16,328 | 15.03 |
| Claude Fable 5.2 | with Cerulion | 99 | 14.9 | 257.9 | 485.8 | 2 | 1.822 | 880,495 | 859,179 | 97.6 | 35,927 | 2.15 |
| Gemini 4 Carbon | without Cerulion | 90 | 16.0 | 200.4 | 1,887.3 | 114 | 2.239 | 860,416 | 58,429 | 6.8 | 62,950 | 2.46 |
| Gemini 4 Carbon | with Cerulion | 95 | 18.3 | 247.7 | 1,092.3 | 130 | 1.286 | 1,159,505 | 663,206 | 57.2 | 22,660 | 1.61 |
| Gemini 4 Argon | without Cerulion | 97 | 37.2 | 379.9 | 1,883.8 | 189 | 5.425 | 2,463,783 | 142,244 | 5.8 | 76,741 | 8.35 |
| Gemini 4 Argon | with Cerulion | 100 | 16.5 | 218.5 | 610.8 | 81 | 1.368 | 1,174,355 | 627,839 | 53.5 | 21,226 | 1.63 |
| Gemini 4 Hydrogen | without Cerulion | 100 | 59.9 | 458.2 | 1,007.0 | 337 | 7.633 | 4,700,914 | 94,256 | 2.0 | 95,520 | 19.57 |
| Gemini 4 Hydrogen | with Cerulion | 100 | 22.6 | 294.3 | 296.9 | 125 | 1.155 | 1,616,643 | 1,007,103 | 62.3 | 22,015 | 1.46 |
| GPT-6.2 Astra | without Cerulion | 100 | 29.8 | 303.4 | 636.6 | 578 | 3.193 | 353,544 | 7,432 | 2.1 | 10,525 | 3.94 |
| GPT-6.2 Astra | with Cerulion | 100 | 20.5 | 296.1 | 261.6 | 369 | 2.453 | 820,286 | 552,715 | 67.4 | 2,283 | 3.19 |
| GPT-6.2 Sol | without Cerulion | 100 | 51.2 | 432.5 | 983.1 | 1,013 | 1.890 | 843,932 | 14,292 | 1.7 | 22,930 | 3.20 |
| GPT-6.2 Sol | with Cerulion | 100 | 28.8 | 304.8 | 340.6 | 533 | 1.128 | 1,285,109 | 783,656 | 61.0 | 4,711 | 1.55 |
| GPT-6.1 Sol | without Cerulion | 100 | 39.9 | 405.2 | 1,100.1 | 852 | 1.353 | 691,562 | 68,988 | 10.0 | 10,073 | 2.29 |
| GPT-6.1 Sol | with Cerulion | 100 | 15.4 | 262.4 | 290.2 | 326 | 0.382 | 622,075 | 462,791 | 74.4 | 1,750 | 0.56 |
| GPT-6 Luna | without Cerulion | 100 | 67.7 | 817.0 | 1,733.2 | 1,204 | 0.259 | 2,528,921 | 150,218 | 5.9 | 38,497 | none solved |
| GPT-6 Luna | with Cerulion | 100 | 17.4 | 197.2 | 401.2 | 313 | 0.037 | 809,603 | 535,276 | 66.1 | 7,864 | 0.13 |
| Grok 4.7 | without Cerulion | 99 | 74.6 | 731.3 | 2,257.5 | 1,033 | 2.570 | 2,931,670 | 2,485,755 | 84.8 | 72,595 | 63.61 |
| Grok 4.7 | with Cerulion | 100 | 29.9 | 372.4 | 816.2 | 384 | 1.347 | 1,777,782 | 1,562,231 | 87.9 | 22,471 | 2.54 |
| GPT-6.1 Terra | without Cerulion | 100 | 49.8 | 481.0 | 773.9 | 743 | 1.104 | 1,105,381 | 80,638 | 7.3 | 15,115 | 1.94 |
| GPT-6.1 Terra | with Cerulion | 100 | 17.6 | 307.7 | 220.0 | 275 | 0.286 | 754,473 | 506,409 | 67.1 | 2,526 | 0.39 |
| Gemini 4 Neon | without Cerulion | 100 | 25.2 | 275.5 | 552.2 | 44 | 4.014 | 1,145,469 | 50,569 | 4.4 | 48,106 | 4.32 |
| Gemini 4 Neon | with Cerulion | 100 | 16.5 | 233.2 | 190.8 | 9 | 1.246 | 1,020,629 | 708,096 | 69.4 | 13,481 | 1.38 |

Wall seconds are wall clock from the episode's start to its end, tool execution and failed requests included. A failed request was sent again inside its episode after a 20 second wait, and a request could wait up to 600 seconds before it timed out, so an arm's seconds carry its endpoint's stalls: read them with the arm's failed requests, printed beside them. Other workloads shared the machines. The 20 episodes that the 90 minute clock ended (5400 seconds each) are in no mean here; leaving them out lowers the mean wall seconds of Gemini 4 Carbon, Gemini 4 Argon and Grok 4.7 without Cerulion, and of Claude Fable 5.2 and Gemini 4 Carbon with it. A statement about speed in wall seconds is read with all of this in mind (RULES.md section 6).

USD is the model's reported token counts of the scored attempt at the list prices of prices.csv on the date in that file; cached input is priced at the cached rate; simulator compute is not included; nothing here is a billed amount. USD per solved start state is the spend of the arm's scored episodes over the start states it solved: the 28 episodes that ended without a score and the other attempts in lost_attempts.csv are not in it (223.00 USD in all), and no figure counts the robot's calibration episodes or the two attempts lost with the stopped machines. Part of the gap in USD between the arms is caching, not only fewer requests: cached input is priced lower, and the arm without Cerulion had the lower cached share of input tokens for 16 of the 16 models. Where an arm's prompt changes near its start from turn to turn, as the published interface's does when its oldest images drop out, less of it can be read from a cache. Tokens in counts every prompt token, cached ones included; tokens out counts reasoning tokens.

## Model cost and wall time per scored episode, without and with Cerulion (descriptive)

| model | USD an episode without Cerulion | with Cerulion | cost with Cerulion | wall seconds an episode without Cerulion | with Cerulion | time with Cerulion |
|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | 1.666 | 0.348 | 4.8 times cheaper | 846.6 | 258.7 | 3.3 times faster |
| GPT-6 Astra | 4.183 | 2.387 | 1.8 times cheaper | 787.9 | 289.4 | 2.7 times faster |
| Claude Opus 5.5 | 8.235 | 0.614 | 13 times cheaper | 902.1 | 242.9 | 3.7 times faster |
| Claude Sonnet 5.5 | 3.509 | 0.366 | 9.6 times cheaper | 533.2 | 233.5 | 2.3 times faster |
| Claude Haiku 5.5 | 0.256 | 0.023 | 11 times cheaper | 1,102.7 | 190.0 | 5.8 times faster |
| Claude Fable 5.2 | 1.503 | 1.822 | 1.2 times dearer | 250.4 | 485.8 | 1.9 times slower |
| Gemini 4 Carbon | 2.239 | 1.286 | 1.7 times cheaper | 1,887.3 | 1,092.3 | 1.7 times faster |
| Gemini 4 Argon | 5.425 | 1.368 | 4.0 times cheaper | 1,883.8 | 610.8 | 3.1 times faster |
| Gemini 4 Hydrogen | 7.633 | 1.155 | 6.6 times cheaper | 1,007.0 | 296.9 | 3.4 times faster |
| GPT-6.2 Astra | 3.193 | 2.453 | 1.3 times cheaper | 636.6 | 261.6 | 2.4 times faster |
| GPT-6.2 Sol | 1.890 | 1.128 | 1.7 times cheaper | 983.1 | 340.6 | 2.9 times faster |
| GPT-6.1 Sol | 1.353 | 0.382 | 3.5 times cheaper | 1,100.1 | 290.2 | 3.8 times faster |
| GPT-6 Luna | 0.259 | 0.037 | 7.0 times cheaper | 1,733.2 | 401.2 | 4.3 times faster |
| Grok 4.7 | 2.570 | 1.347 | 1.9 times cheaper | 2,257.5 | 816.2 | 2.8 times faster |
| GPT-6.1 Terra | 1.104 | 0.286 | 3.9 times cheaper | 773.9 | 220.0 | 3.5 times faster |
| Gemini 4 Neon | 4.014 | 1.246 | 3.2 times cheaper | 552.2 | 190.8 | 2.9 times faster |

Each figure is the arm's USD or wall seconds column in the means table above, a mean over its scored episodes, with what the notes under that table say about each: USD at list price, leaving out what the USD note names, and part of the gap in it caching; wall seconds carrying each endpoint's stalls and failed requests, so a statement about speed is read with them in mind (RULES.md section 6). Each factor is the larger mean of its pair over the smaller, to two decimals under 1.1, one under 10 and a whole number from 10. RULES.md section 3 declares the statement that episodes with Cerulion cost less, per model where one costs more, as this table prints it; no rule declared these factors or a statement about speed.

## How the replies ended (finish reason), scored episodes

| model | arm | replies | tool_calls | stop | length | content_filter | other | requests with no reply | scored episodes with a content_filter reply | ended on content_filter replies |
|---|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | without Cerulion | 5,587 | 5,582 | 5 | 0 | 0 | 0 | 366 | 0 | 0 |
| Gemini 3.8 Flash | with Cerulion | 2,555 | 2,536 | 19 | 0 | 0 | 0 | 126 | 0 | 0 |
| GPT-6 Astra | without Cerulion | 2,991 | 2,991 | 0 | 0 | 0 | 0 | 648 | 0 | 0 |
| GPT-6 Astra | with Cerulion | 1,635 | 1,635 | 0 | 0 | 0 | 0 | 310 | 0 | 0 |
| Claude Opus 5.5 | without Cerulion | 5,510 | 5,510 | 0 | 0 | 0 | 0 | 466 | 0 | 0 |
| Claude Opus 5.5 | with Cerulion | 1,579 | 1,579 | 0 | 0 | 0 | 0 | 201 | 0 | 0 |
| Claude Sonnet 5.5 | without Cerulion | 5,625 | 5,625 | 0 | 0 | 0 | 0 | 344 | 0 | 0 |
| Claude Sonnet 5.5 | with Cerulion | 1,897 | 1,897 | 0 | 0 | 0 | 0 | 202 | 0 | 0 |
| Claude Haiku 5.5 | without Cerulion | 6,536 | 6,536 | 0 | 0 | 0 | 0 | 2,364 | 0 | 0 |
| Claude Haiku 5.5 | with Cerulion | 1,708 | 1,707 | 1 | 0 | 0 | 0 | 333 | 0 | 0 |
| Claude Fable 5.2 | without Cerulion | 1,101 | 764 | 0 | 0 | 337 | 0 | 260 | 94 | 86 |
| Claude Fable 5.2 | with Cerulion | 1,471 | 1,469 | 0 | 2 | 0 | 0 | 200 | 0 | 0 |
| Gemini 4 Carbon | without Cerulion | 1,444 | 1,439 | 3 | 2 | 0 | 0 | 114 | 0 | 0 |
| Gemini 4 Carbon | with Cerulion | 1,743 | 1,740 | 1 | 2 | 0 | 0 | 130 | 0 | 0 |
| Gemini 4 Argon | without Cerulion | 3,605 | 3,603 | 0 | 2 | 0 | 0 | 189 | 0 | 0 |
| Gemini 4 Argon | with Cerulion | 1,647 | 1,644 | 0 | 2 | 1 | 0 | 81 | 1 | 0 |
| Gemini 4 Hydrogen | without Cerulion | 5,990 | 5,989 | 0 | 0 | 1 | 0 | 337 | 1 | 0 |
| Gemini 4 Hydrogen | with Cerulion | 2,261 | 2,261 | 0 | 0 | 0 | 0 | 125 | 0 | 0 |
| GPT-6.2 Astra | without Cerulion | 2,981 | 2,981 | 0 | 0 | 0 | 0 | 578 | 0 | 0 |
| GPT-6.2 Astra | with Cerulion | 2,051 | 2,051 | 0 | 0 | 0 | 0 | 369 | 0 | 0 |
| GPT-6.2 Sol | without Cerulion | 5,116 | 5,116 | 0 | 0 | 0 | 0 | 1,013 | 0 | 0 |
| GPT-6.2 Sol | with Cerulion | 2,876 | 2,876 | 0 | 0 | 0 | 0 | 533 | 0 | 0 |
| GPT-6.1 Sol | without Cerulion | 3,994 | 3,994 | 0 | 0 | 0 | 0 | 852 | 0 | 0 |
| GPT-6.1 Sol | with Cerulion | 1,538 | 1,538 | 0 | 0 | 0 | 0 | 326 | 0 | 0 |
| GPT-6 Luna | without Cerulion | 6,772 | 6,772 | 0 | 0 | 0 | 0 | 1,204 | 0 | 0 |
| GPT-6 Luna | with Cerulion | 1,739 | 1,738 | 1 | 0 | 0 | 0 | 313 | 0 | 0 |
| Grok 4.7 | without Cerulion | 7,383 | 7,305 | 78 | 0 | 0 | 0 | 1,033 | 0 | 0 |
| Grok 4.7 | with Cerulion | 2,985 | 2,985 | 0 | 0 | 0 | 0 | 384 | 0 | 0 |
| GPT-6.1 Terra | without Cerulion | 4,979 | 4,979 | 0 | 0 | 0 | 0 | 743 | 0 | 0 |
| GPT-6.1 Terra | with Cerulion | 1,761 | 1,761 | 0 | 0 | 0 | 0 | 275 | 0 | 0 |
| Gemini 4 Neon | without Cerulion | 2,519 | 2,519 | 0 | 0 | 0 | 0 | 44 | 0 | 0 |
| Gemini 4 Neon | with Cerulion | 1,651 | 1,651 | 0 | 0 | 0 | 0 | 9 | 0 | 0 |

A reply's finish reason is the one its model endpoint returned: `tool_calls` a reply with a tool call, `stop` and `length` one that ended on its own or at the output limit, `content_filter` one the provider's content filter stopped, `other` any other. Every reply of each scored attempt is counted, so the five reasons add up to the replies. A request with no reply is a failed request (sent again) or one the endpoint refused for a request field it does not take (sent again without the field).

4 of the 32 arms had a content_filter reply: Claude Fable 5.2 without Cerulion (337 of its 1,101 replies in scored episodes, in 94 of its 100 scored episodes, 86 of which ended on filtered replies); Gemini 4 Argon without Cerulion (1 of the 155 replies of its 3 unscored episodes, in 1 of them); Gemini 4 Argon with Cerulion (1 of its 1,647 replies in scored episodes, in 1 of its 100 scored episodes, which did not end on it); Gemini 4 Hydrogen without Cerulion (1 of its 5,990 replies in scored episodes, in 1 of its 100 scored episodes, which did not end on it). The unscored episodes' replies are in unscored.csv, per episode.

## Episodes that ended without a score

| model | arm | count | reason | start states |
|---|---|---|---|---|
| Claude Opus 5.5 | without Cerulion | 8 | the model endpoint answered rate limit on every attempt | pro_goal_task_t01_i03, pro_goal_task_t02_i03, pro_spatial_task_t00_i04, pro_spatial_task_t01_i04, pro_spatial_task_t02_i04, pro_object_task_t01_i04, pro_goal_task_t01_i04, pro_goal_swap_t02_i08 |
| Claude Fable 5.2 | with Cerulion | 1 | the 90 minute episode clock ran out | pro_goal_task_t00_i03 |
| Gemini 4 Carbon | without Cerulion | 10 | the 90 minute episode clock ran out | pro_goal_swap_t03_i06, pro_goal_swap_t05_i06, pro_goal_task_t00_i03, pro_goal_swap_t05_i07, pro_goal_task_t01_i04, pro_goal_swap_t02_i08, pro_goal_swap_t02_i09, pro_goal_swap_t03_i09, pro_goal_swap_t09_i10, pro_goal_task_t02_i06 |
| Gemini 4 Carbon | with Cerulion | 5 | the 90 minute episode clock ran out | pro_goal_task_t00_i03, pro_goal_task_t01_i03, pro_goal_task_t00_i05, pro_goal_task_t01_i05, pro_goal_task_t01_i06 |
| Gemini 4 Argon | without Cerulion | 3 | the 90 minute episode clock ran out | pro_goal_task_t00_i04, pro_goal_swap_t00_i08, pro_goal_task_t01_i06 |
| Grok 4.7 | without Cerulion | 1 | the 90 minute episode clock ran out | pro_object_swap_t00_i03 |

Each one counts as not solved, not clean and no false claim for its arm. Their attempts are listed and priced in lost_attempts.csv, as are attempts of scored episodes that are in no row.
