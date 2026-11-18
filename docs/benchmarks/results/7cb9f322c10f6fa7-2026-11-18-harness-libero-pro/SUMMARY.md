# Summary of the grid: 100 start states, 15 models, both arms

Computed by recompute.py from episodes.csv, unscored.csv and not_finished.csv as read at 2026-11-18T17:23:40Z. recompute.py computes this file again from the CSV files; with --check it writes nothing and says whether this file still matches them.

THE GRID WAS STILL RUNNING AT THIS READ. 14 of 15 models had finished all 100 start states in both arms. A model marked * is incomplete: its counts are of the start states that had ended, with n printed, and it gets no verdict. Start states that end first are the short episodes, so an incomplete row is not a fair sample of its list. Incomplete: Claude Haiku 5.5 (64 without, 65 with).

Every start state here is a new start state of a task row that Cerulion was developed on at other start states. The tasks themselves are not new to it.

Arms: `without` is the same model on a published three-tool interface; `with` is the same model on Cerulion's tools. Same start states, same budgets (80 replies, one tool call a reply, 900 control steps), one episode a start state an arm. An episode that ended without a score counts as a failure for its arm and stays in the count.

## The main test, as declared before the first episode

Declared: Gemini 3.8 Flash, SOLVED within the 900-step budget, with Cerulion against without, paired by start state on the grid's 100 start states, exact two-sided sign test on the discordant start states, alpha 0.05, no correction. One test.

Result (complete): with Cerulion 73 of 100 (63.6 to 80.7 percent), without 25 of 100 (17.5 to 34.3 percent). Solved with only: 52. Solved without only: 4. Both: 21. Neither: 23. Exact two-sided sign test p = 1.10e-11. Difference 48.0 points (Newcombe 95 for paired rates: 35.7 to 57.9).

The test passes at 0.05.

## The two secondary tests, declared with it (alpha 0.05 each, apart from the per-model family)

1. GPT-6 Astra, CLEAN (solved at or before the suite's published step limit, with no protocol break, no problem row and no false claim): with Cerulion 61 of 100 (51.2 to 70.0), without 44 of 100 (34.7 to 53.8); with only 26, without only 9, p = 0.00599. Its SOLVED counts, to be read with it: 70 against 71 (with only 13, without only 14, p = 1).

2. Fewer false claims, pooled over every model that ran both arms, one sign test over all discordant pairs of (model, start state) (INCOMPLETE: it pools the pairs ended so far): a false claim without Cerulion only on 203 pairs, with Cerulion only on 126 pairs, p = 2.58e-05. The rate per unsolved episode is in the table below for each model and arm.

## Every model, both arms

| model | arm | finished | solved | Wilson 95, percent | clean | Wilson 95, percent | false claims | per unsolved episode | unscored | not finished |
|---|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | without Cerulion | 100 | 25 | 17.5 to 34.3 | 10 | 5.5 to 17.4 | 30 | 0.40 | 0 | 0 |
| Gemini 3.8 Flash | with Cerulion | 100 | 73 | 63.6 to 80.7 | 68 | 58.3 to 76.3 | 16 | 0.59 | 0 | 0 |
| GPT-6 Astra | without Cerulion | 100 | 71 | 61.5 to 79.0 | 44 | 34.7 to 53.8 | 24 | 0.83 | 0 | 0 |
| GPT-6 Astra | with Cerulion | 100 | 70 | 60.4 to 78.1 | 61 | 51.2 to 70.0 | 14 | 0.47 | 0 | 0 |
| Claude Opus 5.5 | without Cerulion | 100 | 32 | 23.7 to 41.7 | 11 | 6.3 to 18.6 | 16 | 0.24 | 8 | 0 |
| Claude Opus 5.5 | with Cerulion | 100 | 75 | 65.7 to 82.5 | 67 | 57.3 to 75.4 | 5 | 0.20 | 2 | 0 |
| Claude Sonnet 5.5 | without Cerulion | 100 | 16 | 10.1 to 24.4 | 7 | 3.4 to 13.7 | 16 | 0.19 | 0 | 0 |
| Claude Sonnet 5.5 | with Cerulion | 100 | 49 | 39.4 to 58.7 | 36 | 27.3 to 45.8 | 22 | 0.43 | 0 | 0 |
| Claude Haiku 5.5* | without Cerulion | 64 | 4 | 2.5 to 15.0 | 1 | 0.3 to 8.3 | 3 | 0.05 | 0 | 36 |
| Claude Haiku 5.5* | with Cerulion | 65 | 27 | 30.4 to 53.7 | 18 | 18.3 to 39.6 | 13 | 0.34 | 0 | 35 |
| Claude Fable 5.2 | without Cerulion | 100 | 10 | 5.5 to 17.4 | 6 | 2.8 to 12.5 | 4 | 0.04 | 0 | 0 |
| Claude Fable 5.2 | with Cerulion | 100 | 82 | 73.3 to 88.3 | 67 | 57.3 to 75.4 | 9 | 0.50 | 2 | 0 |
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

Wilson intervals are over the finished start states of the arm (scored plus unscored). A false claim is an episode that ends with the model saying it succeeded on a start state that was not solved; per unsolved episode divides them by the arm's finished episodes that were not solved.

## Paired by start state, and the per-model family (15 models, Holm's step-down at 0.05)

The family is every model that ran, the main test's model included, however each came out; the main test above is read alone at 0.05 and does not depend on this table. Thresholds are computed from the p values of all the models as they stand, so they can move until the last column completes. Until then the verdict printed for a complete column can change with them.

### SOLVED

| model | paired start states | with only | without only | both | neither | difference, points (Newcombe 95) | exact p | Holm threshold | verdict |
|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | 100 | 52 | 4 | 21 | 23 | +48.0 (35.7 to 57.9) | 1.10e-11 | 0.0038 | established |
| GPT-6 Astra | 100 | 13 | 14 | 57 | 16 | -1.0 (-11.1 to 9.1) | 1 | 0.0500 | no difference shown |
| Claude Opus 5.5 | 100 | 46 | 3 | 29 | 22 | +43.0 (31.4 to 52.7) | 6.98e-11 | 0.0042 | established |
| Claude Sonnet 5.5 | 100 | 43 | 10 | 6 | 41 | +33.0 (19.6 to 44.8) | 5.55e-06 | 0.0056 | established |
| Claude Haiku 5.5* | 64 | 25 | 2 | 2 | 35 | +35.9 (21.9 to 48.6) | 5.65e-06 | 0.0063 | incomplete at the read of 2026-11-18T17:23:40Z: 64 of 100 |
| Claude Fable 5.2 | 100 | 73 | 1 | 9 | 17 | +72.0 (61.0 to 79.5) | 7.94e-21 | 0.0033 | established |
| Gemini 4 Carbon | 100 | 13 | 19 | 63 | 5 | -6.0 (-17.0 to 5.1) | 0.377 | 0.0167 | no difference shown |
| Gemini 4 Argon | 100 | 29 | 8 | 55 | 8 | +21.0 (9.5 to 31.8) | 7.53e-04 | 0.0071 | established |
| Gemini 4 Hydrogen | 100 | 48 | 8 | 31 | 13 | +40.0 (26.8 to 51.2) | 4.69e-08 | 0.0050 | established |
| GPT-6.2 Astra | 100 | 10 | 14 | 67 | 9 | -4.0 (-13.6 to 5.6) | 0.541 | 0.0250 | no difference shown |
| GPT-6.2 Sol | 100 | 26 | 12 | 47 | 15 | +14.0 (2.1 to 25.3) | 0.0336 | 0.0100 | points up, not established |
| GPT-6.1 Sol | 100 | 22 | 13 | 46 | 19 | +9.0 (-2.5 to 20.1) | 0.175 | 0.0125 | no difference shown |
| GPT-6 Luna | 100 | 29 | 0 | 0 | 71 | +29.0 (20.2 to 38.5) | 3.73e-09 | 0.0045 | established |
| Grok 4.7 | 100 | 51 | 2 | 2 | 45 | +49.0 (37.6 to 58.8) | 3.18e-13 | 0.0036 | established |
| GPT-6.1 Terra | 100 | 27 | 10 | 47 | 16 | +17.0 (5.4 to 28.0) | 0.00763 | 0.0083 | established (column added after the first rows were seen) |

### CLEAN

| model | paired start states | with only | without only | both | neither | difference, points (Newcombe 95) | exact p | Holm threshold | verdict |
|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | 100 | 60 | 2 | 8 | 30 | +58.0 (46.3 to 67.1) | 8.47e-16 | 0.0036 | established |
| GPT-6 Astra | 100 | 26 | 9 | 35 | 30 | +17.0 (5.7 to 27.5) | 0.00599 | 0.0167 | established |
| Claude Opus 5.5 | 100 | 59 | 3 | 8 | 30 | +56.0 (43.9 to 65.5) | 1.72e-14 | 0.0038 | established |
| Claude Sonnet 5.5 | 100 | 34 | 5 | 2 | 59 | +29.0 (17.7 to 39.5) | 2.43e-06 | 0.0083 | established |
| Claude Haiku 5.5* | 64 | 18 | 1 | 0 | 45 | +26.6 (14.4 to 38.7) | 7.63e-05 | 0.0125 | incomplete at the read of 2026-11-18T17:23:40Z: 64 of 100 |
| Claude Fable 5.2 | 100 | 62 | 1 | 5 | 32 | +61.0 (49.8 to 69.8) | 1.39e-17 | 0.0033 | established |
| Gemini 4 Carbon | 100 | 17 | 30 | 41 | 12 | -13.0 (-25.7 to 0.3) | 0.0789 | 0.0250 | no difference shown |
| Gemini 4 Argon | 100 | 50 | 7 | 18 | 25 | +43.0 (29.9 to 53.9) | 4.24e-09 | 0.0056 | established |
| Gemini 4 Hydrogen | 100 | 49 | 2 | 6 | 43 | +47.0 (35.7 to 56.7) | 1.18e-12 | 0.0042 | established |
| GPT-6.2 Astra | 100 | 15 | 11 | 43 | 31 | +4.0 (-5.9 to 13.7) | 0.557 | 0.0500 | no difference shown |
| GPT-6.2 Sol | 100 | 37 | 9 | 23 | 31 | +28.0 (15.4 to 39.2) | 4.06e-05 | 0.0100 | established |
| GPT-6.1 Sol | 100 | 42 | 6 | 19 | 33 | +36.0 (23.7 to 46.6) | 1.01e-07 | 0.0071 | established |
| GPT-6 Luna | 100 | 25 | 0 | 0 | 75 | +25.0 (16.7 to 34.3) | 5.96e-08 | 0.0063 | established |
| Grok 4.7 | 100 | 45 | 1 | 0 | 54 | +44.0 (33.3 to 53.9) | 1.34e-12 | 0.0045 | established |
| GPT-6.1 Terra | 100 | 42 | 2 | 20 | 36 | +40.0 (29.0 to 49.4) | 1.13e-10 | 0.0050 | established (column added after the first rows were seen) |

Tally over the 14 complete columns, printed whole: with Cerulion solved more start states in 11, fewer in 3, the same number in 0. Fewer: GPT-6 Astra (70 against 71, p = 1); Gemini 4 Carbon (76 against 82, p = 0.377); GPT-6.2 Astra (77 against 81, p = 0.541).

### FALSE CLAIM (descriptive for each model; the declared test is the pooled one above)

| model | paired start states | false claim without only | with only | exact p |
|---|---|---|---|---|
| Gemini 3.8 Flash | 100 | 20 | 6 | 0.00936 |
| GPT-6 Astra | 100 | 18 | 8 | 0.0755 |
| Claude Opus 5.5 | 100 | 14 | 3 | 0.0127 |
| Claude Sonnet 5.5 | 100 | 12 | 18 | 0.362 |
| Claude Haiku 5.5* | 64 | 3 | 12 | 0.0352 |
| Claude Fable 5.2 | 100 | 4 | 9 | 0.267 |
| Gemini 4 Carbon | 100 | 6 | 5 | 1 |
| Gemini 4 Argon | 100 | 23 | 4 | 3.11e-04 |
| Gemini 4 Hydrogen | 100 | 20 | 3 | 4.88e-04 |
| GPT-6.2 Astra | 100 | 9 | 7 | 0.804 |
| GPT-6.2 Sol | 100 | 17 | 6 | 0.0347 |
| GPT-6.1 Sol | 100 | 20 | 6 | 0.00936 |
| GPT-6 Luna | 100 | 12 | 14 | 0.845 |
| Grok 4.7 | 100 | 6 | 15 | 0.0784 |
| GPT-6.1 Terra | 100 | 19 | 10 | 0.136 |

## Means per scored episode

| model | arm | scored episodes | replies | control steps | wall seconds | USD, list price | tokens in | of which cached | tokens out | USD per solved start state |
|---|---|---|---|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | without Cerulion | 100 | 55.9 | 538.8 | 846.6 | 1.666 | 4,147,296 | 2,556,882 | 75,171 | 6.67 |
| Gemini 3.8 Flash | with Cerulion | 100 | 25.6 | 224.8 | 258.7 | 0.348 | 1,434,994 | 1,179,094 | 18,020 | 0.48 |
| GPT-6 Astra | without Cerulion | 100 | 29.9 | 305.0 | 787.9 | 4.183 | 423,452 | 49,626 | 7,896 | 5.89 |
| GPT-6 Astra | with Cerulion | 100 | 16.4 | 269.2 | 289.4 | 2.387 | 672,567 | 492,974 | 1,956 | 3.41 |
| Claude Opus 5.5 | without Cerulion | 92 | 59.9 | 497.3 | 902.1 | 8.235 | 1,429,377 | 11,128 | 57,088 | 23.68 |
| Claude Opus 5.5 | with Cerulion | 98 | 15.5 | 258.6 | 278.1 | 2.762 | 928,309 | 457,259 | 15,770 | 3.61 |
| Claude Sonnet 5.5 | without Cerulion | 100 | 56.2 | 609.6 | 533.2 | 3.509 | 1,232,207 | 3,411 | 43,649 | 21.93 |
| Claude Sonnet 5.5 | with Cerulion | 100 | 19.6 | 347.4 | 204.9 | 1.905 | 1,183,123 | 495,965 | 13,801 | 3.89 |
| Claude Haiku 5.5* | without Cerulion | 64 | 64.9 | 655.5 | 1,061.9 | 0.252 | 1,760,591 | 3,385 | 65,564 | 4.04 |
| Claude Haiku 5.5* | with Cerulion | 65 | 19.9 | 312.7 | 321.4 | 0.090 | 1,120,365 | 524,291 | 16,944 | 0.22 |
| Claude Fable 5.2 | without Cerulion | 100 | 11.0 | 76.2 | 250.4 | 1.503 | 105,983 | 21,397 | 16,328 | 15.03 |
| Claude Fable 5.2 | with Cerulion | 98 | 14.7 | 246.3 | 414.4 | 5.501 | 866,206 | 439,253 | 28,604 | 6.57 |
| Gemini 4 Carbon | without Cerulion | 90 | 16.0 | 200.4 | 1,887.3 | 2.239 | 860,416 | 58,429 | 62,950 | 2.46 |
| Gemini 4 Carbon | with Cerulion | 95 | 18.3 | 247.7 | 1,092.3 | 1.286 | 1,159,505 | 663,206 | 22,660 | 1.61 |
| Gemini 4 Argon | without Cerulion | 97 | 37.2 | 379.9 | 1,883.8 | 5.425 | 2,463,783 | 142,244 | 76,741 | 8.35 |
| Gemini 4 Argon | with Cerulion | 100 | 16.5 | 218.5 | 610.8 | 1.368 | 1,174,355 | 627,839 | 21,226 | 1.63 |
| Gemini 4 Hydrogen | without Cerulion | 100 | 59.9 | 458.2 | 1,007.0 | 7.633 | 4,700,914 | 94,256 | 95,520 | 19.57 |
| Gemini 4 Hydrogen | with Cerulion | 100 | 22.6 | 294.3 | 296.9 | 1.155 | 1,616,643 | 1,007,103 | 22,015 | 1.46 |
| GPT-6.2 Astra | without Cerulion | 100 | 29.8 | 303.4 | 636.6 | 3.193 | 353,544 | 7,432 | 10,525 | 3.94 |
| GPT-6.2 Astra | with Cerulion | 100 | 20.5 | 296.1 | 261.6 | 2.453 | 820,286 | 552,715 | 2,283 | 3.19 |
| GPT-6.2 Sol | without Cerulion | 100 | 51.2 | 432.5 | 983.1 | 1.890 | 843,932 | 14,292 | 22,930 | 3.20 |
| GPT-6.2 Sol | with Cerulion | 100 | 28.8 | 304.8 | 340.6 | 1.128 | 1,285,109 | 783,656 | 4,711 | 1.55 |
| GPT-6.1 Sol | without Cerulion | 100 | 39.9 | 405.2 | 1,100.1 | 1.353 | 691,562 | 68,988 | 10,073 | 2.29 |
| GPT-6.1 Sol | with Cerulion | 100 | 15.4 | 262.4 | 290.2 | 0.382 | 622,075 | 462,791 | 1,750 | 0.56 |
| GPT-6 Luna | without Cerulion | 100 | 67.7 | 817.0 | 1,733.2 | 0.259 | 2,528,921 | 150,218 | 38,497 | none solved |
| GPT-6 Luna | with Cerulion | 100 | 17.4 | 197.2 | 401.2 | 0.037 | 809,603 | 535,276 | 7,864 | 0.13 |
| Grok 4.7 | without Cerulion | 99 | 74.6 | 731.3 | 2,257.5 | 2.570 | 2,931,670 | 2,485,755 | 72,595 | 63.61 |
| Grok 4.7 | with Cerulion | 100 | 29.9 | 372.4 | 816.2 | 1.347 | 1,777,782 | 1,562,231 | 22,471 | 2.54 |
| GPT-6.1 Terra | without Cerulion | 100 | 49.8 | 481.0 | 773.9 | 1.104 | 1,105,381 | 80,638 | 15,115 | 1.94 |
| GPT-6.1 Terra | with Cerulion | 100 | 17.6 | 307.7 | 220.0 | 0.286 | 754,473 | 506,409 | 2,526 | 0.39 |

Wall seconds are wall clock from the episode's start to its end, tool execution and failed requests included. USD is the model's reported token counts of the scored attempt at the list prices of prices.csv on the date in that file; cached input is priced at the cached rate; simulator compute is not included; nothing here is a billed amount. Tokens in counts every prompt token, cached ones included; tokens out counts reasoning tokens.

## Episodes that ended without a score

| model | arm | count | reason | start states |
|---|---|---|---|---|
| Claude Opus 5.5 | without Cerulion | 8 | the model endpoint answered rate limit on every attempt | pro_goal_task_t01_i03, pro_goal_task_t02_i03, pro_spatial_task_t00_i04, pro_spatial_task_t01_i04, pro_spatial_task_t02_i04, pro_object_task_t01_i04, pro_goal_task_t01_i04, pro_goal_swap_t02_i08 |
| Claude Opus 5.5 | with Cerulion | 2 | the model endpoint answered rate limit on every attempt | pro_goal_swap_t03_i07, pro_goal_swap_t03_i08 |
| Claude Fable 5.2 | with Cerulion | 2 | the 90 minute episode clock ran out | pro_goal_task_t00_i04, pro_goal_task_t00_i05 |
| Gemini 4 Carbon | without Cerulion | 10 | the 90 minute episode clock ran out | pro_goal_swap_t03_i06, pro_goal_swap_t05_i06, pro_goal_task_t00_i03, pro_goal_swap_t05_i07, pro_goal_task_t01_i04, pro_goal_swap_t02_i08, pro_goal_swap_t02_i09, pro_goal_swap_t03_i09, pro_goal_swap_t09_i10, pro_goal_task_t02_i06 |
| Gemini 4 Carbon | with Cerulion | 5 | the 90 minute episode clock ran out | pro_goal_task_t00_i03, pro_goal_task_t01_i03, pro_goal_task_t00_i05, pro_goal_task_t01_i05, pro_goal_task_t01_i06 |
| Gemini 4 Argon | without Cerulion | 3 | the 90 minute episode clock ran out | pro_goal_task_t00_i04, pro_goal_swap_t00_i08, pro_goal_task_t01_i06 |
| Grok 4.7 | without Cerulion | 1 | the 90 minute episode clock ran out | pro_object_swap_t00_i03 |

Each one counts as not solved, not clean and no false claim for its arm. Attempts that are in no row are listed and priced in lost_attempts.csv.
