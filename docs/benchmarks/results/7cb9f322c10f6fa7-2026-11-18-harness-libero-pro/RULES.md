# The reading rules, each as frozen before the rows of its own lists

This file is the public text of two rule files, each written and frozen before the rows of its own lists. It says
how the numbers in this package are to be read, what may be said from them and what may not. SUMMARY.md and the
SUMMARY.md of each extension folder apply it; recompute.py computes every count and test it names.

| rule file | covers | frozen (UTC) | SHA-256 of the frozen file | what existed at that time |
|---|---|---|---|---|
| the grid's rule | the 100 start states, every model | 2026-11-17T20:49:34Z | 07ecb185801e6c00611441b70cb2beca21b8c931bee48ff3bcc4f1b1f1b657a2 | no grid row; the list of 100 start states was frozen at 21:22:04Z and the first scored episode with Cerulion started at 21:25:40Z |
| the extension's rule | list 2 (400 further start states), the LIBERO-10 list and its 40 development start states | 2026-11-17T21:58:06Z | 39353a2dacdb5234b654d6002e75caa340070dccb3207901da9eecd89a38f0f7 | no extension list and no extension row; the three lists were frozen at 22:12:37Z. The first rows of the grid HAD been seen (section 7) |

What this public text changes against the frozen files: file names, folder names, machine names and the working
names of the people and stages that wrote them are removed; Cerulion is the product's name; sections that
report earlier measurements on other lists and other benchmarks, and the tables of numbers published by other
groups, are left out because none of that data is in this package; the wording of some sentences is shortened;
nine lines are reworded: section 4 (b) to (f) and (h), section 5 items 4 and 5, and the last sentence of section
7's paragraph on unscored and lost start states. No other rule was added, dropped or weakened. The two hashes are
of the frozen files, so a later release of them can be checked against this table.

## 1. Words

- Start state: one stored initial scene of one task of the benchmark. A cell of the lists is one start state.
- Task row: one task of one suite under one perturbation. The lists hold 25 task rows.
- The two arms. `without`: the model on a published three-tool interface (one move tool, done, give up; the
  prompt and tool texts of arXiv 2610.30051, word for word). `with`: the same model on the Cerulion harness's tools.
- Shared by both arms: the start states, the environment seed (142 plus the start state), the sampling seed 1
  sent with each model request (the endpoint of the four Claude models refused the seed and parallel_tool_calls
  fields, so both were dropped and every request that endpoint answered ran without them; the operator still ran
  only the first call of a reply), 80 model replies, one tool call a reply, 900 control steps (45 simulated
  seconds), temperature 1.0, high reasoning effort, a 90 minute clock, the benchmark's own success predicate
  read after the episode. Zero-shot: an empty store, nothing learned on any task carried in. One episode a
  start state an arm.
- SOLVED: the benchmark's success predicate true within the 900-step budget.
- CLEAN: solved at or before the published step limit of the suite (Goal 300, Spatial 220, Object 280, LIBERO-10 520), with no
  protocol break, no problem row and no false claim.
- FALSE CLAIM: the episode ends with the model saying it succeeded on a start state that was not solved.
- Unscored: an episode that ended without a score after the retry rule (section 4 (d)).

## 2. What the start states are, said before any number

1. They are new start states of 25 task rows that the Cerulion harness was developed on at other start states. The tasks
   themselves are not new to it. Task rows that the Cerulion harness never met exist in the benchmark and are on no list here.
2. The result is on one benchmark family, the one the Cerulion harness was built and checked on.
3. The arm with Cerulion reads a record of the robot's own calibration that the other arm does not have. The
   comparison is the whole package against the published interface; it cannot be split into tools, record and
   documents.
4. One episode a start state at temperature 1.0. Provider model versions are known only by the model's name.
5. Other groups publish higher rates on this benchmark under their own protocols. Nothing here supports a
   ranking against any of them.

## 3. The grid: one declared main test, and a correction for everything else

Said before the rows: at 100 start states a column, each run once, a column can show a gain of about 20 points
or more and almost nothing smaller. A gain of 10 points has power 0.54 uncorrected and 0.23 after a ten-model
correction. Most strong models will read "no difference shown", and that is the expected result. A grid of many
models read without a rule invites picking the outcome, the list or the model, so the rule is fixed first.

THE MAIN TEST (one test, alpha 0.05, no correction): Gemini 3.8 Flash, SOLVED within the 900-step budget, with
Cerulion against without, paired by start state on the grid's start states, exact two-sided sign test on the
discordant start states. It is a declared replication of an effect measured before on two earlier lists; a
single test fixed in advance (model, outcome, test, start states) is what closes the choices above.

TWO SECONDARY TESTS, declared, each at alpha 0.05, apart from the per-model family:
1. GPT-6 Astra, CLEAN.
2. Fewer false claims, pooled over all models that ran both arms: for each model and start state, a false claim
   without Cerulion only against a false claim with Cerulion only, one sign test over all discordant pairs. The
   false claims per unsolved episode are printed beside it for each model.

EVERY OTHER MODEL (the per-model family): Holm's step-down correction over the per-model SOLVED tests of all
the models that ran, however they came out. A model that passes is "established". A model at or under 0.05 that
does not pass Holm is "points up, not established" (or "points down"). Otherwise "no difference shown". The same
family rule, separately, for CLEAN. The rule was written for ten models (first threshold 0.005); fifteen ran,
so the first threshold is 0.05 / 15. SUMMARY.md counts the main test's model inside the family as well, which is
the stricter reading; the main test itself is read alone.

ACROSS MODELS, descriptive only: the gain against the model's rate without Cerulion. No significance is claimed
for a trend; the models share start states and the trend was expected before the grid.

Every column is printed: all models, both arms, solved and clean, Wilson intervals, the discordant counts, the
exact p, the Holm verdict, unscored start states counted and named. No model is dropped after its numbers are
seen.

Count needed, said before the rows. From the discordance measured on the two earlier lists (70 start states,
not in this package): the main test needed 15 to 23 start states; GPT-6 Astra CLEAN at least 63 as a declared
secondary; a GPT-6 Astra SOLVED difference of the size seen there needs about 1,200 start states and is not
chased. For each further model the grid answers one question: is the gain 20 points or more (power 0.96 at
alpha 0.05, 0.80 at the corrected alpha with 98 start states).

What must hold for each statement:

| statement | stands if | falls or narrows if |
|---|---|---|
| With Gemini 3.8 Flash, Cerulion solves more start states | the main test passes, with the rate with Cerulion overlapping the earlier interval of 61.5 to 81.9 percent and the rate without overlapping 10.1 to 27.6 percent (two earlier lists, not in this package) | the main test fails: it is printed as two lists that did not replicate a third time, and is not the headline. If it passes with a rate outside the earlier interval: say the earlier draws were lucky |
| With GPT-6 Astra, more solves come inside the published step limit | GPT-6 Astra CLEAN passes on at least 63 start states | not significant: "no solve gain; a step-limit gain not confirmed" |
| Fewer episodes end on a false claim | the pooled test passes, with the rate per unsolved episode printed for each model | per unsolved episode the arm with Cerulion claims falsely as often or more for most models: the statement narrows to "fewer because it solves more" |
| Episodes with Cerulion use fewer requests and cost less | requests and steps are lower in each model's column; USD from one dated price table; cache settings named | a model whose column with Cerulion costs more: print it; the statement becomes per model |
| A cheaper model with Cerulion against a dearer model without | only for pairs named before the grid was read: Gemini 3.8 Flash with Cerulion against GPT-6 Astra without, and against GPT-6.2 Astra without | any other such pair found in the grid is exploratory and labelled so |
| "Cerulion lifts model X" for a further model | X passes Holm on SOLVED | else "points up, not established" or "no difference shown" |

## 4. What must be true of the grid itself

(a) The same start states in both arms of a model; the list frozen and hashed before the first row.
(b) Every start state fresh: none that was developed on, none that any run before the list's freeze used; the
    source of the robot's calibration record named.
(c) The list fixed before the first scored episode; the code snapshot of each arm of each model named.
(d) Retries for infrastructure only, within an episode, by a rule set before (3 attempts; a clock stop is
    final); the episode's attempts in no row counted and priced; an unscored start state is a failure for that
    arm in every rate and is listed.
(e) Both arms of a model run through the same endpoint, with the provider's version string logged from the
    response.
(f) The effort, cache setting and picture mode of each arm of each model printed; every other request setting
    the same in both arms of a model.
(g) A model with one arm missing is printed as incomplete and is in no comparison.
(h) Each arm of each model holds one episode of each start state.

## 5. If a column comes in against Cerulion

1. It is printed in the same table, the same size, and named with its counts in the first paragraph of the
   write-up. It is not rerun, not moved to an appendix, not explained before it is stated.
2. Against Cerulion and significant after Holm: the statement becomes "Cerulion helps models whose rate without it
   is low and hurt model X".
3. Against Cerulion and not significant: "no difference shown for X (a of n with, b of n without, p)". It still
   counts in the tally "Cerulion ahead in k of m models", which is printed whole.
4. The same snapshot, prompt and effort for every episode of an arm of that model.
5. Retries only within an episode, and only where it did not run as declared (an endpoint failure, a crashed
   tool, an unscored start state), by the rule written before, applied to both arms alike.
6. If the main test itself comes in against Cerulion or null: there is no solve statement.

## 6. Cost and time

USD is the model's reported token counts at list price from one dated table (prices.csv), cached input at the
cached rate; nothing is a billed amount; simulator compute is not included. Seconds are the episode's wall
clock and include the time an endpoint stalled or refused requests, so a statement about speed in wall seconds
is read with that in mind.

## 7. The extension: further start states for two models

Said before its rows:

1. THE EXTENSION WAS DECIDED AFTER GRID ROWS WERE SEEN. It was decided at 2026-11-17T21:49:00Z, 97 seconds after
   the first scored grid rows of these two models had been read, and those rows pointed Cerulion's way. The intent
   to run more start states predates the grid, but a reader cannot check a motive, so every extension number is
   printed with this line and the 100-start-state main test stays THE main test. The extension can add
   precision and a replication. It cannot rescue, replace or upgrade the main test.
2. The start states are, again, new start states of task rows the Cerulion harness was developed on. The tasks are not new.
3. "500 an arm" is the field's count for trained policies, not its shape. The published protocol is one suite
   by perturbation column of 10 tasks x 50 start states. Here 500 is 25 task rows x 20 start states over six
   columns (Goal by position 10 tasks x 20; five other columns 3 tasks x 20 each), and it is a pool of two lists.
4. The pooled 500 contains the main test's 100 start states. A pooled test is not a second confirmation.
5. Start states of one task row are not independent, so the task-row count is printed beside every test.
6. GPT-6 Astra SOLVED still cannot be settled: power at 400 start states is 0.341 uncorrected for the
   discordance measured earlier. A null there is the expected result.

What it is: Gemini 3.8 Flash and GPT-6 Astra only, both arms exactly as the grid runs them. List 2 is the same
25 task rows, for each row the next 16 lowest stored start states that nothing had named. A further list on
LIBERO-10 is part of the same rule: its ten tasks x start states 10 to 49, where a start state that something had
named is left out and not replaced (the count for that task falls and is printed), and 40 development start
states (0 to 3 of each task) printed apart and in no test. No count is cut or grown by any result.

The numbers and how each is read:

| number | start states | read |
|---|---|---|
| list 2 alone, a model | 400 | its own list, read alone; the extension's test list for this benchmark |
| the two lists together, a model | 500 = the grid's 100 + list 2's 400 | a declared secondary that makes an estimate: rates, Wilson intervals, the difference with its interval. No statement rests on its p |
| the LIBERO-10 list alone, a model | up to 400 | its own list, read alone; supporting only, never a second independent benchmark |
| the LIBERO-10 development start states, a model | 40 | development: counts only, under the heading "development cells, not a result" |

Never formed: list 2 or the pool with any earlier list; this benchmark pooled with LIBERO-10; the two models
pooled for SOLVED or CLEAN.

The extension family: every (model, list, measure): {Gemini 3.8 Flash, GPT-6 Astra} x {list 2, the LIBERO-10
list} x {SOLVED, CLEAN, FALSE CLAIM} = 12 tests, each an exact two-sided sign test on the discordant start
states. Holm's step-down over the 12 at 0.05: the smallest p is held to 0.05 / 12 = 0.00417, the next to
0.05 / 11, and so on. Two-sided: a test can pass against Cerulion and is then printed and worded as section 5.
Verdicts: passes Holm = "established on the extension list"; at or under 0.05 but not Holm = "points up (or
down), not established"; else "no difference shown". The family is settled only when all four columns are
complete; before that no extension test is called established.

For each test: both arms' counts and Wilson 95 intervals, the difference in points with its Newcombe 95
interval, the discordant counts, the exact p, the threshold, the verdict; the same test on scored pairs only;
and the task-row count (rows where the arm with Cerulion has more, fewer, the same; exact sign test on the untied
rows). If a start-state test passes Holm but its task-row p is above 0.05, the statement must give the row
counts ("on k of the 25 task rows", or "on k of the 10 tasks" on LIBERO-10) and may not say "across the
benchmark" or "across the suite".

How the extension relates to the grid's tests (no statement counted twice). The grid's tests are read on the
grid's start states only. No extension start state enters them; no grid start state enters the extension family.

| the grid's test | the extension's test on the same question | what may be said |
|---|---|---|
| passes | passes | the statement, then "and it held on 400 further start states (a list added after the first grid rows were seen)" |
| passes | does not pass | the statement stands, and the same sentence carries "it did not hold on the 400-cell extension list (counts, p)" |
| does not pass | passes | no upgrade: the grid's rule decides. The extension result may be printed after the sentence that the declared 100-start-state test did not pass |
| does not pass | does not pass | no statement; both printed |

One effect is one statement: the grid's test and the extension's test are never listed as two findings, and
their p values are never multiplied or combined (the two lists share all 25 task rows, the snapshot, the
calibration records and the day).

The pool: printed for each model and measure with its caveat every time: "a pool of two lists: 100 cells
whose first rows had been seen when the other 400 were added; 25 task rows, 20 start states each, six suite by
perturbation columns; not the published 10 tasks x 50 of one column". Beside the pool, always, the two parts
apart. If an arm's rates on the two parts have Wilson 95 intervals that do not overlap, the pooled rate is not
printed for that arm and measure. The pool exists only when both parts are complete for that model.

Declared, descriptive, in no family: the cross-model pair Gemini 3.8 Flash with Cerulion against GPT-6 Astra
without (worded only as "no difference shown" or "differs, outside the corrected family"); requests, steps,
tokens and USD per episode; false claims per unsolved episode; counts by suite and perturbation with no test.
Anything else read from the rows is exploratory and labelled so.

Unscored and lost start states: as section 4 (d). An unscored start state stays in the denominator and enters
the paired test as a failure. More than 5 percent of an arm unscored in a column: the count goes in the
column's heading and in every sentence from it. Each arm of a model holds one episode of each start state, scored
or not.

When a column may be printed: complete (every start state ended in both arms), or incomplete only with its n
and the word "incomplete", at a stop time written down before it arrives, counting the longest prefix of the
frozen list in which every start state has ended in both arms. No column, list, model, arm or measure is
dropped after any of its rows is seen.

Said wherever an extension number appears, before the number:

- "These are new start states of task rows the Cerulion harness was developed on at other start states. They
  are not unseen tasks."
- "The 400-cell lists were added after the first rows of the 100-cell grid had been scored and seen. The
  100-cell test declared beforehand remains the primary result."
- With any LIBERO-10 number: "LIBERO-10's ten tasks are the ones the Cerulion harness was developed and gated on;
  these are 40 other start states of each. Supporting evidence, not a second benchmark."
- With any pooled 500: the pool's caveat, word for word.
- With any number published by others: "Other systems report these under their own protocols; ours is not
  measured under any of them." Two tables under two headings, never one chart, no difference, ratio or rank
  across them.

On LIBERO-10 a test that passes reads "On LIBERO-10, the same direction on N other start states of its ten tasks:
a against b (p)", and one that does not, "On LIBERO-10, no difference shown for the model (a against b of N, p)".
The development start states read "On 40 development start states (not a result): a against b." and nothing
else.

Not said, whatever the rows show: that the tasks are new, held back or never developed on; that this is the
full benchmark, its standard protocol or 50 trials a task; that the extension's existence or size was declared
before the grid's rows (its reading rule was fixed before its own rows; its decision came after the first grid
rows were seen); that anything was confirmed twice or replicated independently; anything resting on the pooled p; any comparison verb, multiple or rank against a
published policy or system; that Cerulion helps strong models solve more, from a test that did not pass; that
Cerulion stops false claims; that LIBERO-10 is a second benchmark; any development number in a headline, chart of
results, pool or test; any number from an incomplete column without its n; any split found in the rows afterwards
worded as a finding.
