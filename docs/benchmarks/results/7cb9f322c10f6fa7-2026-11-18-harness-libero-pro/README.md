# The Cerulion harness on LIBERO-PRO: the results package

Every start state, every count and every episode behind the published Cerulion harness benchmark numbers, with
the script that computes the numbers from the rows.

The benchmark: simulated robot manipulation tasks from LIBERO-PRO, each run twice with the same model. Once
`without` Cerulion, through a published three-tool interface. Once `with` Cerulion, through the Cerulion
harness's tools. Same simulator, same start state, same budget, same scorer. Fifteen models, 100 start states
each, one episode a start state an arm.

Said before any number (RULES.md section 2):

1. They are new start states of 25 task rows that the Cerulion harness was developed on at other start states.
   The tasks themselves are not new to it. Task rows that the Cerulion harness never met exist in the benchmark
   and are on no list here.
2. The result is on one benchmark family, the one the Cerulion harness was built and checked on.
3. The arm with Cerulion reads a record of the robot's own calibration that the other arm does not have. The
   comparison is the whole package against the published interface; it cannot be split into tools, record and
   documents.
4. One episode a start state at temperature 1.0. Provider model versions are known only by the model's name.
5. Other groups publish higher rates on this benchmark under their own protocols. Nothing here supports a
   ranking against any of them.

With them: the arm without Cerulion is the benchmark's published minimal interface, not the best harness there
is. The calibration records were made by the robot's own calibration episodes on development start states of
LIBERO-10 and LIBERO-PRO, and which record each episode read is not recorded. The arm without Cerulion reads the
gripper in one of the two ways the published paper allows, the one that scored lower when both ran in an
earlier check on LIBERO-10 (not in this package); the other way was not run here, so a small gain over this arm
is not shown to hold against it. CLEAN was defined before the first episode with Cerulion on the earlier lists,
and became a claim only after the first of those lists had shown a gain on it; for this grid it was declared
before the first episode.

Two of the models, Gemini 3.8 Flash and GPT-6 Astra, also ran further start states the same way. The 400-cell
lists were added after the first rows of the 100-cell grid had been scored and seen. The 100-cell test declared
beforehand remains the primary result. One list holds 400 further start states of the same 25
LIBERO-PRO task rows. The other is on LIBERO-10. LIBERO-10's ten tasks are the ones the Cerulion harness was
developed and gated on; these are 40 other start states of each (here 38 to 40 a task, 395 in all: five that
earlier files had named were left out, not replaced). Supporting evidence, not a second benchmark. A further 40
LIBERO-10 start states, 0 to 3 of each task, are ones the harness was developed on: development cells, not a
result, printed apart as counts only.

![Success rate per model, without and with Cerulion, ordered by the success rate without it, with the factor of
improvement at the right of each row.](../../../media/harness-success-rate.svg)

![Model cost per successful episode, without and with Cerulion, in the same order, on a linear scale cut at 30 USD,
with the factor at the right of each row.](../../../media/harness-cost-per-success.svg)

Both charts are drawn from this package's CSV files by
[build_harness_charts.py](../../../media/charts/build_harness_charts.py), which takes every count and cost from
recompute.py, so each figure on them is the figure SUMMARY.md prints.

## Start here

- [SUMMARY.md](SUMMARY.md): the main test as it was declared before the first episode, the two secondary tests,
  every model and arm with intervals, the paired counts and exact tests, the two cross-model pairs named before
  the grid, cost and time per episode with each arm's failed requests and cached share. It opens with the lines
  above, states when the rows were read and names every model whose start states had not all ended then, with
  its counts; once every column is complete it goes on with what must be read before any of its numbers, and
  prints each declared test's verdict with the counts that qualify it.
- [extension_list2/SUMMARY.md](extension_list2/SUMMARY.md) and
  [extension_libero10/SUMMARY.md](extension_libero10/SUMMARY.md): the two further lists of the two models, read
  as one family of twelve tests by the extension's rule, with the lines that must be read before any of their
  numbers and the bad news beside the numbers it qualifies.
- [extension_libero10_development/SUMMARY.md](extension_libero10_development/SUMMARY.md): development cells, not
  a result. Counts only.
- [RULES.md](RULES.md): the reading rules, each frozen before the rows of its own lists.
- recompute.py: computes every SUMMARY.md, summary.csv, paired.csv and task_rows.csv again from the CSV files,
  with Python 3 and its standard library only. With `--check` it writes nothing and says whether the files still
  match. Either way it fails if a row of a cells.csv no longer matches its content hash.

## Files

| file | what it holds |
|---|---|
| cells.csv | the 100 start states of the grid in the frozen list's order: suite, perturbation, task, task row, start state, environment seed, the suite's published step limit, the task sentence served to the model, the SHA-256 of the simulator's state at reset, and a content hash of the row |
| episodes.csv | one row for every scored episode, both arms, every model |
| unscored.csv | every episode that ended without a score, with its reason and the list-price cost of its attempts |
| not_finished.csv | every episode not yet ended when the rows were read (empty once the grid is complete) |
| lost_attempts.csv | every attempt that is in no scored row, priced; `before_rerun` is 1 for an attempt of an episode that was started again from attempt 1 (see below) |
| prices.csv | the list prices used, per million tokens, with the page and the date each row was read |
| summary.csv | per model and arm: counts, Wilson intervals, means per scored episode, the cached share of input tokens, the failed requests in all, and the model's paired counts and exact test on SOLVED |
| paired.csv | per model and measure (solved, clean, false claim): the paired counts, the difference with its interval, the exact p, the Holm threshold and the verdict; the grid declares no per-model false-claim test, so its false-claim rows read `descriptive, no test declared` |
| SUMMARY.md | the same as text |
| list_info.json | when the rows were read, the list's hashes and its freeze time |
| transcripts/ | six whole episodes as text: the model's messages, the tool calls and their answers, and the names of the camera frames the model received (the images are not in this package) |
| extension_list2/ | the same files for list 2, 400 further start states of the same 25 task rows run by two of the models, with task_rows.csv (its own SUMMARY.md says how it is read) |
| extension_libero10/ | the same files for the LIBERO-10 list, 395 start states (10 to 49 of each of its ten tasks, less five that earlier files had named) run by the same two models |
| extension_libero10_development/ | the 40 LIBERO-10 development start states (0 to 3 of each task) of the same two models: development cells, not a result; counts only |
| refusals.csv | in each extension folder: every attempt in which the model endpoint answered HTTP 400 "Image processing blocked due to content policy violation", with how many requests it refused, whether that attempt was the scored one, and `before_rerun` as in lost_attempts.csv |
| stopped_attempts.csv | in each extension folder: every attempt that was running when its machine stopped and left no token count on any copy, with its start (for an attempt after a failed one, the earliest it can have started), the machine's stop and the longest it can have run; such an attempt is in no other file and in no total |
| recompute.py | the script |

## Columns of episodes.csv

| column | meaning |
|---|---|
| list | `grid`; `list2` in extension_list2/, `libero10` in extension_libero10/, `libero10_development` in extension_libero10_development/ |
| cell | the start state's id: suite, perturbation, task and start state, as in cells.csv |
| arm | `without` or `with` Cerulion |
| model | the model's public name |
| attempt | which attempt was scored (1 to 3; a retry happens only when the endpoint failed, never because of an outcome) |
| solved | 1 if the benchmark's own success predicate was true within the 900 control steps |
| clean | 1 if solved at or before the suite's published step limit (step_limit) with no protocol break, no problem row and no false claim |
| claimed_success | 1 if the model ended by saying it had succeeded |
| false_claim | 1 if it said so on a start state that was not solved |
| first_success_step | the control step at which the predicate first held, empty if never |
| step_limit | the suite's published step limit for the start state, as in cells.csv (the limit `clean` is read against) |
| requests | model replies used (budget 80) |
| steps | control steps used (budget 900) |
| wall_s | wall clock seconds from the episode's start to its end, tool execution and failed requests included |
| usd_list | the scored attempt's reported token counts at list price; not a billed amount; simulator compute not included |
| price_date | the date the price row was read |
| tokens_in | prompt tokens over the episode, cached ones included |
| tokens_cached | of those, the tokens the endpoint reported as read from its cache |
| tokens_cache_write | of those, the tokens the endpoint reported as written to its cache (priced at the write rate where the table has one) |
| tokens_out | output tokens, reasoning tokens included |
| failed_requests | requests the endpoint refused or dropped; each was sent again |
| ended_by | how the episode ended: the model's claim, the episode ending under it, the model no longer calling tools, or a budget |
| problems | rows the scorer flagged (an episode with one is not clean) |

## How the episodes ran

- Both arms: 80 model replies, one tool call a reply, 900 control steps, temperature 1.0, high reasoning
  effort, the last 4 images kept in context, a 90 minute clock, environment seed 142 plus the start state,
  sampling seed 1 sent with each model request. The endpoint of the four Claude models refused the seed field,
  so it was dropped and every request that endpoint answered ran without a seed. Zero-shot: nothing learned on
  any task is carried in. The score is the benchmark's own success predicate, read from the simulator's log
  after the episode. The model never sees it.
- `without`: three tools (move_to, done, give_up) and the prompt of arXiv 2610.30051, word for word.
- `with`: the Cerulion harness's 44 tools. This arm also reads a record of the robot's own calibration, which
  the other arm does not have.
- The same request settings in both arms of a model. The Claude models carry the prompt cache flag in both arms.
- Up to 3 attempts when the model endpoint fails; a clock stop is final. An episode with no score after that is
  a failure for its arm in every count, and is listed in unscored.csv.
- The simulation ran on cloud machines without a GPU, both arms of a model through the same endpoint.
- The code of both arms is the measured code of the Cerulion harness, with these snapshot ids:
  with Cerulion `sha256:7cb9f322c10f6fa7e64a3f8da7c22c67d67280eca436a03d2d87683496b4d5b8`,
  without `sha256:b73c548a1f0436f52ce41cffc7d4a4a2784e5d6170e2dd2c757c9cabdf146637`.
  The first 16 characters of the first id name this folder.

## The list

The 100 start states are the four lowest stored start states of each of 25 task rows that no earlier list, log
or development run had used. The rule was written before the list was drawn. The list was frozen at
2026-11-17T21:22:04Z, before any episode ran on it; list_info.json holds the SHA-256 of the frozen file and of
its cell lines. Before the freeze every start state was built once with no model and no motion: its state hash
is in cells.csv, and the success predicate was false at reset on every one.

## Things to know before quoting a number

- At this package's read every model had finished all 100 start states in both arms. A model whose start
  states had not all ended at a read is marked incomplete, with its n and the read time: start states that end
  first are the short episodes, so an incomplete row is not a fair sample.
- GPT-6.1 Terra was added to the grid after its first rows had been seen. Its column is marked so in SUMMARY.md
  and on both charts.
- One further model, Gemini Robotics-ER 2 (preview), was tried and left out before any grid row: its endpoint
  failed the arm with Cerulion on 5 of 5 trial attempts. It has no column.
- The four Claude models share one endpoint. To lower the load on it, no episode of theirs started between
  2026-11-18T01:30:07Z and 07:21:18Z (5 h 51 min), so their later start states ran hours after the other
  models' did, in the list's order and both arms alike. For 13 start states of a Claude model (Claude Opus 5.5
  7, Claude Sonnet 5.5 3, Claude Haiku 5.5 2, Claude Fable 5.2 1) the two arms ran on either side of that
  pause.
- Beyond that pause, the two arms of a start state did not always start in the same hour. Start states whose two
  arms started more than 1 h apart, per model: Claude Opus 5.5 8 (up to 7.7 h), Gemini 4 Argon 10 (up to
  1.5 h), Grok 4.7 4, Claude Sonnet 5.5 3, Claude Haiku 5.5 2, Claude Fable 5.2 2, GPT-6 Luna 2, GPT-6.1 Sol 1.
  The Claude counts include the 13 start states across the pause, and Claude Opus 5.5's also one of the two
  start states moved off a stopped machine (below). episodes.csv holds no start times.
- Five start states of GPT-6 Astra's `without` arm were started again from attempt 1 because the harness
  refused to start them before any model call; the refused attempts the records still hold are in
  lost_attempts.csv with 0 replies.
- Two of the machines stopped during the run. Their scored rows are kept. Every episode they had not ended ran
  from attempt 1 on another machine; for two start states of Claude Opus 5.5 this put the `without` arm on a
  different machine from the `with` arm, one of them hours later. The two attempts that were running when their machine
  stopped left no token count, so they are in no file and their cost is in no total. Before a third machine was
  removed, the 94 Claude episodes waiting on it that had not started were moved to other machines (12 of them
  twice) and ran there.
- A network outage of 22 minutes cut the model endpoint off from three of the machines. It ended one episode
  without a score after its 3 attempts (Claude Sonnet 5.5, `without`, pro_goal_task_t00_i06). That episode was
  started again from attempt 1 under RULES.md section 5 (5); its outage attempts are in lost_attempts.csv with
  `before_rerun` 1. A first restart of it was refused by the harness twice before any model call; those two
  attempts left empty folders and are in no file.
- Providers gave no version string beyond the model's name. Grok 4.7 answered from two provider deployments,
  alternating through the whole run. The grid cannot show that no provider changed a model behind its name.
- Wall clock seconds include time an endpoint stalled or refused requests, and other workloads shared the
  machines: one machine ran up to 9 grid episodes at once. SUMMARY.md prints each arm's failed requests beside
  its seconds; the episodes the 90 minute clock ended are in no mean.
- Cost per success counts the scored episodes only. Part of the gap in USD between the arms is caching:
  SUMMARY.md prints each arm's cached share of input tokens, and the arm without Cerulion had the lower share
  for every model.
- On the LIBERO-10 list the provider's image filter refused GPT-6 Astra's `without` arm, and only that arm, on
  tasks 4 and 6 (HTTP 400, "Image processing blocked due to content policy violation"). No start state was run
  again for it: the requests reached the provider and were answered. GPT-6 Astra's LIBERO-10 SOLVED and FALSE
  CLAIM results come from those two tasks; extension_libero10/SUMMARY.md prints this beside every number it
  qualifies.
- Two of the machines also stopped during the extension's run. The episodes they had not ended, all on the
  LIBERO-10 list and its development start states, ran from attempt 1 on three other machines hours later; for
  3 start states this put the two arms of a model on two machines hours apart (GPT-6 Astra l10_t06_i44 and
  l10_t02_i29, Gemini 3.8 Flash l10_t06_i29), and two machines then ran up to 4 GPT-6 Astra episodes at once
  where the protocol allowed 2 a machine. The attempts that were running when the machines stopped are in
  extension_libero10/stopped_attempts.csv and in no total; that SUMMARY.md prices them at the spend a second of
  their model and arm, and no row bounds them. A row written in a stopped machine's last seconds and never
  copied off cannot be ruled out; the two start states that could hold one (Gemini 3.8 Flash, `without`,
  l10_t06_i29 and l10_t05_i30) ran again and were not solved.
- The 40 development start states are the ones the Cerulion harness was developed on. They are in no test, no
  pool, no headline and no chart.

## What was removed

Nothing was removed from the rows. The CSV files name models by their public names; they carry no machine,
address or account. The six transcripts list what was replaced in them in transcripts/README.md.

## Licences

This package is part of the Cerulion repository and is under that repository's licence (AGPL-3.0-only), with two
exceptions. The task sentences in cells.csv and in the transcripts are those of LIBERO and LIBERO-PRO (their
code under the MIT licence, their datasets under CC BY 4.0). The system prompt and tool descriptions of the `without` arm, in its
three transcripts, are from May Espinola, Qifan Zhang and Yu Xiang, "From Reasoning to Action: Benchmarking
Frontier AI Models on LIBERO-10 Robot Manipulation Tasks", arXiv:2610.30051v1, under CC BY 4.0; they were served
word for word, with the robot's name filled in for each start state.
