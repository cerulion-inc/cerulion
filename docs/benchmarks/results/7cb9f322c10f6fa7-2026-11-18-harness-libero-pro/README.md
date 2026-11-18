# The Cerulion harness on LIBERO-PRO: the results package

Every start state, every count and every episode behind the published Cerulion harness benchmark numbers, with
the script that computes the numbers from the rows.

The benchmark: simulated robot manipulation tasks from LIBERO-PRO, each run twice with the same model. Once
`without` Cerulion, through a published three-tool interface. Once `with` Cerulion, through the Cerulion
harness's tools. Same simulator, same start state, same budget, same scorer. Fifteen models, 100 start states
each, one episode a start state an arm.

The start states are new start states of 25 task rows that the Cerulion harness was developed on at other start
states. The tasks themselves are not new to it.

![Success rate per model, without and with Cerulion, ordered by the success rate without it, with the factor of
improvement at the right of each row.](../../../media/harness-success-rate.svg)

![Model cost per successful episode, without and with Cerulion, in the same order, on a linear scale cut at 30 USD,
with the factor at the right of each row.](../../../media/harness-cost-per-success.svg)

Both charts are drawn from this package's CSV files by
[build_harness_charts.py](../../../media/charts/build_harness_charts.py), which takes every count and cost from
recompute.py, so each figure on them is the figure SUMMARY.md prints.

## Start here

- [SUMMARY.md](SUMMARY.md): the main test as it was declared before the first episode, the two secondary tests,
  every model and arm with intervals, the paired counts and exact tests, cost and time per episode. It states
  at its top when the rows were read, and names every model whose start states had not all ended then, with its
  counts.
- [RULES.md](RULES.md): the reading rules, frozen before the rows.
- recompute.py: computes SUMMARY.md, summary.csv and paired.csv again from the CSV files, with Python 3 and its
  standard library only. With `--check` it writes nothing and says whether the files still match.

## Files

| file | what it holds |
|---|---|
| cells.csv | the 100 start states of the grid in the frozen list's order: suite, perturbation, task, task row, start state, environment seed, the suite's published step limit, the task sentence served to the model, the SHA-256 of the simulator's state at reset, and a content hash of the row |
| episodes.csv | one row for every scored episode, both arms, every model |
| unscored.csv | every episode that ended without a score, with its reason and the list-price cost of its attempts |
| not_finished.csv | every episode not yet ended when the rows were read (empty once the grid is complete) |
| lost_attempts.csv | every attempt that is in no scored row, priced; `before_rerun` is 1 for an attempt of an episode that was started again from attempt 1 (see below) |
| prices.csv | the list prices used, per million tokens, with the page and the date each row was read |
| summary.csv | per model and arm: counts, Wilson intervals, means per scored episode, and the model's paired counts and exact test on SOLVED |
| paired.csv | per model and measure (solved, clean, false claim): the paired counts, the difference with its interval, the exact p, the Holm threshold and the verdict |
| SUMMARY.md | the same as text |
| list_info.json | when the rows were read, the list's hashes and its freeze time |
| transcripts/ | six whole episodes as text: the model's messages, the tool calls and their answers, and the names of the camera frames the model received (the images are not in this package) |
| extension_list2/ | the same files for a second list of 400 further start states run by two of the models (its own SUMMARY.md says how it is read) |
| recompute.py | the script |

## Columns of episodes.csv

| column | meaning |
|---|---|
| list | `grid`, or `list2` in extension_list2/ |
| cell | the start state's id: suite, perturbation, task and start state, as in cells.csv |
| arm | `without` or `with` Cerulion |
| model | the model's public name |
| attempt | which attempt was scored (1 to 3; a retry happens only when the endpoint failed, never because of an outcome) |
| solved | 1 if the benchmark's own success predicate was true within the 900 control steps |
| clean | 1 if solved at or before the suite's published step limit (step_limit) with no protocol break, no problem row and no false claim |
| claimed_success | 1 if the model ended by saying it had succeeded |
| false_claim | 1 if it said so on a start state that was not solved |
| first_success_step | the control step at which the predicate first held, empty if never |
| requests | model replies used (budget 80) |
| steps | control steps used (budget 900) |
| operator_s | wall clock seconds from the operator's start to its end, tool execution and failed requests included |
| usd_list | the scored attempt's reported token counts at list price; not a billed amount; simulator compute not included |
| price_date | the date the price row was read |
| tokens_in | prompt tokens over the episode, cached ones included |
| tokens_cached | of those, the tokens the endpoint reported as read from its cache |
| tokens_cache_write | of those, the tokens the endpoint reported as written to its cache (priced at the write rate where the table has one) |
| tokens_out | output tokens, reasoning tokens included |
| failed_requests | requests the endpoint refused or dropped; each was sent again |
| operator_end | how the operator ended: the model's claim, the episode ending under it, the model no longer calling tools, or a budget |
| problems | rows the scorer flagged (an episode with one is not clean) |

## How the episodes ran

- Both arms: 80 model replies, one tool call a reply, 900 control steps, temperature 1.0, high reasoning
  effort, the last 4 images kept in context, a 90 minute clock, environment seed 142 plus the start state,
  operator seed 1. Zero-shot: nothing learned on any task is carried in. The score is the benchmark's own
  success predicate, read from the simulator's log after the episode. The model never sees it.
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

- Models whose 100 start states had not all ended when the rows were read are marked incomplete, with their n
  and the read time. Start states that end first are the short episodes, so an incomplete row is not a fair
  sample.
- GPT-6.1 Terra was added to the grid after its first rows had been seen. Its column is marked so.
- One further model, Gemini Robotics-ER 2 (preview), was tried and left out before any grid row: its endpoint
  failed the arm with Cerulion on 5 of 5 trial attempts. It has no column.
- The four Claude models share one endpoint. No new episode of theirs started from 2026-11-18T01:31:31Z to
  07:21:16Z, to lower the load on that endpoint, so their later start states ran hours after the other models'
  did, in the list's order and both arms alike.
- Five start states of GPT-6 Astra's `without` arm were started again from attempt 1 because the harness
  refused to start them before any model call; the refused attempts the records still hold are in
  lost_attempts.csv with 0 replies.
- Two of the machines stopped during the run. Their scored rows are kept. Every episode they had not ended ran
  from attempt 1 on another machine; for two start states of Claude Opus 5.5 this put the `without` arm on a
  different machine from the `with` arm, one of them hours later. The two attempts that were running when their machine
  stopped left no token count, so they are in no file and their cost is in no total.
- A network outage of 22 minutes cut the model endpoint off from three of the machines. It ended one episode
  without a score after its 3 attempts (Claude Sonnet 5.5, `without`, pro_goal_task_t00_i06). That episode was
  started again from attempt 1 under RULES.md section 5 (5); its outage attempts are in lost_attempts.csv with
  `before_rerun` 1.
- Providers gave no version string beyond the model's name.
- Wall clock seconds include time an endpoint stalled or refused requests.
- RULES.md section 7 also names a list on LIBERO-10. It is not in this package.

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
