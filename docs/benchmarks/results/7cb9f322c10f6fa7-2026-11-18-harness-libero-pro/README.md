# The Cerulion harness on LIBERO-PRO: the results package

Every start state, every count and every episode behind the published Cerulion harness benchmark numbers, with
the script that computes the numbers from the rows.

The benchmark: simulated robot manipulation tasks from LIBERO-PRO, each run twice with the same model. Once
`without` Cerulion, through a published three-tool interface. Once `with` Cerulion, through the Cerulion
harness's tools. Same simulator, same start state, same budget, same scorer. Sixteen models, 100 start states
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
  the grid, cost and time per episode with each arm's failed requests and cached share, and every arm's replies
  by finish reason. It opens with the lines above, states when the rows were read and names every model whose
  start states had not all ended then, with its counts; once every column is complete it goes on with what must
  be read before any of its numbers, and prints each declared test's verdict with the counts that qualify it.
- [extension_list2/SUMMARY.md](extension_list2/SUMMARY.md) and
  [extension_libero10/SUMMARY.md](extension_libero10/SUMMARY.md): the two further lists of the two models, read
  as one family of twelve tests by the extension's rule, with the lines that must be read before any of their
  numbers and the bad news beside the numbers it qualifies.
- [extension_libero10_development/SUMMARY.md](extension_libero10_development/SUMMARY.md): development cells, not
  a result. Counts only.
- [RULES.md](RULES.md): the reading rules, each frozen before the rows of its own lists.
- recompute.py: computes every SUMMARY.md, summary.csv, paired.csv and task_rows.csv again from the CSV files,
  with Python 3 and its standard library only. With `--check` it writes nothing and says whether the files still
  match. Either way it fails if a row of a cells.csv no longer matches its content hash, or if the five replies
  columns of a row of an episodes.csv do not add up to its `requests`.

## Files

| file | what it holds |
|---|---|
| cells.csv | the 100 start states of the grid in the frozen list's order: suite, perturbation, task, task row, start state, environment seed, the suite's published step limit, the task sentence served to the model, the SHA-256 of the simulator's state at reset, and a content hash of the row |
| episodes.csv | one row for every scored episode, both arms, every model |
| unscored.csv | every episode that ended without a score, with its reason, the list-price cost of its attempts, and the replies of those attempts by finish reason with the requests that got no reply (the episodes.csv columns of those names, summed over every attempt the cost covers) |
| not_finished.csv | every episode not yet ended when the rows were read (empty once the grid is complete) |
| lost_attempts.csv | attempts in no scored row of the episodes in episodes.csv and unscored.csv, priced; `before_rerun` is 1 for an attempt from before its episode was started again from attempt 1 after the network outage or a machine stop (see below) |
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
| ended_by | how the episode ended: the model's claim, the episode ending under it, the model no longer calling tools (`stopped_calling_tools`; content_filter_at_end says whether the provider's content filter stopped the replies that ended it), or a budget |
| problems | rows the scorer flagged (an episode with one is not clean) |
| replies_tool_calls | replies of the scored attempt whose finish reason, as the model endpoint returned it, was `tool_calls`: a reply with a tool call |
| replies_stop | replies whose finish reason was `stop`: the reply ended on its own |
| replies_length | replies whose finish reason was `length`: the reply was cut off at the output limit |
| replies_content_filter | replies whose finish reason was `content_filter`: the provider's content filter stopped the reply, and every such reply here came back with no text and no tool call |
| replies_other | replies with any other finish reason; the five replies columns add up to `requests` |
| requests_no_reply | requests of the scored attempt that got no reply, not counted in `requests` (which counts replies): a failed request (counted in failed_requests, sent again) or one the endpoint refused for a request field it does not take (sent again without the field); in unscored.csv they also include the request each attempt ended on, a failure or one the 90 minute clock cut off, which was not sent again |
| content_filter_at_end | the replies in a row at the end of the scored attempt that came back `content_filter`, 0 when its last reply did not; three replies in a row with no tool call end an episode, so 3 or more on an episode that ended `stopped_calling_tools` means the provider's content filter stopped every reply that ended it |

## How the episodes ran

- Both arms: 80 model replies, one tool call a reply, 900 control steps, temperature 1.0, high reasoning
  effort, the newest 4 pictures kept in the conversation, a 90 minute clock, environment seed 142 plus the start
  state, sampling seed 1 sent with each model request. The endpoint of the four Claude models refused the seed and
  parallel_tool_calls fields, so both were dropped and every request that endpoint answered ran without them (the
  operator still ran only the first call of a reply). Zero-shot: nothing learned on
  any task is carried in. The score is the benchmark's own success predicate, read from the simulator's log
  after the episode. The model never sees it.
- `without`: three tools (move_to, done, give_up) and the prompt of arXiv 2610.30051, word for word.
- `with`: the Cerulion harness's 44 tools. This arm also reads a record of the robot's own calibration, which
  the other arm does not have.
- Pictures beyond the newest 4: every arm swapped each of them for a short text inside its older turn (the
  harness's `swap` picture mode), except the four Claude columns with Cerulion. Those kept every picture where it
  was and marked their opening (the tools, the system text and the first message) for the prompt cache (`append`,
  the harness's default picture mode for a Claude model with Cerulion).
- The same request settings in both arms of a model, apart from the picture handling and the cache mark above.
  The Claude models carry the prompt cache flag in both arms.
- Up to 3 attempts when the model endpoint fails; a clock stop is final. An episode that the network outage, a
  machine stop or a harness refusal before any model call left without a score was started again from attempt 1
  (see below); any other episode with no score after its attempts is a failure for its arm in every count, and
  is listed in unscored.csv.
- The simulation ran on cloud machines with the CPU renderer (OSMesa); no GPU was used. Both arms of a model ran
  through the same endpoint.
- The code of both arms is the measured code of the Cerulion harness, with these snapshot ids:
  with Cerulion `sha256:7cb9f322c10f6fa7e64a3f8da7c22c67d67280eca436a03d2d87683496b4d5b8`,
  without `sha256:b73c548a1f0436f52ce41cffc7d4a4a2784e5d6170e2dd2c757c9cabdf146637`.
  The first 16 characters of the first id name this folder. The four Claude columns with Cerulion ran
  `sha256:4f8bbe79fdf38df1770c068ec470fd3d5a629fd371f7288887cb3d275f9f747e`: the tree of the first id with the
  `append` picture mode added to two of its operator files, `bench/operators/chat_support.py` and
  `bench/operators/chat_operator.py`.
- To run a start state yourself: `cerulion rcp bench --model MODEL --cell CELL`
  ([docs/harness.md](../../../harness.md#how-cerulion-rcp-bench-reproduces-it)) runs it in both arms with the
  same budgets, temperature and scorer. With a Claude model the arm with Cerulion runs `append` with the cache
  mark, as the four Claude columns here ran; every other model and arm runs `swap`. `cerulion rcp bench --verify`
  checks the harness folder against `sha256:4f8bbe79fdf38df1770c068ec470fd3d5a629fd371f7288887cb3d275f9f747e` with
  Cerulion and `sha256:6f040ca78bae0efd437005afc04ead78c4295e1e471d854b3e7ff0f9ea01f2a4` without: the two trees
  above with `append` added, which in `swap` mode send what those trees sent.

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
- GPT-6.1 Terra was added to the grid after its first rows had been seen, and Gemini 4 Neon after every other
  column had finished and been read. Both columns are marked so in SUMMARY.md and on both charts.
- Gemini 4 Neon's 200 episodes ran on 2026-11-21 from 06:54Z to 09:39Z, after the other 15 columns' rows
  had been read at 2026-11-19T20:35:04Z. They ran on one machine, up to 8 at once, with other work on that
  machine; the two arms of each start state started within 4 minutes of each other. Its price row was read on
  2026-11-21. With it each per-model family holds 16 tests, so every Holm threshold under 0.05 moved: GPT-6.1
  Terra's SOLVED test (p = 0.00763) is under its threshold with 15 models (0.0083) and over it with 16 (0.0071),
  so it reads "points up, not established". Every other verdict reads the same with 15 models as with 16.
- One further model, Gemini Robotics-ER 2 (preview), was tried and left out before any grid row: its endpoint
  failed the arm with Cerulion on 5 of 5 trial attempts. It has no column.
- The four Claude models share one endpoint. To lower the load on it, no episode of theirs started between
  2026-11-18T01:30:07Z and 07:21:18Z (5 h 51 min), so the later start states of their arms without Cerulion ran
  hours after the other models' did, in the list's order.
- For the other models the two arms of a start state did not always start in the same hour. Start states whose
  two arms started more than 1 h apart, per model: Gemini 4 Argon 10 (up to 1.5 h), Grok 4.7 4, GPT-6 Luna 2,
  GPT-6.1 Sol 1. episodes.csv holds no start times.
- Five start states of GPT-6 Astra's `without` arm were started again from attempt 1 because the harness
  refused to start them before any model call; the refused attempts the records still hold are in
  lost_attempts.csv with 0 replies.
- Two of the machines stopped during the run. Every episode they had not ended ran from attempt 1 on another
  machine. The two attempts that were running when their machine stopped left no token count, so they are in no
  file and their cost is in no total. Before a third machine was removed, the 47 Claude episodes without
  Cerulion waiting on it that had not started were moved to other machines (6 of them twice) and ran there.
- A network outage of 22 minutes cut the model endpoint off from three of the machines. It ended one episode
  without a score after its 3 attempts (Claude Sonnet 5.5, `without`, pro_goal_task_t00_i06). That episode was
  started again from attempt 1 under RULES.md section 5 (5); its outage attempts are in lost_attempts.csv with
  `before_rerun` 1. A first restart of it was refused by the harness twice before any model call; those two
  attempts left empty folders and are in no file.
- Providers gave no version string beyond the model's name. Grok 4.7 answered from two provider deployments,
  alternating through the whole run. The grid cannot show that no provider changed a model behind its name.
- Wall clock seconds include time an endpoint stalled or refused requests, and other workloads shared the
  machines: one machine ran up to 14 grid episodes at once. SUMMARY.md prints each arm's failed requests beside
  its seconds; the episodes the 90 minute clock ended are in no mean.
- Cost per success counts the scored episodes only. Part of the gap in USD between the arms is caching:
  SUMMARY.md prints each arm's cached share of input tokens, and the arm without Cerulion had the lower share
  for every model. The four Claude columns with Cerulion also kept their older pictures and marked their opening
  for the prompt cache (How the episodes ran), which their arms without Cerulion did not.
- The provider's content filter stopped many of Claude Fable 5.2's replies in its arm without Cerulion and
  none in its arm with Cerulion. A stopped reply comes back with finish reason `content_filter`, and most of
  that arm's episodes ended on three of them in a row, with no claim from the model. SUMMARY.md names every
  arm with such a reply under "Read these first", with its counts; only an arm whose unsolved episodes ended
  on filtered replies also gets its false-claim rates read with them. episodes.csv counts every reply by its
  finish reason.
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
