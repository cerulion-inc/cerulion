# Development cells, not a result: 40 LIBERO-10 start states, 2 models, both arms

Computed by ../recompute.py from this folder's CSV files as read at 2026-11-18T17:23:55Z. Every start state here has ended in both arms of both models.

Read this line before any number here:

- The 400-cell lists were added after the first rows of the 100-cell grid had been scored and seen. The 100-cell test declared beforehand remains the primary result. That test is the grid's main test, in ../SUMMARY.md.

These are start states 0 to 3 of each LIBERO-10 task, the start states Cerulion was developed on. They are in no test, no pool, no headline and no chart, and nothing here is a result: counts only.

## Development cells, not a result

| model | arm | solved | clean | false claims | unscored | scored |
|---|---|---|---|---|---|---|
| Gemini 3.8 Flash | with Cerulion | 24 of 40 | 24 of 40 | 7 of 40 | 0 | 40 |
| Gemini 3.8 Flash | without Cerulion | 8 of 40 | 6 of 40 | 5 of 40 | 0 | 40 |
| GPT-6 Astra | with Cerulion | 22 of 40 | 21 of 40 | 11 of 40 | 0 | 40 |
| GPT-6 Astra | without Cerulion | 19 of 40 | 13 of 40 | 11 of 40 | 1 | 39 |

The sentence the reading rule allows, and nothing else: "On 40 development start states (not a result): 24 against 8." (Gemini 3.8 Flash); "On 40 development start states (not a result): 22 against 19." (GPT-6 Astra).

Ended without a score (a failure for its arm in the counts above): GPT-6 Astra without Cerulion l10_t06_i02, 3 attempts: the model endpoint refused every attempt: HTTP 400, Image processing blocked due to content policy violation.
