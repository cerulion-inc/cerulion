# Six whole episodes

Three models, both arms each, on one start state a model. The start state for a model is the first one in the frozen list's order on which both arms were scored and one arm solved it while the other did not. No other choice was made.

| model | start state | arm | solved | folder | camera frames named |
|---|---|---|---|---|---|
| Gemini 3.8 Flash | pro_goal_swap_t01_i06 | without Cerulion | no | [gemini-3-8-flash_without_pro_goal_swap_t01_i06](gemini-3-8-flash_without_pro_goal_swap_t01_i06/transcript.md) | 36 |
| Gemini 3.8 Flash | pro_goal_swap_t01_i06 | with Cerulion | yes | [gemini-3-8-flash_with_pro_goal_swap_t01_i06](gemini-3-8-flash_with_pro_goal_swap_t01_i06/transcript.md) | 24 |
| GPT-6 Astra | pro_goal_swap_t04_i06 | without Cerulion | no | [gpt-6-astra_without_pro_goal_swap_t04_i06](gpt-6-astra_without_pro_goal_swap_t04_i06/transcript.md) | 68 |
| GPT-6 Astra | pro_goal_swap_t04_i06 | with Cerulion | yes | [gpt-6-astra_with_pro_goal_swap_t04_i06](gpt-6-astra_with_pro_goal_swap_t04_i06/transcript.md) | 3 |
| Claude Opus 5.5 | pro_goal_swap_t01_i06 | without Cerulion | no | [claude-opus-5-5_without_pro_goal_swap_t01_i06](claude-opus-5-5_without_pro_goal_swap_t01_i06/transcript.md) | 152 |
| Claude Opus 5.5 | pro_goal_swap_t01_i06 | with Cerulion | yes | [claude-opus-5-5_with_pro_goal_swap_t01_i06](claude-opus-5-5_with_pro_goal_swap_t01_i06/transcript.md) | 14 |

Each folder holds transcript.md (readable), transcript.jsonl (the operator's log, one record a line: start, request, strict_endpoint, reply, tool_call, frames, http_error, notice, end; a strict_endpoint record says the endpoint refused a request field, which was dropped before the request was sent again). The camera images are not in this package: each is named by its file name where the model received it.

Removed or replaced in every transcript, and nothing else: the camera images themselves, the folder the episode ran in (frame paths now read frames/NAME.png), the address of the model endpoint, the endpoint's own text in failed requests (the status code is kept), the endpoint's extra usage fields beyond the token counts, the endpoint's tool call identifiers (now call_1, call_2, in order), the endpoint's name for the model (now the model's public name), the product's working name (now Cerulion), and typographic dashes in any text (now a hyphen). No endpoint returned reasoning text, so the transcripts hold none.

The full text of every tool the model could call is in the measured code of the Cerulion harness.
