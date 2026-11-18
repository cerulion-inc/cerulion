# SPDX-License-Identifier: AGPL-3.0-only
"""List prices used to turn token counts into a cost, and the arithmetic. Standard library only.

The table was read from the model makers' public price pages on the date in DATE (one row was read a day later,
see its note). Prices are USD per million tokens. A cost printed from this table is a list-price estimate from
the token counts your endpoint reported; it is not a bill. Your endpoint may charge other prices.
"""

DATE = "2026-11-17"
SOURCES = {
    "openai": "https://developers.openai.com/api/docs/pricing",
    "google": "https://cloud.google.com/vertex-ai/generative-ai/pricing",
    "anthropic": "https://platform.claude.com/docs/en/about-claude/pricing",
    "xai": "https://docs.x.ai/docs/models",
}
# key: (input, cached input, output, cache write, long-prompt threshold in tokens, long input, long cached input,
#       long output, source)
PRICES = {
    "claude-opus-5.5": (4.00, 0.20, 20.00, 5.00, None, None, None, None, "anthropic"),
    "claude-sonnet-5.5": (2.00, 0.10, 10.00, 2.50, None, None, None, None, "anthropic"),
    "claude-haiku-5.5": (0.10, 0.01, 0.50, 0.125, 100000, 0.50, 0.05, 2.50, "anthropic"),
    "claude-fable-5.2": (8.00, 0.20, 40.00, 10.00, None, None, None, None, "anthropic"),
    "gemini-4-argon": (2.00, 0.10, 10.00, None, None, None, None, None, "google"),
    "gemini-4-carbon": (2.00, 0.10, 10.00, None, None, None, None, None, "google"),
    "gemini-4-hydrogen": (1.50, 0.075, 7.50, None, None, None, None, None, "google"),
    "gemini-3.8-flash": (0.75, 0.075, 3.75, None, None, None, None, None, "google"),
    "gpt-6.2-astra": (8.00, 0.40, 40.00, None, 272000, 16.00, 0.80, 60.00, "openai"),
    "gpt-6.2-sol": (2.00, 0.10, 10.00, None, 272000, 4.00, 0.20, 15.00, "openai"),
    "gpt-6-astra": (10.00, 1.00, 50.00, None, 272000, 20.00, 2.00, 75.00, "openai"),
    "gpt-6.1-sol": (2.00, 0.10, 10.00, None, 272000, 4.00, 0.20, 15.00, "openai"),
    "gpt-6-luna": (0.10, 0.01, 0.50, None, 272000, 0.20, 0.02, 0.75, "openai"),
    # read 2026-11-18, standard tier, short context
    "gpt-6.1-terra": (1.00, 0.05, 5.00, None, None, None, None, None, "openai"),
    "grok-4.7": (2.00, 0.50, 6.00, None, 200000, 4.00, 1.00, 12.00, "xai"),
}
# A model name maps to a table key when it holds the word on the left (the first match wins).
NAMES = [
    ("claude-opus-5-5", "claude-opus-5.5"), ("claude-opus-5.5", "claude-opus-5.5"),
    ("claude-sonnet-5-5", "claude-sonnet-5.5"), ("claude-sonnet-5.5", "claude-sonnet-5.5"),
    ("claude-haiku-5-5", "claude-haiku-5.5"), ("claude-haiku-5.5", "claude-haiku-5.5"),
    ("claude-fable-5-2", "claude-fable-5.2"), ("claude-fable-5.2", "claude-fable-5.2"),
    ("gemini-4-argon", "gemini-4-argon"), ("gemini-4-carbon", "gemini-4-carbon"),
    ("gemini-4-hydrogen", "gemini-4-hydrogen"), ("gemini-3.8-flash", "gemini-3.8-flash"),
    ("gpt-6.2-astra", "gpt-6.2-astra"), ("gpt-6.2-sol", "gpt-6.2-sol"), ("gpt-6-astra", "gpt-6-astra"),
    ("gpt-6.1-sol", "gpt-6.1-sol"), ("gpt-6.1-terra", "gpt-6.1-terra"), ("gpt-6-luna", "gpt-6-luna"),
    ("grok-4.7", "grok-4.7"),
]


def key_of(model):
    """The table key for a model name, or None when the table has no row for it."""
    for word, key in NAMES:
        if word in str(model):
            return key
    return None


def tokens(usage):
    """(prompt, cached, cache write, completion with reasoning, reasoning) from one reply's usage record."""
    p = usage.get("prompt_tokens", usage.get("input_tokens"))
    c = usage.get("completion_tokens", usage.get("output_tokens"))
    d = usage.get("prompt_tokens_details") or {}
    cached = d.get("cached_tokens") if isinstance(d, dict) else None
    if not isinstance(cached, int):
        cached = usage.get("cache_read_input_tokens") or 0
    write = usage.get("cache_creation_input_tokens") or 0
    od = usage.get("completion_tokens_details") or {}
    think = od.get("reasoning_tokens") if isinstance(od, dict) else None
    total = usage.get("total_tokens")
    extra_think = 0
    if isinstance(think, int) and think > 0 and isinstance(p, int) and isinstance(c, int) \
            and isinstance(total, int) and total >= p + c + think:
        extra_think = think   # some endpoints count reasoning apart from completion
    p, c = (p or 0), (c or 0) + extra_think
    cached = min(max(0, cached), p)
    write = min(max(0, write), p - cached)
    return p, cached, write, c, (think or 0)


def cost_of(usages, key):
    """USD at list price for a list of usage records, or None when the table has no row for the key."""
    row = PRICES.get(key)
    if row is None:
        return None
    inp, cin, out, cw, thr, linp, lcin, lout, _ = row
    total = 0.0
    for u in usages:
        p, cached, write, c, _ = tokens(u)
        i, ci, o = (linp, lcin, lout) if thr is not None and p > thr else (inp, cin, out)
        w = cw if cw is not None else i
        total += ((p - cached - write) * i + cached * ci + write * w + c * o) / 1e6
    return round(total, 6)


def usages_of(operator_log):
    """Every reply's usage record from an operator log (JSON lines)."""
    import json
    out = []
    try:
        with open(operator_log, encoding="utf-8") as f:
            for line in f:
                try:
                    rec = json.loads(line)
                except ValueError:
                    continue
                if rec.get("kind") == "reply" and isinstance(rec.get("usage"), dict):
                    out.append(rec["usage"])
    except OSError:
        pass
    return out
