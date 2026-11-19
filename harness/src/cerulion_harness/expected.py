# SPDX-License-Identifier: AGPL-3.0-only
"""What one episode took in the measured runs, for the start line of run-cell. Standard library only.

Each row is the grid list's row of summary.csv in the harness results package
(docs/benchmarks/results/7cb9f322c10f6fa7-2026-11-18-harness-libero-pro), copied as that file prints it: the
scored episodes, then the mean per scored episode of model replies, operator seconds (the operator's wall time:
the model's replies and the tool calls they ran) and USD at list price, for the plain setup (the column "without")
and the Cerulion harness setup ("with"). A model is listed once all 100 cells of both its columns finished. Keys
are the price table's (prices.key_of).
"""
from cerulion_harness import prices

MEASURED = {
    "claude-fable-5.2": {"plain": ("100", "11.01", "250.375", "1.5032"), "rcp": ("98", "14.7449", "414.449", "5.5015")},
    "claude-opus-5.5": {"plain": ("92", "59.8913", "902.0826", "8.235"), "rcp": ("98", "15.5102", "278.0592", "2.762")},
    "claude-sonnet-5.5": {"plain": ("100", "56.25", "533.18", "3.5087"), "rcp": ("100", "19.63", "204.883", "1.9055")},
    "gemini-3.8-flash": {"plain": ("100", "55.87", "846.644", "1.6665"), "rcp": ("100", "25.55", "258.659", "0.3479")},
    "gemini-4-argon": {"plain": ("97", "37.1649", "1883.7619", "5.4247"), "rcp": ("100", "16.47", "610.791", "1.3681")},
    "gemini-4-carbon": {"plain": ("90", "16.0444", "1887.3378", "2.2393"),
                        "rcp": ("95", "18.3474", "1092.2895", "1.2855")},
    "gemini-4-hydrogen": {"plain": ("100", "59.9", "1007.01", "7.6335"), "rcp": ("100", "22.61", "296.925", "1.155")},
    "gpt-6-astra": {"plain": ("100", "29.91", "787.911", "4.1827"), "rcp": ("100", "16.35", "289.396", "2.3867")},
    "gpt-6-luna": {"plain": ("100", "67.72", "1733.225", "0.2586"), "rcp": ("100", "17.39", "401.234", "0.0367")},
    "gpt-6.1-sol": {"plain": ("100", "39.94", "1100.091", "1.3528"), "rcp": ("100", "15.38", "290.159", "0.3823")},
    "gpt-6.1-terra": {"plain": ("100", "49.79", "773.932", "1.1043"), "rcp": ("100", "17.61", "220.032", "0.286")},
    "gpt-6.2-astra": {"plain": ("100", "29.81", "636.613", "3.1929"), "rcp": ("100", "20.51", "261.575", "2.453")},
    "gpt-6.2-sol": {"plain": ("100", "51.16", "983.089", "1.89"), "rcp": ("100", "28.76", "340.556", "1.1284")},
    "grok-4.7": {"plain": ("99", "74.5758", "2257.498", "2.5703"), "rcp": ("100", "29.85", "816.231", "1.347")},
}


def expected_line(model, arms, budget, clock_s):
    """The start line's expectation: what one episode of each setup took in the measured runs, and where an
    episode stops whatever the model does."""
    stop = (f"an episode stops at {budget['model_replies']} replies, {budget['control_steps']} control steps or "
            f"{clock_s:g} s")
    rows = MEASURED.get(prices.key_of(model))
    if rows is None:
        return f"expected: no measured run of this model; {stop}"
    parts = [f"{arm} {rows[arm][2]} operator seconds, {rows[arm][1]} replies, {rows[arm][3]} USD at list price "
             f"(mean of {rows[arm][0]} scored episodes)" for arm in arms]
    return f"expected from the measured runs of {prices.key_of(model)}: " + "; ".join(parts) + f"; {stop}"
