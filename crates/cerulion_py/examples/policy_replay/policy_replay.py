"""Replay a recorded bag through a policy and compare its actions with the recorded ones.

``recorded_steps()`` walks a bag written by ``cerulion bag record`` and pairs each
action record with the newest record on every observation topic logged before it,
in log order. That is the observation the policy acted on when the robot logs a
new observation only after publishing the action for the previous one, and the
recorder logged frames in the order the policy received them. A frame logged
while the policy was still computing pairs a newer observation with that action,
and the replayed action can then differ from the recorded one (for a policy that
keeps state, so can the steps after it).
``assert_replay_matches()`` feeds those observations to a policy in order and
compares each returned action with the recorded one, exactly (the default) or
within a tolerance.

Copy this file next to your tests; it needs only ``cerulion`` and NumPy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, Iterator, Mapping, Sequence

import numpy as np

import cerulion


@dataclass(frozen=True)
class Step:
    """One recorded action and the observation records it was paired with."""

    index: int
    observation: Mapping[str, cerulion.Record]
    action: cerulion.Record


def _checked_topics(observation_topics, action_topic):
    if isinstance(observation_topics, str):
        observation_topics = [observation_topics]
    topics = list(observation_topics)
    if not topics:
        raise ValueError("observation_topics is empty")
    if len(set(topics)) != len(topics):
        raise ValueError(f"observation_topics names a topic twice: {topics}")
    if action_topic in topics:
        raise ValueError(f"{action_topic!r} is both an observation topic and the action topic")
    return topics


class RecordedSteps:
    """Iterator of ``Step`` over ``(topic, Record)`` pairs in log order.

    ``skipped`` counts the action records that yielded no step because some
    observation topic had no record before them (the start of a recording).
    It is final once the iterator is exhausted.
    """

    def __init__(
        self,
        records: Iterable[tuple[str, cerulion.Record]],
        observation_topics: Sequence[str],
        action_topic: str,
    ):
        self.observation_topics = _checked_topics(observation_topics, action_topic)
        self.action_topic = action_topic
        self.skipped = 0
        self._records = iter(records)
        self._latest: dict[str, cerulion.Record] = {}
        self._index = 0

    def __iter__(self) -> Iterator[Step]:
        return self

    def __next__(self) -> Step:
        for topic, record in self._records:
            if topic == self.action_topic:
                if len(self._latest) < len(self.observation_topics):
                    self.skipped += 1
                    continue
                step = Step(self._index, dict(self._latest), record)
                self._index += 1
                return step
            if topic in self.observation_topics:
                self._latest[topic] = record
        raise StopIteration


def recorded_steps(bag, observation_topics: Sequence[str], action_topic: str) -> RecordedSteps:
    """The recorded steps of an open ``cerulion.Bag``.

    Reads only the named topics; an unknown topic raises ``ValueError`` from
    ``Bag.messages()``.
    """
    observation_topics = _checked_topics(observation_topics, action_topic)
    topics = observation_topics + [action_topic]
    return RecordedSteps(bag.messages(topics), observation_topics, action_topic)


@dataclass(frozen=True)
class ReplayResult:
    steps: int
    skipped: int
    max_abs_error: float


def assert_replay_matches(
    steps: RecordedSteps,
    policy: Callable[[Step], np.ndarray],
    recorded_action: Callable[[cerulion.Record], np.ndarray],
    *,
    atol: float = 0.0,
    rtol: float = 0.0,
    max_report: int = 5,
    min_steps: int = 1,
) -> ReplayResult:
    """Run ``policy`` on every step in order and compare with the recorded action.

    ``policy(step)`` returns the action for that step's observation;
    ``recorded_action(record)`` decodes the recorded action record. With
    ``atol == rtol == 0`` (the default) the two must have the same dtype and
    shape and equal bytes. Otherwise they must have the same shape and each
    element must equal the recorded one or satisfy
    ``abs(replayed - recorded) <= atol + rtol * abs(recorded)``; under a
    tolerance a NaN in either array is a mismatch and an infinity matches only
    an equal infinity.

    Raises ``AssertionError`` naming the first ``max_report`` mismatching
    steps (index, sequence, timestamp, and the shape, dtype or largest error)
    and the mismatch count,
    or when fewer than ``min_steps`` steps were replayed.
    """
    exact = atol == 0.0 and rtol == 0.0
    mismatches = []
    count = 0
    worst = 0.0
    for step in steps:
        replayed = np.asarray(policy(step))
        recorded = np.asarray(recorded_action(step.action))
        count += 1
        if replayed.shape != recorded.shape:
            mismatches.append((step, f"shape {replayed.shape} != recorded {recorded.shape}"))
            continue
        if exact and replayed.dtype != recorded.dtype:
            mismatches.append((step, f"dtype {replayed.dtype} != recorded {recorded.dtype}"))
            continue
        with np.errstate(invalid="ignore", over="ignore"):
            error = np.abs(replayed.astype(np.float64) - recorded.astype(np.float64))
            bound = atol + rtol * np.abs(recorded.astype(np.float64))
        equal = replayed == recorded
        if exact:
            equal |= np.isnan(replayed) & np.isnan(recorded)
        error = np.where(equal, 0.0, error)
        largest = float(np.max(error)) if error.size else 0.0
        finite_error = error[np.isfinite(error)]
        if finite_error.size:
            worst = max(worst, float(np.max(finite_error)))
        if exact:
            ok = replayed.tobytes() == recorded.tobytes()
        else:
            finite = np.isfinite(replayed) & np.isfinite(recorded)
            ok = bool(np.all(equal | (finite & (error <= bound))))
        if not ok:
            why = "bytes differ (max abs error 0)" if largest == 0 else f"max abs error {largest:.6g}"
            mismatches.append((step, why))
    if mismatches:
        rule = "byte-exact" if exact else f"atol={atol:g} rtol={rtol:g}"
        lines = [f"{len(mismatches)} of {count} replayed actions differ from the recording ({rule}):"]
        for step, why in mismatches[:max_report]:
            lines.append(
                f"  step {step.index} (action sequence {step.action.sequence}, "
                f"timestamp_ns {step.action.timestamp_ns}): {why}"
            )
        raise AssertionError("\n".join(lines))
    if count < min_steps:
        raise AssertionError(
            f"replayed {count} steps, fewer than min_steps={min_steps} "
            f"({steps.skipped} action records came before every observation topic had a record)"
        )
    return ReplayResult(count, steps.skipped, worst)
