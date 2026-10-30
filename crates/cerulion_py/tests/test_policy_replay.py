"""The policy replay helper shipped in examples/policy_replay/.

Records here are built in the test (synthetic frames on made-up topics, or typed sensor_msgs messages encoded in the test) or come from
the fixture's oracle bag.
"""

import struct
import sys
from pathlib import Path

import numpy as np
import pytest

import cerulion

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples" / "policy_replay"))
from policy_replay import RecordedSteps, assert_replay_matches, recorded_steps  # noqa: E402

from test_bag import HASH_A, HASH_B, write_bag  # noqa: E402


def synthetic_record(sequence, values, dtype=np.float32):
    payload = np.asarray(values, dtype=dtype).tobytes()
    header = struct.pack(
        "<QIIIIQQ", 1, cerulion.WIRE_HEADER_SIZE + len(payload), 0, 0, sequence, 1_000 + sequence, 0
    )
    return cerulion.Record(header + payload)


def decode(record):
    return np.frombuffer(record.payload, dtype=np.float32).copy()


def synthetic_log():
    """cam and state interleaved with actions; the first action precedes any state."""
    return [
        ("/synthetic/cam", synthetic_record(0, [0.0])),
        ("/synthetic/action", synthetic_record(0, [9.0])),
        ("/synthetic/state", synthetic_record(0, [1.0])),
        ("/synthetic/action", synthetic_record(1, [1.0])),
        ("/synthetic/cam", synthetic_record(1, [2.0])),
        ("/synthetic/cam", synthetic_record(2, [3.0])),
        ("/synthetic/other", synthetic_record(0, [99.0])),
        ("/synthetic/action", synthetic_record(2, [4.0])),
    ]


def steps():
    return RecordedSteps(synthetic_log(), ["/synthetic/cam", "/synthetic/state"], "/synthetic/action")


def summed(step):
    return decode(step.observation["/synthetic/cam"]) + decode(step.observation["/synthetic/state"])


def test_steps_pair_each_action_with_the_newest_observations():
    it = steps()
    paired = list(it)
    assert it.skipped == 1
    assert [s.index for s in paired] == [0, 1]
    assert [s.action.sequence for s in paired] == [1, 2]
    assert paired[0].observation["/synthetic/cam"].sequence == 0
    assert paired[1].observation["/synthetic/cam"].sequence == 2
    assert paired[1].observation["/synthetic/state"].sequence == 0
    assert set(paired[1].observation) == {"/synthetic/cam", "/synthetic/state"}


def test_topic_arguments_are_checked():
    with pytest.raises(ValueError, match="empty"):
        RecordedSteps([], [], "/a")
    with pytest.raises(ValueError, match="both an observation topic and the action topic"):
        RecordedSteps([], ["/a", "/b"], "/a")
    assert RecordedSteps([], "/a", "/b").observation_topics == ["/a"]


def test_exact_replay_passes_and_reports_counts():
    result = assert_replay_matches(steps(), summed, decode)
    assert (result.steps, result.skipped, result.max_abs_error) == (2, 1, 0.0)


def test_exact_replay_fails_on_one_ulp():
    def nudged(step):
        out = summed(step)
        if step.index == 1:
            out = np.nextafter(out, np.float32(np.inf))
        return out

    with pytest.raises(AssertionError) as failure:
        assert_replay_matches(steps(), nudged, decode)
    message = str(failure.value)
    assert message.startswith("1 of 2 replayed actions differ from the recording (byte-exact):")
    assert "step 1 (action sequence 2, timestamp_ns 1002)" in message


def test_exact_replay_fails_on_dtype_and_shape():
    with pytest.raises(AssertionError, match="dtype float64 != recorded float32"):
        assert_replay_matches(steps(), lambda s: summed(s).astype(np.float64), decode)
    with pytest.raises(AssertionError, match=r"shape \(2,\) != recorded \(1,\)"):
        assert_replay_matches(steps(), lambda s: np.repeat(summed(s), 2), decode)


def test_tolerance_bounds_the_error():
    def off(step):
        return summed(step) + np.float32(1e-3)

    result = assert_replay_matches(steps(), off, decode, atol=2e-3)
    assert result.max_abs_error == pytest.approx(1e-3, rel=1e-3)
    with pytest.raises(AssertionError, match="atol=0.0005 rtol=0"):
        assert_replay_matches(steps(), off, decode, atol=5e-4)
    result = assert_replay_matches(steps(), off, decode, rtol=2e-3)
    assert result.steps == 2


def test_nan_is_a_mismatch_under_a_tolerance():
    with pytest.raises(AssertionError, match="2 of 2"):
        assert_replay_matches(steps(), lambda s: np.array([np.nan], np.float32), decode, atol=1.0)


def test_too_few_steps_fails():
    with pytest.raises(AssertionError, match="replayed 0 steps, fewer than min_steps=1 .1 action records"):
        assert_replay_matches(
            RecordedSteps(synthetic_log()[:2], ["/synthetic/cam", "/synthetic/state"], "/synthetic/action"),
            summed,
            decode,
        )


def test_replay_reads_a_bag(fixture_bin, tmp_path):
    """The oracle bag logs a0 b0 a1 a2 b1 a3 b2 a4: b0, b1, b2 pair with a0, a2, a3."""
    path = tmp_path / "bag.mcap"
    write_bag(fixture_bin, path)
    seen = []

    def oracle_policy(step):
        record = step.observation["/py_bag/a"]
        assert record.schema_hash == HASH_A
        seen.append(record.sequence)
        return np.frombuffer(step.action.payload, dtype=np.uint8)

    def raw_action(record):
        assert record.schema_hash == HASH_B
        return np.frombuffer(record.payload, dtype=np.uint8)

    with cerulion.open_bag(path) as bag:
        result = assert_replay_matches(recorded_steps(bag, "/py_bag/a", "/py_bag/b"), oracle_policy, raw_action)
    assert (result.steps, result.skipped) == (3, 0)
    assert seen == [0, 2, 3]

    with cerulion.open_bag(path) as bag:
        with pytest.raises(ValueError):
            list(recorded_steps(bag, "/py_bag/missing", "/py_bag/b"))


def test_recorded_steps_takes_any_iterable_of_topics(fixture_bin, tmp_path):
    path = tmp_path / "bag.mcap"
    write_bag(fixture_bin, path)
    with cerulion.open_bag(path) as bag:
        steps = recorded_steps(bag, (t for t in ["/py_bag/a"]), "/py_bag/b")
        assert steps.observation_topics == ["/py_bag/a"]
        assert len(list(steps)) == 3


def test_a_topic_named_twice_is_refused():
    with pytest.raises(ValueError, match="names a topic twice"):
        RecordedSteps([], ["/a", "/a"], "/b")


def test_equal_infinities_match_under_a_tolerance():
    inf = np.array([np.inf, -np.inf], np.float32)
    log = [("/o", synthetic_record(0, [0.0])), ("/act", synthetic_record(0, inf))]
    result = assert_replay_matches(RecordedSteps(log, "/o", "/act"), lambda s: inf.copy(), decode, atol=1e-3)
    assert result.steps == 1


def test_a_tolerance_accepts_another_dtype():
    result = assert_replay_matches(steps(), lambda s: summed(s).astype(np.float64), decode, atol=1e-6)
    assert result.steps == 2


def test_the_report_names_at_most_max_report_steps():
    log = [("/o", synthetic_record(0, [0.0]))]
    log += [("/act", synthetic_record(i, [1.0])) for i in range(7)]
    with pytest.raises(AssertionError) as failure:
        assert_replay_matches(RecordedSteps(log, "/o", "/act"), lambda s: np.float32([2.0]), decode, max_report=3)
    lines = str(failure.value).splitlines()
    assert lines[0].startswith("7 of 7 replayed actions differ")
    assert [line.split(" (")[0].strip() for line in lines[1:]] == ["step 0", "step 1", "step 2"]


def load_template():
    import importlib.util

    path = Path(__file__).resolve().parents[1] / "examples" / "policy_replay" / "test_policy_replay.py"
    spec = importlib.util.spec_from_file_location("policy_replay_template", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


HEADER = {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""}
IMAGE_DEFAULTS = {"header": HEADER, "is_bigendian": 0}


def encoded_record(message_class, values):
    from cerulion._typed import _encode_message, _schema_arg

    schemas, name = _schema_arg(message_class, None, "encoded_record")
    return cerulion.Record(bytes(_encode_message(schemas, name, values, 0)))


def test_template_decoders_read_recorded_messages():
    template = load_template()
    from cerulion.msgs.sensor_msgs import Image, JointState

    height, width, pad = 2, 3, 4
    step = width * 3 + pad
    rows = np.arange(height * step, dtype=np.uint8).reshape(height, step)
    image = encoded_record(
        Image,
        {**IMAGE_DEFAULTS, "height": height, "width": width, "step": step, "encoding": "rgb8", "data": rows.reshape(-1)},
    )
    pixels = template.image_array(image)
    assert pixels.shape == (height, width, 3)
    assert pixels.flags.owndata and pixels.flags.writeable
    np.testing.assert_array_equal(pixels.reshape(height, width * 3), rows[:, : width * 3])

    mono = encoded_record(
        Image,
        {**IMAGE_DEFAULTS, "height": 1, "width": 1, "step": 1, "encoding": "mono8", "data": np.zeros(1, np.uint8)},
    )
    with pytest.raises(AssertionError, match="decodes rgb8, the bag has mono8"):
        template.image_array(mono)

    action = encoded_record(
        JointState, {"header": HEADER, "name": b"", "position": [0.5, -1.25], "velocity": [], "effort": []}
    )
    decoded = template.recorded_action(action)
    assert decoded.dtype == np.float64
    np.testing.assert_array_equal(decoded, [0.5, -1.25])


def test_an_infinity_matches_only_an_equal_infinity():
    inf = np.array([np.inf, np.inf], np.float32)
    log = [("/o", synthetic_record(0, [0.0])), ("/act", synthetic_record(0, inf))]
    with pytest.raises(AssertionError, match=r"(?s)1 of 1 .*max abs error inf"):
        assert_replay_matches(
            RecordedSteps(log, "/o", "/act"), lambda s: np.float32([0.0, -np.inf]), decode, rtol=1e-3
        )


def test_max_abs_error_ignores_matched_infinities():
    recorded = np.array([np.inf, 1.0], np.float32)
    log = [("/o", synthetic_record(0, [0.0])), ("/act", synthetic_record(0, recorded))]
    result = assert_replay_matches(
        RecordedSteps(log, "/o", "/act"), lambda s: np.float32([np.inf, 1.5]), decode, atol=1.0
    )
    assert result.max_abs_error == 0.5


def test_rtol_is_relative_to_the_recorded_action():
    # recorded 1.0, replayed 2.0: rtol 0.6 of the recorded value bounds the
    # error at 0.6 (a mismatch); the same rtol of the replayed value would be 1.2.
    log = [("/o", synthetic_record(0, [0.0])), ("/act", synthetic_record(0, [1.0]))]
    with pytest.raises(AssertionError, match="1 of 1"):
        assert_replay_matches(RecordedSteps(log, "/o", "/act"), lambda s: np.float32([2.0]), decode, rtol=0.6)


def test_a_mismatch_report_ignores_matched_infinities():
    recorded = np.array([np.inf, 1.0], np.float32)
    log = [("/o", synthetic_record(0, [0.0])), ("/act", synthetic_record(0, recorded))]
    with pytest.raises(AssertionError, match="max abs error 2$"):
        assert_replay_matches(
            RecordedSteps(log, "/o", "/act"), lambda s: np.float32([np.inf, 3.0]), decode, atol=1.0
        )


def test_nan_on_both_sides_is_a_mismatch_under_a_tolerance_only():
    nan = np.array([np.nan], np.float32)
    log = [("/o", synthetic_record(0, [0.0])), ("/act", synthetic_record(0, nan))]
    with pytest.raises(AssertionError, match="1 of 1"):
        assert_replay_matches(RecordedSteps(log, "/o", "/act"), lambda s: nan.copy(), decode, atol=1.0)
    result = assert_replay_matches(RecordedSteps(log, "/o", "/act"), lambda s: nan.copy(), decode)
    assert (result.steps, result.max_abs_error) == (1, 0.0)


def test_an_exact_mismatch_of_zero_magnitude_says_the_bytes_differ():
    log = [("/o", synthetic_record(0, [0.0])), ("/act", synthetic_record(0, [0.0]))]
    with pytest.raises(AssertionError, match=r"bytes differ \(max abs error 0\)"):
        assert_replay_matches(RecordedSteps(log, "/o", "/act"), lambda s: np.float32([-0.0]), decode)
