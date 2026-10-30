"""Template: replay a recorded bag through a LeRobot policy and assert the actions.

Copy this file and ``policy_replay.py`` into your test directory, then edit the
constants below to name your topics, cameras and checkpoint. Run it with:

    REPLAY_BAG=session.mcap REPLAY_POLICY=<hub repo or local checkpoint> pytest test_policy_replay.py

The bag is one ``cerulion bag record`` of the robot running the policy. This
template assumes camera frames as ``sensor_msgs/Image`` (``rgb8``), the arm
state as ``sensor_msgs/JointState`` positions, and each action published as a
``sensor_msgs/JointState`` whose ``position`` holds the action vector.

Record from the policy's first step. A policy that keeps state between calls
(an action chunk queue, temporal ensembling) gives the recorded actions only
when the replay feeds it the same observations from the same starting state:
start the recorder before the policy acts, and the replay calls ``reset()``
once and then every recorded step in order.

Exact or within tolerance:

* Byte-exact replay holds on CPU with deterministic algorithms on, when the
  replay runs the same weights, library versions, CPU model and ``torch``
  thread count as the recording. Set ``REPLAY_THREADS`` to the
  ``torch.get_num_threads()`` the recording ran with (default 1, for a
  recording made after ``torch.set_num_threads(1)``). Leave ``REPLAY_ATOL``
  and ``REPLAY_RTOL`` unset for that check.
* A GPU or MPS replay, or a recording made on a different device type, gives
  actions within a tolerance, not equal bytes. Set ``REPLAY_ATOL`` (and
  ``REPLAY_RTOL``) to the bound you accept.
* A policy that samples noise when it acts (flow matching, diffusion) repeats
  its actions only if the recording seeded the same generator the same way
  (this template seeds 0 before loading the checkpoint);
  otherwise compare within a tolerance wide enough for the sampling spread.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

import cerulion
from cerulion.msgs.sensor_msgs import Image, JointState
from policy_replay import assert_replay_matches, recorded_steps

CAMERAS = {
    "/camera/top/image": "observation.images.top",
    "/camera/wrist/image": "observation.images.wrist",
}
STATE_TOPIC = "/arm/joint_states"
ACTION_TOPIC = "/arm/command"
TASK = "pick up the cube"

BAG = os.environ.get("REPLAY_BAG")
CHECKPOINT = os.environ.get("REPLAY_POLICY")
DEVICE = os.environ.get("REPLAY_DEVICE", "cpu")
ATOL = float(os.environ.get("REPLAY_ATOL", "0"))
RTOL = float(os.environ.get("REPLAY_RTOL", "0"))
THREADS = int(os.environ.get("REPLAY_THREADS", "1"))

pytestmark = pytest.mark.skipif(
    not (BAG and CHECKPOINT), reason="set REPLAY_BAG and REPLAY_POLICY to run the replay test"
)


def image_array(record):
    """A recorded rgb8 ``sensor_msgs/Image`` as an owned (height, width, 3) uint8 array."""
    image = record.view(Image)
    assert image.encoding == "rgb8", f"this template decodes rgb8, the bag has {image.encoding}"
    rows = np.asarray(image.data, dtype=np.uint8).reshape(image.height, image.step)
    return rows[:, : image.width * 3].reshape(image.height, image.width, 3).copy()


def recorded_action(record):
    return np.asarray(record.view(JointState).position, dtype=np.float64)


@pytest.fixture(scope="module")
def policy():
    # Plain imports: with REPLAY_BAG and REPLAY_POLICY set, a missing torch or
    # LeRobot fails the test instead of skipping it.
    import torch
    import lerobot.processor
    from lerobot.policies.factory import get_policy_class, make_pre_post_processors
    from lerobot.configs.policies import PreTrainedConfig

    if DEVICE == "cpu":
        torch.use_deterministic_algorithms(True)
        torch.set_num_threads(THREADS)
    torch.manual_seed(0)

    config = PreTrainedConfig.from_pretrained(CHECKPOINT)
    model = get_policy_class(config.type).from_pretrained(CHECKPOINT)
    model.config.device = DEVICE
    model.to(DEVICE).eval()
    model.reset()
    preprocess, postprocess = make_pre_post_processors(
        model.config,
        pretrained_path=CHECKPOINT,
        preprocessor_overrides={"device_processor": {"device": DEVICE}},
    )
    # A relative-action processor step anchors each chunk to the state it was
    # predicted from only when bound to the policy's action queue, as LeRobot's
    # own rollout binds it. Releases without that step have no binding to make.
    bind = getattr(lerobot.processor, "bind_relative_anchor", None)
    if bind is not None:
        bind(model, preprocess)

    def act(step):
        observation = {
            key: torch.from_numpy(image_array(step.observation[topic])).permute(2, 0, 1).float().div_(255.0)
            for topic, key in CAMERAS.items()
        }
        state = np.asarray(step.observation[STATE_TOPIC].view(JointState).position, dtype=np.float32)
        observation["observation.state"] = torch.from_numpy(state)
        observation["task"] = TASK
        with torch.inference_mode():
            action = postprocess(model.select_action(preprocess(observation)))
        # This template assumes the robot widened the float32 action into
        # JointState's float64 positions; float32 to float64 is exact, so the
        # bytes stay comparable.
        return action.squeeze(0).cpu().numpy().astype(np.float64)

    return act


def test_replayed_actions_match_the_recording(policy):
    with cerulion.open_bag(BAG) as bag:
        steps = recorded_steps(bag, [*CAMERAS, STATE_TOPIC], ACTION_TOPIC)
        result = assert_replay_matches(steps, policy, recorded_action, atol=ATOL, rtol=RTOL)
    print(f"replayed {result.steps} steps, skipped {result.skipped}, max abs error {result.max_abs_error:.3g}")
