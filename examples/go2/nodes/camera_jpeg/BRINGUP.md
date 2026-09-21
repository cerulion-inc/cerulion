<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# Go2 camera JPEG node, bring-up

Bring-up notes for running the `camera_jpeg` node against the real Go2 front
camera on the companion computer (the EDU's Jetson, or any Linux machine on
the robot LAN).

## What the node does

`#[cerulion_node]` **data-triggered transcoder**: it consumes
`unitree_go/Go2FrontVideoData` on `/go2/camera/h264` (the `dds_bridge`'s raw
route for the robot's `/frontvideostream` DDS topic), feeds each H.264 access
unit to a GStreamer `appsrc`, and republishes the decoded frame as
`sensor_msgs/CompressedImage` (`format = "jpeg"`) on `/go2/camera/jpeg`. The
decode/encode chain is probed at start:

| Host | Pipeline |
|-----|----------|
| Jetson (nv elements present) | `appsrc ! h264parse ! nvv4l2decoder ! nvjpegenc ! appsink` |
| Software fallback | `appsrc ! h264parse ! avdec_h264 ! videoconvert ! jpegenc ! appsink` |

The choice is logged at `INFO` (`camera transcode pipeline selected`).

### The camera is a TOPIC, not a socket

The node takes its video from a Cerulion topic, not from a `udpsrc` joined to
the vendor's `230.1.1.1:1720` H.264 multicast group, for two reasons:

1. **The multicast route is fragile.** On a companion with WiFi,
   `ip route get 230.1.1.1` can resolve to the WiFi interface rather than the
   robot LAN, so it needs a per-machine route or interface fix before a
   single frame can arrive.
2. **The same video is already a plain DDS topic.** `/frontvideostream` carries
   exactly the bytes a decoder wants, and "the camera is a topic" is what
   generalizes to an unfamiliar robot, no group, no port, no interface,
   and it rides the transport Cerulion records and replays.

So there is **no networking setup for the camera**. Nothing to route,
no udev rule (the stream is not a `/dev/video*` device), and the
raw-vs-RTP question is closed by construction: the bridge hands the node
elementary-stream Annex-B access units, one per sample.

## The two-rendition trap (read this before debugging a garbled picture)

`/frontvideostream` carries **two independent H.264 renditions interleaved on
one topic**, 640x360 and 1280x720, one of each per `time_frame`
(SPS-confirmed; see `../../schemas/unitree_go/msg/Go2FrontVideoData.msg`).
They are separate encodes with separate parameter sets, so feeding both to one
decoder is not "two streams at once", it is one corrupt stream.

The node decodes exactly one, chosen by `video_height`. The default is 720; set
`CAMERA_TARGET_HEIGHT` to pick the other:

```bash
# default (no env var): the 1280x720 rendition
DDS_BRIDGE_CONFIG=graphs/go2.bridge.yaml cerulion graph run bridge --release --single-process

# the low-bandwidth 640x360 twin
CAMERA_TARGET_HEIGHT=360 DDS_BRIDGE_CONFIG=graphs/go2.bridge.yaml \
  cerulion graph run bridge --release --single-process
```

`CAMERA_TARGET_HEIGHT=0` (or an unparseable value) falls back to the default
with a warn; any other height outside {360, 720} is honoured as asked but warns
at launch, because nothing this robot has ever published matches it and the
alternative is a black screen with a climbing skip counter.

The sibling rendition is dropped silently (it is half of every frame pair, so a
log line per frame would be a flood) but COUNTED, and the count is printed in
the node's teardown summary along with every other skip class.

## Waiting for a keyframe

A decoder cannot start mid-GOP, so the node pushes **nothing** until an access
unit carrying an SPS (NAL type 7) arrives. The Go2 sends SPS+PPS with every
IDR, so this clears itself within one keyframe interval; while it is waiting
you get ONE `warn!` ("camera is WAITING FOR A KEYFRAME"), repeats demoted to
`debug!`. If that warn never clears, the stream really is not carrying
parameter sets, check the bridge, not the decoder.

The gate re-closes on a pipeline rebuild (a fresh decoder has no parameter
sets), so a recovered pipeline waits for the next keyframe rather than being
fed a mid-GOP slice.

## Prerequisites on the companion computer

```bash
# Headers (to BUILD with capture), camera_jpeg's `gstreamer` feature is
# NON-default, and `cerulion node build` probes for these:
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

# Elements (to RUN): h264parse (-bad), jpegenc (-good), avdec_h264 (libav,
# software arm only), x264enc (-ugly, the loopback test's generator).
# nvv4l2decoder / nvjpegenc ship with L4T.
sudo apt install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                 gstreamer1.0-plugins-ugly gstreamer1.0-libav

cerulion node build camera_jpeg --release   # probes, then adds --features gstreamer
```

A build WITHOUT the feature has no decoder, and the node's `init()` fails the
graph build naming itself, loud, at launch. It never runs inert.

> **Artifact-overwrite hazard.** A plain `cargo build --release` rebuilds
> `camera_jpeg` WITHOUT capture and overwrites the same
> `target/release/libcamera_jpeg.*` the `node build` line produced. Re-run
> `cerulion node build camera_jpeg --release` after any workspace rebuild. See
> the crate's `Cargo.toml` for the full note.

## Verifying the stream without the node

The camera is a Cerulion topic, so the check is a topic check, not a
`gst-launch`:

```bash
cerulion topic hz /go2/camera/h264      # the bridge's raw route, expect ~2x the
                                        # frame rate (both renditions)
cerulion topic hz /go2/camera/jpeg      # the node's output, expect 1x
```

If `/go2/camera/h264` is silent, the problem is upstream of this node entirely:
check `DDS_BRIDGE_CONFIG`, the `/frontvideostream` mapping in
`graphs/go2.bridge.yaml`, and `only_networks`/`GO2_IFACE`.

## Measure latency and throughput

The node stamps `header.stamp` from the **node clock** at publish, so that a
recording replays to the same stamps, and carries the gst buffer **PTS** on the
internal `JpegFrame` (diagnostic only). The node clock is a replay clock, not
an elapsed-work timer.

> **Clock-domain caveat (do NOT subtract PTS from node time):** the buffer PTS
> lives in the GStreamer **pipeline clock** domain; `self.now_ns()` is the
> Cerulion **node clock** with its own epoch. `now_ns - pts_ns` is NOT a
> latency. The wire `time_frame` is a THIRD epoch (the camera's own counter)
> and is likewise not subtractable, which is why the pipeline uses
> `appsrc do-timestamp=true` rather than deriving a PTS from it.

Measure the legs with same-clock comparisons only:

1. **In-pipeline (decode + JPEG encode):** GStreamer's own latency tracer on a
   standalone harness of the SAME element chain, fed by the loopback generator
   rather than the live topic:
   ```bash
   GST_TRACERS=latency GST_DEBUG=GST_TRACER:7 gst-launch-1.0 \
     videotestsrc num-buffers=300 ! video/x-raw,format=I420,width=1280,height=720,framerate=30/1 ! \
     x264enc tune=zerolatency key-int-max=30 ! video/x-h264,profile=constrained-baseline ! \
     h264parse ! nvv4l2decoder ! nvjpegenc ! fakesink 2>&1 | grep latency
   ```
   This isolates the NVDEC + JPEG-encode cost with no Cerulion code involved.
2. **Node-side (access unit in → JPEG out):** the codec is asynchronous and
   decoding can outlive the tick that fed it, so the JPEG a tick emits is
   generally not the one that tick pushed in, and the node clock is a replay
   clock rather than an elapsed-work timer. To get a number, tag each input
   access unit, match it to the output frame it produced, and read a
   **monotonic** clock at both points; or take the figure from GStreamer's
   latency tracer in step 1, which already correlates buffers.
3. **Bridge → publish (on the companion):** this measures **throughput**, not
   latency: `cerulion topic hz` on both `/go2/camera/h264` and
   `/go2/camera/jpeg` reports the interarrival rate of each topic, and two
   rates say nothing about how long any one frame took. A latency here needs
   the same correlation and single clock as step 2.
4. **End-to-end (capture → desk viewer):** a CROSS-machine comparison, so
   either sync the clocks first (NTP/chrony against the same source; note
   residual skew in the report) or report jitter + throughput instead of
   absolute latency.

## Frame loss, and what overload actually looks like

Everything the NODE discards is counted and printed in the teardown summary:
the sibling rendition, pre-keyframe access units, unknown heights, empty
payloads, access units that ARRIVED while the pipeline was down
(`aus_arrived_while_pipeline_down`, counted before the rendition filter, so on
this two-rendition topic it is roughly **twice** the decodable loss), access
units a dying pipeline refused (`aus_admitted` minus `aus_pushed`), stale
JPEGs, and JPEGs above the output's `max_slice_len` **ceiling** (dropped, never
truncated). A frame merely bigger than the publisher's current adaptive loan is
**not** a loss, it spills and publishes whole.

Two loss channels are **not** the node's to count, so look for them elsewhere:

1. **Input eviction.** The `h264` input is a normal `drop_oldest` Cerulion
   input, so when the node falls behind the runtime evicts the OLDEST access
   units before the node sees them. The runtime counts them on the input's
   `drop_oldest` counter, but be aware there is **no CLI that renders it**:
   `NodeHandle::backpressure_drop_oldest_count("h264")` is an in-process
   API, so on the robot the reachable evidence is the teardown summary plus a
   `topic hz` divergence (`/go2/camera/h264` at ~2x the frame rate while
   `/go2/camera/jpeg` sags below 1x), not a number you can print.
2. **`appsink max-buffers=2 drop=true`.** GStreamer keeps at most two queued
   JPEGs, drops older ones internally and reports nothing, so those drops are
   not in the node's counter. What it drops is by construction stale: the node
   applies the same latest-wins rule one layer out, and counts it there.

**The overload signature.** This node does not degrade into "a few dropped
frames". Queue eviction can force the decoder to wait for a keyframe, and the
failure has a recognisable shape:

> input queue overflows → the runtime evicts old access units → the decoder
> sees a mid-GOP **gap** → it errors → the pipeline is torn down and rebuilt
> after 1 s → the rebuilt decoder has no parameter sets, so the keyframe gate
> re-closes → nothing decodes until the next IDR.

So one overflow costs about `1 s + one keyframe interval` of video, and under
sustained load it repeats. In the teardown summary that reads as
`pipeline_builds` > 1 plus a climbing `skipped_waiting_for_keyframe`; in the
log, repeated `camera transcode pipeline FATAL` errors and
`WAITING FOR A KEYFRAME` warns. Cross-check the two `topic hz` rates (above) to
confirm eviction is the trigger rather than a genuinely corrupt stream.

There is one more permanent-stall shape worth recognising, unrelated to load:
automatic end-of-stream recovery is not implemented, so if the pipeline reaches
**end-of-stream** it accepts pushes forever and never yields another JPEG.
Restart the graph. The signature is exactly one `reached end-of-stream` warn
plus a teardown summary where `aus_pushed` kept climbing while `published`
stopped. An `appsrc` fed by a live topic has no legitimate EOS, so this shape
is not expected; the reasoning is recorded in `capture.rs`.

The input depth is deliberately left at the default rather than sized to a GOP:
the live keyframe interval is unmeasured, and an explicit depth makes the
service-create order on `/go2/camera/h264` load-bearing (at the default either
order is safe). Measure both before choosing a number, see the node's module docs.

## Verifying on the robot (and why `replay` is not the check)

```bash
cerulion topic hz /go2/camera/h264      # ~2x the frame rate (both renditions)
cerulion topic hz /go2/camera/jpeg      # 1x, the decoded rendition
# then Ctrl-C the graph and read the `camera_jpeg teardown summary` line:
# published / aus_admitted / aus_pushed / every skip class / pipeline_builds.
```

> **Byte-exact verification does not cover this node's payload.** `bag play
> --resim all --verify` re-executes the current cdylibs and **byte-diffs** the
> frames they produce against the recording, and this node's payload is
> whatever a GStreamer decode + JPEG encode emitted at that moment, from a
> pipeline that is not in the bag, on an encoder (`nvjpegenc` vs `jpegenc`, and
> either one's version) that nothing records. JPEG output is not guaranteed to
> reproduce across encoders or versions, so a replay of a bag containing
> `/go2/camera/jpeg` may report a byte mismatch on that topic.
>
> Check stream rates and the error and drop counters on hardware, and compare
> decoded content when you are evaluating image changes. The crate's own tests
> cover the fire schedule, `header.stamp` (a pure function of the scheduler
> clock), the constant `frame_id`/`format`, and the admission, metadata and
> lifecycle decisions in `h264` / `capture`. End-to-end reproducibility of the
> encoded output is a separate question, and is not covered by those tests.

## Running the loopback e2e (no robot needed)

On any machine with GStreamer + the H.264/JPEG plugins:

```bash
cargo test -p camera_jpeg --features gstreamer --test loopback_e2e_test -- --nocapture
```

`--features gstreamer` is REQUIRED: the capture feature is non-default
(so the rest of `examples/go2` builds on a machine with no GStreamer), and the rig is
compiled only under it. Omit the flag and the binary FAILS naming the correct
command, it never reports the vacuous `0 passed` an empty test binary would.

It runs `videotestsrc ! x264enc ! video/x-h264,profile=constrained-baseline !
h264parse config-interval=-1 ! appsink` (the NVDEC-compatible test stream:
constrained-baseline profile + in-band SPS/PPS), feeds those access units
through the PRODUCTION `TranscodeLoop`, and asserts valid 1280x720 JPEGs come
out. A second arm labels the same access units with the sibling height and
asserts NOTHING is decoded, the two-rendition pin over a real encoder. A
missing GStreamer / element fails LOUDLY with the missing names (never a silent
pass); a mid-stream pipeline death surfaces as an attributable bus-error panic
(the pipeline is bus-monitored).

The GStreamer-free half of the suite, the same targets the
`demos-go2` CI job runs on every PR:

```bash
cargo test -p camera_jpeg --no-default-features --features cdylib --lib --test node_publish_test
```

Note what that covers: the pure JPEG reader, the admission gate, the transcode
state machine, the **pipeline-description string oracles** (`pipeline_desc` is
deliberately NOT feature-gated, so the strings that decide whether the robot
decodes anything are pinned without GStreamer), and the node's
trigger→gate→publish path over real iceoryx2 with a scripted decoder. Only the
gst RUNTIME, `loopback_e2e_test`, needs a machine with the plugins installed.

## Validate your hardware

These are not established for every robot and every companion computer, so
check them on yours:

- The Jetson pipeline is not validated. Confirm that `nvv4l2decoder !
  nvjpegenc` links directly, or whether an `nvvidconv` is needed between them.
- Measure the pipeline latency (leg 1) and the two topic rates (leg 3), using
  the methods above.
- Measure the `/go2/camera/jpeg` byte rate over your desk link. Whether the
  desk should decode H.264 itself instead depends on that measurement.
- Confirm the camera TF `frame_id` (`front_camera` here) matches your Go2's TF
  tree.

## Payload ownership

The camera input borrows H.264 bytes from shared memory. GStreamer needs an
owned access unit because decoding can outlive the tick. The appsink then
copies its JPEG into an owned frame, and the node copies that frame into its
SHM output; an adaptive-loan spill may add another output copy. Codec-internal
work is additional. Local subscribers borrow the published SHM frame directly.
This example shows how shared-memory transport connects to an asynchronous video codec.
