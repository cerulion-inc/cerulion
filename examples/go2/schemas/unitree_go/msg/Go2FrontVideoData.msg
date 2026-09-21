# unitree_go/Go2FrontVideoData — the Go2 front-camera H.264 stream, as the
# ROBOT ACTUALLY PUBLISHES IT.
#
# ── WHY THIS FILE EXISTS ────────────────────────────────────────────────────
# This is a DELIBERATE workspace-store override of the `unitree_go` ament
# package's own Go2FrontVideoData.msg. The ament copy (the community
# `unitree_ros2` reconstruction, also vendored into autonomy_stack_go2) says:
#
#     uint64 time_frame
#     uint8[] video720p
#     uint8[] video360p
#     uint8[] video180p
#
# The Go2 firmware does NOT publish that layout. The writer is a bare DDS app
# (`ros2 topic info -v /frontvideostream` reports node name
# `_CREATED_BY_BARE_DDS_APP_`), so it never had to agree with the ROS package,
# and it doesn't. Decoding a real sample against the 4-field definition reads
# `video720p`'s length prefix off the field that actually holds the frame
# HEIGHT (360 or 720 — values that look like plausible lengths), consumes that
# many bytes of H.264 as if it were `video720p`'s payload, and then reads
# `video360p`'s length prefix from the middle of the H.264 stream. That
# garbage length is what produced the live failure:
#
#     CDR body for 'unitree_go/Go2FrontVideoData' field 'video360p' has a
#     hostile length/count prefix 2170829756 over only 1760 remaining bytes
#
# The byte budget was doing its job: the length really was nonsense. The
# definition was wrong. Because the store is consulted BEFORE any acquisition
# rung, this file also stops `cerulion ros2 attach` from
# re-harvesting the wrong ament copy for this type.
#
# ── HOW THE LAYOUT BELOW WAS ESTABLISHED ────────────────────────────────────
# 60 consecutive raw samples were captured off the live robot (an rclpy
# `raw=True` subscription — serialized bytes, no ROS deserialization) and
# checked against this layout. All 60 satisfy, EXACTLY:
#
#     body_len == 16 + video_data_len + cdr_trailing_pad
#
# and in all 60 the bytes at body offset 16 are the H.264 Annex-B start code
# `00 00 00 01`. Three of those captures are committed as decode fixtures
# under nodes/dds_bridge/tests/fixtures/ and are pinned by
# nodes/dds_bridge/tests/frontvideostream_wire_test.rs, which additionally
# proves the round-trip `encode(decode(body)) == body` — i.e. this layout
# accounts for every byte, rather than merely happening to parse.
#
# `video_height` is NOT a guess: across the 60 samples it took only the values
# 360 and 720, one of each per `time_frame` (the camera publishes both
# renditions), and the H.264 SPS carried in the IDR samples decodes to
# 640x360 for the 360 samples and 1280x720 for the 720 samples. The field is
# the rendition's frame height.
#
# NOT verified: the vendor's own field NAMES. `time_frame` is upstream's;
# `video_height` and `video_data` are ours, chosen to describe what the bytes
# provably are. The Go2 runs ROS 2 Foxy, which has no
# `~/get_type_description` service, and the publisher is a bare DDS app with
# no type-description endpoint, so there is no authoritative name source to
# consult. The LAYOUT is evidence; the NAMES are description.
#
# ── FIELDS ──────────────────────────────────────────────────────────────────
# time_frame   monotonic frame stamp from the camera; the 360 and 720
#              renditions of one capture share a value.
# video_height 360 or 720 — the rendition's frame height, matching the H.264
#              SPS (640x360 / 1280x720). Use it to tell the two renditions
#              apart on one topic.
# video_data   ONE H.264 Annex-B access unit (start-code delimited, `00 00 00
#              01`). An IDR access unit carries SPS + PPS + IDR; every other
#              access unit is a single non-IDR slice. This is exactly what
#              `h264parse ! nvv4l2decoder` wants, and exactly what a
#              desk-side H.264 decoder wants — no depayloading, no
#              re-containerizing.
uint64 time_frame
uint32 video_height
uint8[] video_data
