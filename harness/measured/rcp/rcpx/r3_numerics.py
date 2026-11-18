# SPDX-License-Identifier: AGPL-3.0-only
"""Round three numerics: the word rules, image numerics and counts that are NOT limits
on speed, force, travel or time. They are the same on every robot because they describe words, pixels and answer
sizes, not motion. rcpx/r3_limits.py serves every name here as its own (from rcpx.r3_numerics import *), so readers
keep writing r3_limits.NAME; the limits themselves are functions of the bounds object there."""

# ---------------------------------------------------------------- 3.1 result envelope
SAID_MAX_WORDS = 45          # said is one sentence a weak model reads whole
NEXT_WHY_MAX_WORDS = 12      # a ready call's why is a reason, not a paragraph
PICTURES_MAX = 1             # one frame per answer (a composite where two frames matter): the harness keeps 6 images
IMAGE_MAX_W, IMAGE_MAX_H = 640, 480   # the harness never resizes an image this size, so pixel arguments stay exact
BRIEF_MAX_BYTES = 900        # a brief result besides pictures; the session's own brief limit stays 2400

# ---------------------------------------------------------------- 3.4 side words
RADIAL_NEAR_MM = 30.0        # nearer the base axis than this a thing has no radial direction of its own: +x
FRONT_MIN_MM = 15.0          # nearer a device centre than this (horizontally) no front is defined
VERTICAL_Z = 0.7             # a direction whose vertical part is over this (45 deg from level) reads up or down
AWAY_DEG = 30.0              # within this of the radial direction: "away from the robot"
TOWARD_DEG = 150.0           # this far or more from it: "toward the robot"

# ---------------------------------------------------------------- 3.5 colour words (OpenCV HSV: H 0 to 180)
V_BLACK = 45                 # darker than this is black whatever the hue (no hue reads in the dark)
S_GREY = 45                  # less saturated than this has no hue: a grey by its value
V_DARK_GREY, V_GREY = 100, 170   # grey bands by value: dark grey, grey, then white
BROWN_H, BROWN_S, BROWN_V = 25, 60, 130   # a dark, saturated orange-red reads brown
RED_H_LO, RED_H_HI = 8, 170  # red wraps round H 0
ORANGE_H, YELLOW_H, GREEN_H, BLUE_H, PURPLE_H = 20, 34, 85, 130, 160   # hue band upper ends
BRIGHT_GREEN_V, BRIGHT_GREEN_S = 180, 120   # a lit green lamp reads bright green
GLOW_REL = 1.4               # a glowing patch is at least this times the frame's median value
GLOW_V, GLOW_S = 150, 90     # ... and at least this bright and this saturated (a coloured light)
SATURATED_V, SATURATED_S = 230, 45   # or a saturated light: so bright the camera reads it nearly white
COMPACT_FILL = 0.5           # a compact region fills at least this share of its bounding box (a disc fills 0.785)
COMPACT_MAX_SHARE = 1.0 / 16.0   # ... and its box covers at most this share of the frame (a lamp, not a lit wall)

# ---------------------------------------------------------------- 3.6 sheet words and do lines
QUOTE_MAX_CHARS = 160        # a do line quotes at most this much of the part's own direction text, cut on a word
SURFACE_UP_DEG = 30.0        # a tool surface faces the hand's up when its normal is within this

# ---------------------------------------------------------------- 4.1 selfsee
MOUNT_MARGIN_MM = 20.0       # region B: the mount grown by the pad ring (equal to survey's MOUNT_MARGIN_M x 1000)
MOUNT_TOP_MARGIN_MM = 15.0   # region B reaches this above the mount top
TOP_VIEW_DEG, DOWN_VIEW_DEG, LEVEL_VIEW_DEG = 65.0, 25.0, -15.0   # view kinds by elevation
ROLL_WARN_DEG = 60.0         # a roll this large can make a standing thing look as if it lies flat
SELF_GRID_PX = 16            # the own-body ray grid step
VIEW_BLOCK_MAX_BYTES = 400   # the view block on every look
DRAW_CENTRE_FRAC = 0.2       # labels stay out of the image's central 20 percent
MARK_RING_PX = 4             # mark refusal: the 8 pixels this far around the marked one ...
MARK_RING_HITS = 3           # ... refuse when this many show the robot

# ---------------------------------------------------------------- 4.2 finish
ACT_NEAR_MIN_MM = 15.0       # an act is on a part when within this of its located point ...
ACT_NEAR_SIZE_FRAC = 0.6     # ... or within this share of its largest rough size
MARK_IN_FRAC = 0.45          # an object is on its mark within this share of the mark's rough size
MARK_OUT_FRAC = 0.5          # ... and contradicted beyond this share
REGION_GROW_PX = 20          # the device region: the survey box projected and grown this much
REGION_DISC_MM = 100.0       # ... else a disc this big round the last contact
HARD_CAP = 3                 # hard claim answers per episode; the next claim is recorded, gate_overridden
UNTRIED_MAX = 3              # untried calls in one answer
GIVE_UP_LEFT_FRAC = 1.0 / 3.0   # a give-up with more than this share of calls and seconds left is answered
EXEMPT_RECENT_CALLS = 3      # a recent blocking refusal within this many calls exempts a give-up

# ---------------------------------------------------------------- 4.3 act: push and lift
WAY_UP_Z = 0.5               # a push line pointing up more than this (30 deg) is a lift, refused way_up
WITNESS_MIN_PX = 24          # the naming look serves as witness view when the part spans this many pixels there
RAY_H_MIN = 0.3              # P2: a naming ray's horizontal part counts when it is this share of its length
CONFIRM_TRIES = 2            # unconfirmed contacts tolerated before not_found
MOVED_NCC, MOVED_SEARCH_PX = 0.8, 60   # a patch is found again at this correlation within this many pixels

# ---------------------------------------------------------------- 4.4 witness
ALIGN_SHIFT_PX, ALIGN_TURN_DEG = 6.0, 2.0   # frames that need more to line up compare as unclear
LIGHT_CHANGED_FRAC = 0.15    # a gain this far off 1 is a light change
DIFF_MIN, DIFF_SCALE = 12.0, 40.0 / 128.0   # change threshold: max(12, 40 x m / 128) levels
DIFF_NEAR_PX = 2             # a pixel changed only when no pixel this near in the other frame matches it
COMPONENT_MIN_PX = 30        # changes smaller than this are noise
DEVICE_MOVED_PX = 4.0        # the whole device box moved this far as one: device_moved
DEVICE_MOVED_NCC = 0.8
REGION_MIN_PX = 12           # a target region's radius at least this
RING_PX = 25                 # the ring round a target region
CHANGED_SHARE = 0.08         # changed yes: this share of the region changed ...
CHANGED_RING_MULT, CHANGED_RING_ADD = 3.0, 0.01   # ... and at least 3 x the ring's share + 0.01
NO_SHARE = 0.02              # a solid no: at most this share changed ...
SOLID_MIN_PX = 24            # ... on a region this big, in a frame not dim
CHANGES_MAX = 4
TRIPTYCH_W = 640
CORRECTIVES_MAX = 2          # look like: corrective moves after the first
CORRECTIVE_LEAD_MULT = 2.0   # ... each joint's lead clipped to twice its last shortfall

# ---------------------------------------------------------------- 4.5 sight
PATCH_SIZES_PX = (21, 31, 45, 61)
FRAME_KEEP_FRAC = 0.8        # the bracket stays in the central 80 percent of each extra view
ROLLS_TRIED = 12
EPI_BAND_PX = 6
NCC_PEAK, NCC_RUNNER_FRAC, NCC_RUNNER_PX = 0.75, 0.9, 8
MATCH_ERR_PX, POINT_ERR_PX, ON_TABLE_PX = 1.0, 2.0, 2.0
CONTACT_UP_DEG = 30.0        # on=contact_N: only on a face within this of up
PART_CHECK_PX = 9
VIEW_EL_DEG, VIEW_EL_ABOVE_DEG = 25.0, 75.0
VIEW_EL_STEPS_DEG = (10.0, 20.0)
VIEW_AZ_STEP_DEG = 20.0
AROUND_MAX = 4
AROUND_TILE_PX = (320, 240)
SURVEY_SHEET_COLS, SURVEY_TILE_W = 4, 320   # survey_see.sheet(tiles, cols=4, width=320)
FITS_LO, FITS_HI = 0.6, 1.8  # a candidate fits when every size ratio lies in this band
