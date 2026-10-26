#!/usr/bin/env bash
# test_android_e2e_probe_verdict.sh: the oracle table for
# tools/scripts/android_e2e_probe_verdict.sh.
#
# The verdict script reads the one report line the Android e2e's direct data
# demand probe prints and decides whether it matches what a posture must do.
# Every report below is written by hand in the probe's line shape. The first
# case is the line CI's emulator run printed for the paired app; the open and
# schema mismatch lines use the counts a manual open run gave. Each case states
# the exit status and the exact stdout, or a fragment of it, that the verdict
# must give. One case per behaviour; the first failing assertion prints one
# `FAIL:` line and exits 1. The case count is asserted against CASES_EXPECTED
# at the end: a case deleted or never reached fails the run.
# Portable: bash 3.2+, GNU and BSD userland. No network, no device.

set -euo pipefail

CASES_EXPECTED=33

script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
verdict="$script_dir/android_e2e_probe_verdict.sh"

cases=0
case_name=
output=
status=0

fail() {
    printf 'FAIL: %s: %s\n' "${case_name:-setup}" "$1" >&2
    printf '%s\n' '--- output ---' "$output" >&2
    exit 1
}

# run_case NAME ARG... : run the verdict, keep its stdout and exit status.
run_case() {
    case_name=$1
    shift
    cases=$((cases + 1))
    status=0
    output=$("$verdict" "$@" 2>&1) || status=$?
}

expect_status() {
    [ "$status" -eq "$1" ] || fail "exit status $status, expected $1"
}

expect_exact() {
    [ "$output" = "$1" ] || fail "output is not exactly: $1"
}

expect_output() {
    case "$output" in
        *"$1"*) ;;
        *) fail "output does not contain: $1" ;;
    esac
}

# report FIELD=VALUE... : one probe report line. The fields come in the order
# the probe prints them; a case overrides only the ones it is about.
report() {
    local passes=5 enabled=0 refused=5 wire_header=0 mismatch=0 decode=0 reinject=0
    local lan_refused=0 wire=0 frames=0
    local schema=std_msgs/String
    local arg
    for arg in "$@"; do
        case "$arg" in
            passes=*) passes=${arg#*=} ;;
            enabled_acks=*) enabled=${arg#*=} ;;
            refused_acks=*) refused=${arg#*=} ;;
            wire_header_refused_acks=*) wire_header=${arg#*=} ;;
            schema_mismatch_drops=*) mismatch=${arg#*=} ;;
            decode_errors=*) decode=${arg#*=} ;;
            reinject_failure_drops=*) reinject=${arg#*=} ;;
            lan_refused_drops=*) lan_refused=${arg#*=} ;;
            wire_frames=*) wire=${arg#*=} ;;
            frames=*) frames=${arg#*=} ;;
            schema=*) schema=${arg#*=} ;;
            *) printf 'report: unknown field %s\n' "$arg" >&2; exit 1 ;;
        esac
    done
    printf 'demand_probe topic=/app/heartbeat schema=%s window_s=10 passes=%s enabled_acks=%s refused_acks=%s wire_header_refused_acks=%s schema_mismatch_drops=%s decode_errors=%s reinject_failure_drops=%s lan_refused_drops=%s wire_frames=%s frames=%s\n' \
        "$schema" "$passes" "$enabled" "$refused" "$wire_header" "$mismatch" "$decode" \
        "$reinject" "$lan_refused" "$wire" "$frames"
}

# A real run interleaves the probe's warn-level log lines with its report.
log_line='2026-10-20T10:00:00.000000Z  WARN cerulion_core::transport::network: demand REFUSED'

paired_ok=$(report)
open_ok=$(report enabled_acks=4 refused_acks=0 wire_frames=20 frames=20)
mismatch_ok=$(report schema=std_msgs/Int32 enabled_acks=4 refused_acks=0 \
    schema_mismatch_drops=20 wire_frames=20 frames=0)

# --- paired ------------------------------------------------------------------
run_case paired_every_demand_refused paired "$log_line
$paired_ok"
expect_status 0
expect_exact 'passes=5 enabled_acks=0 refused_acks=5 wire_header_refused_acks=0 schema_mismatch_drops=0 decode_errors=0 reinject_failure_drops=0 lan_refused_drops=0 wire_frames=0 frames=0'

run_case paired_one_grant paired "$(report enabled_acks=1 refused_acks=4)"
expect_status 1
expect_output 'a paired app granted 1 demand(s)'

run_case paired_frame_on_the_wire paired "$(report wire_frames=2)"
expect_status 1
expect_output 'a paired app sent 2 frame(s)'

# A frame the probe's subscriber received that never crossed its ingress came
# by another path (a host netd bridging into shared memory, say).
run_case paired_frame_off_the_ingress paired "$(report frames=1)"
expect_status 1
expect_output 'frames=1 is more than wire_frames=0'

# One GET per pass reaches one peer, the app, so it answers at most once a
# pass. More acks than passes means another peer answered too, and its refusal
# would be counted as the app's posture.
run_case paired_more_acks_than_passes paired "$(report refused_acks=6)"
expect_status 1
expect_output '6 ack(s) from 5 pass(es): more than one peer answered'

run_case paired_no_answer paired "$(report refused_acks=0)"
expect_status 1
expect_output 'no demand was refused'

# The probe's own LAN access is open, so its ingress must never refuse a frame.
run_case paired_lan_refused paired "$(report lan_refused_drops=1 wire_frames=1)"
expect_status 1
expect_output "the probe's ingress refused 1 frame(s) on its own LAN posture"

run_case paired_wire_header_refusal paired "$(report refused_acks=4 wire_header_refused_acks=1)"
expect_status 1
expect_output 'refused 1 demand(s) on the wire header'

# --- open --------------------------------------------------------------------
run_case open_granted_and_delivered open "$open_ok"
expect_status 0
expect_exact 'passes=5 enabled_acks=4 refused_acks=0 wire_header_refused_acks=0 schema_mismatch_drops=0 decode_errors=0 reinject_failure_drops=0 lan_refused_drops=0 wire_frames=20 frames=20'

# The refused ack is the same bytes for every refusal reason, so an open app
# that refuses even one demand makes a paired refusal prove nothing.
run_case open_one_refusal open "$(report enabled_acks=4 refused_acks=1 wire_frames=20 frames=20)"
expect_status 1
expect_output 'an open app refused 1 demand(s)'

run_case open_granted_no_frame open "$(report enabled_acks=4 refused_acks=0)"
expect_status 1
expect_output 'an open app delivered no frame'

run_case open_never_granted open "$(report refused_acks=0)"
expect_status 1
expect_output 'an open app granted no demand'

run_case open_schema_drops open "$(report enabled_acks=4 refused_acks=0 schema_mismatch_drops=1 wire_frames=21 frames=20)"
expect_status 1
expect_output 'dropped 1 frame(s) on the schema'

run_case open_more_dropped_than_received open "$(report enabled_acks=4 refused_acks=0 schema_mismatch_drops=21 wire_frames=20)"
expect_status 1
expect_output 'schema_mismatch_drops=21 is more than wire_frames=20'

run_case open_decode_errors open "$(report enabled_acks=4 refused_acks=0 decode_errors=1 wire_frames=20 frames=19)"
expect_status 1
expect_output 'the ingress dropped 1 frame(s) as undecodable'

run_case open_more_delivered_than_received open "$(report enabled_acks=4 refused_acks=0 wire_frames=2 frames=3)"
expect_status 1
expect_output 'frames=3 is more than wire_frames=2'

# --- schema mismatch -----------------------------------------------------------
run_case mismatch_every_frame_dropped schema-mismatch "$mismatch_ok"
expect_status 0
expect_exact 'passes=5 enabled_acks=4 refused_acks=0 wire_header_refused_acks=0 schema_mismatch_drops=20 decode_errors=0 reinject_failure_drops=0 lan_refused_drops=0 wire_frames=20 frames=0'

run_case mismatch_frame_delivered schema-mismatch "$(report schema=std_msgs/Int32 enabled_acks=4 refused_acks=0 schema_mismatch_drops=17 wire_frames=20 frames=3)"
expect_status 1
expect_output 'delivered 3 frame(s) under the wrong schema'

run_case mismatch_one_refusal schema-mismatch "$(report schema=std_msgs/Int32 enabled_acks=4 refused_acks=1 schema_mismatch_drops=20 wire_frames=20)"
expect_status 1
expect_output 'an open app refused 1 demand(s)'

run_case mismatch_never_granted schema-mismatch "$(report schema=std_msgs/Int32 enabled_acks=0 refused_acks=0 schema_mismatch_drops=20 wire_frames=20)"
expect_status 1
expect_output 'an open app granted no demand'

# Every frame must die on the schema; one lost another way is not the check.
run_case mismatch_some_not_on_the_schema schema-mismatch "$(report schema=std_msgs/Int32 enabled_acks=4 refused_acks=0 schema_mismatch_drops=17 wire_frames=20)"
expect_status 1
expect_output 'schema_mismatch_drops=17 is not every one of wire_frames=20'

run_case mismatch_reinject_failures schema-mismatch "$(report schema=std_msgs/Int32 enabled_acks=4 refused_acks=0 schema_mismatch_drops=18 reinject_failure_drops=2 wire_frames=20)"
expect_status 1
expect_output 'the ingress dropped 2 frame(s) on local re-injection'

run_case mismatch_no_drop schema-mismatch "$(report schema=std_msgs/Int32 enabled_acks=4 refused_acks=0)"
expect_status 1
expect_output 'no frame was dropped on the schema'

# --- the report itself -------------------------------------------------------
run_case no_report paired "$log_line"
expect_status 1
expect_output 'no probe report line'

run_case two_reports paired "$paired_ok
$paired_ok"
expect_status 1
expect_output '2 probe report lines'

# `wire_frames=` must never be read as `frames=`.
run_case frames_missing paired "${paired_ok% frames=0}"
expect_status 1
expect_output "field 'frames' is missing"

run_case field_not_a_number paired "${paired_ok% frames=0} frames=x"
expect_status 1
expect_output "field 'frames' is not a whole number: x"

run_case field_twice paired "$paired_ok frames=0"
expect_status 1
expect_output "field 'frames' appears 2 times"

run_case no_pass paired "$(report passes=0 refused_acks=0)"
expect_status 1
expect_output 'the demander ran no pass'

# --- usage -------------------------------------------------------------------
run_case unknown_expectation granted "$paired_ok"
expect_status 2
expect_output 'usage: android_e2e_probe_verdict.sh paired|open|schema-mismatch <probe-output>'

run_case missing_output paired
expect_status 2
expect_output 'usage: android_e2e_probe_verdict.sh'

# --- determinism: the same report gives the same bytes ---------------------------
run_case paired_twice paired "$paired_ok"
first=$output
second=$("$verdict" paired "$paired_ok")
[ "$first" = "$second" ] || fail "two verdicts on one report differ: $second"

run_case open_twice open "$open_ok"
first=$output
second=$("$verdict" open "$open_ok")
[ "$first" = "$second" ] || fail "two verdicts on one report differ: $second"

case_name=
[ "$cases" -eq "$CASES_EXPECTED" ] ||
    fail "ran $cases case(s), CASES_EXPECTED is $CASES_EXPECTED"
printf 'test_android_e2e_probe_verdict: all %s cases passed\n' "$cases"
