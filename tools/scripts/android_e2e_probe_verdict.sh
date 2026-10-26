#!/usr/bin/env bash
# android_e2e_probe_verdict.sh: decide whether one run of the Android e2e's
# direct data demand probe (crates/test_fixtures/android_e2e_demand_probe)
# matches what the app's LAN posture must do.
#
#   android_e2e_probe_verdict.sh paired|open|schema-mismatch <probe-output>
#
# <probe-output> is everything the probe printed. It must hold exactly one
# `demand_probe ...` report line; other lines (logs) are ignored.
#
#   paired           every answered demand is refused, none is granted, and no
#                    frame reaches the probe's ingress.
#   open             the demand is granted, none is refused, and frames are
#                    delivered. The app sends the same refused ack for every
#                    refusal reason (posture, allow-list miss, unknown topic), so
#                    this control is what shows that a paired refusal comes from
#                    the posture: the same topic and schema ARE served when open.
#   schema-mismatch  the probe expected a schema the topic does not carry: the
#                    demand is granted, frames reach the ingress, and every one
#                    is dropped on the schema hash, so none is delivered.
#
# In every expectation the ingress must lose no frame to a decode error, a
# local re-injection failure or its own LAN posture (the probe's is open): a
# frame lost that way would hide behind the counts the rules above read.
#
# Exit 0 prints the counters on one line. Exit 1 prints why the report fails.
# Exit 2 is a usage error. The verdict reads only its arguments, so the same
# report always gives the same bytes. Oracle table:
# tools/scripts/test_android_e2e_probe_verdict.sh. Portable: bash 3.2+.

set -euo pipefail
# The report line is split into words below; no word may glob.
set -f

usage() {
    printf '%s\n' 'usage: android_e2e_probe_verdict.sh paired|open|schema-mismatch <probe-output>' >&2
    exit 2
}

[ "$#" -eq 2 ] || usage
expect=$1
case "$expect" in
    paired | open | schema-mismatch) ;;
    *) usage ;;
esac

reports=$(printf '%s\n' "$2" | grep '^demand_probe ' || true)
if [ -z "$reports" ]; then
    count=0
else
    count=$(printf '%s\n' "$reports" | wc -l | tr -d ' ')
fi
reject() {
    printf '%s; report: %s\n' "$1" "${reports:-none}"
    exit 1
}
[ "$count" -ne 0 ] || reject 'no probe report line'
[ "$count" -eq 1 ] || reject "$count probe report lines, expected 1"

# field NAME: the value of the one `NAME=` token on the report line.
field() {
    local name=$1 token value='' seen=0
    for token in $reports; do
        case "$token" in
            "$name="*)
                seen=$((seen + 1))
                value=${token#*=}
                ;;
        esac
    done
    [ "$seen" -ne 0 ] || reject "field '$name' is missing"
    [ "$seen" -eq 1 ] || reject "field '$name' appears $seen times"
    case "$value" in
        '' | *[!0-9]*) reject "field '$name' is not a whole number: $value" ;;
    esac
    printf '%s\n' "$value"
}

# A reject inside `field` exits only its command substitution, with the
# reason on its stdout, so each read passes that reason on and exits.
passes=$(field passes) || { printf '%s\n' "$passes"; exit 1; }
enabled=$(field enabled_acks) || { printf '%s\n' "$enabled"; exit 1; }
refused=$(field refused_acks) || { printf '%s\n' "$refused"; exit 1; }
wire_header=$(field wire_header_refused_acks) || { printf '%s\n' "$wire_header"; exit 1; }
mismatch=$(field schema_mismatch_drops) || { printf '%s\n' "$mismatch"; exit 1; }
decode=$(field decode_errors) || { printf '%s\n' "$decode"; exit 1; }
reinject=$(field reinject_failure_drops) || { printf '%s\n' "$reinject"; exit 1; }
lan_refused=$(field lan_refused_drops) || { printf '%s\n' "$lan_refused"; exit 1; }
wire=$(field wire_frames) || { printf '%s\n' "$wire"; exit 1; }
frames=$(field frames) || { printf '%s\n' "$frames"; exit 1; }

[ "$passes" -ge 1 ] || reject 'the demander ran no pass in the window'
# Every frame the ingress received is either delivered or dropped, so neither
# count can be more than the frames that crossed the network.
[ "$frames" -le "$wire" ] || reject "frames=$frames is more than wire_frames=$wire"
[ "$mismatch" -le "$wire" ] ||
    reject "schema_mismatch_drops=$mismatch is more than wire_frames=$wire"
[ "$wire_header" -eq 0 ] ||
    reject "the app refused $wire_header demand(s) on the wire header (a build mismatch)"
[ "$decode" -eq 0 ] || reject "the ingress dropped $decode frame(s) as undecodable"
[ "$reinject" -eq 0 ] ||
    reject "the ingress dropped $reinject frame(s) on local re-injection"
[ "$lan_refused" -eq 0 ] ||
    reject "the probe's ingress refused $lan_refused frame(s) on its own LAN posture"

case "$expect" in
    paired)
        [ "$enabled" -eq 0 ] || reject "a paired app granted $enabled demand(s)"
        [ "$wire" -eq 0 ] || reject "a paired app sent $wire frame(s) to the probe"
        [ "$refused" -ge 1 ] || reject 'no demand was refused, so the posture was not exercised'
        ;;
    open)
        [ "$refused" -eq 0 ] || reject "an open app refused $refused demand(s)"
        [ "$enabled" -ge 1 ] || reject 'an open app granted no demand'
        [ "$mismatch" -eq 0 ] || reject "an open app dropped $mismatch frame(s) on the schema"
        [ "$frames" -ge 1 ] || reject 'an open app delivered no frame'
        ;;
    schema-mismatch)
        [ "$refused" -eq 0 ] || reject "an open app refused $refused demand(s)"
        [ "$enabled" -ge 1 ] || reject 'an open app granted no demand'
        [ "$frames" -eq 0 ] || reject "the probe delivered $frames frame(s) under the wrong schema"
        [ "$mismatch" -eq "$wire" ] ||
            reject "schema_mismatch_drops=$mismatch is not every one of wire_frames=$wire"
        [ "$mismatch" -ge 1 ] || reject 'no frame was dropped on the schema, so the check was not exercised'
        ;;
esac

printf 'passes=%s enabled_acks=%s refused_acks=%s wire_header_refused_acks=%s schema_mismatch_drops=%s decode_errors=%s reinject_failure_drops=%s lan_refused_drops=%s wire_frames=%s frames=%s\n' \
    "$passes" "$enabled" "$refused" "$wire_header" "$mismatch" "$decode" "$reinject" \
    "$lan_refused" "$wire" "$frames"
