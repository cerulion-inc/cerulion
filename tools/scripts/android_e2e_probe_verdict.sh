#!/usr/bin/env bash
# android_e2e_probe_verdict.sh: decide whether one run of the Android e2e's
# direct data demand probe (crates/test_fixtures/android_e2e_demand_probe)
# matches what the app's LAN posture must do.
#
#   android_e2e_probe_verdict.sh paired|open|schema-mismatch|connect <app-identity> <probe-output>
#
# <app-identity> is the robot identity the app runs under: every counted ack
# must come from it. <probe-output> is everything the probe printed. It must hold exactly one
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
#   connect          what open must do, over the link the app dialed: the
#                    probe listened and dialed nothing (`dialer=app`). Every
#                    other expectation reads an app the probe dialed
#                    (`dialer=probe`).
#
# In every expectation the acks must come from the app alone (`ack_peers` is
# exactly <app-identity>, or empty when no ack was counted): an ack from any
# other peer, such as a host netd that answers the wildcard demand with
# `refused` for a topic it does not produce, would be counted as the app's.
# `ack_peer_overflow` and `ack_peer_overlong` must be 0: each counts acks from
# a peer the probe did not name, so ack_peers would be incomplete.
# The ingress must also lose no frame to a decode error, a
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
    printf '%s\n' 'usage: android_e2e_probe_verdict.sh paired|open|schema-mismatch|connect <app-identity> <probe-output>' >&2
    exit 2
}

[ "$#" -eq 3 ] || usage
expect=$1
app=$2
case "$expect" in
    paired | open | schema-mismatch | connect) ;;
    *) usage ;;
esac
# A robot identity: 1 to 63 ASCII letters, digits, '_' or '-'.
case "$app" in
    '' | *[!A-Za-z0-9_-]*) usage ;;
esac
[ "${#app}" -le 63 ] || usage

reports=$(printf '%s\n' "$3" | grep '^demand_probe ' || true)
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

# raw_field NAME: the value of the one `NAME=` token on the report line,
# which may be empty.
raw_field() {
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
    printf '%s\n' "$value"
}

# field NAME: the one `NAME=` value, which must be a whole number.
field() {
    local value
    value=$(raw_field "$1") || { printf '%s\n' "$value"; exit 1; }
    case "$value" in
        '' | *[!0-9]*) reject "field '$1' is not a whole number: $value" ;;
    esac
    printf '%s\n' "$value"
}

# A reject inside `field` exits only its command substitution, with the
# reason on its stdout, so each read passes that reason on and exits.
dialer=$(raw_field dialer) || { printf '%s\n' "$dialer"; exit 1; }
case "$dialer" in
    probe | app) ;;
    *) reject "field 'dialer' is not probe or app: $dialer" ;;
esac
passes=$(field passes) || { printf '%s\n' "$passes"; exit 1; }
enabled=$(field enabled_acks) || { printf '%s\n' "$enabled"; exit 1; }
refused=$(field refused_acks) || { printf '%s\n' "$refused"; exit 1; }
wire_header=$(field wire_header_refused_acks) || { printf '%s\n' "$wire_header"; exit 1; }
peers=$(raw_field ack_peers) || { printf '%s\n' "$peers"; exit 1; }
# Robot names joined by ','; empty when no ack was counted.
case ",$peers," in
    *[!A-Za-z0-9_,-]* | *,,*)
        [ -z "$peers" ] || reject "field 'ack_peers' is not a list of robot names: $peers"
        ;;
esac
overflow=$(field ack_peer_overflow) || { printf '%s\n' "$overflow"; exit 1; }
overlong=$(field ack_peer_overlong) || { printf '%s\n' "$overlong"; exit 1; }
mismatch=$(field schema_mismatch_drops) || { printf '%s\n' "$mismatch"; exit 1; }
decode=$(field decode_errors) || { printf '%s\n' "$decode"; exit 1; }
reinject=$(field reinject_failure_drops) || { printf '%s\n' "$reinject"; exit 1; }
lan_refused=$(field lan_refused_drops) || { printf '%s\n' "$lan_refused"; exit 1; }
wire=$(field wire_frames) || { printf '%s\n' "$wire"; exit 1; }
frames=$(field frames) || { printf '%s\n' "$frames"; exit 1; }

[ "$passes" -ge 1 ] || reject 'the demander ran no pass in the window'
# Each pass sends one GET per selector and blocks for its replies. The probe
# reaches one peer, the app, so the app answers at most once a pass: more acks
# than passes means another peer answered, and its acks are not the app's.
# The probe names at most a capped number of peers, each of bounded length:
# an ack from a peer it did not name leaves ack_peers incomplete.
[ "$overflow" -eq 0 ] ||
    reject "ack_peer_overflow=$overflow ack(s) came from a peer the probe did not name, so ack_peers is incomplete"
[ "$overlong" -eq 0 ] ||
    reject "ack_peer_overlong=$overlong ack(s) came from a peer name too long to store, so ack_peers is incomplete"
acks=$((enabled + refused + wire_header))
[ "$acks" -le "$passes" ] ||
    reject "$acks ack(s) from $passes pass(es): more than one peer answered"
if [ "$acks" -eq 0 ]; then
    [ -z "$peers" ] || reject "ack_peers=$peers names a peer, but no ack was counted"
else
    [ "$peers" = "$app" ] ||
        reject "the ack(s) came from ${peers:-no peer}, not only from the app $app"
fi
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
    connect)
        [ "$dialer" = app ] ||
            reject "dialer=$dialer, but the connect case needs the app to dial the probe"
        ;;
    *)
        [ "$dialer" = probe ] ||
            reject "dialer=$dialer, but the $expect case needs the probe to dial the app"
        ;;
esac
case "$expect" in
    paired)
        [ "$enabled" -eq 0 ] || reject "a paired app granted $enabled demand(s)"
        [ "$wire" -eq 0 ] || reject "a paired app sent $wire frame(s) to the probe"
        [ "$refused" -ge 1 ] || reject 'no demand was refused, so the posture was not exercised'
        ;;
    open | connect)
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

printf 'dialer=%s passes=%s enabled_acks=%s refused_acks=%s wire_header_refused_acks=%s ack_peers=%s schema_mismatch_drops=%s decode_errors=%s reinject_failure_drops=%s lan_refused_drops=%s wire_frames=%s frames=%s\n' \
    "$dialer" "$passes" "$enabled" "$refused" "$wire_header" "$peers" "$mismatch" "$decode" "$reinject" \
    "$lan_refused" "$wire" "$frames"
