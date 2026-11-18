#!/bin/sh
# modem-check.sh — read-only check of the rosemary modem.
#
# Everything here only ASKS the modem something. No registration is changed, no
# PDP context is activated, no SMS is sent, no call is dialled: the command list is
# ATI, AT+CGMR, AT+CPIN?, AT+CSQ, AT+CFUN?, AT+CREG?, AT+CEREG?, AT+COPS? (query
# form) and AT+CGDCONT? (query form), plus the driver's own state file and the
# kernel interfaces.
#
# Two rules this script obeys by construction:
#   - every AT command has a timeout, so a wedged modem cannot hang the check;
#   - unique numbers never reach the output. IMEI, IMSI and ICCID are masked before
#     anything is printed (the owner's rule for logs), and so is any run of 14 or
#     more digits, which is what those identifiers look like on the wire.
#
# Usage:  modem-check.sh [AT-PORT]
#         AT_PORT=/dev/wwan1at0 modem-check.sh
#
# When ModemManager holds the AT port the open fails with EBUSY; the script says so
# and shows the same commands to run through mmcli instead, because stopping the
# daemon is a change and this script does not make changes.

set -u

AT_PORT=${1:-${AT_PORT:-}}
AT_TIMEOUT=${AT_TIMEOUT:-3}          # seconds per AT command
AT_TICKS=$((AT_TIMEOUT * 10))        # the reader wakes ten times a second
STATE_FILE=${STATE_FILE:-/sys/devices/platform/10209000.modem/state}
SYSFS_WWAN=${SYSFS_WWAN:-/sys/class/wwan}

WORK=${TMPDIR:-/tmp}/modem-check.$$
mkdir -p "$WORK" 2>/dev/null || WORK=/tmp
trap 'rm -rf "$WORK"' EXIT INT TERM

mask() {
    # The identifiers: 14+ digits in a row (IMEI 15, IMSISV 16, ICCID 18-20).
    # Also drop the value of any line whose keyword names an identifier, whatever
    # its length, so a short or oddly formatted one is still not printed.
    sed -E \
        -e 's/^([[:space:]]*\+?(CGSN|CIMI|CCID|ICCID|GSN|IMEI|IMSI)[[:space:]]*:).*$/\1 <masked>/I' \
        -e 's/[0-9]{14,}/<masked-id>/g'
}

say() {
    # section line, never masked (it is ours)
    printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

show() {
    # data line: indented, timestamped, masked
    printf '[%s]     %s\n' "$(date +%H:%M:%S)" "$(printf '%s' "$*" | mask)"
}

find_at_port() {
    # The driver's AT port is the wwan<instance>at0 it created; the instance is not
    # fixed, because another WWAN device may own wwan0.
    for d in "$SYSFS_WWAN"/*at0; do
        [ -e "$d" ] || continue
        if grep -qx 'DRIVER=rosemary-ccci' "$d/device/uevent" 2>/dev/null; then
            printf '/dev/%s\n' "$(basename "$d")"
            return 0
        fi
    done
    # fall back to the first one that exists at all, and say which
    for d in "$SYSFS_WWAN"/*at0; do
        [ -e "$d" ] || continue
        printf '/dev/%s\n' "$(basename "$d")"
        return 0
    done
    return 1
}

# ---- the AT exchange ---------------------------------------------------

# Read one answer into $WORK/reply.
#
# The read is a background dd plus a bounded wait for the final result code, rather
# than a pipeline that has to be interrupted: a pipeline needs timeout(1) to kill dd
# and awk to exit early for dd to see SIGPIPE, and when either step behaves
# differently the whole check hangs on a port that will never answer. This way the
# reader is always stopped by us, on a tick count, whatever the modem does.
at_read() {
    _reply=$WORK/reply
    : >"$_reply"
    dd bs=1 count=4096 <&3 >"$_reply" 2>/dev/null &
    _dd=$!
    _ticks=0
    while [ "$_ticks" -lt "$AT_TICKS" ]; do
        sleep 0.1
        _ticks=$((_ticks + 1))
        grep -qE '^(OK|ERROR|\+CME ERROR|\+CMS ERROR|NO CARRIER)$' "$_reply" 2>/dev/null && break
    done
    # SIGKILL, and no wait: a reader blocked in a device read does not always die on
    # SIGTERM, and waiting for it is what turns a wedged port into a wedged script.
    kill -9 "$_dd" 2>/dev/null
    tr -d '\r' <"$_reply" | grep -v '^$'
}

at_cmd() {
    _cmd=$1
    _out=

    if ! printf '%s\r' "$_cmd" >&3 2>/dev/null; then
        say "$_cmd — the port refused the write"
        return 2
    fi
    _out=$(at_read)

    say "$_cmd"
    if [ -z "$_out" ]; then
        show "(no answer within ${AT_TIMEOUT}s)"
        return 1
    fi
    printf '%s\n' "$_out" | while IFS= read -r line; do
        [ -n "$line" ] && show "$line"
    done
    if printf '%s\n' "$_out" | grep -qE '^(ERROR|\+CME ERROR|\+CMS ERROR)'; then
        return 1
    fi
    return 0
}

# ---- the check ---------------------------------------------------------

printf 'rosemary modem check — read only, no registration change, no SMS, no calls\n'

say "driver state ($STATE_FILE)"
if [ -r "$STATE_FILE" ]; then
    show "$(cat "$STATE_FILE" 2>/dev/null)"
else
    show "(absent: is rosemary_ccci loaded and the modem probed?)"
fi

say "ports"
if ls "$SYSFS_WWAN" >/dev/null 2>&1; then
    for d in "$SYSFS_WWAN"/*; do
        [ -e "$d" ] || continue
        drv=$(sed -n 's/^DRIVER=//p' "$d/device/uevent" 2>/dev/null)
        show "$(printf '/dev/%s  driver %s' "$(basename "$d")" "${drv:-?}")"
    done
else
    show "(no /sys/class/wwan: the driver is not loaded)"
fi

say "network interfaces"
if command -v ip >/dev/null 2>&1; then
    _links=$(ip -br link show 2>/dev/null | awk '$1 ~ /^(wwan|ccmni)/ { print }')
    if [ -n "$_links" ]; then
        printf '%s\n' "$_links" | while IFS= read -r l; do show "$l"; done
    else
        show "(none: create the data interface with rosemary-modem-datalink)"
    fi
else
    show "(iproute2 not installed)"
fi
# the binding attribute only our links carry, and where each link points
for f in /sys/class/net/wwan*/md_interface /sys/class/net/ccmni*/md_interface; do
    [ -e "$f" ] || continue
    show "$(printf '%s = %s' "$f" "$(cat "$f" 2>/dev/null)")"
done

if [ -z "$AT_PORT" ]; then
    AT_PORT=$(find_at_port) || AT_PORT=
fi

if [ -z "$AT_PORT" ]; then
    say "AT"
    show "(no AT port: the modem has not reached state ready, so the driver has not created one)"
    exit 2
fi

say "AT port $AT_PORT"
# One open for the whole session: a character device opened and closed per command
# drops the line, and a WWAN AT port is exclusive anyway, so a second opener is the
# EBUSY we report here.
if ! exec 3<>"$AT_PORT" 2>/dev/null; then
    show "(cannot open: ModemManager or another process holds it)"
    show "run the same queries through it instead, for example:"
    show "  mmcli -m 0 --command='AT+CSQ'"
    show "  mmcli -m 0 --command='AT+CEREG?'"
    exit 2
fi

# NB: only query forms. AT+COPS=? would start a network scan and AT+COPS=<n> would
# change the operator; AT+CGDCONT alone would change the context. None of those.
_rc=0
at_cmd "AT"            || _rc=1
at_cmd "ATI"           || _rc=1
at_cmd "AT+CGMR"       || _rc=1
at_cmd "AT+CPIN?"      || _rc=1
at_cmd "AT+CSQ"        || _rc=1
at_cmd "AT+CFUN?"      || _rc=1
at_cmd "AT+CREG?"      || _rc=1
at_cmd "AT+CEREG?"     || _rc=1
at_cmd "AT+COPS?"      || _rc=1
at_cmd "AT+CGDCONT?"   || _rc=1

exec 3>&- 2>/dev/null

say "done"
if [ "$_rc" -eq 0 ]; then
    show "every query answered OK"
else
    show "at least one query failed or timed out — see the lines above"
fi
exit "$_rc"
