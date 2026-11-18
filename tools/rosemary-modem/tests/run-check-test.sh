#!/bin/sh
# run-check-test.sh — test modem-check.sh on the host, against a fake AT modem.
#
# Three things are asserted, and each of them has failed in real life at least once
# in this port:
#   1. every query in the list is actually sent, and the answers appear;
#   2. the injected error (AT+CPIN? -> +CME ERROR: 10) is reported and makes the
#      script exit non-zero, while the queries after it still run;
#   3. no unique identifier reaches the output: neither the IMEI nor the IMSI of the
#      fake modem may appear, and no run of 14+ digits may appear at all.
#
# The fake modem runs on a pty inside this test; nothing here touches a phone.

set -u

here=$(cd "$(dirname "$0")" && pwd)
check="$here/../modem-check.sh"
fake="$here/fake-at-modem.py"
work=$(mktemp -d "${TMPDIR:-/tmp}/modem-check-test.XXXXXX")
link="$work/wwan-test-at0"
out="$work/out.txt"
fail=0

cleanup() {
    [ -n "${modem_pid:-}" ] && kill "$modem_pid" 2>/dev/null
    rm -rf "$work"
}
trap cleanup EXIT INT TERM

command -v python3 >/dev/null 2>&1 || { echo "SKIP: python3 not available"; exit 77; }

python3 "$fake" "$link" >"$work/fake.out" 2>"$work/fake.err" &
modem_pid=$!
# wait for the pty to exist (bounded)
i=0
while [ ! -e "$link" ] && [ "$i" -lt 50 ]; do
    sleep 0.1
    i=$((i + 1))
done
if [ ! -e "$link" ]; then
    echo "FAIL: the fake modem did not start"; sed 's/^/  /' "$work/fake.err"; exit 1
fi

AT_PORT="$link" AT_TIMEOUT=2 STATE_FILE=/nonexistent SYSFS_WWAN=/nonexistent \
    "$check" >"$out" 2>&1
rc=$?

say() { printf '%-58s %s\n' "$1" "$2"; }

# 1. every query was sent and answered
for cmd in AT ATI AT+CGMR AT+CPIN? AT+CSQ AT+CFUN? AT+CREG? AT+CEREG? AT+COPS? AT+CGDCONT?; do
    if grep -qF "] $cmd" "$out"; then :; else
        say "query $cmd sent" "FAIL"; fail=1
    fi
done
[ "$fail" -eq 0 ] && say "all ten queries sent" "ok"

# the answers arrived, not just the questions
if grep -q 'MOLY.LR13.R1.MP.V89.6.P79' "$out" && grep -q '+CSQ: 17,99' "$out" \
   && grep -q '+CEREG: 0,5' "$out" && grep -q '28202' "$out"; then
    say "answers present (ATI, CSQ, CEREG, COPS)" "ok"
else
    say "answers present" "FAIL"; fail=1
fi

# 2. the injected error is reported, the run continues, the exit code says so
if grep -q '^\[.*\]     +CME ERROR: 10' "$out"; then
    say "injected error reported (+CME ERROR: 10)" "ok"
else
    say "injected error reported" "FAIL"; fail=1
fi
# the query after the failing one still ran: CGDCONT comes last
if grep -q 'AT+CGDCONT?' "$out" && grep -q '+CGDCONT: 1,"IP","internet"' "$out"; then
    say "the run continues past the error" "ok"
else
    say "the run continues past the error" "FAIL"; fail=1
fi
if [ "$rc" -ne 0 ]; then
    say "exit code non-zero on a failed query ($rc)" "ok"
else
    say "exit code non-zero on a failed query" "FAIL (was 0)"; fail=1
fi

# 3. no identifiers in the output
if grep -q '861234567890123' "$out"; then
    say "IMEI masked" "FAIL (found in output)"; fail=1
else
    say "IMEI masked" "ok"
fi
if grep -q '250010123456789' "$out"; then
    say "IMSI masked" "FAIL (found in output)"; fail=1
else
    say "IMSI masked" "ok"
fi
if grep -qE '[0-9]{14,}' "$out"; then
    say "no run of 14+ digits anywhere" "FAIL"; grep -nE '[0-9]{14,}' "$out" | sed 's/^/    /'; fail=1
else
    say "no run of 14+ digits anywhere" "ok"
fi
# ... while the ordinary numbers survive, so the masking is not simply blanking lines
if grep -q '+CSQ: 17,99' "$out"; then
    say "ordinary values survive masking" "ok"
else
    say "ordinary values survive masking" "FAIL"; fail=1
fi

echo
if [ "$fail" -eq 0 ]; then
    echo "PASS: modem-check.sh behaves as required"
    exit 0
fi
echo "FAILED — output follows:"
sed 's/^/  /' "$out"
exit 1
