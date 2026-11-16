#!/usr/bin/env python3
"""A fake AT modem on a pty, for testing modem-check.sh without a phone.

It answers the queries the check sends, and it does three things on purpose:

  * AT+CPIN? answers "+CME ERROR: 10" (SIM not inserted) — the injected error: the
    check must report it, keep going, and exit non-zero;
  * ATI and AT+CGSN carry a 15-digit IMEI and an IMSI, which must never appear in
    the check's output;
  * every answer is framed with CRLF and terminated by OK/ERROR, the way the modem
    frames them.

Usage: fake-at-modem.py <pty-link-path>
It prints nothing on stdout except the slave device path, then serves until killed.
"""

import os
import select
import sys
import tty

IMEI = "861234567890123"
IMSI = "250010123456789"

# command -> the lines the modem answers before its final result code
ANSWERS = {
    "AT": ["OK"],
    "ATI": [
        "MTK2",
        "MOLY.LR13.R1.MP.V89.6.P79",
        "IMEI: " + IMEI,
        "OK",
    ],
    "AT+CGMR": ["MOLY.LR13.R1.MP.V89.6.P79", "OK"],
    # the injected error
    "AT+CPIN?": ["+CME ERROR: 10"],
    "AT+CSQ": ["+CSQ: 17,99", "OK"],
    "AT+CFUN?": ["+CFUN: 0", "OK"],
    "AT+CREG?": ["+CREG: 0,5", "OK"],
    "AT+CEREG?": ["+CEREG: 0,5", "OK"],
    "AT+COPS?": ['+COPS: 0,0,"28202",6', "OK"],
    "AT+CGDCONT?": ['+CGDCONT: 1,"IP","internet","10.64.3.21",0,0', "OK"],
    # a URC that must not be mistaken for a command reply
    "AT+CGSN": ["+CGSN: " + IMEI, "OK"],
    "AT+CIMI": ["+CIMI: " + IMSI, "OK"],
}


def serve(master_fd, slave_path):
    buf = b""
    with open(slave_path, "rb", buffering=0) as port:
        # the port is raw already; make that explicit for the pty
        tty.setraw(port.fileno())
        while True:
            r, _, _ = select.select([master_fd], [], [], 1.0)
            if not r:
                continue
            try:
                chunk = os.read(master_fd, 4096)
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            while b"\r" in buf:
                line, buf = buf.split(b"\r", 1)
                cmd = line.decode(errors="replace").strip()
                if not cmd:
                    continue
                lines = ANSWERS.get(cmd.upper(), ["ERROR"])
                reply = "".join("\r\n%s" % l for l in lines) + "\r\n"
                os.write(master_fd, reply.encode())


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: fake-at-modem.py <pty-link-path>")
    link = sys.argv[1]
    master_fd, slave_fd = os.openpty()
    slave_path = os.ttyname(slave_fd)
    if os.path.lexists(link):
        os.unlink(link)
    os.symlink(slave_path, link)
    print(slave_path, flush=True)
    try:
        serve(master_fd, slave_path)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
