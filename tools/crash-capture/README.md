# crash-capture — kdump for the pocophone, the part that does not depend on DRAM

This phone comes back from every reset with DRAM at its power-up state (see
`scratch/wt/connectivity/REPORT.md`, "pstore on rosemary, part 2"), so ramoops can
never hold a crash log. The log has to be taken out of memory by a second kernel
that runs *without* a reset: kdump, standard Linux, `kexec -p`.

What lives here is the capture side - the second kernel's initramfs and the small
program inside it. The trial side (reserving `crashkernel=`, loading the capture
kernel on a candidate boot) is in `poco-m5s/tools-phone/`.

    ./build-initramfs.sh            # -> out/crash-initramfs-<utc>.cpio.gz (32 KB)
    ./tests/run-tests.sh            # 5 cases, each one a planted fault

On the phone, one-time setup:

    mkdir -p /usr/lib/poco/crash
    cp /boot/Image-capture              /usr/lib/poco/crash/Image
    cp crash-initramfs-*.cpio.gz        /usr/lib/poco/crash/initramfs.cpio.gz
    install -m755 crash-kexec-load.sh   /usr/local/bin/crash-kexec-load.sh
    install -m644 poco-crash-kexec.service /etc/systemd/system/
    systemctl daemon-reload && systemctl enable poco-crash-kexec

`Image-capture` is any trial Image: the capture kernel only needs to mount
userdata, read `/proc/vmcore` and reboot. It is the same kernel as the trial one
on purpose - one less variable to get wrong after a crash.

What the capture kernel does: mounts `/proc`, `/sys`, `/dev` and the userdata
filesystem, then tries the phone's own `vmcore-dmesg` and `makedumpfile`
(through a chroot, because they and their libraries live on userdata), and
writes `--dump-dmesg` output to `/var/lib/poco/crash/<timestamp>.txt`. No full
vmcore: that would be gigabytes. If neither tool exists it writes the vmcore's
ELF header plus a line saying so, so a capture never comes back empty and
ambiguous.

`tests/run-tests.sh` runs the same source natively with `-DCRASH_TEST`, so the
dumpers become scripts the test plants: a dumper that exits 1, a dumper that
exits 0 writing nothing, no dumper at all, and a boot without `/proc/vmcore`.
All five must behave as required, including the empty-output case, which is the
one that silently "succeeds" if the code is careless.
