#!/usr/bin/env python3
"""Assemble the userspace environment of the MT6785 GNSS engine from a super image.

Host only: nothing here runs a vendor binary, and nothing here touches a phone.
It reads the dynamic-partition metadata, carves the four partitions it needs,
extracts the vendor GPS files and - the part that matters - resolves their ELF
dependency closure, so the file set in filelist.txt is the closure and not a
guess.

    ./assemble.py --super super.img --out DIR [--list-only]

The output tree is laid out the way the vendor engine expects to see the
filesystem (see README.md in this directory for why each piece is there).
"""
import argparse
import hashlib
import os
import re
import shutil
import struct
import subprocess
import sys
import zipfile

GEOM_MAGIC = 0x616C4467
HDR_MAGIC = 0x414C5030

# The files the engine consists of, as the vendor ships them. Everything else is
# resolved from their NEEDED entries.
VENDOR_FILES = [
    "/bin/mnld",
    "/bin/mtk_agpsd",
    "/lib64/libmnl.so",
    "/lib64/libviagpsrpc.so",
    "/etc/MNL_Config.xml",
    "/etc/gnss/agps_profiles_conf2.xml",
]

# Where a SONAME is looked for, in load order. The bionic runtime apex comes
# first because that is what the linker actually loads; /system/lib64 only holds
# symlinks into it.
SEARCH = [
    ("runtime", ["/lib64/bionic", "/lib64"]),
    ("vendor_b", ["/lib64", "/lib"]),
    ("system_b", ["/lib64", "/lib", "/system/lib64", "/system/lib"]),
    ("system_ext_b", ["/lib64", "/lib"]),
    ("product_b", ["/lib64", "/lib"]),
]

PARTITIONS = ["system_b", "system_ext_b", "product_b", "vendor_b"]


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def debugfs(img, cmd):
    out = run(["debugfs", "-R", cmd, img])
    return out.stdout + out.stderr


def stat_of(img, path):
    return debugfs(img, "stat " + path)


def exists(img, path):
    out = stat_of(img, path)
    return "Inode:" in out and "File not found" not in out


def regular(img, path):
    out = stat_of(img, path)
    if "Inode:" not in out or "File not found" in out:
        return False
    if "Type: symlink" in out:
        return False
    m = re.search(r"Size: (\d+)", out)
    return bool(m) and int(m.group(1)) > 0


def listdir(img, path):
    out = debugfs(img, "ls -l " + path)
    names = []
    for line in out.splitlines():
        parts = line.split()
        if line.startswith(" ") and len(parts) >= 7:
            names.append(parts[-1])
    return names


def dump(img, path, dest):
    if not exists(img, path):
        return False
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    debugfs(img, "dump %s %s" % (path, dest))
    if os.path.exists(dest) and os.path.getsize(dest) > 0:
        return True
    # A symlink dumps as an empty file: follow it once.
    m = re.search(r'Fast link dest: "(.+?)"', stat_of(img, path))
    if not m:
        return False
    target = m.group(1)
    base = os.path.dirname(path)
    real = target if target.startswith("/") else base + "/" + target
    debugfs(img, "dump %s %s" % (real, dest))
    return os.path.exists(dest) and os.path.getsize(dest) > 0


def needed(binary):
    out = run(["readelf", "-d", binary]).stdout
    return re.findall(r"\(NEEDED\).*\[(.+?)\]", out)


def read_metadata(super_path):
    """Partition name -> (byte offset, byte size), from the live metadata slot."""
    with open(super_path, "rb") as f:
        f.seek(4096)
        geom = f.read(52)
        if struct.unpack_from("<I", geom, 0)[0] != GEOM_MAGIC:
            raise SystemExit("no dynamic-partition geometry in %s" % super_path)
        slot = None
        for probe in range(4096, 4096 * 64, 4096):
            f.seek(probe)
            if f.read(4) == struct.pack("<I", HDR_MAGIC):
                slot = probe
                break
        if slot is None:
            raise SystemExit("no metadata header in %s" % super_path)
        # Several slots exist; take the one that actually maps partitions.
        best = None
        probe = slot
        while probe < slot + 3 * 65536:
            f.seek(probe)
            head = f.read(256)
            if head[:4] == struct.pack("<I", HDR_MAGIC):
                hsize = struct.unpack_from("<I", head, 8)[0]
                p_off, p_num, p_sz = struct.unpack_from("<III", head, 80)
                e_off, e_num, e_sz = struct.unpack_from("<III", head, 92)
                if e_num > 0 and (best is None or e_num > best[0]):
                    best = (e_num, probe, hsize, p_off, p_num, p_sz,
                            e_off, e_num, e_sz)
            probe += 65536
        if not best:
            raise SystemExit("no metadata slot maps any partition")
        _, base, hsize, p_off, p_num, p_sz, e_off, e_num, e_sz = best
        tables = base + hsize
        f.seek(tables + p_off)
        parts = f.read(p_num * p_sz)
        f.seek(tables + e_off)
        exts = f.read(e_num * e_sz)
    extents = [struct.unpack_from("<QIII", exts, i * e_sz) for i in range(e_num)]
    out = {}
    for i in range(p_num):
        name, _attr, first, num, _grp = struct.unpack_from("<36sIIII", parts,
                                                           i * p_sz)
        name = name.rstrip(b"\0").decode()
        start = 0
        total = 0
        for k in range(num):
            nsec, ttype, tdata, _src = extents[first + k]
            if ttype != 0:
                raise SystemExit("partition %s has a non-linear extent" % name)
            if k == 0:
                start = tdata * 512
            total += nsec * 512
        if total:
            out[name] = (start, total)
    return out


def carve(super_path, layout, part, dest):
    start, size = layout[part]
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    if os.path.exists(dest) and os.path.getsize(dest) == size:
        return
    out = run(["dd", "if=" + super_path, "of=" + dest, "bs=1M",
               "iflag=skip_bytes,count_bytes", "skip=%d" % start,
               "count=%d" % size, "status=none"])
    if out.returncode:
        raise SystemExit("dd failed for %s: %s" % (part, out.stderr))


def extract_runtime_apex(system_img, workdir):
    apex = os.path.join(workdir, "com.android.runtime.apex")
    dump(system_img, "/system/apex/com.android.runtime.apex", apex)
    payload = os.path.join(workdir, "runtime", "apex_payload.img")
    os.makedirs(os.path.dirname(payload), exist_ok=True)
    with zipfile.ZipFile(apex) as z:
        with z.open("apex_payload.img") as src, open(payload, "wb") as dst:
            dst.write(src.read())
    return payload


# The bionic runtime lives in the apex, and nothing on this side of the chroot puts it
# where the linker looks for it: Android generates /linkerconfig/ld.config.txt for that.
# The tree does not generate one, so the runtime's libraries are linked into /system/lib64
# the way the vendor image does with its own symlinks, and the links are *relative* so the
# tree also resolves on the host (which is what makes it verifiable before it is deployed).
RUNTIME_LIBS = ["libc.so", "libm.so", "libdl.so", "libdl_android.so"]


def move_tree(src, dst):
    if not os.path.isdir(src):
        return
    os.makedirs(dst, exist_ok=True)
    for name in os.listdir(src):
        s = os.path.join(src, name)
        d = os.path.join(dst, name)
        if os.path.isdir(s) and not os.path.islink(s):
            move_tree(s, d)
        else:
            if os.path.lexists(d):
                os.remove(d)
            shutil.move(s, d)


def build_tree(root, apex_img):
    """Turn the extracted per-partition stage into the tree mnld expects to run in."""
    runtime = os.path.join(root, "apex", "com.android.runtime")
    os.makedirs(runtime, exist_ok=True)

    # The apex payload: the linker and the bionic libraries, which the closure walk
    # cannot find by SONAME alone.
    for d in ("/bin", "/lib64"):
        debugfs(apex_img, "rdump %s %s" % (d, runtime))

    move_tree(os.path.join(root, "system_b", "system"), os.path.join(root, "system"))
    move_tree(os.path.join(root, "system_b"), os.path.join(root, "system"))
    move_tree(os.path.join(root, "vendor_b"), os.path.join(root, "vendor"))
    move_tree(os.path.join(root, "system_ext_b"), os.path.join(root, "system_ext"))
    move_tree(os.path.join(root, "product_b"), os.path.join(root, "product"))
    move_tree(os.path.join(root, "runtime"), os.path.join(runtime, "lib64"))
    for stale in ("system_b", "vendor_b", "system_ext_b", "product_b", "runtime"):
        shutil.rmtree(os.path.join(root, stale), ignore_errors=True)

    # /system/bin/linker64 in Android is a symlink into the runtime apex, and so are the
    # bionic libraries; relative links resolve the same way inside the chroot.
    os.makedirs(os.path.join(root, "system", "bin"), exist_ok=True)
    os.makedirs(os.path.join(root, "system", "lib64"), exist_ok=True)
    link = os.path.join(root, "system", "bin", "linker64")
    if os.path.lexists(link):
        os.remove(link)
    os.symlink("../../../apex/com.android.runtime/bin/linker64", link)
    for name in RUNTIME_LIBS:
        link = os.path.join(root, "system", "lib64", name)
        if os.path.lexists(link):
            os.remove(link)
        os.symlink("../../apex/com.android.runtime/lib64/bionic/" + name, link)

    # What mnld writes into, and the pseudo filesystems start.sh bind-mounts over.
    for d in ("data/vendor/gps", "data/vendor/log/gps", "data/agps_supl",
              "data/local/tmp", "dev", "proc", "sys", "tmp"):
        os.makedirs(os.path.join(root, d), exist_ok=True)
    os.chmod(os.path.join(root, "tmp"), 0o1777)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--super", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--list-only", action="store_true")
    ap.add_argument("--keep-images", action="store_true")
    args = ap.parse_args()

    work = os.path.join(args.out, "images")
    root = os.path.join(args.out, "root")
    os.makedirs(work, exist_ok=True)

    layout = read_metadata(args.super)
    for part in PARTITIONS:
        if part not in layout:
            raise SystemExit("partition %s not in the image" % part)
        carve(args.super, layout, part, os.path.join(work, part + ".img"))

    runtime = extract_runtime_apex(os.path.join(work, "system_b.img"), work)
    images = {p: os.path.join(work, p + ".img") for p in PARTITIONS}
    images["runtime"] = runtime

    rows = []
    # vendor files first, then their closure
    for path in VENDOR_FILES:
        dest = os.path.join(root, "vendor", path.lstrip("/"))
        if not dump(images["vendor_b"], path, dest):
            raise SystemExit("cannot extract %s" % path)
        rows.append((os.path.basename(path), "vendor_b", path,
                     os.path.getsize(dest)))

    seen = set()
    queue = []
    for name, _part, _path, _size in list(rows):
        for dep in needed(os.path.join(root, "vendor",
                                       _path.lstrip("/"))):
            queue.append(dep)
    while queue:
        name = queue.pop(0)
        if name in seen:
            continue
        seen.add(name)
        found = None
        for part, dirs in SEARCH:
            for d in dirs:
                candidate = d + "/" + name
                if regular(images[part], candidate):
                    found = (part, candidate)
                    break
            if found:
                break
        if not found:
            rows.append((name, "MISSING", "", 0))
            continue
        part, path = found
        dest = os.path.join(root, part, path.lstrip("/"))
        dump(images[part], path, dest)
        rows.append((name, part, path, os.path.getsize(dest)))
        for dep in needed(dest):
            if dep not in seen:
                queue.append(dep)

    if not args.list_only:
        build_tree(root, images["runtime"])

    lines = ["%-48s %-12s %10s  %s" %
             ("file", "partition", "bytes", "path in the tree")]
    for name, part, path, size in sorted(rows, key=lambda r: (r[1], r[0])):
        lines.append("%-48s %-12s %10d  %s" % (name, part, size, path))
    total = sum(r[3] for r in rows)
    lines.append("")
    lines.append("%d files, %.1f MiB" % (len(rows), total / 1048576))
    text = "\n".join(lines)
    print(text)
    listpath = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "filelist.txt")
    if not args.list_only:
        with open(listpath, "w") as f:
            f.write(text + "\n")

    missing = [r[0] for r in rows if r[1] == "MISSING"]
    if missing:
        print("\nMISSING: %s" % ", ".join(missing), file=sys.stderr)
        return 1
    return 0


sys.exit(main())
