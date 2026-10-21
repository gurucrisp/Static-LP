#!/usr/bin/env python3
"""The firmware version the slave reports in its $H hello frame (SLAVE_FW_VERSION in slave_features.h) must be the VERSION file as major*10000 + minor*100 + patch.
A release that forgets to bump one of them would report the wrong firmware to the master. Exit 0 = they agree."""
import os, re, sys
root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ver = open(os.path.join(root, "VERSION")).read().strip()
m = re.fullmatch(r"(\d+)\.(\d+)\.(\d+)", ver)
if not m:
    sys.exit("VERSION is not major.minor.patch: %r" % ver)
want = int(m.group(1)) * 10000 + int(m.group(2)) * 100 + int(m.group(3))
cfg = open(os.path.join(root, "slave", "Core", "Inc", "slave_features.h")).read()
g = re.search(r"#define\s+SLAVE_FW_VERSION\s+(\d+)UL", cfg)
if not g:
    sys.exit("SLAVE_FW_VERSION not found in slave_features.h")
have = int(g.group(1))
if have != want:
    sys.exit("MISMATCH: VERSION %s means %d, SLAVE_FW_VERSION is %d" % (ver, want, have))
print("OK: VERSION %s = SLAVE_FW_VERSION %d" % (ver, have))
