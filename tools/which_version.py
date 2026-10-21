#!/usr/bin/env python3
"""Which version of the firmware is in this folder?

    python tools/which_version.py slave  <path to your slave  Core folder>
    python tools/which_version.py master <path to your master Core folder>

Compares every file of your folder, byte for byte, with every tagged version in this repository
and tells you which tag it matches best and which files differ. Needs git on the PATH.
slave_config.h is your settings file and is reported separately (it is expected to differ)."""
import os, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SETTINGS = {"slave_config.h"}

def git(*args):
    r = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit("git failed: " + " ".join(args) + "\n" + r.stderr)
    return r.stdout

def main():
    if len(sys.argv) != 3 or sys.argv[1] not in ("slave", "master"):
        sys.exit(__doc__)
    project, folder = sys.argv[1], sys.argv[2]
    if not os.path.isdir(folder):
        sys.exit("not a folder: " + folder)

    mine = {}                                    # relative path -> git blob hash of YOUR file (no CRLF filtering)
    for dp, _, fns in os.walk(folder):
        for fn in fns:
            full = os.path.join(dp, fn)
            rel = os.path.relpath(full, folder).replace("\\", "/")
            mine[rel] = git("hash-object", "--no-filters", full).strip()

    prefix = project + "/Core/"
    rows = []
    for commit in git("rev-list", "--reverse", "HEAD").split():
        for tag in git("tag", "--points-at", commit).split():
            tree = {}
            for line in git("ls-tree", "-r", tag, "--", project + "/Core").splitlines():
                meta, path = line.split("\t", 1)
                tree[path[len(prefix):]] = meta.split()[2]
            if not tree:
                continue
            same = [p for p in tree if p in mine and mine[p] == tree[p] and p not in SETTINGS]
            diff = [p for p in tree if p in mine and mine[p] != tree[p] and p not in SETTINGS]
            miss = [p for p in tree if p not in mine and p not in SETTINGS]
            rows.append((tag, tree, same, diff, miss))

    print("%-20s %6s %10s %8s" % ("version", "same", "different", "missing"))
    for tag, tree, same, diff, miss in rows:
        print("%-20s %6d %10d %8d" % (tag, len(same), len(diff), len(miss)))
    exact = [r for r in rows if not r[3] and not r[4]]
    print()
    if exact:
        tags = ", ".join(r[0] for r in exact)
        if len(exact) == 1:
            print("EXACT MATCH: your %s folder is version %s." % (project, tags))
        else:
            print("EXACT MATCH with %d versions (this project did not change between them): %s" % (len(exact), tags))
            print("  -> the newest of them is %s." % exact[-1][0])
        tag, tree, same, diff, miss = exact[-1]
    else:
        # closest = fewest differing + missing files; on a tie the NEWER version
        tag, tree, same, diff, miss = min(reversed(rows), key=lambda r: len(r[3]) + len(r[4]))
        print("No exact match. Closest version: %s  (%d files differ, %d missing)" % (tag, len(diff), len(miss)))
        for p in diff: print("  DIFFERENT: " + p)
        for p in miss: print("  MISSING:   " + p)
    for s in SETTINGS:
        if s in tree and s in mine:
            print("  settings file %s: %s" % (s, "same as that version" if mine[s] == tree[s] else "differs (expected - it holds your settings)"))

main()
