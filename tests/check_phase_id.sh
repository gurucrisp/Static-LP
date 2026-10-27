#!/bin/sh
# v0.9.53: SLAVE_PHASE_ID is gone (the master knows the phase from the UART port). slave_link.h builds without it, and a config that still defines it is refused at build time, so a leftover line is noticed.
S=$1; D=$(mktemp -d); bad=0; n=0
printf '#include "slave_link.h"\nint main(void) { return 0; }\n' > "$D/t.c"
ok()  { n=$((n+1)); gcc -std=c99 -fsyntax-only -I"$S/Inc" $2 "$D/t.c" 2>/dev/null || { echo "FAIL (does not build): $1"; bad=1; }; }
no()  { n=$((n+1)); gcc -std=c99 -fsyntax-only -I"$S/Inc" $2 "$D/t.c" 2>/dev/null && { echo "FAIL (builds): $1"; bad=1; }; }
ok "no SLAVE_PHASE_ID" ""
no "a leftover Y is refused" "-DSLAVE_PHASE_ID='Y'"
no "a leftover B is refused" "-DSLAVE_PHASE_ID='B'"
no "a leftover R is refused" "-DSLAVE_PHASE_ID='R'"
n=$((n+1)); grep -q "SLAVE_PHASE_ID" "$S/Src/slave_link.c" && { echo "FAIL: slave_link.c still uses SLAVE_PHASE_ID"; bad=1; }
rm -rf "$D"
if [ $bad = 0 ]; then echo "  $n cases: ALL PASSED"; exit 0; else exit 1; fi
