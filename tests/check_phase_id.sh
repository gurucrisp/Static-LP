#!/bin/sh
# SLAVE_PHASE_ID (slave_link.h): 'R' by default, 'Y' or 'B' from the config, anything else is refused at build time.
S=$1; D=$(mktemp -d); bad=0; n=0
printf '#include "slave_link.h"\nchar c = SLAVE_PHASE_ID;\nint main(void) { return 0; }\n' > "$D/t.c"
ok()  { n=$((n+1)); gcc -std=c99 -fsyntax-only -I"$S/Inc" $2 "$D/t.c" 2>/dev/null || { echo "FAIL (does not build): $1"; bad=1; }; }
no()  { n=$((n+1)); gcc -std=c99 -fsyntax-only -I"$S/Inc" $2 "$D/t.c" 2>/dev/null && { echo "FAIL (builds): $1"; bad=1; }; }
ok "the default (R)" ""
ok "Y" "-DSLAVE_PHASE_ID='Y'"
ok "B" "-DSLAVE_PHASE_ID='B'"
ok "R explicitly" "-DSLAVE_PHASE_ID='R'"
no "X is refused" "-DSLAVE_PHASE_ID='X'"
no "a lower case letter is refused" "-DSLAVE_PHASE_ID='y'"
rm -rf "$D"
if [ $bad = 0 ]; then echo "  $n cases: ALL PASSED"; exit 0; else exit 1; fi
