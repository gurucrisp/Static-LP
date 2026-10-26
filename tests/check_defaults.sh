#!/bin/sh
# The defaults of SLAVE_AUTOTUNE, SLAVE_TUNE_MODE, SLAVE_TUNE_APPLY, SLAVE_TUNE_ENFORCE and SLAVE_IOUT (slave_features.h), in a directory with an EMPTY slave_config.h so that only the command line decides.
S=$1; HERE=$(cd "$(dirname "$0")" && pwd); D=$(mktemp -d)
cp "$S/Inc/slave_features.h" "$D/"; : > "$D/slave_config.h"; cp "$HERE/test_slave_defaults.c" "$D/"
bad=0; n=0
chk() {   # name flags AT TM AP EN IO
  name=$1; flags=$2; shift 2; n=$((n+1))
  if gcc -std=c99 -fsyntax-only -I"$D" $flags -DX_AT=$1 -DX_TM=$2 -DX_AP=$3 -DX_EN=$4 -DX_IO=$5 "$D/test_slave_defaults.c" 2>"$D/err"; then :; else echo "FAIL: $name"; head -3 "$D/err"; bad=1; fi
}
err() {   # name flags: this combination must NOT build (an explicit contradiction)
  name=$1; flags=$2; n=$((n+1))
  if gcc -std=c99 -fsyntax-only -I"$D" $flags -DX_AT=0 -DX_TM=0 -DX_AP=0 -DX_EN=0 -DX_IO=0 "$D/test_slave_defaults.c" 2>/dev/null; then echo "FAIL (it builds): $name"; bad=1; fi
}
P="-DSLAVE_POWER_STAGE=3 -DSLAVE_MASTER_CONTROL=1 -DSLAVE_LINK_V2=1"
chk "production build (stage 3, master control, link v2): everything on" "$P" 1 1 1 1 1
chk "stage 3 without a master: everything off" "-DSLAVE_POWER_STAGE=3" 0 0 0 0 0
chk "stage 2 with a master: everything off" "-DSLAVE_POWER_STAGE=2 -DSLAVE_MASTER_CONTROL=1 -DSLAVE_LINK_V2=1" 0 0 0 0 0
chk "stage 1 with a master: everything off" "-DSLAVE_POWER_STAGE=1 -DSLAVE_MASTER_CONTROL=1 -DSLAVE_LINK_V2=1" 0 0 0 0 0
chk "nothing defined: everything off" "" 0 0 0 0 0
chk "production, AUTOTUNE 0: the autotune switches all follow it" "$P -DSLAVE_AUTOTUNE=0" 0 0 0 0 1
chk "production, TUNE_APPLY 0: ENFORCE follows it" "$P -DSLAVE_TUNE_APPLY=0" 1 1 0 0 1
chk "production, TUNE_MODE 0: ENFORCE follows it" "$P -DSLAVE_TUNE_MODE=0" 1 0 1 0 1
chk "production, TUNE_ENFORCE 0: the rest stays on" "$P -DSLAVE_TUNE_ENFORCE=0" 1 1 1 0 1
chk "production, IOUT 0" "$P -DSLAVE_IOUT=0" 1 1 1 1 0
chk "production, everything explicitly 0" "$P -DSLAVE_AUTOTUNE=0 -DSLAVE_TUNE_MODE=0 -DSLAVE_TUNE_APPLY=0 -DSLAVE_TUNE_ENFORCE=0 -DSLAVE_IOUT=0" 0 0 0 0 0
chk "no master, but explicitly asked for: stays as asked (AUTOTUNE only)" "-DSLAVE_POWER_STAGE=3 -DSLAVE_AUTOTUNE=1 -DSLAVE_TUNE_MODE=0 -DSLAVE_TUNE_APPLY=0" 1 0 0 0 0
err "TUNE_ENFORCE 1 without TUNE_APPLY is refused at build time" "$P -DSLAVE_TUNE_APPLY=0 -DSLAVE_TUNE_ENFORCE=1"
err "TUNE_MODE 1 without AUTOTUNE is refused at build time" "$P -DSLAVE_AUTOTUNE=0 -DSLAVE_TUNE_MODE=1"
rm -rf "$D"
if [ $bad = 0 ]; then echo "  $n cases: ALL PASSED"; exit 0; else exit 1; fi
