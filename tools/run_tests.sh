#!/bin/sh
# Host tests for the slave firmware logic (no hardware, needs gcc).
#   tools/run_tests.sh quick   (default)     tools/run_tests.sh full   (adds the slow simulations)     NPROC=8 tools/run_tests.sh = 8 parallel jobs
# Runs: regulator equivalence, link frame + polarity helper, master display text, and the sequencer + relay-rule suite
# for stage 1, stage 2 and stage 3 with EVERY combination of relay contact types, plus the hand-driven-relay builds.
MODE=${1:-quick}
ROOT=$(cd "$(dirname "$0")/.." && pwd); T=$ROOT/tests; S=$ROOT/slave/Core; OUT=$(mktemp -d)
INC="-I$S/Inc -I$T/stubs"
SRC="$S/Src/slave_ctrl.c $S/Src/slave_reg.c $S/Src/slave_relay.c"
fail=0
cd "$T" || exit 1

# the master display test uses the compose function cut out of the CURRENT master_monitor.c
awk '/^static void put_text\(/{f=1} /^void Mon_Compose\(/{g=1} (f||g){print} g&&/^}/{exit}' "$ROOT/master/Core/Src/master_monitor.c" > old_snippets/mon_compose.inc

line() { printf '  %-58s %s\n' "$1" "$2"; }
ok()   { echo "$1" | grep -q "RESULT: ALL PASSED\|BIT-IDENTICAL"; }

gcc -O0 -ffp-contract=off -w $INC test_reg_equiv.c "$S/Src/slave_reg.c" -o $OUT/reg -lm && r=$($OUT/reg | grep RESULT) ; ok "$r" && line "regulator equivalence (old code vs new)" PASS || { line "regulator equivalence" FAIL; fail=1; }
gcc -O1 -w $INC study_reg_plant.c "$S/Src/slave_reg.c" -o $OUT/regpl -lm && r=$($OUT/regpl | grep RESULT) ; ok "$r" && line "regulator vs the MEASURED plant (stability margin)" PASS || { line "regulator vs the MEASURED plant" FAIL; fail=1; }
gcc -O1 -w $INC test_reg_softstart.c "$S/Src/slave_reg.c" -o $OUT/regss -lm && r=$($OUT/regss | grep RESULT) ; ok "$r" && line "regulator soft-start ramp vs an independent model" PASS || { line "regulator soft-start ramp" FAIL; fail=1; }
gcc -O1 -w $INC study_reg_curve.c "$S/Src/slave_reg.c" -o $OUT/regcv -lm && r=$($OUT/regcv | grep RESULT) ; ok "$r" && line "regulator vs the MEASURED gain CURVE near 230 V (no hunting)" PASS || { line "regulator vs the MEASURED gain CURVE" FAIL; fail=1; }
gcc -O1 -w $INC test_meas_sign_link.c "$S/Src/link_proto.c" -o $OUT/lnk -lm && r=$($OUT/lnk | grep -E "RESULT") ; ok "$r" && line "polarity helper + link frame" PASS || { line "polarity helper + link frame" FAIL; fail=1; }
gcc -std=c99 -O2 -Wall -Wextra -I"$S/Inc" -I"$T" test_link_v2.c "$S/Src/link_v2.c" "$S/Src/link_proto.c" -o $OUT/lv2 && r=$($OUT/lv2 | grep -E "RESULT") ; ok "$r" && line "link library v2 (examples, ranges, fuzz, mutation)" PASS || { line "link library v2" FAIL; fail=1; }
gcc -std=c99 -O2 -Wall -Wextra -Wshadow -I"$S/Inc" test_slave_link2.c "$S/Src/slave_link2.c" "$S/Src/link_v2.c" "$S/Src/link_proto.c" -o $OUT/sl2 && r=$($OUT/sl2 | grep -E "RESULT") ; ok "$r" && line "slave link logic v2 (hello, E cadence, K retry, wrap)" PASS || { line "slave link logic v2" FAIL; fail=1; }
python3 "$ROOT/tools/check_version.py" > /dev/null 2>&1 && line "firmware version in the hello = VERSION file" PASS || { line "firmware version (tools/check_version.py)" FAIL; fail=1; }
MC_="$ROOT/master/Core"
gcc -std=c99 -O2 -Wall -Wextra -Wshadow -I"$MC_/Inc" test_master_store.c "$MC_/Src/m_store.c" "$MC_/Src/m_settings.c" "$MC_/Src/m_log.c" -o $OUT/mst && r=$($OUT/mst | grep -E "RESULT") ; ok "$r" && line "master core: storage, settings, fault log (power cuts, bit flips)" PASS || { line "master core: storage, settings, fault log" FAIL; fail=1; }
gcc -std=c99 -O2 -Wall -Wextra -Wshadow -I"$MC_/Inc" test_master_faults.c "$MC_/Src/m_faults.c" "$MC_/Src/m_settings.c" "$MC_/Src/m_store.c" "$MC_/Src/m_log.c" -o $OUT/mfa && r=$($OUT/mfa | grep -E "RESULT") ; ok "$r" && line "master core: fault and restart engine (scenarios, fuzz)" PASS || { line "master core: fault and restart engine" FAIL; fail=1; }
gcc -std=c99 -O2 -Wall -Wextra -Wshadow -Wno-unused-parameter -I"$MC_/Inc" test_master_ui.c "$MC_/Src/m_ui.c" "$MC_/Src/m_keys.c" "$MC_/Src/m_faults.c" "$MC_/Src/m_settings.c" "$MC_/Src/m_store.c" "$MC_/Src/m_log.c" -o $OUT/mui && r=$($OUT/mui | grep -E "RESULT") ; ok "$r" && line "master core: keys and screens (Set-Mode, pages, log viewer, fuzz)" PASS || { line "master core: keys and screens" FAIL; fail=1; }
gcc -std=c99 -O2 -Wall -Wextra -Wshadow -Wno-unused-parameter -I"$MC_/Inc" test_master_app.c "$MC_/Src/m_app.c" "$MC_/Src/m_ctl.c" "$MC_/Src/m_ui.c" "$MC_/Src/m_keys.c" "$MC_/Src/m_faults.c" "$MC_/Src/m_settings.c" "$MC_/Src/m_store.c" "$MC_/Src/m_log.c" -o $OUT/mapp && r=$($OUT/mapp | grep -E "RESULT") ; ok "$r" && line "master core: the application end to end (raw keys, flash, restarts)" PASS || { line "master core: the application end to end" FAIL; fail=1; }
# HAL names that exist in one STM32G0 HAL and not in another break the IDE build while the PC build passes: keep known offenders out of the sources
if grep -rn "FLASH_FLAG_ALL_ERRORS" "$ROOT/master/Core/Src" "$ROOT/slave/Core/Src" | grep -v "^[^:]*:[0-9]*:/\*" > /dev/null; then line "HAL names not in every G0 HAL (FLASH_FLAG_ALL_ERRORS)" FAIL; fail=1; else line "no HAL names that the G070 HAL lacks (FLASH_FLAG_ALL_ERRORS)" PASS; fi
gcc -std=c99 -O2 -Wall -Wextra -Wshadow -I"$MC_/Inc" test_master_ctl.c "$MC_/Src/m_ctl.c" "$MC_/Src/m_settings.c" "$MC_/Src/m_store.c" -o $OUT/mctl && r=$($OUT/mctl | grep -E "RESULT") ; ok "$r" && line "master core: what the master tells the slave (run, set voltage, clear counter)" PASS || { line "master core: what the master tells the slave" FAIL; fail=1; }
if grep -qE "^#define[[:space:]]+SLAVE_INJ_GUARD[[:space:]]+1" "$ROOT/slave/Core/Inc/slave_features.h" && grep -qE "^#define[[:space:]]+MASTER_TEST_NO_VOLTAGE_CUTS[[:space:]]+0" "$ROOT/master/Core/Inc/master_monitor.h"; then line "safe defaults: injection guard ON, master voltage cuts ON" PASS; else line "safe defaults (SLAVE_INJ_GUARD 1, MASTER_TEST_NO_VOLTAGE_CUTS 0)" FAIL; fail=1; fi
gcc -O1 -w $INC test_mon_state.c "$S/Src/link_proto.c" -o $OUT/mon && r=$($OUT/mon | grep -E "RESULT") ; ok "$r" && line "master display text" PASS || { line "master display text" FAIL; fail=1; }

# ready-made configs (configs/*.h): the structure checker must pass, and SLAVE_BENCH_AUTORUN must match the file name (a "...autorun0" file is the only one that
# may NOT start by itself: a generator slip once shipped files with autorun 0 that would never have started the converter)
cfg_bad=0; cfg_n=0
for f in "$ROOT"/configs/*.h; do
    cfg_n=$((cfg_n+1)); a=$(grep -E "^#define[[:space:]]+SLAVE_BENCH_AUTORUN[[:space:]]" "$f" | awk '{print $3}')
    case "$f" in *autorun0*|*SCOPE*|*MASTER*) want=0;; *) want=1;; esac
    python3 "$ROOT/tools/check_config.py" "$f" > /dev/null 2>&1 || { echo "    config structure problem: $f"; cfg_bad=1; }
    [ "$a" = "$want" ] || { echo "    autorun is '$a', expected $want: $f"; cfg_bad=1; }
    case "$f" in *PEAK*) grep -qE "^#define[[:space:]]+SLAVE_CONV_FIRST_AT_PEAK[[:space:]]+1" "$f" || { echo "    PEAK file without the peak setting: $f"; cfg_bad=1; };; esac
    case "$f" in *retry3s*) grep -qE "^#define[[:space:]]+CTRL_RESTART_STEP_MS[[:space:]]+3000UL" "$f" || { echo "    retry3s file without the 3 s retry: $f"; cfg_bad=1; };; esac
    case "$f" in *settle5s*) grep -qE "^#define[[:space:]]+SLAVE_LEGACY_W_SETTLE_MS[[:space:]]+5000U" "$f" || { echo "    settle5s file without the 5 s settle: $f"; cfg_bad=1; };; esac
    case "$f" in *STAGE2*) grep -qE "^#define[[:space:]]+SLAVE_POWER_STAGE[[:space:]]+2" "$f" || { echo "    STAGE2 file that is not stage 2: $f"; cfg_bad=1; };; esac
    case "$f" in *STAGE2*5pct*) grep -qE "^#define[[:space:]]+CTRL_STAGE2_DUTY_COUNTS[[:space:]]+160U" "$f" || { echo "    5pct file without 160 ticks: $f"; cfg_bad=1; };; esac
    case "$f" in *STAGE2*2pct*) grep -qE "^#define[[:space:]]+CTRL_STAGE2_DUTY_COUNTS[[:space:]]+64U" "$f" || { echo "    2pct file without duty 64: $f"; cfg_bad=1; };; esac
    case "$f" in *STAGE2*3pct*) grep -qE "^#define[[:space:]]+CTRL_STAGE2_DUTY_COUNTS[[:space:]]+96U" "$f" || { echo "    3pct file without duty 96: $f"; cfg_bad=1; };; esac
    case "$f" in *STAGE2*120V*) grep -qE "^#define[[:space:]]+CTRL_VIN_MIN_V[[:space:]]+80.0f" "$f" || { echo "    120V file without the 90 V window: $f"; cfg_bad=1; };; esac
    case "$f" in *STAGE3*) grep -qE "^#define[[:space:]]+SLAVE_POWER_STAGE[[:space:]]+3" "$f" || { echo "    STAGE3 file that is not stage 3: $f"; cfg_bad=1; };; esac
    case "$f" in *STAGE3*cap640*) grep -qE "^#define[[:space:]]+CTRL_STAGE3_MAX_EFF_TICKS[[:space:]]+640U" "$f" || { echo "    cap640 file without 640: $f"; cfg_bad=1; };; esac
    case "$f" in *STAGE3*cap1600*) grep -qE "^#define[[:space:]]+CTRL_STAGE3_MAX_EFF_TICKS[[:space:]]+1600U" "$f" || { echo "    cap1600 file without 1600: $f"; cfg_bad=1; };; esac
    case "$f" in *STAGE3*cap2000*) grep -qE "^#define[[:space:]]+CTRL_STAGE3_MAX_EFF_TICKS[[:space:]]+2000U" "$f" || { echo "    cap2000 file without 2000: $f"; cfg_bad=1; };; esac
    case "$f" in *fast_start*) { grep -qE "^#define[[:space:]]+CTRL_SOFTSTART_SLOW_SLOPE_VPS[[:space:]]+10.0f" "$f" && grep -qE "^#define[[:space:]]+CTRL_SOFTSTART_FAST_SLOPE_VPS[[:space:]]+30.0f" "$f" && grep -qE "^#define[[:space:]]+CTRL_SOFTSTART_FINAL_SLOPE_VPS[[:space:]]+10.0f" "$f" && grep -qE "^#define[[:space:]]+CTRL_SOFTSTART_SLOW_TIME_MS[[:space:]]+500UL" "$f"; } || { echo "    fast_start file without the four soft-start settings: $f"; cfg_bad=1; };; esac
    case "$f" in *STAGE3*cap2752*) grep -qE "^#define[[:space:]]+CTRL_STAGE3_MAX_EFF_TICKS[[:space:]]+2752U" "$f" || { echo "    cap2752 file without 2752: $f"; cfg_bad=1; };; esac
    case "$f" in *MASTER*) ;; *STAGE3*) grep -qE "^#define[[:space:]]+CTRL_VOUT_MAX_V[[:space:]]+2[56][05]\.0f" "$f" || { echo "    STAGE3 file without an output guard of 250, 260 or 265 V: $f"; cfg_bad=1; };; esac     # the master-control files keep the slave's built-in 280 V guard (the master owns the limits)
    case "$f" in *MASTER*) { grep -qE "^#define[[:space:]]+SLAVE_MASTER_CONTROL[[:space:]]+1" "$f" && grep -qE "^#define[[:space:]]+SLAVE_LINK_V2[[:space:]]+1" "$f"; } || { echo "    MASTER file without SLAVE_MASTER_CONTROL 1 and SLAVE_LINK_V2 1: $f"; cfg_bad=1; };; esac
    case "$f" in *MASTER*) grep -qE "^#define[[:space:]]+SLAVE_GLITCH_SKIP[[:space:]]+1" "$f" || { echo "    a master-control config without SLAVE_GLITCH_SKIP 1 (a rare zero-cross glitch would latch the load): $f"; cfg_bad=1; };; esac
    case "$f" in *MASTER*) if grep -qE "^#define[[:space:]]+(CTRL_VIN_MIN_V|CTRL_VIN_MAX_V|CTRL_VOUT_MAX_V)[[:space:]]" "$f"; then echo "    a master-control config narrows the slave's voltage envelope (the master owns the limits): $f"; cfg_bad=1; fi;; esac
    case "$f" in *NOCUTS*) ;; *) if grep -qE "^#define[[:space:]]+SLAVE_TEST_NO_VOLTAGE_CUTS[[:space:]]+1" "$f"; then echo "    the no-voltage-cuts TEST switch is on in a standard config: $f"; cfg_bad=1; fi;; esac
    case "$f" in *flt_latency*) grep -qE "^#define[[:space:]]+BRINGUP_FLT_TEST[[:space:]]+1" "$f" || { echo "    flt_latency file without BRINGUP_FLT_TEST 1: $f"; cfg_bad=1; };; esac
    case "$f" in *SCOPE*) { grep -qE "^#define[[:space:]]+SLAVE_POWER_STAGE[[:space:]]+0" "$f" && grep -qE "^#define[[:space:]]+BRINGUP_PWM_MODE[[:space:]]+1" "$f"; } || { echo "    SCOPE file must be stage 0 with the debug PWM: $f"; cfg_bad=1; };; esac
    case "$f" in *settle10s*) grep -qE "^#define[[:space:]]+SLAVE_LEGACY_W_SETTLE_MS[[:space:]]+10000U" "$f" || { echo "    settle10s file without the 10 s settle: $f"; cfg_bad=1; };; esac
done
[ $cfg_bad = 0 ] && line "ready-made configs ($cfg_n files): structure + autorun + settings" PASS || { line "ready-made configs" FAIL; fail=1; }

# shared source: these files are compiled by BOTH projects and by the PC tests, so the two copies must be byte-identical
shared_bad=0
for f in link_proto.h link_v2.h; do cmp -s "$ROOT/slave/Core/Inc/$f" "$ROOT/master/Core/Inc/$f" || { echo "    shared file differs between slave and master: $f"; shared_bad=1; }; done
for f in link_proto.c link_v2.c; do cmp -s "$ROOT/slave/Core/Src/$f" "$ROOT/master/Core/Src/$f" || { echo "    shared file differs between slave and master: $f"; shared_bad=1; }; done
[ $shared_bad = 0 ] && line "shared link sources identical in slave and master" PASS || { line "shared link sources" FAIL; fail=1; }

# protection path (D25): interrupt priorities named and ordered, FLT alone at the top, the shutdown the first statement of both FLT functions
if python3 "$ROOT/tools/check_irq.py" > $OUT/irq.txt 2>&1; then line "protection path: FLT highest priority, shutdown first" PASS; else line "protection path (tools/check_irq.py)" FAIL; cat $OUT/irq.txt; fail=1; fi

# source guard: the output contactor pins must be driven like the old firmware's start-up (CONT1 = PC0 HIGH = OFF). v0.7.x left them unconfigured (high impedance).
if grep -q "HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)" "$S/Src/main.c" && grep -q "GPIO_InitStruct.Pin   = GPIO_PIN_0;" "$S/Src/main.c"; then line "output contactor pins driven OFF at start-up (source guard)" PASS; else line "output contactor pins driven OFF at start-up (source guard)" FAIL; fail=1; fi

JOBS="$OUT/jobs.txt"; : > "$JOBS"
echo "s1 -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=1 -DSLAVE_RELAY_SH_CONTACT=1 -DSLAVE_RELAY_W_CONTACT=2" >> "$JOBS"
for st in 2 3; do for sh in 1 2 3; do for w in 1 2 3; do
  [ $sh = 3 ] && [ $w = 3 ] && continue                      # neither relay a shunt: a deliberate build error
  echo "s${st}_sh${sh}_w${w} -DSLAVE_POWER_STAGE=$st -DSLAVE_RELAYS_MODE=1 -DSLAVE_RELAY_SH_CONTACT=$sh -DSLAVE_RELAY_W_CONTACT=$w" >> "$JOBS"
done; done; done
echo "s1_manual -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=0" >> "$JOBS"
echo "s2_manual -DSLAVE_POWER_STAGE=2 -DSLAVE_RELAYS_MODE=0 -DSLAVE_RELAYS_CONFIRMED=1" >> "$JOBS"
echo "s3_manual -DSLAVE_POWER_STAGE=3 -DSLAVE_RELAYS_MODE=0 -DSLAVE_RELAYS_CONFIRMED=1" >> "$JOBS"
echo "s1_first_pos -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=0 -DSLAVE_CONV_FIRST_POL=1" >> "$JOBS"
echo "s1_first_neg -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=0 -DSLAVE_CONV_FIRST_POL=-1" >> "$JOBS"
echo "s1_legacy_first_neg -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=2 -DSLAVE_CONV_FIRST_POL=-1" >> "$JOBS"
echo "s3_legacy_softstart -DSLAVE_POWER_STAGE=3 -DSLAVE_RELAYS_MODE=2 -DCTRL_STAGE3_MAX_EFF_TICKS=2752U -DTEST_LEGACY_REG -DTEST_SOFTSTART" >> "$JOBS"
echo "s3_glitch_skip -DSLAVE_POWER_STAGE=3 -DSLAVE_RELAYS_MODE=2 -DCTRL_STAGE3_MAX_EFF_TICKS=2752U -DTEST_LEGACY_REG -DSLAVE_GLITCH_SKIP=1" >> "$JOBS"
echo "s3_legacy_fast_start -DSLAVE_POWER_STAGE=3 -DSLAVE_RELAYS_MODE=2 -DCTRL_STAGE3_MAX_EFF_TICKS=2752U -DCTRL_SOFTSTART_SLOW_SLOPE_VPS=10.0f -DCTRL_SOFTSTART_FAST_SLOPE_VPS=30.0f -DCTRL_SOFTSTART_FINAL_SLOPE_VPS=10.0f -DCTRL_SOFTSTART_SLOW_TIME_MS=500UL -DTEST_LEGACY_REG -DTEST_SOFTSTART" >> "$JOBS"
echo "s3_legacy_reg -DSLAVE_POWER_STAGE=3 -DSLAVE_RELAYS_MODE=2 -DTEST_LEGACY_REG" >> "$JOBS"
echo "s3_input_steps -DSLAVE_GLITCH_SKIP=1 -DSLAVE_POWER_STAGE=3 -DSLAVE_RELAYS_MODE=2 -DTEST_LEGACY_REG -DSLAVE_MASTER_CONTROL=1 -DSLAVE_LINK_V2=1 -DSLAVE_BENCH_AUTORUN=0 -DTEST_MC -DCTRL_STAGE3_MAX_EFF_TICKS=2752U -I$ROOT/master/Core/Inc $S/Src/slave_link2.c $S/Src/link_v2.c $S/Src/link_proto.c $ROOT/master/Core/Src/m_ctl.c $ROOT/master/Core/Src/m_settings.c $ROOT/master/Core/Src/m_store.c $ROOT/master/Core/Src/m_app.c $ROOT/master/Core/Src/m_ui.c $ROOT/master/Core/Src/m_keys.c $ROOT/master/Core/Src/m_faults.c $ROOT/master/Core/Src/m_log.c" >> "$JOBS"
echo "s3_no_voltage_cuts -DSLAVE_GLITCH_SKIP=1 -DSLAVE_POWER_STAGE=3 -DSLAVE_RELAYS_MODE=2 -DTEST_LEGACY_REG -DSLAVE_MASTER_CONTROL=1 -DSLAVE_LINK_V2=1 -DSLAVE_BENCH_AUTORUN=0 -DTEST_MC -DSLAVE_TEST_NO_VOLTAGE_CUTS=1 -I$ROOT/master/Core/Inc $S/Src/slave_link2.c $S/Src/link_v2.c $S/Src/link_proto.c $ROOT/master/Core/Src/m_ctl.c $ROOT/master/Core/Src/m_settings.c $ROOT/master/Core/Src/m_store.c $ROOT/master/Core/Src/m_app.c $ROOT/master/Core/Src/m_ui.c $ROOT/master/Core/Src/m_keys.c $ROOT/master/Core/Src/m_faults.c $ROOT/master/Core/Src/m_log.c" >> "$JOBS"
echo "s3_master_control -DSLAVE_GLITCH_SKIP=1 -DSLAVE_POWER_STAGE=3 -DSLAVE_RELAYS_MODE=2 -DTEST_LEGACY_REG -DSLAVE_MASTER_CONTROL=1 -DSLAVE_LINK_V2=1 -DSLAVE_BENCH_AUTORUN=0 -DTEST_MC -I$ROOT/master/Core/Inc $S/Src/slave_link2.c $S/Src/link_v2.c $S/Src/link_proto.c $ROOT/master/Core/Src/m_ctl.c $ROOT/master/Core/Src/m_settings.c $ROOT/master/Core/Src/m_store.c $ROOT/master/Core/Src/m_app.c $ROOT/master/Core/Src/m_ui.c $ROOT/master/Core/Src/m_keys.c $ROOT/master/Core/Src/m_faults.c $ROOT/master/Core/Src/m_log.c" >> "$JOBS"
echo "s2_legacy_inv -DSLAVE_POWER_STAGE=2 -DSLAVE_RELAYS_MODE=2 -DTEST_LEGACY_INV" >> "$JOBS"
echo "s1_peak -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=2 -DSLAVE_CONV_FIRST_AT_PEAK=1 -DTEST_FIRST_AT_PEAK" >> "$JOBS"
echo "s1_quick_retry -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=2 -DCTRL_RESTART_STEP_MS=3000UL -DCTRL_RESTART_MAX_MS=3000UL -DCTRL_LATCH_AFTER=10U -DTEST_QUICK_RETRY" >> "$JOBS"
echo "s1_settle5 -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=2 -DSLAVE_LEGACY_W_SETTLE_MS=5000U -DTEST_SETTLE_OVERRIDE" >> "$JOBS"
echo "s1_low_voltage -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=2 -DCTRL_VIN_MIN_V=80.0f -DTEST_LOW_VOLTAGE" >> "$JOBS"
echo "s1_legacy_relays -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=2 -DEXPECT_LEGACY_DEFAULTS" >> "$JOBS"
echo "s1_legacy_tf_off -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=2 -DSLAVE_TF_CHECK=0" >> "$JOBS"
echo "s1_legacy_lows_old -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=2 -DSLAVE_LOWSIDES_OFF_AFTER=2 -DSLAVE_CONV_FIRST_POL=0" >> "$JOBS"
echo "s1_legacy_lows_old_neg -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=2 -DSLAVE_LOWSIDES_OFF_AFTER=2 -DSLAVE_CONV_FIRST_POL=-1" >> "$JOBS"
echo "s1_legacy_no_lows -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=2 -DSLAVE_LEGACY_LOWSIDES=0" >> "$JOBS"
echo "s1_conv_swap -DSLAVE_POWER_STAGE=1 -DSLAVE_RELAYS_MODE=1 -DSLAVE_RELAY_SH_CONTACT=1 -DSLAVE_RELAY_W_CONTACT=2 -DSLAVE_CONV_SWAP=1" >> "$JOBS"
echo "s2_conv_swap -DSLAVE_POWER_STAGE=2 -DSLAVE_RELAYS_MODE=1 -DSLAVE_RELAY_SH_CONTACT=1 -DSLAVE_RELAY_W_CONTACT=2 -DSLAVE_CONV_SWAP=1" >> "$JOBS"

NPROC=${NPROC:-$(nproc 2>/dev/null || echo 4)}      # override with: NPROC=8 tools/run_tests.sh
echo "  running $(wc -l < "$JOBS") sequencer / relay-rule builds ($MODE mode, $NPROC in parallel) ..."
# run the builds in parallel batches (POSIX sh: background jobs + wait)
run_one() {
  name=$1; shift
  if gcc -O1 -w $INC -DBRINGUP_REPORT_MODE=2 "$@" test_ctrl.c $SRC -o "$OUT/$name" -lm 2>/dev/null; then
    # the low-voltage job changes the input window, so only its own test applies (older tests assume the 140 V default)
    only=""; [ "$name" = "s1_quick_retry" ] && only="quick retry"; [ "$name" = "s3_master_control" ] && only="master control"; [ "$name" = "s3_no_voltage_cuts" ] && only="no voltage cuts"; [ "$name" = "s3_input_steps" ] && only="input steps"; [ "$name" = "s3_glitch_skip" ] && only="glitch skip"; { [ "$name" = "s3_legacy_softstart" ] || [ "$name" = "s3_legacy_fast_start" ]; } && only="soft start"; [ "$name" = "s1_low_voltage" ] && only="low input voltage"; [ "$name" = "s1_settle5" ] && only="longer W-RLY settle"
    if [ "$MODE" = "full" ]; then r=$(ONLY="$only" timeout 900 "$OUT/$name" 2>&1); else r=$(ONLY="$only" SKIP_SLOW=1 timeout 900 "$OUT/$name" 2>&1); fi
    r=$(echo "$r" | grep -E "checks,|RESULT" | tr -s ' ' | tr '\n' ' ')
  else
    r="does not compile"
  fi
  case "$r" in *"ALL PASSED"*) echo "PASS $name $r" > "$OUT/$name.res";; *) echo "FAIL $name $r" > "$OUT/$name.res";; esac
}
n=0
while read -r name flags; do
  run_one "$name" $flags &
  n=$((n+1)); [ $((n % NPROC)) -eq 0 ] && wait
done < "$JOBS"
wait
for f in "$OUT"/*.res; do
  res=$(cat "$f"); case "$res" in PASS*) line "$(echo "$res" | cut -d' ' -f2)" "$(echo "$res" | cut -d' ' -f3-)";; *) line "$(echo "$res" | cut -d' ' -f2)" "$res"; fail=1;; esac
done
echo
[ $fail = 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"
rm -rf "$OUT"
exit $fail
