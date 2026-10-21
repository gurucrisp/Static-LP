#!/usr/bin/env python3
"""Checks a slave_config.h BEFORE you build it.

    python tools/check_config.py <path>/Core/Inc/slave_config.h

Reports: an unterminated /* comment, an #if / #ifdef / #ifndef without its #endif (the classic damage from editing the file: the include
guard's final #endif is the LAST line), a last line that is not #endif, more than one active CTRL_VIN_MIN_V line, and prints the main settings."""
import re, sys

def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    text = open(sys.argv[1], encoding="utf8", errors="replace").read()
    errors = []

    # 1) comments: find an unterminated /*
    pos, line_of = 0, lambda i: text.count("\n", 0, i) + 1
    clean = []
    while True:
        a = text.find("/*", pos)
        if a < 0:
            clean.append(text[pos:]); break
        b = text.find("*/", a + 2)
        clean.append(text[pos:a])
        if b < 0:
            errors.append("unterminated comment opened at line %d (a '/*' without its '*/'): everything after it, including the final #endif, is swallowed" % line_of(a))
            break
        inner = text.find("/*", a + 2, b)
        if inner >= 0:
            errors.append("line %d: a '/*' inside the comment that starts at line %d: that comment was probably left OPEN (its '*/' is missing) and "
                          "swallowed the lines in between" % (line_of(inner), line_of(a)))
        clean.append(re.sub(r"[^\n]", " ", text[a:b + 2]))        # keep the line numbers
        pos = b + 2
    code = "".join(clean)

    # 2) conditional balance
    stack, last_directive = [], None
    for n, raw in enumerate(code.split("\n"), 1):
        m = re.match(r"\s*#\s*(\w+)", raw)
        if not m:
            continue
        d = m.group(1)
        last_directive = (n, d)
        if d in ("if", "ifdef", "ifndef"):
            stack.append((n, raw.strip()))
        elif d == "endif":
            if not stack:
                errors.append("line %d: #endif without a matching #if" % n)
            else:
                stack.pop()
    for n, raw in stack:
        errors.append("line %d: '%s' is never closed by an #endif%s" % (n, raw, "  <- this is the include guard: the file's LAST line must be  #endif /* SLAVE_CONFIG_H */" if "SLAVE_CONFIG_H" in raw else ""))

    # 3) the last directive must be the guard's #endif
    if not stack and last_directive and last_directive[1] != "endif":
        errors.append("the last preprocessor line (%d) is '#%s', not the final #endif" % last_directive)

    # 4) active CTRL_VIN_MIN_V lines
    vin = [(n, raw.strip()) for n, raw in enumerate(code.split("\n"), 1) if re.match(r"\s*#\s*define\s+CTRL_VIN_MIN_V\b", raw)]
    if len(vin) > 1:
        errors.append("%d active CTRL_VIN_MIN_V lines (%s): only ONE may be active" % (len(vin), ", ".join("line %d" % n for n, _ in vin)))

    # 5) the main settings
    print("settings found in %s:" % sys.argv[1])
    for key in ("SLAVE_POWER_STAGE", "SLAVE_RELAYS_MODE", "SLAVE_BENCH_AUTORUN", "BRINGUP_PWM_MODE", "BRINGUP_REPORT_MODE", "BRINGUP_FLT_TEST", "SLAVE_CONV_SWAP"):
        hit = [(n, re.sub(r"\s+", " ", raw.strip())) for n, raw in enumerate(code.split("\n"), 1) if re.match(r"\s*#\s*define\s+%s\b" % key, raw)]
        print("  %-22s %s" % (key, ("line %d: %s" % hit[0]) if hit else "(not set here: the default applies)"))
    print("  %-22s %s" % ("CTRL_VIN_MIN_V", ("line %d: %s" % vin[0]) if vin else "(default: trip below 140 V, start from 150 V)"))

    # 6) an older config does not define SLAVE_TF_CHECK: the transformer-connection check (v0.6.11) is then silently OFF
    if not re.search(r"^\s*#\s*define\s+SLAVE_TF_CHECK\b", code, flags=re.M) and "SLAVE_TF_CHECK" not in code:
        print("\nNOTE: this file does not know SLAVE_TF_CHECK (a config from before v0.6.11): the transformer-connection check will be OFF.")
        print("      Use a ready-made config from slave_config_ready_made.zip (or the slave_config.h of the current release).")

    if "SLAVE_CONV_FIRST_AT_PEAK" not in code:
        print("\nNOTE: this file does not know SLAVE_CONV_FIRST_AT_PEAK (a config from before v0.6.15): the peak-start experiment option is not available with it.")
    for key in ("SLAVE_LEGACY_W_SETTLE_MS", "SLAVE_CONV_FIRST_AT_PEAK", "CTRL_RESTART_STEP_MS", "CTRL_LATCH_AFTER", "CTRL_STAGE2_DUTY_COUNTS", "CTRL_VIN_MIN_V", "CTRL_VIN_MAX_V", "CTRL_VOUT_MAX_V", "CTRL_STAGE3_MAX_EFF_TICKS", "CTRL_SOFTSTART_SLOW_SLOPE_VPS", "CTRL_SOFTSTART_FAST_SLOPE_VPS", "CTRL_SOFTSTART_FINAL_SLOPE_VPS", "CTRL_SOFTSTART_SLOW_TIME_MS"):
        hit = [(n, re.sub(r"\s+", " ", raw.strip())) for n, raw in enumerate(code.split("\n"), 1) if re.match(r"\s*#\s*define\s+%s\b" % key, raw)]
        if hit and key not in ("SLAVE_CONV_FIRST_AT_PEAK", "CTRL_VIN_MIN_V", "CTRL_STAGE2_DUTY_COUNTS", "CTRL_VIN_MAX_V", "CTRL_VOUT_MAX_V", "CTRL_STAGE3_MAX_EFF_TICKS", "CTRL_SOFTSTART_SLOW_SLOPE_VPS", "CTRL_SOFTSTART_FAST_SLOPE_VPS", "CTRL_SOFTSTART_FINAL_SLOPE_VPS", "CTRL_SOFTSTART_SLOW_TIME_MS") or (hit and "#ifndef" not in code.split("\n")[hit[0][0]-2]):
            print("  %-26s line %d: %s" % (key, hit[0][0], hit[0][1]))

    if errors:
        print("\nPROBLEMS - do NOT build this file:")
        for e in errors:
            print("  - " + e)
        sys.exit(1)
    print("\nOK: the preprocessor structure is intact (%d lines)." % (text.count("\n") + 1))

main()
