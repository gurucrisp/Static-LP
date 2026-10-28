#!/usr/bin/env python3
"""Source-level checks of the slave's protection path (decision D25). Unit tests cannot see these: they are about WHAT THE SOURCE SAYS, not what the PC simulation does.

 1. every HAL_NVIC_SetPriority call in slave/Core/Src names its priority with IRQ_PRIO_* (no bare numbers);
 2. the FLT interrupt (EXTI2_3_IRQn) is the ONLY one at IRQ_PRIO_FLT, and the header's levels are strictly ordered;
 3. the FIRST statement of EXTI2_3_IRQHandler (stm32g0xx_it.c) is SlavePower_AllOffNow(); OR (v0.9.62, the FLT glitch filter) it is SlaveZC_FltIrqFiltered(); under
    #if (SLAVE_FLT_FILTER_US > 0U) ... with the plain SlavePower_AllOffNow(); first in its #else, and in slave_zc.c that function does nothing but wait, clear the edge
    flags and read the pin before its judge, and SlavePower_AllOffNow(); is the statement right after the judge;
 4. the FIRST statement of SlaveCtrl_OnFltIrq (slave_ctrl.c) is the FLT shutdown;
 3b. (v0.9.66) with the cycle-by-cycle limit the handler is '#if CBC SlaveZC_FltIrqCbc(); #elif FILTER SlaveZC_FltIrqFiltered(); #else the shutdown', the CBC handler's first
    statement is SlavePower_InverterForceOffNow(); and that function is exactly the two forced-inactive writes (TIM1, TIM15);
 5. (v0.9.65) the FLT shutdown is SlavePower_FltOffNow(): in slave_power.c, with SLAVE_FLT_KEEP_LOWS its first two statements force BOTH inverter channels inactive
    (TIM1 and TIM15 CCMR1, OC1M_2: high sides off at once, low sides on), then the converter off; without it, SlavePower_AllOffNow(); and SlavePower_InverterEnable()
    restores PWM mode 1 on both timers.
In rules 3 and 4 "the FLT shutdown" is SlavePower_FltOffNow(); (v0.9.65; before: SlavePower_AllOffNow();).

Exit code 0 = all fine. Run from anywhere: python3 tools/check_irq.py"""
import os, re, sys

root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
src = os.path.join(root, "slave", "Core", "Src")
inc = os.path.join(root, "slave", "Core", "Inc")
problems = []


def read(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def strip_comments(s):
    s = re.sub(r"/\*.*?\*/", "", s, flags=re.S)
    return re.sub(r"//[^\n]*", "", s)


def first_statement(code, func):
    """the first statement inside the body of 'func', skipping preprocessor lines and comments"""
    m = re.search(r"(?m)^[A-Za-z_][\w \t\*]*\b" + re.escape(func) + r"\s*\([^)]*\)\s*\{", code)
    if not m:
        return None
    body = code[m.end():]
    lines = [l.strip() for l in body.splitlines()]
    for l in lines:
        if not l or l.startswith("#"):
            continue
        return l.split(";")[0] + ";" if ";" in l else l
    return None


# the header
hdr = read(os.path.join(inc, "slave_irq_prio.h"))
lv = {k: int(v) for k, v in re.findall(r"#define\s+(IRQ_PRIO_\w+)\s+(\d+)U?", hdr)}
if not ({"IRQ_PRIO_FLT", "IRQ_PRIO_TIMING", "IRQ_PRIO_MEAS", "IRQ_PRIO_LINK"} <= set(lv)):
    problems.append("slave_irq_prio.h does not define IRQ_PRIO_FLT / _TIMING / _MEAS / _LINK")
elif not (lv["IRQ_PRIO_FLT"] < lv["IRQ_PRIO_TIMING"] < lv["IRQ_PRIO_MEAS"] < lv["IRQ_PRIO_LINK"] <= 3):
    problems.append("the priority levels are not strictly ordered FLT < TIMING < MEAS < LINK <= 3: %r" % lv)

# 1 and 2: every NVIC call
calls = []
for fn in sorted(os.listdir(src)):
    if not fn.endswith(".c"):
        continue
    code = strip_comments(read(os.path.join(src, fn)))
    for m in re.finditer(r"HAL_NVIC_SetPriority\s*\(\s*(\w+)\s*,\s*([^,]+?)\s*,\s*([^)]+?)\s*\)", code):
        calls.append((fn, m.group(1), m.group(2)))
for fn, irq, prio in calls:
    if not re.fullmatch(r"IRQ_PRIO_(FLT|TIMING|MEAS|LINK)", prio):
        problems.append("%s: HAL_NVIC_SetPriority(%s, %s, ...) uses a bare number or an unknown name" % (fn, irq, prio))
link = [irq for fn, irq, prio in calls if prio == "IRQ_PRIO_LINK"]
if any(irq != "USART1_IRQn" for irq in link):
    problems.append("IRQ_PRIO_LINK is for the link receive interrupt (USART1_IRQn) only, found: %r" % link)
flt = [irq for fn, irq, prio in calls if prio == "IRQ_PRIO_FLT"]
if flt != ["EXTI2_3_IRQn"]:
    problems.append("IRQ_PRIO_FLT must be used by EXTI2_3_IRQn and nothing else, found: %r" % flt)
if not any(irq == "EXTI2_3_IRQn" for _, irq, _ in calls):
    problems.append("no HAL_NVIC_SetPriority call for EXTI2_3_IRQn (the FLT interrupt) was found")

# 3 and 4: the first statement of the two FLT functions
SHUT = "SlavePower_FltOffNow();"
it = strip_comments(read(os.path.join(src, "stm32g0xx_it.c")))
fs = first_statement(it, "EXTI2_3_IRQHandler")
if fs == "SlaveZC_FltIrqCbc();":
    # v0.9.66: #if CBC -> SlaveZC_FltIrqCbc(); #elif FILTER -> SlaveZC_FltIrqFiltered(); #else the plain shutdown
    m = re.search(r"void\s+EXTI2_3_IRQHandler\s*\(\s*void\s*\)\s*\{(.*?)\n\}", it, flags=re.S)
    lines = [l.strip() for l in (m.group(1) if m else "").splitlines() if l.strip()]
    ok = (len(lines) >= 7 and lines[0].startswith("#if (SLAVE_FLT_CBC != 0)") and lines[1] == "SlaveZC_FltIrqCbc();" and lines[2].startswith("#elif (SLAVE_FLT_FILTER_US > 0U)")
          and lines[3] == "SlaveZC_FltIrqFiltered();" and lines[4] == "#else" and lines[5] == SHUT)
    if not ok:
        problems.append("EXTI2_3_IRQHandler: expected '#if (SLAVE_FLT_CBC != 0) SlaveZC_FltIrqCbc(); #elif (SLAVE_FLT_FILTER_US > 0U) SlaveZC_FltIrqFiltered(); #else %s ...'" % SHUT)
    zc = strip_comments(read(os.path.join(src, "slave_zc.c")))
    if first_statement(zc, "SlaveZC_FltIrqCbc") != "SlavePower_InverterForceOffNow();":
        problems.append("slave_zc.c SlaveZC_FltIrqCbc: the first statement must be SlavePower_InverterForceOffNow(); (found %r)" % first_statement(zc, "SlaveZC_FltIrqCbc"))
    pw0 = strip_comments(read(os.path.join(src, "slave_power.c")))
    m = re.search(r"void\s+SlavePower_InverterForceOffNow\s*\(\s*void\s*\)\s*\{(.*?)\n\}", pw0, flags=re.S)
    body = [l.strip() for l in (m.group(1) if m else "").splitlines() if l.strip()]
    if body != ["TIM1->CCMR1  = (TIM1->CCMR1  & ~TIM_CCMR1_OC1M) | TIM_CCMR1_OC1M_2;", "TIM15->CCMR1 = (TIM15->CCMR1 & ~TIM_CCMR1_OC1M) | TIM_CCMR1_OC1M_2;"]:
        problems.append("slave_power.c SlavePower_InverterForceOffNow: must be exactly the two forced-inactive writes (TIM1, TIM15), found %r" % body)
    # and the filtered path inside it must still be right
    fs = "SlaveZC_FltIrqFiltered();"
    it_filtered_checked_by_cbc = True
if fs == "SlaveZC_FltIrqFiltered();":
    m = re.search(r"void\s+EXTI2_3_IRQHandler\s*\(\s*void\s*\)\s*\{(.*?)\n\}", it, flags=re.S)
    body = m.group(1) if m else ""
    lines = [l.strip() for l in body.splitlines() if l.strip()]
    ok = (len(lines) >= 5 and lines[0].startswith("#if (SLAVE_FLT_FILTER_US > 0U)") and lines[1] == "SlaveZC_FltIrqFiltered();" and lines[2] == "#else"
          and lines[3] == SHUT)
    ok = ok or ("it_filtered_checked_by_cbc" in globals())          # v0.9.66: the layout was checked above with the CBC branch first
    if not ok:
        problems.append("EXTI2_3_IRQHandler: the filtered form must be '#if (SLAVE_FLT_FILTER_US > 0U) ... SlaveZC_FltIrqFiltered(); #else %s ...'" % SHUT)
    zc = strip_comments(read(os.path.join(src, "slave_zc.c")))
    m = re.search(r"void\s+SlaveZC_FltIrqFiltered\s*\(\s*void\s*\)\s*\{(.*?)\n\}", zc, flags=re.S)
    st = [x.strip() for x in (m.group(1) if m else "").split(";") if x.strip()]
    allowed = ("const uint32_t cyc", "uint8_t asserted", "FltFilter_Wait(", "__HAL_GPIO_EXTI_CLEAR_FALLING_IT(", "__HAL_GPIO_EXTI_CLEAR_RISING_IT(", "asserted = ")
    j = next((i for i, x in enumerate(st) if x.startswith("if (FltFilter_Judge(")), None)
    if (m is None) or (j is None) or any(not x.startswith(allowed) for x in st[:j]) or (j + 1 >= len(st)) or (st[j + 1] != SHUT[:-1]):
        problems.append("slave_zc.c SlaveZC_FltIrqFiltered: only the wait, the flag clears and the pin read before the judge, and %s right after it" % SHUT)
elif fs != SHUT:
    problems.append("EXTI2_3_IRQHandler: the first statement is %r, expected %r" % (fs, SHUT))
ct = strip_comments(read(os.path.join(src, "slave_ctrl.c")))
fs2 = first_statement(ct, "SlaveCtrl_OnFltIrq")
if fs2 != SHUT:
    problems.append("SlaveCtrl_OnFltIrq: the first statement is %r, expected %r" % (fs2, SHUT))

# 5: the FLT shutdown itself
pw = read(os.path.join(src, "slave_power.c"))
m = re.search(r"void\s+SlavePower_FltOffNow\s*\(\s*void\s*\)\s*\{(.*?)\n\}", strip_comments(pw), flags=re.S)
body = [l.strip() for l in (m.group(1) if m else "").splitlines() if l.strip()]
want = ["#if (SLAVE_FLT_KEEP_LOWS != 0)",
        "TIM1->CCMR1  = (TIM1->CCMR1  & ~TIM_CCMR1_OC1M) | TIM_CCMR1_OC1M_2;",
        "TIM15->CCMR1 = (TIM15->CCMR1 & ~TIM_CCMR1_OC1M) | TIM_CCMR1_OC1M_2;",
        "SlavePower_ConverterOffNow();", "#else", "SlavePower_AllOffNow();", "#endif"]
if body != want:
    problems.append("slave_power.c SlavePower_FltOffNow: expected exactly %r, found %r" % (want, body))
m = re.search(r"uint8_t\s+SlavePower_InverterEnable\s*\(\s*void\s*\)\s*\{(.*?)\n\}", strip_comments(pw), flags=re.S)
en = m.group(1) if m else ""
for t in ("TIM1", "TIM15"):
    if not re.search(re.escape(t) + r"->CCMR1\s*=\s*\(" + re.escape(t) + r"->CCMR1\s*&\s*~TIM_CCMR1_OC1M\)\s*\|\s*\(TIM_CCMR1_OC1M_2\s*\|\s*TIM_CCMR1_OC1M_1\)", en):
        problems.append("slave_power.c SlavePower_InverterEnable does not restore PWM mode 1 on %s (after SlavePower_FltOffNow forced it inactive)" % t)
if "#include \"slave_features.h\"" not in pw:
    problems.append("slave_power.c does not include slave_features.h: SLAVE_FLT_KEEP_LOWS would be undefined there")

if problems:
    print("PROTECTION PATH PROBLEMS:")
    for p in problems:
        print("  -", p)
    sys.exit(1)
print("OK: %d NVIC calls all use IRQ_PRIO_* names; FLT (EXTI2_3) alone at level %d, timing at %d, measurement at %d, link receive at %d; the FLT shutdown (high sides off, low sides on) is the first statement of both FLT functions"
      % (len(calls), lv["IRQ_PRIO_FLT"], lv["IRQ_PRIO_TIMING"], lv["IRQ_PRIO_MEAS"], lv["IRQ_PRIO_LINK"]))
