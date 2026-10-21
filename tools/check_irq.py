#!/usr/bin/env python3
"""Source-level checks of the slave's protection path (decision D25). Unit tests cannot see these: they are about WHAT THE SOURCE SAYS, not what the PC simulation does.

 1. every HAL_NVIC_SetPriority call in slave/Core/Src names its priority with IRQ_PRIO_* (no bare numbers);
 2. the FLT interrupt (EXTI2_3_IRQn) is the ONLY one at IRQ_PRIO_FLT, and the header's levels are strictly ordered;
 3. the FIRST statement of EXTI2_3_IRQHandler (stm32g0xx_it.c) is SlavePower_AllOffNow();
 4. the FIRST statement of SlaveCtrl_OnFltIrq (slave_ctrl.c) is SlavePower_AllOffNow();

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
it = strip_comments(read(os.path.join(src, "stm32g0xx_it.c")))
fs = first_statement(it, "EXTI2_3_IRQHandler")
if fs != "SlavePower_AllOffNow();":
    problems.append("EXTI2_3_IRQHandler: the first statement is %r, expected 'SlavePower_AllOffNow();'" % fs)
ct = strip_comments(read(os.path.join(src, "slave_ctrl.c")))
fs2 = first_statement(ct, "SlaveCtrl_OnFltIrq")
if fs2 != "SlavePower_AllOffNow();":
    problems.append("SlaveCtrl_OnFltIrq: the first statement is %r, expected 'SlavePower_AllOffNow();'" % fs2)

if problems:
    print("PROTECTION PATH PROBLEMS:")
    for p in problems:
        print("  -", p)
    sys.exit(1)
print("OK: %d NVIC calls all use IRQ_PRIO_* names; FLT (EXTI2_3) alone at level %d, timing at %d, measurement at %d, link receive at %d; the shutdown is the first statement of both FLT functions"
      % (len(calls), lv["IRQ_PRIO_FLT"], lv["IRQ_PRIO_TIMING"], lv["IRQ_PRIO_MEAS"], lv["IRQ_PRIO_LINK"]))
