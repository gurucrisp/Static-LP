/* Tests Mon_Compose (cut verbatim out of master_monitor.c) for the new state / fault line. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "link_proto.h"
#define MON_LINK_TIMEOUT_MS 500U
#define MON_LCD_COLS 20U
#define MON_LCD_ROWS 4U
#include "old_snippets/mon_struct.inc"
#include "old_snippets/mon_compose.inc"

static long checks, fails;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("  FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

static void show(const char *tag, const MasterMon_t *m)
{
    char rows[MON_LCD_ROWS][MON_LCD_COLS + 1U]; int r;
    Mon_Compose(m, 1000U, rows);
    printf("  %s\n", tag);
    for (r = 0; r < 4; r++) printf("     |%s|\n", rows[r]);
    for (r = 0; r < 4; r++) CHECK(strlen(rows[r]) == MON_LCD_COLS, "row %d is %zu chars", r, strlen(rows[r]));
}

static const char *pad20(const char *t) { static char b[8][24]; static int k; char *o = b[k++ & 7]; snprintf(o, 24, "%-20s", t); return o; }

int main(void)
{
    MasterMon_t m; char rows[4][21];
    memset(&m, 0, sizeof m);
    m.have_frame = 1U; m.t_last_ms = 900U;
    m.last.phase = 'R'; m.last.vin_dv = 2304; m.last.vout_dv = 2298; m.last.freq_chz = 5002;
    m.last.flags = LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID;

    printf("Master display with the slave's power-stage state\n");
    m.last.state = 0; m.last.fault = 0;  show("state 0, fault 0 (power stage off / stopped): unchanged", &m);
    Mon_Compose(&m, 1000U, rows); CHECK(strncmp(rows[0], "SLAVE R LINK OK RAW", 19) == 0, "unchanged row 0: '%s'", rows[0]);
    m.last.state = 1; m.last.fault = 0;  show("starting", &m);
    Mon_Compose(&m, 1000U, rows); CHECK(strncmp(rows[0], "SLAVE R START F00", 17) == 0, "row 0: '%s'", rows[0]);
    m.last.state = 2; m.last.fault = 0;  show("running", &m);
    Mon_Compose(&m, 1000U, rows); CHECK(strncmp(rows[0], "SLAVE R RUN   F00", 17) == 0, "row 0: '%s'", rows[0]);
    m.last.state = 3; m.last.fault = 8;  show("fault 8 (ZC lost)", &m);
    Mon_Compose(&m, 1000U, rows); CHECK(strncmp(rows[0], "SLAVE R FAULT F08", 17) == 0, "row 0: '%s'", rows[0]);
    m.last.state = 3; m.last.fault = 300; Mon_Compose(&m, 1000U, rows); CHECK(strncmp(rows[0], "SLAVE R FAULT F99", 17) == 0, "fault capped at 99: '%s'", rows[0]);
    m.last.state = 7; m.last.fault = 0;   Mon_Compose(&m, 1000U, rows); CHECK(strncmp(rows[0], "SLAVE R ?", 9) == 0, "unknown state: '%s'", rows[0]);
    /* fault context on row 3 (flags bits 8..15; bit 15 = valid) */
    m.last.state = 3; m.last.fault = 2;
    m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID | ((0x80U | 0x20U | (1U << 3) | 2U) << 8));   /* CONV, mode B, FLT low, BKIN high */
    show("fault with context: converter state, FLT still LOW", &m);
    Mon_Compose(&m, 1000U, rows);
    CHECK(strncmp(rows[0], "SLAVE R FAULT F02", 17) == 0, "row 0: '%s'", rows[0]);
    CHECK(strcmp(rows[3], "@CONV M:B FLT:L BK:H") == 0, "row 3: '%s'", rows[3]);
    m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID | ((0x80U | 0x40U | (2U << 3) | 4U) << 8));   /* UNSH, mode F, FLT high, BKIN low */
    Mon_Compose(&m, 1000U, rows);
    CHECK(strcmp(rows[3], "@UNSH M:F FLT:H BK:L") == 0, "row 3: '%s'", rows[3]);
    m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID | ((0x80U | 0x00U | (1U << 3) | 6U) << 8));   /* RSHT */
    Mon_Compose(&m, 1000U, rows);
    CHECK(strcmp(rows[3], "@RSHT M:B FLT:H BK:H") == 0, "row 3: '%s'", rows[3]);
    m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID | ((0x80U | 0x00U | (1U << 3) | 3U) << 8));   /* INVS */
    Mon_Compose(&m, 1000U, rows);
    CHECK(strcmp(rows[3], "@INVS M:B FLT:H BK:H") == 0, "row 3: '%s'", rows[3]);
    m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID);      /* fault but no context: normal row 3 */
    Mon_Compose(&m, 1000U, rows);
    CHECK(strncmp(rows[3], "50.02Hz", 7) == 0, "without a context the normal row 3 stays: '%s'", rows[3]);
    /* bench diagnostics counters on rows 1 and 2: F<idle>/<running>, K<idle>/<running>; CLIP wins if both are shown */
    m.last.state = 1; m.last.fault = 0; m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID);
    m.last.diag = (uint16_t)(3U | (1U << 5) | (0U << 10) | (2U << 13));
    Mon_Compose(&m, 1000U, rows);
    CHECK(strncmp(rows[1], "Vin  ", 5) == 0 && strncmp(&rows[1][13], "F3/1", 4) == 0, "row 1: '%s'", rows[1]);
    CHECK(strncmp(&rows[2][13], "K0/2", 4) == 0, "row 2: '%s'", rows[2]);
    m.last.diag = 0;
    Mon_Compose(&m, 1000U, rows);
    CHECK(rows[1][13] == ' ' && rows[2][13] == ' ', "no counters shown when nothing was seen");
    m.last.diag = 0x0400U;                                   /* break idle 1 only: K shown, F not */
    Mon_Compose(&m, 1000U, rows);
    CHECK(rows[1][13] == ' ' && strncmp(&rows[2][13], "K1/0", 4) == 0, "only the break counter: '%s' / '%s'", rows[1], rows[2]);
    m.last.diag = 0;

    /* passive mapping check on row 3 (non-fault states): M<agree>/<disagree>; a trip word (bit 15) never shows it */
    m.last.state = 0; m.last.fault = 0; m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID);
    m.last.diag2 = (uint16_t)(0x4000U | 99U | (0U << 7));
    m.t_last_ms = 900U; Mon_Compose(&m, 1000U, rows);
    CHECK(strncmp(rows[3], "50.02Hz M99/0 L0", 16) == 0, "mapping on row 3: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x4000U | 0U | (90U << 7));
    Mon_Compose(&m, 1000U, rows);
    CHECK(strncmp(rows[3], "50.02Hz M0/90 L0", 16) == 0, "inverted mapping on row 3: '%s'", rows[3]);
    m.last.diag2 = 0;
    Mon_Compose(&m, 1000U, rows);
    CHECK(strncmp(rows[3], "50.02Hz L0 B0 f0", 16) == 0, "no mapping data: the normal statistics: '%s'", rows[3]);

    /* stage 3 regulator state: BOOST / BUCK / FREEZE page on every other 2 s slot (diag2 bit 12 set, bits 15..13 clear) */
    m.last.state = 2; m.last.fault = 0; m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID);
    m.last.diag2 = (uint16_t)(0x1000U | 0x0800U | 116U);                                    /* boost, 116 x 4 = 464 ticks */
    m.t_last_ms = 900U;  Mon_Compose(&m, 1000U, rows); CHECK(strncmp(rows[3], "50.02Hz L0 B0 f0", 16) == 0, "reg page: even 2 s slot shows the normal row: '%s'", rows[3]);
    m.t_last_ms = 1900U; Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], pad20("BOOST 464t")) == 0, "boost: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x1000U | 78U);                                                /* buck, 312 ticks */
    Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], pad20("BUCK 312t")) == 0, "buck: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x1000U | 0x0800U | 0x0400U | 160U);                          /* boost AT THE CAP, 640 ticks */
    Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], pad20("BOOST 640t CAP")) == 0, "boost at the cap: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x1000U | 0x0800U);                                            /* frozen: zero ticks */
    Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], pad20("FREEZE")) == 0, "frozen: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x1000U | 0x0800U | 1023U);                                    /* the largest value */
    Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], pad20("BOOST 4092t")) == 0, "largest: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x2000U | (1U << 11) | 32U);                                  /* a transformer word is still the TF page, not this one */
    Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], "TF OK +3.2V         ") == 0, "the TF page is not mistaken for the regulator page: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x1000U | 0x0800U | 116U);                                    /* a fault page wins over the regulator page */
    m.last.state = 3; m.last.fault = 2; m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID | ((0x80U | 0x00U | (1U << 3) | 2U) << 8));
    Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], "@CONV M:B FLT:H BK:H") == 0, "fault page wins over the regulator page: '%s'", rows[3]);
    m.last.state = 1; m.last.fault = 0; m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID);
    m.last.diag2 = 0;

    /* transformer-connection check: TF page on every other 2 s slot (diag2 bit 13 set, bits 15 and 14 clear) */
    m.last.state = 1; m.last.fault = 0; m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID);
    m.last.diag2 = (uint16_t)(0x2000U | (1U << 11) | 32U);                                  /* OK, +3.2 V */
    m.t_last_ms = 900U;  Mon_Compose(&m, 1000U, rows); CHECK(strncmp(rows[3], "50.02Hz L0 B0 f0", 16) == 0, "even 2 s slot: the normal row: '%s'", rows[3]);
    m.t_last_ms = 1900U; Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], "TF OK +3.2V         ") == 0, "odd 2 s slot: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x2000U | (2U << 11) | ((uint16_t)(-30) & 0x07FFU));          /* FLIPPED, -3.0 V */
    Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], "TF FLIP -3.0V       ") == 0, "flipped: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x2000U | (3U << 11) | 2U);                                    /* inconclusive, +0.2 V */
    Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], "TF ? +0.2V          ") == 0, "inconclusive: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x2000U | (1U << 11) | 1023U);                                 /* the extremes */
    Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], "TF OK +102.3V       ") == 0, "positive extreme: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x2000U | (2U << 11) | 0x0400U);
    Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], "TF FLIP -102.4V     ") == 0, "negative extreme: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x2000U | (1U << 11) | 0U);                                    /* exactly 0.0 V */
    Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], "TF OK +0.0V         ") == 0, "zero: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x4000U | 99U);                                                /* the mapping check is not mistaken for it */
    Mon_Compose(&m, 2000U, rows); CHECK(strncmp(rows[3], "50.02Hz M99/0 L0", 16) == 0, "mapping word on an odd slot: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x8000U | 0x2000U | 5U);                                       /* a trip word is not mistaken for it either */
    m.last.state = 3; m.last.fault = 2; m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID | ((0x80U | 0x00U | (1U << 3) | 2U) << 8));
    Mon_Compose(&m, 2000U, rows); CHECK(strncmp(rows[3], "dt=", 3) == 0, "trip word on an odd slot: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x2000U | (1U << 11) | 32U);                                  /* the fault page replaces the TF row */
    Mon_Compose(&m, 2000U, rows); CHECK(strcmp(rows[3], "@CONV M:B FLT:H BK:H") == 0, "fault page wins over the TF page: '%s'", rows[3]);
    m.last.state = 1; m.last.fault = 0; m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID);
    m.last.diag2 = 0;

    /* fault row alternates (every 2 s) with the trip timing: dt=<ms> D<+|-|0> <ON|off> */
    m.last.state = 3; m.last.fault = 2;
    m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID | ((0x80U | 0x00U | (1U << 3) | 2U) << 8));
    m.last.diag2 = (uint16_t)(0x8000U | 0x0400U | 3U);                 /* 3 x 16 us = 48 us after the event, positive diagonal on */
    m.t_last_ms = 3900U; Mon_Compose(&m, 4000U, rows);          CHECK(strcmp(rows[3], "@CONV M:B FLT:H BK:H") == 0, "even 2 s slot shows the context: '%s'", rows[3]);
    m.t_last_ms = (2000U) - ((2000U) > 0U ? 100U : 0U); Mon_Compose(&m, 2000U, rows);       CHECK(strcmp(rows[3], "dt=0.04ms D+ ON     ") == 0, "odd 2 s slot shows the timing: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x8000U | 0x0800U | 607U);               /* 607 x 16 = 9712 us, negative diagonal */
    m.t_last_ms = (4000U + 2000U) - ((4000U + 2000U) > 0U ? 100U : 0U); Mon_Compose(&m, 4000U + 2000U, rows); CHECK(strcmp(rows[3], "dt=9.71ms D- ON     ") == 0, "negative half near turn-off: '%s'", rows[3]);
    m.last.diag2 = (uint16_t)(0x8000U | 100U);                          /* no diagonal on */
    m.t_last_ms = (2000U) - ((2000U) > 0U ? 100U : 0U); Mon_Compose(&m, 2000U, rows);       CHECK(strcmp(rows[3], "dt=1.60ms D0 off    ") == 0, "no diagonal energised: '%s'", rows[3]);
    m.last.diag2 = 0;                                                    /* nothing captured: the context stays on both slots */
    m.t_last_ms = (2000U) - ((2000U) > 0U ? 100U : 0U); Mon_Compose(&m, 2000U, rows);       CHECK(strcmp(rows[3], "@CONV M:B FLT:H BK:H") == 0, "no capture: context on the odd slot too: '%s'", rows[3]);
    m.last.state = 1; m.last.fault = 0; m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID);
    m.t_last_ms = 900U;                                                  /* back to the time base the other cases use */

    /* waiting in SYNC: row 3 lists what is missing */
    m.last.state = 1; m.last.fault = 0;
    m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID | ((0x80U | 0x10U) << 8));            /* B */
    show("START, waiting: a break input reads low", &m);
    Mon_Compose(&m, 1000U, rows);
    CHECK(strcmp(rows[0], "SLAVE R START F00   ") == 0, "row 0: '%s'", rows[0]);
    CHECK(strcmp(rows[3], "WAIT: B             ") == 0, "row 3: '%s'", rows[3]);
    m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID | ((0x80U | 0x01U | 0x08U | 0x10U) << 8)); /* Z F B */
    Mon_Compose(&m, 1000U, rows);
    CHECK(strcmp(rows[3], "WAIT: Z F B         ") == 0, "row 3: '%s'", rows[3]);
    m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID | ((0x80U | 0x7FU) << 8));            /* all seven */
    Mon_Compose(&m, 1000U, rows);
    CHECK(strcmp(rows[3], "WAIT: Z V M F B O R ") == 0, "row 3 (all): '%s'", rows[3]);
    m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID | (0x80U << 8));                     /* nothing missing */
    Mon_Compose(&m, 1000U, rows);
    CHECK(strcmp(rows[3], "WAIT: -             ") == 0, "row 3: '%s'", rows[3]);
    m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID);                                     /* START without a wait byte: normal row 3 */
    Mon_Compose(&m, 1000U, rows);
    CHECK(strncmp(rows[3], "50.02Hz", 7) == 0, "START without context: normal row 3, got '%s'", rows[3]);
    m.last.state = 2; m.last.fault = 0; m.last.flags = (uint16_t)(LINK_FLAG_MEAS_VALID | LINK_FLAG_FREQ_VALID);
    /* rows 1..3 must be unaffected by the state */
    m.last.state = 2; Mon_Compose(&m, 1000U, rows);
    CHECK(strncmp(rows[1], "Vin  230.4 V", 12) == 0 && strncmp(rows[2], "Vout 229.8 V", 12) == 0 && strncmp(rows[3], "50.02Hz", 7) == 0, "rows 1-3 changed");
    printf("\n  %ld checks, %ld failures\n  RESULT: %s\n", checks, fails, fails ? "FAILURES" : "ALL PASSED");
    return fails != 0;
}
