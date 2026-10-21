/* Tests of what changed in existing modules:
 *  1) SlaveMeas_SignFromRing (cut verbatim out of slave_meas.c): the integer polarity
 *     helper used in the zero-cross interrupt and by the 20 kHz guard.
 *  2) The link frame carrying the controller's state and fault code.
 * Build: gcc -O1 -Wall -I.. test_meas_sign_link.c ../link_proto.c -o t -lm */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "link_proto.h"

#define MEAS_BUF_FRAMES        1024U
#define MEAS_CHANNELS          3U
#define MEAS_SIGN_BACK_FRAMES  4U
#define MEAS_SIGN_SPAN_FRAMES  8U
#define MEAS_SIGN_MIN_COUNTS   20
#define MEAS_SIGN_MIN_SUM      (MEAS_SIGN_MIN_COUNTS * (int32_t)MEAS_SIGN_SPAN_FRAMES)
#define MEAS_GUARD_SPAN_FRAMES 2U
#define MEAS_GUARD_MIN_COUNTS  40

#include "old_snippets/sign_from_ring.inc"

static volatile uint16_t ring[MEAS_BUF_FRAMES * MEAS_CHANNELS];
static long checks, fails;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails < 15) { printf("  FAIL: " __VA_ARGS__); printf("\n"); } } } while (0)
static uint32_t rng = 7U;
static double noise(double a) { rng = rng * 1664525U + 1013904223U; return a * ((double)(rng >> 8) / 8388608.0 - 1.0); }

static double sample_at(uint32_t k, double amp, double phase)       /* 20 kHz, 50 Hz */
{
    return 2048.0 + amp * sin(2.0 * 3.14159265358979 * 50.0 * (double)k / 20000.0 + phase);
}

int main(void)
{
    uint32_t amps[3] = { 784, 1290, 1680 };      /* 140 V, 230 V, 300 V RMS */
    int ai;
    long wrong_sign = 0, zeros_when_big = 0, nonzero_when_tiny = 0, tested = 0;

    printf("Integer polarity helper (verbatim from slave_meas.c)\n");

    for (ai = 0; ai < 3; ai++)
    {
        double amp = (double)amps[ai];
        uint32_t fd;
        /* fill: frame f holds time f (sample index); channel 1 carries a DIFFERENT signal (opposite sign) */
        uint32_t f;
        for (f = 0; f < MEAS_BUF_FRAMES; f++)
        {
            ring[f * 3 + 0] = (uint16_t)lround(sample_at(f, amp, 0.0) + noise(4.0));
            ring[f * 3 + 1] = (uint16_t)lround(sample_at(f, amp, 3.14159265358979) + noise(4.0));
            ring[f * 3 + 2] = 1234U;
        }
        /* 1024 frames = 51.2 ms = 2.56 cycles: the ring is not a whole number of cycles, so the wrap point is a
         * discontinuity - exercise every frames_done including those whose window straddles it. */
        for (fd = 0; fd < MEAS_BUF_FRAMES; fd++)
        {
            int32_t dev; int8_t s;
            double avg = 0.0; uint32_t k;
            for (k = 0; k < MEAS_SIGN_SPAN_FRAMES; k++)
            {
                uint32_t idx = (fd + MEAS_BUF_FRAMES - 1U - MEAS_SIGN_BACK_FRAMES - k) % MEAS_BUF_FRAMES;
                avg += ring[idx * 3];
            }
            avg = avg / MEAS_SIGN_SPAN_FRAMES - 2048.0;
            s = SlaveMeas_SignFromRing(ring, MEAS_BUF_FRAMES, fd, MEAS_CHANNELS, 0U,
                                       2048 * (int32_t)MEAS_SIGN_SPAN_FRAMES, MEAS_SIGN_BACK_FRAMES,
                                       MEAS_SIGN_SPAN_FRAMES, MEAS_SIGN_MIN_SUM, &dev);
            tested++;
            CHECK(dev == (int32_t)lround(avg * MEAS_SIGN_SPAN_FRAMES) || labs(dev - (int32_t)lround(avg * MEAS_SIGN_SPAN_FRAMES)) <= 1,
                  "amp %u fd %u: deviation sum %d vs expected %.1f", amps[ai], fd, dev, avg * MEAS_SIGN_SPAN_FRAMES);
            if (avg >  MEAS_SIGN_MIN_COUNTS + 0.5) { if (s != 1)  wrong_sign++; if (s == 0) zeros_when_big++; }
            if (avg < -MEAS_SIGN_MIN_COUNTS - 0.5) { if (s != -1) wrong_sign++; if (s == 0) zeros_when_big++; }
            if (fabs(avg) < MEAS_SIGN_MIN_COUNTS - 0.5 && s != 0) nonzero_when_tiny++;
            /* the other channel (opposite sign signal) must not leak in */
            {
                int8_t s1 = SlaveMeas_SignFromRing(ring, MEAS_BUF_FRAMES, fd, MEAS_CHANNELS, 1U,
                                                   2048 * (int32_t)MEAS_SIGN_SPAN_FRAMES, MEAS_SIGN_BACK_FRAMES,
                                                   MEAS_SIGN_SPAN_FRAMES, MEAS_SIGN_MIN_SUM, 0);
                if (s != 0 && s1 != 0) CHECK(s1 == -s, "amp %u fd %u: channel isolation (ch0 %d ch1 %d)", amps[ai], fd, s, s1);
            }
        }
    }
    CHECK(wrong_sign == 0,        "%ld windows gave the wrong sign", wrong_sign);
    CHECK(zeros_when_big == 0,    "%ld windows well above the dead band returned 0", zeros_when_big);
    CHECK(nonzero_when_tiny == 0, "%ld windows inside the dead band returned a sign", nonzero_when_tiny);
    printf("  %ld windows over 3 amplitudes (140/230/300 V), every ring position incl. wrap: wrong signs %ld\n", tested, wrong_sign);

    /* bias error: a +/-12 count error in the stored bias must not flip the sign of a large signal */
    {
        uint32_t f; int8_t sp, sn;
        for (f = 0; f < MEAS_BUF_FRAMES; f++) { ring[f * 3] = (uint16_t)lround(2048.0 + 900.0); }      /* DC high */
        sp = SlaveMeas_SignFromRing(ring, MEAS_BUF_FRAMES, 500, 3, 0, (2048 + 12) * 8, 4, 8, 160, 0);
        sn = SlaveMeas_SignFromRing(ring, MEAS_BUF_FRAMES, 500, 3, 0, (2048 - 12) * 8, 4, 8, 160, 0);
        CHECK(sp == 1 && sn == 1, "a 12-count bias error must not change the sign of a 900-count signal");
        for (f = 0; f < MEAS_BUF_FRAMES; f++) { ring[f * 3] = 2048; }
        sp = SlaveMeas_SignFromRing(ring, MEAS_BUF_FRAMES, 500, 3, 0, 2048 * 8, 4, 8, 160, 0);
        CHECK(sp == 0, "a flat signal at the bias must give 0");
        printf("  bias error +/-12 counts tolerated on a large signal; flat signal -> 0\n");
    }

    /* the 2-sample GUARD variant: sign appears only beyond ~10 V (40 counts) of mains */
    {
        uint32_t f; int8_t s; int first_nonzero = -1, i;
        /* sine crossing zero upwards at frame 600 */
        for (f = 0; f < MEAS_BUF_FRAMES; f++) ring[f * 3] = (uint16_t)lround(sample_at((uint32_t)((int)f - 600 + 4000), 1290.0, 0.0) );
        for (i = 0; i < 40; i++)
        {
            s = SlaveMeas_SignFromRing(ring, MEAS_BUF_FRAMES, (uint32_t)(600 + i), 3, 0, 2048 * 2, 0U, 2U, 40 * 2, 0);
            if (s != 0 && first_nonzero < 0) first_nonzero = i;
        }
        CHECK(first_nonzero >= 0 && first_nonzero <= 8, "guard needs %d frames after the zero to see the sign (limit 8 = 400 us)", first_nonzero);
        printf("  guard variant: sign visible %d frames (%d us) after a zero crossing at 230 V\n", first_nonzero, first_nonzero * 50);
    }

    /* ---- link frame with the controller's state and fault ---- */
    printf("Link frame with state / fault\n");
    {
        int st, fc;
        for (st = 0; st <= 3; st++)
            for (fc = 0; fc <= 11; fc++)
            {
                char buf[LINK_STATUS_MAX_LEN + 8]; LinkStatus_t a, b; int32_t n; LinkRx_t rx; int32_t r = 0; int32_t i;
                memset(&a, 0, sizeof a);
                a.phase = 'R'; a.seq = (uint8_t)(st * 12 + fc); a.vin_dv = 2304; a.vout_dv = 2298; a.freq_chz = 5002;
                a.state = (uint8_t)st; a.flags = 0x13; a.fault = (uint16_t)fc; a.ntc_raw = 1871;
                n = Link_BuildStatus(buf, sizeof buf, &a);
                CHECK(n > 0 && n <= (int32_t)LINK_STATUS_MAX_LEN, "build failed or too long (%d)", n);
                memset(&rx, 0, sizeof rx);
                for (i = 0; i < n; i++) r = LinkRx_Feed(&rx, (uint8_t)buf[i], &b);
                CHECK(r == 1 || r > 0, "frame with state %d fault %d not accepted (r=%d)", st, fc, r);
                CHECK(b.state == a.state && b.fault == a.fault && b.vin_dv == a.vin_dv && b.seq == a.seq,
                      "state %d fault %d changed in transit (got %d/%d)", st, fc, b.state, b.fault);
            }
        printf("  48 state x fault combinations build, parse and round-trip intact\n");
        /* flags with the fault-context byte in bits 8..15 (up to 0xFFFF) must survive and stay within the frame limit */
        {
            unsigned ctx; long n_ok = 0;
            for (ctx = 0; ctx < 256; ctx++)
            {
                char buf[LINK_STATUS_MAX_LEN + 8]; LinkStatus_t a, b; int32_t n, i, r = 0; LinkRx_t rx;
                memset(&a, 0, sizeof a);
                a.phase = 'R'; a.seq = (uint8_t)ctx; a.vin_dv = 2304; a.vout_dv = 2298; a.freq_chz = 5002;
                a.state = 3; a.flags = (uint16_t)(0x13U | (ctx << 8)); a.fault = 11; a.ntc_raw = 4095;
                n = Link_BuildStatus(buf, sizeof buf, &a);
                CHECK(n > 0 && n <= (int32_t)LINK_STATUS_MAX_LEN, "ctx 0x%02X: frame length %d", ctx, n);
                memset(&rx, 0, sizeof rx);
                for (i = 0; i < n; i++) r = LinkRx_Feed(&rx, (uint8_t)buf[i], &b);
                CHECK(r > 0 && b.flags == a.flags && b.fault == 11 && b.state == 3, "ctx 0x%02X not intact (flags 0x%04X vs 0x%04X)", ctx, b.flags, a.flags);
                if (r > 0 && b.flags == a.flags) n_ok++;
            }
            printf("  all 256 fault-context bytes in flags bits 8..15 round-trip intact (%ld/256)\n", n_ok);
        }
        /* the OPTIONAL last field "diag": every value round-trips; a legacy frame without it still parses (diag = 0);
         * the wrong number of fields is rejected */
        {
            unsigned d; long ok2 = 0;
            for (d = 0; d <= 65535U; d += 257U)
            {
                char buf[LINK_STATUS_MAX_LEN + 8]; LinkStatus_t a, b; int32_t n, i, r = 0; LinkRx_t rx;
                memset(&a, 0, sizeof a);
                a.phase = 'B'; a.seq = 200; a.vin_dv = 9999; a.vout_dv = 12345; a.freq_chz = 6500; a.state = 2; a.flags = 0xFFFFU;
                a.fault = 65535U; a.ntc_raw = 4095; a.diag = (uint16_t)d; a.diag2 = (uint16_t)(65535U - d);
                n = Link_BuildStatus(buf, sizeof buf, &a);
                CHECK(n > 0 && n <= (int32_t)LINK_STATUS_MAX_LEN, "diag %u: worst-case frame length %d", d, n);
                memset(&rx, 0, sizeof rx);
                for (i = 0; i < n; i++) r = LinkRx_Feed(&rx, (uint8_t)buf[i], &b);
                CHECK(r > 0 && b.diag == a.diag && b.diag2 == a.diag2 && b.ntc_raw == 4095 && b.fault == 65535U, "diag %u / diag2 not intact (got %u / %u)", d, b.diag, b.diag2);
                if (r > 0 && b.diag == a.diag) ok2++;
            }
            printf("  optional diag field: %ld values round-trip intact in the worst-case frame\n", ok2);
        }
        {
            static const char *bodies[] = {
                "S,R,7,2304,2298,5002,1,19,0,1871",            /* legacy: no diag      -> valid, diag 0 */
                "S,R,7,2304,2298,5002,1,19,0,1871,513",        /* with diag            -> valid, diag 513 */
                "S,R,7,2304,2298,5002,1,19,0",                 /* ntc missing          -> rejected */
                "S,R,7,2304,2298,5002,1,19,0,1871,513,9",      /* with diag AND diag2   -> valid, diag 513, diag2 9 */
                "S,R,7,2304,2298,5002,1,19,0,1871,513,9,1",    /* one field too many   -> rejected */
                "S,R,7,2304,2298,5002,1,19,0,1871,",           /* empty diag           -> rejected */
            };
            static const int expect_ok[] = { 1, 1, 0, 1, 0, 0 };
            static const unsigned expect_diag[] = { 0, 513, 0, 513, 0, 0 };
            unsigned k;
            for (k = 0; k < 6; k++)
            {
                char frame[LINK_MAX_LINE]; LinkStatus_t b; LinkRx_t rx; int32_t r = 0; uint32_t i, L;
                uint8_t cs = Link_Checksum(bodies[k], (uint32_t)strlen(bodies[k]));
                L = (uint32_t)snprintf(frame, sizeof frame, "$%s*%02X\r\n", bodies[k], cs);
                memset(&rx, 0, sizeof rx); memset(&b, 0, sizeof b);
                for (i = 0; i < L; i++) { int32_t q = LinkRx_Feed(&rx, (uint8_t)frame[i], &b); if (q > 0) r = q; }
                CHECK((r > 0) == expect_ok[k], "frame %u ('%s'): expected %s", k, bodies[k], expect_ok[k] ? "valid" : "rejected");
                if (expect_ok[k]) CHECK(b.diag == expect_diag[k] && b.ntc_raw == 1871, "frame %u: diag %u ntc %u", k, b.diag, b.ntc_raw);
                if (k == 3) CHECK(b.diag2 == 9U, "frame 3: diag2 %u", b.diag2);
            }
            printf("  legacy 8- and 9-field frames still parse (diag / diag2 = 0); a missing ntc, 11 fields and an empty diag are rejected\n");
        }
    }

    printf("\n  %ld checks, %ld failures\n  RESULT: %s\n", checks, fails, fails ? "FAILURES" : "ALL PASSED");
    return fails ? 1 : 0;
}
