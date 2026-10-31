/* Tests for the slave's flash driver (slave_flash.c) against a model of the HAL flash API. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "main.h"
#include "slave_flash.h"
static int tot = 0, bad = 0;
#define CHECK(c, ...) do { tot++; if (!(c)) { bad++; printf("      FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
FLASH_TypeDef flash_m; uint32_t _sidata, _sdata, _edata;
static uint8_t mem[0x80000]; static uint32_t image_end = 0x08020000UL; static int unlocked = 0, last_banks = 0; static uint32_t last_page = 0; static int erase_calls = 0, prog_calls = 0, fail_next_prog = 0;
uint32_t SlaveFlash_ImageEnd(void) { return image_end; }
uint8_t SlaveFlashTest_Rd8(uint32_t a) { return mem[a - 0x08000000UL]; }
static int erase_noop = 0;
HAL_StatusTypeDef HAL_FLASH_Unlock(void) { unlocked = 1; return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { unlocked = 0; return HAL_OK; }
/* v0.9.79: a model of the option-byte HAL: OBProgram changes only USERType's bits of OPTR (as the G0 HAL does for OPTIONBYTE_USER, keeping the read protection);
 * Launch would reset the MCU: the model counts it and returns, the test then boots again with OBLRSTF set */
RCC_TypeDef rcc_m; static int ob_unlocked = 0, ob_writes = 0, ob_launches = 0, ob_fail = 0, ob_ignore = 0; static uint32_t ob_last_type = 0U;
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void) { if (!unlocked) return HAL_ERROR; ob_unlocked = 1; return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void) { ob_unlocked = 0; return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_OB_Launch(void) { if (!ob_unlocked) return HAL_ERROR; ob_launches++; return HAL_OK; }
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *ob)
{
    if (!ob_unlocked || ob->OptionType != OPTIONBYTE_USER || ob_fail) return HAL_ERROR;
    ob_writes++; ob_last_type = ob->USERType;
    if (!ob_ignore) flash_m.OPTR = (flash_m.OPTR & ~ob->USERType) | (ob->USERConfig & ob->USERType);
    return HAL_OK;
}
static uint32_t hal_err = 0U; uint32_t HAL_FLASH_GetError(void) { return hal_err; }
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint64_t data)
{
    uint32_t o = addr - 0x08000000UL, i; (void)type; prog_calls++;
    if (!unlocked || (addr & 7UL) || o + 8U > sizeof mem || fail_next_prog) { fail_next_prog = 0; return HAL_ERROR; }
    for (i = 0U; i < 8U; i++) if (mem[o + i] != 0xFFU) return HAL_ERROR;                       /* programming over a non-erased byte fails, as in the real part */
    for (i = 0U; i < 8U; i++) mem[o + i] = (uint8_t)(data >> (8U * i));
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *err)
{
    uint32_t base = 0U; erase_calls++; if (erase_noop) { *err = 0xFFFFFFFFUL; return HAL_OK; }                  /* the model of a HAL that says OK and does nothing */ last_banks = (int)e->Banks; last_page = e->Page; *err = 0xFFFFFFFFUL;
    if (!unlocked) return HAL_ERROR;
    if (flash_m.OPTR & FLASH_OPTR_DBANK) base = (e->Banks == FLASH_BANK_2) ? 0x40000UL : 0U;
    else if (e->Banks == FLASH_BANK_2) return HAL_ERROR;
    memset(&mem[base + e->Page * 2048UL], 0xFF, 2048U);
    return HAL_OK;
}
static int rd(uint32_t a, uint8_t *b, uint32_t n) { memcpy(b, &mem[a - 0x08000000UL], n); return 0; }
static void fresh(int dbank) { memset(mem, 0xFF, sizeof mem); flash_m.OPTR = dbank ? FLASH_OPTR_DBANK : 0U; image_end = 0x08020000UL; unlocked = 0; erase_calls = prog_calls = 0; fail_next_prog = 0; }

int main(void)
{
    uint8_t in[16], out[16]; int i;
    for (i = 0; i < 16; i++) in[i] = (uint8_t)(0xA0 + i);
    /* erase and program the two calibration pages in DUAL-bank mode: bank 2, pages 126 and 127 */
    fresh(1); memset(&mem[0x3F000], 0x00, 4096);
    CHECK(g_slave_flash.erase(0x0803F800UL) == 0 && last_banks == (int)FLASH_BANK_1 && last_page == 127U, "dual bank: 0x0803F800 is bank 1, page 127 (bank %d, page %u)", last_banks, (unsigned)last_page);
    CHECK(g_slave_flash.erase(0x0803F000UL) == 0 && last_banks == (int)FLASH_BANK_1 && last_page == 126U, "dual bank: 0x0803F000 is bank 1, page 126 (bank %d, page %u)", last_banks, (unsigned)last_page);
    CHECK(mem[0x3F000] == 0xFFU && mem[0x3FFFF] == 0xFFU, "both pages are erased");
    /* single-bank mode: bank 1, pages 254 and 255 */
    fresh(0); memset(&mem[0x3F000], 0x00, 4096);
    CHECK(g_slave_flash.erase(0x0803F800UL) == 0 && last_banks == (int)FLASH_BANK_1 && last_page == 127U, "single bank: the SAME numbers, page 127 of bank 1 (bank %d, page %u)", last_banks, (unsigned)last_page);
    CHECK(g_slave_flash.erase(0x0803F000UL) == 0 && last_banks == (int)FLASH_BANK_1 && last_page == 126U, "single bank: page 126 of bank 1, never above 127 (%u)", (unsigned)last_page);
    /* programming: by double words, read back; the same place again fails because it is no longer erased */
    CHECK(g_slave_flash.prog(0x0803F000UL, in, 16U) == 0 && rd(0x0803F000UL, out, 16U) == 0 && memcmp(in, out, 16U) == 0 && prog_calls == 2, "16 bytes are programmed as 2 double words and read back (%d calls)", prog_calls);
    CHECK(g_slave_flash.prog(0x0803F000UL, in, 8U) != 0, "programming over a written place fails");
    CHECK(unlocked == 0, "the flash is locked again after every operation");
    /* a failing program: reported */
    fresh(1); fail_next_prog = 1; CHECK(g_slave_flash.prog(0x0803F000UL, in, 8U) != 0 && unlocked == 0, "a failing HAL program is reported and the flash is locked again");
    /* the driver refuses everything outside the four pages (profile 124, 125, calibration 126, 127): the program can never be touched */
    fresh(1); memset(mem, 0x00, 0x1000);
    CHECK(g_slave_flash.erase(0x08000000UL) != 0 && erase_calls == 0 && mem[0] == 0x00U, "erasing the first page (the start of the program) is refused, nothing called");
    CHECK(g_slave_flash.erase(0x0803D800UL) != 0 && g_slave_flash.erase(0x08040000UL) != 0 && g_slave_flash.erase(0x08040800UL) != 0 && erase_calls == 0, "erasing the page below the four, the page above and the start of bank 2 is refused");
    CHECK(g_slave_flash.prog(0x0803D000UL, in, 8U) != 0 && g_slave_flash.prog(0x0803DFF8UL, in, 16U) != 0 && g_slave_flash.prog(0x0803FFF8UL, in, 16U) != 0 && prog_calls == 0, "programming below the pages, across the start of the first or across the end of the last is refused");
    CHECK(g_slave_flash.prog(0x0807F001UL, in, 8U) != 0 && g_slave_flash.prog(0x0803F000UL, in, 7U) != 0, "a misaligned address or length is refused");
    CHECK(g_slave_flash.erase(0x0803F400UL) != 0, "an address that is not the start of a page is refused");
    /* the program image has grown into the calibration pages: everything is refused */
    fresh(1); image_end = 0x0803E100UL; CHECK(g_slave_flash.erase(0x0803F800UL) != 0 && g_slave_flash.prog(0x0803F800UL, in, 8U) != 0 && g_slave_flash.erase(0x0803E800UL) != 0 && erase_calls == 0 && prog_calls == 0, "an image that reaches 0x0803E000: the driver refuses to write anything");
    /* the two profile pages are writable like the calibration ones: 124 and 125 of bank 1, in both flash modes */
    fresh(1); CHECK(g_slave_flash.erase(0x0803E000UL) == 0 && last_banks == (int)FLASH_BANK_1 && last_page == 124U && g_slave_flash.erase(0x0803E800UL) == 0 && last_page == 125U, "dual bank: the profile pages are bank 1, pages 124 and 125 (page %u)", (unsigned)last_page);
    fresh(0); CHECK(g_slave_flash.erase(0x0803E000UL) == 0 && last_banks == (int)FLASH_BANK_1 && last_page == 124U && g_slave_flash.erase(0x0803E800UL) == 0 && last_page == 125U, "single bank: the same numbers (page %u)", (unsigned)last_page);
    fresh(1); image_end = 0x08020000UL; CHECK(g_slave_flash.prog(0x0803E000UL, in, 16U) == 0 && g_slave_flash.prog(0x0803E800UL, in, 16U) == 0, "and they can be programmed");
    /* the store on top of it: save, load, and a second save goes to the OTHER page */
    fresh(1); { SStore_t st; uint8_t pay[4] = { 0xA2, 0x27, 0x0F, 0x27 }, got[4] = { 0, 0, 0, 0 }; SFlash_t fl; fl.read = rd; fl.prog = g_slave_flash.prog; fl.erase = g_slave_flash.erase;
      SStore_Init(&st, &fl, SFLASH_CAL_PAGE_A, SFLASH_CAL_PAGE_B, 1U, 4U);
      CHECK(SStore_Load(&st, got) == 0U, "nothing stored yet");
      CHECK(SStore_Save(&st, pay) == 0 && SStore_Load(&st, got) != 0U && memcmp(pay, got, 4U) == 0, "a record is saved and read back through the store");
      { int cur1 = st.cur; pay[0] = 0x11; CHECK(SStore_Save(&st, pay) == 0 && st.cur != cur1 && SStore_Load(&st, got) != 0U && got[0] == 0x11U, "the second save goes to the other slot (%d -> %d)", cur1, st.cur); } }
    /* the diagnostic: why the last failure happened, packed as reason x 1,000,000 + dual x 100,000 + flags */
    fresh(1); hal_err = 0x0080U; fail_next_prog = 1; (void)g_slave_flash.prog(0x0803F800UL, in, 8U);
    CHECK(SlaveFlash_Diag() == 3U * 1000000UL + 100000UL + 0x0080UL, "a HAL program error with flags 0x0080 in dual-bank mode: reason 3, dual 1, flags 0080 (%lu)", (unsigned long)SlaveFlash_Diag());
    fresh(0); (void)g_slave_flash.prog(0x0807F001UL, in, 8U); CHECK(SlaveFlash_Diag() == 2U * 1000000UL, "misaligned, single bank: reason 2, dual 0 (%lu)", (unsigned long)SlaveFlash_Diag());
    fresh(1); (void)g_slave_flash.erase(0x08000000UL); CHECK(SlaveFlash_Diag() == 1U * 1000000UL + 100000UL, "refused by the guard: reason 1 (%lu)", (unsigned long)SlaveFlash_Diag());
    /* an erase the HAL reports as OK that did not blank the page: caught, with the offset of the first byte that is not blank */
    fresh(1); erase_noop = 0; (void)g_slave_flash.erase(0x0803F800UL); memcpy(&mem[0x3F800 + 40], in, 16); erase_noop = 1;
    CHECK(g_slave_flash.erase(0x0803F800UL) != 0 && SlaveFlash_Diag() == 5U * 1000000UL + 100000UL + 40U, "an erase that did nothing: refused with reason 5 and offset 40 (%lu)", (unsigned long)SlaveFlash_Diag());
    erase_noop = 0; CHECK(g_slave_flash.erase(0x0803F800UL) == 0 && mem[0x3F800 + 40] == 0xFFU, "a working erase passes the blank check");
    printf("  slave flash driver (against a HAL model): bank and page arithmetic in both modes, double-word programming, only the two calibration pages, the image check, the store on top\n");
    /* ---- v0.9.79: the brown-out reset at power-up ---- */
    {   const uint32_t factory = 0xFFFFFEAAUL & ~FLASH_OPTR_BOR_EN & ~FLASH_OPTR_DBANK;     /* RDP 0, BOR off */
        flash_m.OPTR = factory; rcc_m.CSR = 0U; unlocked = 0; ob_writes = ob_launches = ob_fail = ob_ignore = 0;
        SlaveFlash_EnsureBor();
        CHECK(ob_writes == 1 && ob_launches == 1, "BOR off at power-up: written once and the option bytes reloaded (writes %d, launches %d)", ob_writes, ob_launches);
        CHECK(ob_last_type == (FLASH_OPTR_BOR_EN | FLASH_OPTR_BORF_LEV | FLASH_OPTR_BORR_LEV), "only the BOR fields were in the write (0x%08lX)", (unsigned long)ob_last_type);
        CHECK(SlaveFlash_BorOk() == 1U && (flash_m.OPTR & 0xFFUL) == 0xAAUL && (flash_m.OPTR & ~(FLASH_OPTR_BOR_EN | FLASH_OPTR_BORF_LEV | FLASH_OPTR_BORR_LEV)) == (factory & ~(FLASH_OPTR_BOR_EN | FLASH_OPTR_BORF_LEV | FLASH_OPTR_BORR_LEV)), "BOR on, the read protection and every other bit unchanged (0x%08lX)", (unsigned long)flash_m.OPTR);
        CHECK(!unlocked && !ob_unlocked, "the flash and the option bytes are locked again");
        /* the boot after the reload: nothing more */
        rcc_m.CSR = RCC_CSR_OBLRSTF; SlaveFlash_EnsureBor();
        CHECK(ob_writes == 1 && ob_launches == 1 && SlaveFlash_BorResult() == 0U, "the boot after the reload finds BOR on: no second write (writes %d, result %u)", ob_writes, (unsigned)SlaveFlash_BorResult());
        /* later power-ups: nothing */
        rcc_m.CSR = 0U; SlaveFlash_EnsureBor(); SlaveFlash_EnsureBor();
        CHECK(ob_writes == 1 && ob_launches == 1, "later power-ups: nothing written (writes %d)", ob_writes);
        /* a write that does not take (the option bytes come back unchanged): ONE attempt, the boot after the reload gives up (no reset loop) */
        flash_m.OPTR = factory; rcc_m.CSR = 0U; ob_writes = ob_launches = 0; ob_ignore = 1;
        SlaveFlash_EnsureBor(); CHECK(ob_writes == 1 && ob_launches == 1, "a write that does not take: one attempt and one reload");
        rcc_m.CSR = RCC_CSR_OBLRSTF; SlaveFlash_EnsureBor();
        CHECK(ob_writes == 1 && ob_launches == 1 && SlaveFlash_BorResult() == 2U && SlaveFlash_BorOk() == 0U, "the boot after that reload still finds BOR off: it gives up, no second attempt, no reset loop (writes %d, result %u)", ob_writes, (unsigned)SlaveFlash_BorResult());
        /* the HAL refuses the write: no reload, it runs on and reports it */
        flash_m.OPTR = factory; rcc_m.CSR = 0U; ob_writes = ob_launches = 0; ob_ignore = 0; ob_fail = 1;
        SlaveFlash_EnsureBor();
        CHECK(ob_launches == 0 && SlaveFlash_BorResult() == 2U && !unlocked && !ob_unlocked, "the HAL refuses the write: no reload, it runs on, reports it, everything locked (launches %d, result %u)", ob_launches, (unsigned)SlaveFlash_BorResult());
        ob_fail = 0;
        /* BOR already on at level 3: nothing at all */
        flash_m.OPTR = factory | FLASH_OPTR_BOR_EN | FLASH_OPTR_BORF_LEV | FLASH_OPTR_BORR_LEV; ob_writes = ob_launches = 0;
        SlaveFlash_EnsureBor();
        CHECK(ob_writes == 0 && ob_launches == 0 && SlaveFlash_BorOk() == 1U, "BOR already on: nothing written");
    }
    printf("  %d checks, %d failures\n  RESULT: %s\n", tot, bad, bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
