/*
 * wram.c  –  8 KiB switchable Work‑RAM pager for MMC3 / MMC6
 *            (updated July 2025 – fixes and robustness tweaks)
 *
 *  • Obeys enable / write‑protect bits written via $A001
 *  • Maps the currently‑selected SRAM bank into CPU pages $6000–7FFF
 *  • Ignores writes when the window is disabled or write‑protected
 *
 *  Changes from the original version
 *  ---------------------------------
 *    – “Dead page” is now a full 8 KiB, so page 7 access is in‑bounds.
 *    – WRAM starts **disabled** after power‑on, matching real MMC3/6.
 *    – add_write_handler() refuses to install a duplicate gate.
 *    – mmc_bankwram() accepts both 4 KiB and 8 KiB requests.
 *    – Extra comments on MMC6’s inverted WP bit.
 */

#include <stdint.h>
#include <string.h>
#include "nes.h"
#include "nes6502.h"
#include "wram.h"

/*──────────────────── Module‑scope state ────────────────────*/
static uint8_t *base     = NULL;   /* full SRAM blob supplied by cart       */
static int      banks    = 1;      /* number of 8 KiB pages                 */
static int      cur_bank = 0;      /* currently mapped page index           */
static bool     wram_en  = false;  /* enable bit – **false after reset**    */
static bool     wram_wp  = false;  /* write‑protect bit (true = read‑only)  */

#define NES           (nes_getcontextptr())
#define WINDOW_START  0x6000
#define WINDOW_END    0x7FFF
#define PAGE_SIZE     0x2000      /* 8 KiB */

/* Local "open‑bus" page – always returns $FF */
static uint8_t dead_page[PAGE_SIZE]; /* no initializer */

/*──────────────────── Internal helpers ──────────────────────*/
static void remap_page(void)
{
    nes_t *ctx = nes_getcontextptr();
    if (!ctx)                            /* core not initialised yet?  */
        return;

    uint8_t *page0 = (wram_en && base) ? (base + cur_bank * PAGE_SIZE)
                                       : dead_page;

    ctx->cpu->mem_page[6] = page0;
    ctx->cpu->mem_page[7] = page0 + 0x1000;
}

/* $6000‑7FFF write gate */
static void wram_write(uint32_t addr, uint8_t val)
{
    /* Writes are ignored when window disabled, unmapped or write-protected */
    if (!wram_en || !base || wram_wp)
        return;

    /* Inlined bank_writebyte() – faster, no extra symbol needed */
    NES->cpu->mem_page[addr >> NES6502_BANKSHIFT]
                     [addr &  NES6502_BANKMASK] = val;
}

/* Push our handler into the machine’s write‑handler table (idempotent) */
static void add_write_handler(nes_t *nes)
{
    /* Bail out if someone already installed exactly the same gate */
    for (int j = 0; nes->writehandler[j].write_func; ++j)
        if (nes->writehandler[j].min_range  == WINDOW_START &&
            nes->writehandler[j].max_range  == WINDOW_END   &&
            nes->writehandler[j].write_func == wram_write)
            return;                       /* already present */

    int i = 0;
    while (nes->writehandler[i].write_func && i < MAX_MEM_HANDLERS - 1)
        ++i;

    nes->writehandler[i].min_range  = WINDOW_START;
    nes->writehandler[i].max_range  = WINDOW_END;
    nes->writehandler[i].write_func = wram_write;

    nes->writehandler[i + 1].min_range  = 0xFFFFFFFF;   /* sentinel */
    nes->writehandler[i + 1].write_func = NULL;
}

/*──────────────────── Public MMC3 API ───────────────────────*/
/* mapper calls this when the program writes the $A000/$A001 pair */
void mmc_bankwram(int size_kib, uint32_t addr, uint8_t bank)
{
    /* Accept 8 KiB (standard) or 4 KiB (after‑market) requests     */
    if (addr != WINDOW_START || (size_kib != 8 && size_kib != 4) || banks == 0)
        return;

    /* For 4 KiB paging we simply treat the bank number as 8 KiB‑aligned.
       Home‑brew boards that really page half‑windows can supply their
       own mapper code if they need finer control. */
    cur_bank = bank % banks;
    remap_page();
}

void nes_set_wram_enable(bool enable)
{
    wram_en = enable;
    remap_page();
}

/* For MMC3: protect == 1 means read‑only.
 * For MMC6 the polarity is **inverted** – its WP bit is active‑low –
 * so the mapper layer should call this with !bit for MMC6 carts. */
void nes_set_wram_write_protect(bool protect)
{
    wram_wp = protect;
}

/*──────────────────── Cart‑boot hook ────────────────────────*/
/*──────────────────── Cart-boot hook ────────────────────────*/
void wram_init(struct nes_s *nes)
{
    if (!nes->rominfo->sram)
        return;

    base     = nes->rominfo->sram;
    banks    = nes->rominfo->sram_banks ? nes->rominfo->sram_banks : 1;
    cur_bank = 0;

    memset(dead_page, 0xFF, sizeof dead_page);

    /* power-on state = disabled & writable */
    wram_en  = false;
    wram_wp  = false;

    /* 1.  Install the write gate so CPU writes are captured                     */
    add_write_handler(nes);

    /* 2.  DO **NOT** call remap_page() here – the CPU context isn’t wired yet   */
}
