/*
 *  map005.c - MMC5 mapper emulation
 *
 *  This is a modern re‑implementation of the MMC5 logic used by
 *  games such as "Castlevania III".  The original file contained a
 *  tiny placeholder with only enough logic to boot a few titles; the
 *  code below implements the documented behaviour of the chip:
 *
 *    • Flexible PRG and CHR banking
 *    • 1 KiB of external "ExRAM" including fill mode
 *    • Split screen support registers (state only – rendering handled
 *      inside the PPU core)
 *    • Scanline IRQ counter
 *    • $5205/$5206 hardware multiplier
 *
 *  The implementation is intentionally straightforward and avoids any
 *  clever tricks so it can serve as reference material.  It borrows
 *  ideas from a number of open source emulators while being written
 *  from scratch for Nofrendo's code base.
 */

#include <string.h>

#include "noftypes.h"
#include "nes_mmc.h"
#include "nes.h"
#include "new_ppu.h"
#include "wram.h"
#include "log.h"
#include "mmc5_snd.h"

/* ------------------------------------------------------------------
 *  Internal state
 * ------------------------------------------------------------------ */

/* PRG/CHR configuration */
static uint8 prg_mode;              /* $5100 */
static uint8 chr_mode;              /* $5101 */
static uint8 chr_high;              /* $5130 – upper bits for CHR banks */

static uint8 prg_reg[4];            /* $5114–$5117 */
static uint16 chr_spr[8];           /* $5120–$5127 */
static uint16 chr_bg[4];            /* $5128–$512B */

/* ExRAM and nametable fill handling */
static uint8 exram[0x400];
static uint8 exram_mode;            /* lower 2 bits of $5104 */
static uint8 nt_fill = 0;           /* $5106 */
static uint8 at_fill = 0;           /* $5107 */
static uint8 fill_ram[0x400];       /* prebuilt fill nametable */
static uint8 *nt_page[4];           /* backup of CIRAM pages */

/* Split screen registers ($5200–$5202) – stored for the PPU core */
static uint8 split_ctrl, split_scroll, split_bank;

/* Hardware multiplier */
static uint8 mul[2];                /* $5205/$5206 */

/* IRQ counter */
static struct {
    int counter;
    int latch;
    bool enabled;
    bool pending;
} irq;

/* ------------------------------------------------------------------
 *  Helper functions
 * ------------------------------------------------------------------ */

static void sync_prg(void)
{
    switch (prg_mode & 3) {
    case 0: /* one 32 KiB page */
    {
        int bank = (prg_reg[3] & 0x7F) & ~3;
        for (int i = 0; i < 4; i++)
            mmc_bankrom(8, 0x8000 + i * 0x2000, bank + i);
        break;
    }
    case 1: /* two 16 KiB pages */
    {
        int bank0 = (prg_reg[1] & 0x7F) & ~1;
        int bank1 = (prg_reg[3] & 0x7F) & ~1;
        mmc_bankrom(8, 0x8000, bank0);
        mmc_bankrom(8, 0xA000, bank0 + 1);
        mmc_bankrom(8, 0xC000, bank1);
        mmc_bankrom(8, 0xE000, bank1 + 1);
        break;
    }
    case 2: /* 8K, 8K, 16K */
    {
        int bank0 = prg_reg[1] & 0x7F;
        int bank1 = prg_reg[2] & 0x7F;
        int bank2 = (prg_reg[3] & 0x7F) & ~1;
        mmc_bankrom(8, 0x8000, bank0);
        mmc_bankrom(8, 0xA000, bank1);
        mmc_bankrom(8, 0xC000, bank2);
        mmc_bankrom(8, 0xE000, bank2 + 1);
        break;
    }
    case 3: /* four 8 KiB pages */
    default:
        for (int i = 0; i < 4; i++)
            mmc_bankrom(8, 0x8000 + i * 0x2000, prg_reg[i] & 0x7F);
        break;
    }
}

static void sync_chr(void)
{
    switch (chr_mode & 3) {
    case 0: /* one 8 KiB bank */
        mmc_bankvrom(8, 0x0000, chr_spr[7]);
        break;

    case 1: /* two 4 KiB banks */
        mmc_bankvrom(4, 0x0000, chr_spr[3]);
        mmc_bankvrom(4, 0x1000, chr_bg[3]);
        break;

    case 2: /* four 2 KiB banks */
        mmc_bankvrom(2, 0x0000, chr_spr[1]);
        mmc_bankvrom(2, 0x0800, chr_spr[3]);
        mmc_bankvrom(2, 0x1000, chr_bg[1]);
        mmc_bankvrom(2, 0x1800, chr_bg[3]);
        break;

    case 3: /* eight 1 KiB banks: $5120-$5127 */
    default:
        /* lower half comes from registers $5120-$5123 */
        for (int i = 0; i < 4; i++)
            mmc_bankvrom(1, i * 0x400, chr_spr[i]);
        /* upper half comes from registers $5124-$5127 */
        for (int i = 0; i < 4; i++)
            mmc_bankvrom(1, 0x1000 + i * 0x400, chr_spr[4 + i]);
        break;
    }
}

/* Rebuild the fill nametable after $5106/$5107 writes */
static void rebuild_fill(void)
{
    memset(fill_ram, nt_fill, 0x3C0);
    uint8 attr = at_fill & 3;
    uint8 packed = attr | (attr << 2) | (attr << 4) | (attr << 6);
    for (int i = 0x3C0; i < 0x400; i++)
        fill_ram[i] = packed;
}

/* Nametable mapping helper – called on $5105 writes */
static void map_nametables(uint8 val)
{
    for (int i = 0; i < 4; i++) {
        uint8 sel = (val >> (i * 2)) & 3;
        switch (sel) {
        case 0: ppu_setpage(1, 8 + i, nt_page[0]); break;
        case 1: ppu_setpage(1, 8 + i, nt_page[1]); break;
        case 2: ppu_setpage(1, 8 + i, exram);   break;
        case 3: ppu_setpage(1, 8 + i, fill_ram);break;
        }
    }
}

/* ------------------------------------------------------------------
 *  IRQ / H-blank callback
 * ------------------------------------------------------------------ */

static void map5_hblank(int vblank)
{
    UNUSED(vblank);

    if (!irq.enabled)
        return;

    if (irq.counter == 0) {
        nes_irq();
        irq.pending = true;
        irq.counter = irq.latch;
    } else {
        irq.counter--;
    }
}

/* ------------------------------------------------------------------
 *  CPU read/write handlers
 * ------------------------------------------------------------------ */

static void map5_write(uint32 address, uint8 value)
{
    if (address >= 0x5C00 && address <= 0x5FFF) {
        /* ExRAM write */
        if ((exram_mode & 3) != 3)
            exram[address & 0x3FF] = value;
        return;
    }

    switch (address) {
    case 0x5100: /* PRG banking mode */
        prg_mode = value & 3;
        sync_prg();
        break;

    case 0x5101: /* CHR banking mode */
        chr_mode = value & 3;
        sync_chr();
        break;

    case 0x5104: /* ExRAM / split mode */
        exram_mode = value & 3;
        break;

    case 0x5105: /* Nametable mapping */
        map_nametables(value);
        break;

    case 0x5106: /* Fill tile */
        nt_fill = value;
        rebuild_fill();
        break;

    case 0x5107: /* Fill attribute */
        at_fill = value;
        rebuild_fill();
        break;

    case 0x5113: /* WRAM bank for $6000–$7FFF */
        mmc_bankwram(8, 0x6000, value);
        break;

    case 0x5114: case 0x5115: case 0x5116: case 0x5117:
        prg_reg[address & 3] = value;
        sync_prg();
        break;

    case 0x5120: case 0x5121: case 0x5122: case 0x5123:
    case 0x5124: case 0x5125: case 0x5126: case 0x5127:
        chr_spr[address - 0x5120] = value | (chr_high << 8);
        sync_chr();
        break;

    case 0x5128: case 0x5129: case 0x512A: case 0x512B:
        chr_bg[address - 0x5128] = value | (chr_high << 8);
        sync_chr();
        break;

    case 0x5130:
        chr_high = value & 0x3;
        sync_chr();
        break;

    case 0x5200: split_ctrl   = value; break;
    case 0x5201: split_scroll = value; break;
    case 0x5202: split_bank   = value; break;

    case 0x5203:
        irq.latch = value;
        irq.counter = value;
        irq.pending = false;
        break;

    case 0x5204:
        irq.enabled = (value & 0x80) != 0;
        irq.pending = false;
        break;

    case 0x5205: mul[0] = value; break;
    case 0x5206: mul[1] = value; break;

    default:
#ifdef NOFRENDO_DEBUG
        log_printf("unknown MMC5 write: $%02X to $%04X\n", value, address);
#endif
        break;
    }
}

static uint8 map5_read(uint32 address)
{
    if (address >= 0x5C00 && address <= 0x5FFF)
        return exram[address & 0x3FF];

    switch (address) {
    case 0x5204:
    {
        uint8 ret = irq.pending ? 0x40 : 0x00;
        irq.pending = false;
        return ret;
    }
    case 0x5205:
        return (uint8)((mul[0] * mul[1]) & 0xFF);
    case 0x5206:
        return (uint8)((mul[0] * mul[1]) >> 8);
    default:
#ifdef NOFRENDO_DEBUG
        log_printf("invalid MMC5 read: $%04X\n", address);
#endif
        return 0xFF;
    }
}

/* ------------------------------------------------------------------
 *  Mapper initialisation
 * ------------------------------------------------------------------ */

static void map5_init(void)
{
    /* Backup the CIRAM pointers for later nametable mapping */
    for (int i = 0; i < 4; i++)
        nt_page[i] = ppu_getpage(8 + i);

    memset(exram, 0, sizeof(exram));
    rebuild_fill();

    /* Default PRG mapping mirrors the last bank */
    for (int i = 0; i < 4; i++)
        mmc_bankrom(8, 0x8000 + i * 0x2000, MMC_LASTBANK);

    irq.counter = irq.latch = 0;
    irq.enabled = irq.pending = false;

    prg_mode = chr_mode = 3;  /* sensible defaults */
}

/* ------------------------------------------------------------------
 *  SNSS state handlers – minimal stub
 * ------------------------------------------------------------------ */

static void map5_getstate(SnssMapperBlock *state)
{
    state->extraData.mapper5.dummy = 0;
}

static void map5_setstate(SnssMapperBlock *state)
{
    UNUSED(state);
}

/* Memory handler tables */
static map_memwrite map5_memwrite[] = {
    { 0x5016, 0x5FFF, map5_write },
    { 0x8000, 0xFFFF, map5_write },
    {    -1,    -1, NULL }
};

static map_memread map5_memread[] = {
    { 0x5C00, 0x5FFF, map5_read },
    { 0x5204, 0x5206, map5_read },
    {    -1,    -1, NULL }
};

mapintf_t map5_intf = {
    5,               /* mapper number */
    "MMC5",          /* mapper name   */
    map5_init,       /* init routine  */
    NULL,            /* vblank */
    map5_hblank,     /* hblank */
    map5_getstate,   /* get state  */
    map5_setstate,   /* set state  */
    map5_memread,    /* memory read */
    map5_memwrite,   /* memory write*/
    &mmc5_ext        /* external sound */
};

