#include "noftypes.h"
#include "nes_mmc.h"
#include "nes.h"
#include "libsnss.h"
#include "nes_rom.h"
#include "new_ppu.h"
#include <string.h>
#include "nes6502.h"  

#define TRACE_MMC3 0

#if defined(TRACE_MMC3) && TRACE_MMC3
#   include <stdio.h>
#   define LOG(...)  do{ fprintf(stderr,__VA_ARGS__);}while(0)
#else
#   define LOG(...)  ((void)0)
#endif

#ifndef INLINE
# define INLINE static inline
#endif

/* ───────────────── configuration ──────────────────────────────────── */
#define MAP4_PPU_EDGE_IRQ  1   /* 1 = proper A12 edge, 0 = HBlank tick */

/* 1 = "new/normal" (IRQ when counter==0 after clock), 0 = "old/alternate" (IRQ only on decrement-to-0) */
#ifndef MMC3_IRQ_NEWSTYLE
#define MMC3_IRQ_NEWSTYLE 1
#endif

/* ───────────────────────── state ──────────────────────────────────── */
typedef struct {
    uint8 counter, latch;
    bool  enabled;
    bool  reload_flag;
} irq_t;

static irq_t  irq;
static uint8  reg8000;          /* last $8000 value               */
static uint16 vrombase;         /* 0x0000 or 0x1000               */
static uint8  prg_bank6;        /* last R6 value                  */
static uint8  r7_prg_bank;      /* last R7 value (0xA000 bank)    */
static bool   fourscreen;
static bool   wram_en, wram_wp;
static uint8  wram_bank; 
static uint8 chr_reg[6];



#define FIXED_LAST(c)   ((c)->rom_banks*2-1)
#define FIXED_PENULT(c) ((c)->rom_banks*2-2)

/* ─────────────── IRQ helper ───────────────────────────────────────── */
/* Call once on every rising edge of PPU-A12 */
//
INLINE void map4_clock_irq(void)
{
    bool decremented = false;
    if (irq.reload_flag || irq.counter == 0) {
        irq.counter     = irq.latch;
        irq.reload_flag = false;
    } else {
        irq.counter--;
        decremented = true;
    }
    if (irq.enabled) {
#if MMC3_IRQ_NEWSTYLE
        if (irq.counter == 0) nes_irq();              /* "new/normal" */
#else
        if (decremented && irq.counter == 0) nes_irq();/* "old/alternate" */
#endif
    }
}


/* ─────────────── PPU bus hook ─────────────────────────────────────── */
#if MAP4_PPU_EDGE_IRQ
/* ─────────────── PPU bus hook ───────────────────────────────────────
   Treat as "rising-edge already debounced" callback - just clock IRQ 
   unconditionally. PPU handles A12 edge detection and debouncing.     */
void map4_ppu_tick(uint16 addr)
{
    (void)addr; /* Address no longer needed */
    map4_clock_irq();
}

#endif /* MAP4_PPU_EDGE_IRQ */

/* ─────────────── CPU write handler ────────────────────────────────── */
static void map4_write(uint32 a, uint8 v)
{
    switch (a & 0xE001)
    {
    /* $8000 – bank select ------------------------------------------------*/
    case 0x8000: {
        uint8 old_d7   = reg8000 & 0x80;
        uint8 old_mode = reg8000 & 0x40;      /* D6: PRG mode bit        */

        reg8000  = v;
        vrombase = (v & 0x80) ? 0x1000 : 0x0000;

        /* ─── refresh all six CHR windows if D7 toggled ─── */
        if (old_d7 != (v & 0x80)) {
            /* R0 / R1 = 2 KiB even‑only windows */
            uint8 r0 = chr_reg[0] & 0xFE;
            uint8 r1 = chr_reg[1] & 0xFE;
            mmc_bankvrom(1, vrombase ^ 0x0000, r0);
            mmc_bankvrom(1, vrombase ^ 0x0400, r0 + 1);
            mmc_bankvrom(1, vrombase ^ 0x0800, r1);
            mmc_bankvrom(1, vrombase ^ 0x0C00, r1 + 1);

            /* R2–R5 = four 1 KiB windows */
            mmc_bankvrom(1, vrombase ^ 0x1000, chr_reg[2]);
            mmc_bankvrom(1, vrombase ^ 0x1400, chr_reg[3]);
            mmc_bankvrom(1, vrombase ^ 0x1800, chr_reg[4]);
            mmc_bankvrom(1, vrombase ^ 0x1C00, chr_reg[5]);
        }

        /* fix penultimate bank (always visible) */
        mmc_bankrom(8, (v & 0x40) ? 0x8000 : 0xC000,
                       FIXED_PENULT(mmc_getinfo()));

        /* swap R6 target if PRG mode bit (D6) flipped */
        if (old_mode != (v & 0x40))
            mmc_bankrom(8, (v & 0x40) ? 0xC000 : 0x8000, prg_bank6);
        break;
    }

    /* $8001 – bank data --------------------------------------------------*/
    case 0x8001: {
        switch (reg8000 & 7)
        {
        case 0: v &= 0xFE;
                chr_reg[0] = v;
                mmc_bankvrom(1, vrombase ^ 0x0000, v);
                mmc_bankvrom(1, vrombase ^ 0x0400, v+1);      break;
        case 1: v &= 0xFE;
                chr_reg[1] = v;
                mmc_bankvrom(1, vrombase ^ 0x0800, v);
                mmc_bankvrom(1, vrombase ^ 0x0C00, v+1);      break;
        case 2: chr_reg[2] = v;  mmc_bankvrom(1, vrombase ^ 0x1000, v);        break;
        case 3: chr_reg[3] = v; mmc_bankvrom(1, vrombase ^ 0x1400, v);        break;
        case 4: chr_reg[4] = v; mmc_bankvrom(1, vrombase ^ 0x1800, v);        break;
        case 5: chr_reg[5] = v;mmc_bankvrom(1, vrombase ^ 0x1C00, v);        break;
        case 6: prg_bank6 = v;
                mmc_bankrom(8, (reg8000 & 0x40) ? 0xC000 : 0x8000, prg_bank6);
                break;
        case 7: r7_prg_bank = v; mmc_bankrom(8, 0xA000, v);  break;
        }
        break;
    }

    /* $A000 – nametable mirroring --------------------------------------- */
    case 0xA000:
        if (!fourscreen) {
            if ((v & 1) == 0)  ppu_mirror(0,0,1,1);   /* 0 = horizontal (A10) */
            else               ppu_mirror(0,1,0,1);   /* 1 = vertical   (A11) */
        }
        break;

    /* $A001 – WRAM enable/protect --------------------------------------- */
    case 0xA001:
        wram_en = !!(v & 0x80);            /* MMC3: D7 enable */
        wram_wp = !!(v & 0x40);            /* MMC3: D6 write-protect (1 = deny) */
        nes_set_wram_enable(wram_en);
        nes_set_wram_write_protect(wram_wp);
        mmc_bankwram(8, 0x6000, 0);        /* MMC3: ignore low bits (no bank select) */
        break;

    /* $C000 – IRQ latch -------------------------------------------------- */
    case 0xC000: irq.latch = v; 
    #ifdef TRACE_MMC3  
        //LOG("map4_write: $C000 → set latch to ", v, "\n"); 
    #endif
    break;

    /* $C001 – IRQ reload ------------------------------------------------- */
    case 0xC001: irq.reload_flag = true;  irq.counter = 0;
    #ifdef TRACE_MMC3  
        //LOG("map4_write: $C001 → reload\n"); 
    #endif
    break;
    /* $E000 – IRQ disable / ack ----------------------------------------- */
    case 0xE000: irq.enabled = false; nes_irq_ack();  
    #ifdef TRACE_MMC3  
        //LOG("map4_write: $E000 → IRQ disable + ack\n"); 
    #endif
 break;

    /* $E001 – IRQ enable ------------------------------------------------- */
    case 0xE001: irq.enabled = true;  
        #ifdef TRACE_MMC3 
            //LOG("map4_write: $E001 → IRQ enable\n");
        #endif;
        break;
    }
    /*#ifdef TRACE_MMC3
        if ((a & 0xE001) == 0xC000 || (a & 0xE001) == 0xC001 ||
            (a & 0xE001) == 0xE000 || (a & 0xE001) == 0xE001)
        {
            LOG("WR %04X <- %02X | cnt=%02X lat=%02X rf=%d en=%d line=%d\n",
                (uint16)(a & 0xE001), v,
                irq.counter, irq.latch, irq.reload_flag, irq.enabled,
                ext_irq_line);
        }
    #endif */
    
}

/* ─────────────── HBlank fallback (edge IRQ off) ───────────────────── */
static void map4_hblank(int vb)
{
#if !MAP4_PPU_EDGE_IRQ
    if (!vb)
        map4_clock_irq();
#else
    (void)vb;
#endif
}

/* ─────────────── Save-state helpers ───────────────────────────────── */
static void map4_getstate(SnssMapperBlock *s)
{
    s->extraData.mapper4.irqCounter        = irq.counter;
    s->extraData.mapper4.irqLatchCounter   = irq.latch;
    s->extraData.mapper4.irqCounterEnabled = irq.enabled;
    s->extraData.mapper4.last8000Write     = reg8000;
    s->extraData.mapper4.fill1[0]          = irq.reload_flag;
    s->extraData.mapper4.fill1[1]          = wram_en;
    s->extraData.mapper4.fill1[2]          = wram_wp;
    s->extraData.mapper4.fill1[3]          = prg_bank6;
    s->extraData.mapper4.fill1[4]          = r7_prg_bank;
    /* Save CHR registers in remaining fill space */
    memcpy(&s->extraData.mapper4.fill1[5], chr_reg, 6);
}

static void map4_setstate(SnssMapperBlock *s)
{
    irq.counter      = s->extraData.mapper4.irqCounter;
    irq.latch        = s->extraData.mapper4.irqLatchCounter;
    irq.enabled      = s->extraData.mapper4.irqCounterEnabled;
    reg8000          = s->extraData.mapper4.last8000Write;
    irq.reload_flag  = s->extraData.mapper4.fill1[0];
    wram_en          = s->extraData.mapper4.fill1[1];
    wram_wp          = s->extraData.mapper4.fill1[2];
    prg_bank6        = s->extraData.mapper4.fill1[3];
    r7_prg_bank      = s->extraData.mapper4.fill1[4];
    /* Restore CHR registers from remaining fill space */
    memcpy(chr_reg, &s->extraData.mapper4.fill1[5], 6);

    nes_set_wram_enable(wram_en);
    nes_set_wram_write_protect(wram_wp);
    
    /* Restore banking configuration */
    vrombase = (reg8000 & 0x80) ? 0x1000 : 0x0000;
    
    /* Restore PRG banks */
    mmc_bankrom(8, (reg8000 & 0x40) ? 0xC000 : 0x8000, prg_bank6);
    mmc_bankrom(8, (reg8000 & 0x40) ? 0x8000 : 0xC000, FIXED_PENULT(mmc_getinfo()));
    mmc_bankrom(8, 0xA000, r7_prg_bank);
    mmc_bankrom(8, 0xE000, FIXED_LAST(mmc_getinfo()));
    
    /* Restore CHR banks */
    uint8 r0 = chr_reg[0] & 0xFE;
    uint8 r1 = chr_reg[1] & 0xFE;
    mmc_bankvrom(1, vrombase ^ 0x0000, r0);
    mmc_bankvrom(1, vrombase ^ 0x0400, r0 + 1);
    mmc_bankvrom(1, vrombase ^ 0x0800, r1);
    mmc_bankvrom(1, vrombase ^ 0x0C00, r1 + 1);
    mmc_bankvrom(1, vrombase ^ 0x1000, chr_reg[2]);
    mmc_bankvrom(1, vrombase ^ 0x1400, chr_reg[3]);
    mmc_bankvrom(1, vrombase ^ 0x1800, chr_reg[4]);
    mmc_bankvrom(1, vrombase ^ 0x1C00, chr_reg[5]);
}

/* ─────────────── Power-on / reset ─────────────────────────────────── */
static void map4_init(void)
{
     rominfo_t *cart = mmc_getinfo();
     memset(&irq,    0, sizeof irq);
     memset(chr_reg, 0, sizeof chr_reg);
     wram_bank = 0;
    reg8000    = 0;  vrombase = 0;  prg_bank6 = 0;  r7_prg_bank = 0;
    fourscreen = !!(cart->flags & ROM_FLAG_FOURSCREEN);
    wram_en    = false; wram_wp = false;

    /* PRG layout */
    mmc_bankrom(8, 0xC000, FIXED_PENULT(cart));
    mmc_bankrom(8, 0xE000, FIXED_LAST(cart));
    mmc_bankrom(8, 0x8000, prg_bank6);
    mmc_bankrom(8, 0xA000, 0);

    /* CHR layout */
    mmc_bankvrom(8, 0x0000, 0);

#if MAP4_PPU_EDGE_IRQ
    ppu_set_mapper_hook(map4_ppu_tick);
#endif
}

/* ─────────────── memory-write table & public iface ────────────────── */
static map_memwrite map4_memwrite[] = {
    { 0x8000, 0xFFFF, map4_write },
    {    -1,     -1, NULL }
};

mapintf_t map4_intf = {
    4, "MMC3",
    map4_init, NULL,
    map4_hblank,
    map4_getstate, map4_setstate,
    NULL, map4_memwrite, NULL
};
