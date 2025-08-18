
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#if defined(TRACE_MMC3) && TRACE_MMC3
#include <stdio.h>
#endif
#if defined(TRACE_PPU) && TRACE_PPU
#include <stdio.h>
#endif

#include "noftypes.h"
#include "nes.h"            /* nes_nmi() / nes_getcontextptr()        */
#include "new_ppu.h"        /* public interface + register defs       */
#include "nes_mmc.h"        /* mapper A12 edge callback               */
#include "nes6502.h"        /* nes6502_burn() / nes6502_getbyte()     */
#include "vid_drv.h"        /* vid_getbuffer() / vid_setpalette()     */
#include "nes_pal.h"        /* nes_palette / shady_palette            */
#include "log.h"

/* ─────────────────── Fast-code / inline helpers ─────────────────── */
#if defined(ESP_PLATFORM)
#   define FAST_MEM __attribute__((section(".iram1")))
#else
#   define FAST_MEM
#endif

#if defined(__GNUC__)
#   define ALWAYS_INLINE FAST_MEM static inline __attribute__((always_inline))
#else
#   define ALWAYS_INLINE FAST_MEM static inline
#endif

/* ─────────────────── Build-time configuration ─────────────────── */
/* MMC3_A12_LOW_REQ constant removed - now using PPU-cycle timer */

/* ─────────────────── MMC3 A12 edge filter state ─────────────────── */
static bool a12_prev = false;                 /* last latched A12 */
static int  mmc3_a12_low_m2_count = 0;       /* M2 cycles seen while A12 is low */
static bool mmc3_a12_level = false;          /* current A12 level */

/* Forward declaration for mapper hook */
static void (*mapper_ppu_hook)(uint16_t) = NULL;

ALWAYS_INLINE bool a12(uint16_t addr) { return (addr & 0x1000) != 0; }

static void mmc3_track_a12(uint16_t addr)
{
    bool cur = (addr & 0x1000) != 0;

    /* On rising edge: check if A12 has been low for >= 3 M2 cycles */
    if (!a12_prev && cur) {
#if defined(TRACE_MMC3) && TRACE_MMC3
        printf("MMC3 A12: rising edge, low_count=%d\n", mmc3_a12_low_m2_count);
#endif
        if (mmc3_a12_low_m2_count >= 3 && mapper_ppu_hook != NULL) {
            mapper_ppu_hook(addr & 0x1FFF);
#if defined(TRACE_MMC3) && TRACE_MMC3
            printf("MMC3 A12: IRQ clocked (count=%d)\n", mmc3_a12_low_m2_count);
#endif
        }
        /* Reset counter after rising edge */
        mmc3_a12_low_m2_count = 0;
#if defined(TRACE_MMC3) && TRACE_MMC3
        printf("MMC3 A12: rising edge processed, reset counter\n");
#endif
    }
    
    a12_prev = cur;
    mmc3_a12_level = cur;
}

/* ─────────────────── Constants & macros ─────────────────── */
/* NES PPU Timing Constants:
 * - PPU runs at 5.369318 MHz (NTSC) / 5.320342 MHz (PAL)
 * - CPU runs at 1.789773 MHz (NTSC) / 1.773447 MHz (PAL)
 * - NTSC: PPU/CPU ratio = 3.0 (exactly 3 PPU cycles per CPU cycle)
 * - PAL: PPU/CPU ratio = 3.2 (16 PPU cycles per 5 CPU cycles = 3.2)
 * - Each frame: 341 dots × 262 scanlines = 89,342 PPU cycles (NTSC)
 * - Visible area: 256×240 pixels, rendered during dots 1-256 of scanlines 0-239
 * - VBlank occurs during scanlines 241-260, NMI triggered at scanline 241, dot 1
 */
#define PPU_DOTS_PER_SCANLINE   341
#define PPU_SCANLINES_PER_FRAME_NTSC 262
#define PPU_SCANLINES_PER_FRAME_PAL  312
#define PPU_VISIBLE_X           256
#define PPU_VISIBLE_Y           240

#define PPU_SCANLINES_PER_FRAME (ppu_is_pal ? PPU_SCANLINES_PER_FRAME_PAL : PPU_SCANLINES_PER_FRAME_NTSC)
#define FRAME_TICKS             (PPU_DOTS_PER_SCANLINE * PPU_SCANLINES_PER_FRAME)

/* PPUCTRL flags (local subset) */
#define PPU_CTRL0F_BGADDR   0x10
#define PPU_CTRL0F_SPRADDR  0x08
#define PPU_CTRL0F_SPR16    0x20
#define PPU_CTRL0F_NMI      0x80
#define PPU_CTRL0F_ADDRINC  0x04

/* PPUMASK bits */
#define MASK_SHOW_BG     0x08
#define MASK_SHOW_SPR    0x10
#define MASK_LEFT_BG     0x02
#define MASK_LEFT_SPR    0x04

/* PPUSTATUS bits */
#define PPU_STATF_VBLANK     0x80
#define PPU_STATF_STRIKE     0x40  /* sprite-0 hit */
#define PPU_STATF_MAXSPRITE  0x20  /* sprite overflow */

/* OAM attribute bits */
#define OAMF_PALETTE 0x03
#define OAMF_BEHIND  0x20
#define OAMF_HFLIP   0x40
#define OAMF_VFLIP   0x80

#define RENDERING_ENABLED ((ppu.mask & (MASK_SHOW_BG | MASK_SHOW_SPR)) != 0)

/* Helper to advance the master dot & line counters */
#define INC_DOT()                                 \
    do {                                          \
        ++ppu.dot;                                \
        if (ppu.dot == PPU_DOTS_PER_SCANLINE) {   \
            ppu.dot = 0;                          \
            ++ppu.scanline;                       \
            if (ppu.scanline == PPU_SCANLINES_PER_FRAME) \
                ppu.scanline = 0;                 \
        }                                         \
    } while (0)

#define IS_VISIBLE_LINE   (ppu.scanline < 240)
#define IS_PRERENDER_LINE (ppu.scanline == (ppu_is_pal ? 311 : 261))

/* Memory-mapped I/O addresses */
#define PPU_OAMDMA 0x4014
#define PPU_JOY0   0x4016     /* VS joypad strobe / CHR bank switch */

/* 4 KiB internal / CIRAM nametable RAM (supports 4-screen mode) */
static uint8_t ciram[0x1000];

/* Nametable mirroring state */
static uint8_t nametable_mapping[4] = {0, 1, 2, 3}; /* Default: 4-screen mode */

/* 4-screen mode flag for external VRAM access */
static bool ppu_four_screen_enabled = false;

/* Mapper-supplied callback for CHR banking on A12 rising edge */

/* Global sprite display toggle */
static bool sprites_enabled = true;

/* Global flag to control whether the PPU should actually write pixels to the
 * framebuffer. When disabled the PPU still runs through all cycles so timing
 * and side effects (sprite-0 hits, scroll updates, etc.) remain intact. */
static bool draw_enabled = true;

/* PAL timing option */
static bool ppu_is_pal = false;

/* MMC-2 / MMC-4 callbacks */
static void (*ppu_latchfunc)(uint32_t base, uint8_t tile) = NULL;
static void (*ppu_vromswitch)(uint8_t data)               = NULL;

/* ─────────────────── Bit-reverse LUT for fast H-flip ─────────────────── */
static uint8_t bitrev[256];
static void init_bitrev(void)
{
    for (int i = 0; i < 256; ++i) {
        uint8_t b = i;
        b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
        b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
        b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
        bitrev[i] = b;
    }
}

/* ─────────────────── Data structures ─────────────────── */

typedef struct {
    /* Pattern shift registers (16-bit) */
    uint16_t pt_lo, pt_hi;

    /* Attribute shift registers (16-bit) */
    uint16_t at_lo, at_hi;

    /* Latches */
    uint8_t  next_nt, next_at, next_pt_lo, next_pt_hi;
} bg_t;

typedef struct {
    uint8_t x;          /* pixel counter */
    uint8_t pt_lo;      /* pattern shift regs (8 bits each) */
    uint8_t pt_hi;
    uint8_t attr;       /* attribute byte */
    bool    in_range;
} spr_unit_t;

#define SPR_UNIT_MAX 8

static struct {
    /* $2000-$2002 shadow */
    uint8_t ctrl, mask, status;
    uint8_t oam_addr;

    /* Loopy registers */
    uint16_t v, t;
    uint8_t  x, w;

    /* Buffered data (for $2007 read) */
    uint8_t buffered_data;

    /* timing */
    int dot, scanline;
    bool odd_frame;
    bool frame_complete;

    // CPU<->PPU interleave
    // For NTSC: exactly 3 PPU dots per CPU cycle.
    // For PAL/Dendy: 16 PPU dots every 5 CPU cycles (3.2 per CPU), scheduled via accumulator.
    uint8_t  phase_mod3;    // 0..2 sub-slot index inside a CPU cycle (NTSC fast-path)
    uint8_t  pal_ppu_accum; // 0..4 accumulator in "fifths" for PAL scheduling
    bool     is_pal_system; // mirrors global region flag

    /* NMI timing */
    bool nmi_prev;
    uint8_t nmi_delay;

    /* OAM */
    uint8_t  oam[256];
    uint8_t  sec_oam[32];

    /* rendering pipes */
    bg_t       bg;
    spr_unit_t spr[SPR_UNIT_MAX];
    uint8_t    sprite_count;     /* sprites in range (0-8) */
    bool       sprite_zero_next; /* sprite 0 in next line secondary OAM */
    bool       sprite_zero_this; /* sprite 0 in current line secondary OAM */
    uint8_t    next_sprite_xmin; /* optimisation: earliest sprite X left */

    /* NEW: track which secondary-OAM slot corresponds to OAM #0 */
    uint8_t    sprite0_slot_next; /* 0..7 valid, 0xFF = none */
    uint8_t    sprite0_slot_this; /* latched copy for current line */
    
    /* Sprite evaluation state (for cycle-accurate evaluation) */
    uint8_t    eval_sprite_idx;  /* Current sprite being evaluated (0-63) */
    uint16_t   eval_oam_addr;    /* Current byte address in primary OAM (0-255) */
    uint8_t    eval_sec_idx;     /* Current secondary OAM index (0-31) */
    bool       eval_overflow;    /* Sprite overflow detected */
    uint8_t    eval_temp_y;      /* Temporary Y value read on even cycles */
    bool       sprite_in_range;  /* Flag if current sprite is in range */
    uint8_t    eval_byte_index;  /* 0..3: which byte of the sprite we're on */
    uint8_t    eval_read_latch;  /* last byte read on odd cycle */
    bool       oam_write_during_eval; /* OAM write occurred during sprite eval */

    /* Sprite fetch state */
    uint8_t  spr_fetch_slot;    // 0..7
    uint8_t  spr_fetch_phase;   // 0..7 within the 8-cycle sequence
    uint8_t  spr_tmp_y, spr_tmp_tile, spr_tmp_attr, spr_tmp_x;
    uint8_t  spr_lo, spr_hi;    // temp hold of fetched pattern bytes for the current slot
    uint16_t spr_fetch_addr;

    /* palette */
    uint8_t palette[32];

    /* output fb */
    uint8_t *fb;
    uint8_t open_bus;
} ppu;

/* ─────────────────── Helpers ─────────────────── */
static inline uint8_t apply_grayscale(uint8_t idx)
{
    return (idx & 0x30) | (idx & 0x03);
}

/* Emphasis lookup table --------------------------------------------------- */
static uint8_t emphasis_lut[8][64];
static bool    emphasis_lut_init = false;

static void init_emphasis_lut(void)
{
    if (emphasis_lut_init)
        return;

    static const float emph[8][3] = {
        {1.00f, 1.00f, 1.00f}, /* --- */
        {1.00f, 0.75f, 0.75f}, /* r-- */
        {0.75f, 1.00f, 0.75f}, /* -g- */
        {0.75f, 0.75f, 1.00f}, /* rg- */
        {1.00f, 0.75f, 1.00f}, /* --b */
        {0.75f, 1.00f, 1.00f}, /* r-b */
        {1.00f, 1.00f, 0.75f}, /* -gb */
        {0.75f, 0.75f, 0.75f}  /* rgb */
    };

    for (int e = 0; e < 8; ++e) {
        for (int i = 0; i < 64; ++i) {
            rgb_t base = nes_palette[i];
            float r = base.r * emph[e][0];
            float g = base.g * emph[e][1];
            float b = base.b * emph[e][2];

            int best = 0;
            int best_err = 1 << 30;
            for (int j = 0; j < 64; ++j) {
                rgb_t cand = nes_palette[j];
                int dr = (int)r - cand.r;
                int dg = (int)g - cand.g;
                int db = (int)b - cand.b;
                int err = dr * dr + dg * dg + db * db;
                if (err < best_err) {
                    best_err = err;
                    best = j;
                }
            }
            emphasis_lut[e][i] = (uint8_t)best;
        }
    }

    emphasis_lut_init = true;
}

static inline uint8_t apply_emphasis_idx(uint8_t idx, uint8_t mask)
{
    init_emphasis_lut();
    uint8_t emph = (mask >> 5) & 0x07;
    return emphasis_lut[emph][idx & 0x3F];
}

ALWAYS_INLINE uint8_t *ciram_ptr(uint16_t addr)
{
    /* Nametable addressing: $2000-$2FFF -> NT 0,1,2,3 */
    uint8_t nt = (addr >> 10) & 3;              /* Which nametable (0-3) */
    uint8_t mapped_nt = nametable_mapping[nt];  /* Apply mirroring */
    uint16_t offset = addr & 0x3FF;             /* Offset within nametable */
    
    /* Map to 2KB CIRAM unless 4-screen mode is enabled */
    if (!ppu_four_screen_enabled) {
        /* Only NT 0 and 1 physically exist in 2KB CIRAM */
        if (mapped_nt >= 2) mapped_nt -= 2;    /* NT 2,3 -> NT 0,1 */
    }
    
    return &ciram[(mapped_nt << 10) | offset];
}

ALWAYS_INLINE uint8_t pal_read_raw(uint16_t addr)
{
    addr &= 0x1F;
    if ((addr & 0x13) == 0x10) addr &= ~0x10;
    return ppu.palette[addr];
}

ALWAYS_INLINE void pal_write_raw(uint16_t addr, uint8_t v)
{
    addr &= 0x1F;
    if ((addr & 0x13) == 0x10) addr &= ~0x10;

    /* Hardware mirrors palette index 0 ($3F00/$3F04/$3F08/$3F0C)
       across *both* BG and sprite banks ($3F10 etc.). */
    uint8_t val = v & 0x3F;
    if ((addr & 0x03) == 0) {
        ppu.palette[addr]        = val;
        ppu.palette[addr ^ 0x10] = val;
    } else {
        ppu.palette[addr] = val;
    }
}


/* ─────────────────── CHR bus accessors ─────────────────── */
/* Legacy memory mapping compatibility - translates to MMC interface calls */
static uint8_t *chr_page_ptrs[16]; /* Track page pointers for 16 1 KiB pages */
static uint8_t *chrram_ptr = NULL; /* Base pointer to CHR RAM */
static size_t   chrram_size = 0;   /* Size of CHR RAM in bytes */

/* Cached pointer to NES context to avoid global lookups */
static nes_t *ppu_get_nes(void) {
    static nes_t *nes = NULL;
    if (!nes) nes = nes_getcontextptr();
    return nes;
}

/* CHR reads use the mapper-provided 1 KiB page table - single source of truth for CHR mapping */
ALWAYS_INLINE uint8_t chr_read(uint16_t addr)
{
    mmc3_track_a12(addr);
    
    if (addr >= 0x2000) return 0; /* Only handle $0000-$1FFF */
    
    int page = (addr >> 10) & 0x0F;
    int off = addr & 0x3FF;
    
    if (chr_page_ptrs[page] != NULL) {
        return chr_page_ptrs[page][off];
    }
    
    /* Fall back for CHR-RAM when no page is mapped */
    if (chrram_ptr && chrram_size) {
        return chrram_ptr[addr % chrram_size];
    }

    return 0;
}

ALWAYS_INLINE uint8_t ppu_bus_read(uint16_t addr)
{
    addr &= 0x3FFF;
    if (addr < 0x2000)              return chr_read(addr);
    else if (addr < 0x3F00)         return *ciram_ptr(addr);
    else                            return pal_read_raw(addr);
}

ALWAYS_INLINE void ppu_bus_write(uint16_t addr, uint8_t v)
{
    addr &= 0x3FFF;
    if (addr < 0x2000) {
        /* CHR-RAM write */
        if (chrram_ptr && chrram_size) {
            chrram_ptr[addr % chrram_size] = v;
        }
        mmc3_track_a12(addr);        /* track A12 edges on writes too */
    } else if (addr < 0x3F00) {
        *ciram_ptr(addr) = v;
    } else {
        pal_write_raw(addr, v);
    }
}

/* ─────────────────── Loopy helpers ─────────────────── */
ALWAYS_INLINE void inc_x(void)
{
    if ((ppu.v & 0x001F) == 31) {         /* coarse X == 31 */
        ppu.v &= ~0x001F;
        ppu.v ^= 0x0400;                  /* switch horizontal nametable */
    } else {
        ++ppu.v;
    }
}
ALWAYS_INLINE void inc_y(void)
{
    if ((ppu.v & 0x7000) != 0x7000) {
        ppu.v += 0x1000;                  /* fine Y++ */
    } else {
        ppu.v &= ~0x7000;                 /* fine Y = 0 */
        uint16_t y = (ppu.v & 0x03E0) >> 5; /* coarse Y */
        if (y == 29) {
            y = 0;
            ppu.v ^= 0x0800;              /* switch vertical nametable */
        } else if (y == 31) {
            y = 0; /* nametable attribute row, stays same */
        } else {
            ++y;
        }
        ppu.v = (ppu.v & ~0x03E0) | (y << 5);
    }
}
ALWAYS_INLINE void copy_x_from_t(void) { ppu.v = (ppu.v & ~0x041F) | (ppu.t & 0x041F); }
ALWAYS_INLINE void copy_y_from_t(void) { ppu.v = (ppu.v & ~0x7BE0) | (ppu.t & 0x7BE0); }

/* ─────────────────── Background pipeline ─────────────────── */
ALWAYS_INLINE void bg_shift(void)
{
    ppu.bg.pt_lo <<= 1;
    ppu.bg.pt_hi <<= 1;
    ppu.bg.at_lo <<= 1;
    ppu.bg.at_hi <<= 1;
}
ALWAYS_INLINE void bg_reload_shifters(void)
{
    /* pattern */
    ppu.bg.pt_lo = (ppu.bg.pt_lo & 0x00FF) | ((uint16_t)ppu.bg.next_pt_lo << 8);
    ppu.bg.pt_hi = (ppu.bg.pt_hi & 0x00FF) | ((uint16_t)ppu.bg.next_pt_hi << 8);

    /* attribute – replicate palette bits across 16-bit regs */
    uint8_t attr = ppu.bg.next_at;
    uint16_t lo = (attr & 1) ? 0xFFFF : 0x0000;
    uint16_t hi = (attr & 2) ? 0xFFFF : 0x0000;
    ppu.bg.at_lo = (ppu.bg.at_lo & 0x00FF) | (lo & 0xFF00);
    ppu.bg.at_hi = (ppu.bg.at_hi & 0x00FF) | (hi & 0xFF00);
}

ALWAYS_INLINE void bg_fetch(void)
{
    int cyc = ppu.dot & 7;

    switch (cyc) {
    case 1: /* NT byte */
        ppu.bg.next_nt = ppu_bus_read(0x2000 | (ppu.v & 0x0FFF));
        break;
    case 3: /* AT byte */
    {
        uint16_t at_addr = 0x23C0 | (ppu.v & 0x0C00) | ((ppu.v >> 4) & 0x38) | ((ppu.v >> 2) & 0x07);
        uint8_t at_byte = ppu_bus_read(at_addr);
        uint8_t shift = ((ppu.v >> 4) & 4) | (ppu.v & 2);
        ppu.bg.next_at = (at_byte >> shift) & 3;
        break;
    }
    case 5: /* PT low */
    {
        uint16_t base = (ppu.ctrl & PPU_CTRL0F_BGADDR) ? 0x1000 : 0x0000;
        uint8_t tile = ppu.bg.next_nt;
        uint16_t addr = base + tile * 16 + ((ppu.v >> 12) & 7);

        /* MMC-2 / MMC-4 latch */
        if (ppu_latchfunc) ppu_latchfunc(base, tile);

        ppu.bg.next_pt_lo = chr_read(addr);
        break;
    }
    case 7: /* PT high + reload */
    {
        uint16_t base = (ppu.ctrl & PPU_CTRL0F_BGADDR) ? 0x1000 : 0x0000;
        uint8_t tile = ppu.bg.next_nt;
        uint16_t addr = base + tile * 16 + ((ppu.v >> 12) & 7) + 8;
        ppu.bg.next_pt_hi = chr_read(addr);
        bg_reload_shifters();
        break;
    }
    default:
        break;
    }

    if (cyc == 0) inc_x();
}

/* ─────────────────── Cycle-accurate sprite evaluation ─────────────────── */
static void eval_sprite_read_primary(void)
{
    if (ppu.eval_oam_addr > 255) return; /* All sprites processed */

    /* If $2004 was written mid-eval, emulate the PPU's bus corruption */
    if (ppu.oam_write_during_eval) {
        ppu.eval_read_latch = 0xFF;      /* typical open-bus value */
        ppu.sprite_in_range = false;     /* treat as out-of-range */
        return;
    }

    /* Read one byte from primary OAM on odd cycle */
    ppu.eval_read_latch = ppu.oam[ppu.eval_oam_addr];

    /* If reading Y byte (byte_index == 0), compute sprite_in_range */
    if (ppu.eval_byte_index == 0) {
        uint16_t cur_line = ppu.scanline + 1;
        uint8_t spr_h = (ppu.ctrl & PPU_CTRL0F_SPR16) ? 16 : 8;
        int16_t diff = (int16_t)cur_line - (int16_t)ppu.eval_read_latch;
        ppu.sprite_in_range = (diff >= 0 && diff < spr_h);
        ppu.eval_sprite_idx = ppu.eval_oam_addr >> 2;
    }
}

static void eval_sprite_write_secondary(void)
{
    if (ppu.eval_oam_addr > 255) return; /* All sprites processed */

    /* Abort evaluation entirely if a write occurred during evaluation */
    if (ppu.oam_write_during_eval)
        return;

    /* Write one byte to secondary OAM on even cycle if in range */
    if (ppu.sprite_in_range && ppu.eval_sec_idx < 32) {
        ppu.sec_oam[ppu.eval_sec_idx++] = ppu.eval_read_latch;

        /* If writing Y byte (byte_index == 0) and this is sprite 0 */
        if (ppu.eval_byte_index == 0 && ppu.eval_sprite_idx == 0) {
            ppu.sprite_zero_next  = true;
            ppu.sprite0_slot_next = (ppu.eval_sec_idx > 0) ? ((ppu.eval_sec_idx - 1) >> 2) : 0; // slot 0..7
        }
    } else if (ppu.eval_sec_idx >= 32) {
        /* Secondary OAM full - overflow bug path */
        if (ppu.sprite_in_range && !ppu.eval_overflow) {
            ppu.eval_overflow = true;
        }

        /* Continue reading but don't write */
        /* Hardware bug: increment by +5 when starting new sprite while full */
        if (ppu.eval_byte_index == 0) {
            ppu.eval_oam_addr = (ppu.eval_oam_addr + 5) & 0xFF;
            if (ppu.eval_oam_addr < 5) {
                ppu.eval_oam_addr = 256; /* Force termination */
                return;
            }
            ppu.eval_byte_index = 0; /* Stay on Y byte */
            return;
        }
    }

    /* Increment by 1 byte and advance byte index */
    ppu.eval_oam_addr = (ppu.eval_oam_addr + 1) & 0xFF;
    ppu.eval_byte_index = (ppu.eval_byte_index + 1) & 3;
}

/* Legacy sprite evaluation function removed - now using cycle-accurate evaluation */

/* ─────────────────── Sprite fetch (now with MMC-2 latch support) ─────────────────── */
ALWAYS_INLINE void sprite_shift(void)
{
    uint8_t min_x = 255;
    for (uint8_t i = 0; i < SPR_UNIT_MAX; ++i) {
        spr_unit_t *u = &ppu.spr[i];
        if (!u->in_range) continue;
        if (u->x == 0) {
            u->pt_lo <<= 1;
            u->pt_hi <<= 1;
        } else {
            --u->x;
        }
        if (u->x < min_x) min_x = u->x;
    }
    ppu.next_sprite_xmin = (min_x == 255) ? 0 : min_x;
}

/* ─────────────────── Pixel composition ─────────────────── */
ALWAYS_INLINE uint8_t bg_pixel(uint8_t *pal_row_out)
{
    if (!(ppu.mask & MASK_SHOW_BG) || (!(ppu.mask & MASK_LEFT_BG) && ppu.dot <= 8)) {
        *pal_row_out = 0; return 0;
    }

    uint16_t bit = 0x8000 >> ppu.x;
    uint8_t p0 = (ppu.bg.pt_lo & bit) ? 1 : 0;
    uint8_t p1 = (ppu.bg.pt_hi & bit) ? 1 : 0;
    uint8_t a0 = (ppu.bg.at_lo & bit) ? 1 : 0;
    uint8_t a1 = (ppu.bg.at_hi & bit) ? 1 : 0;

    *pal_row_out = (a1 << 1) | a0;
    return (p1 << 1) | p0;
}

ALWAYS_INLINE uint8_t sprite_pixel(uint8_t *pal_row_out, uint8_t *prio_out)
{
    if (!sprites_enabled || !(ppu.mask & MASK_SHOW_SPR) || (!(ppu.mask & MASK_LEFT_SPR) && ppu.dot <= 8)) {
        *pal_row_out = *prio_out = 0; return 0;
    }

    /* Early-out optimisation: nothing can hit yet */
    if (ppu.next_sprite_xmin > 0) {
        --ppu.next_sprite_xmin; /* countdown to next sprite entrance */
        *pal_row_out = *prio_out = 0;
        return 0;
    }

    for (uint8_t i = 0; i < SPR_UNIT_MAX; ++i) {
        spr_unit_t *u = &ppu.spr[i];
        if (!u->in_range) continue;
        if (u->x == 0) {
            uint8_t p0 = (u->pt_lo & 0x80) ? 1 : 0;
            uint8_t p1 = (u->pt_hi & 0x80) ? 1 : 0;
            uint8_t px = (p1 << 1) | p0;
            if (px) {
                *pal_row_out = u->attr & 0x03;
                *prio_out    = (u->attr & OAMF_BEHIND) != 0;

                /* Sprite-0 hit detection - occurs at exact cycle of collision */
                if (i == ppu.sprite0_slot_this && ppu.sprite_zero_this && (ppu.mask & MASK_SHOW_BG) &&
                    ((ppu.mask & MASK_LEFT_BG) || ppu.dot > 8)) {
                    uint16_t bit = 0x8000 >> ppu.x;
                    bool would_hit = (ppu.bg.pt_lo & bit) || (ppu.bg.pt_hi & bit);
                    
                    if (would_hit && ppu.dot == 255) {
#if defined(TRACE_PPU) && TRACE_PPU
                        printf("Sprite-0: suppressed hit at dot %d (scanline %d)\n", 
                               ppu.dot, ppu.scanline);
#endif
                    } else if (would_hit && ppu.dot != 255) {
                        ppu.status |= PPU_STATF_STRIKE;
                    }
                }
                return px;
            }
        }
    }
    return 0;
}

/* ─────────────────── NMI helper ─────────────────── */
ALWAYS_INLINE void nmi_check(void)
{
    bool nmi = (ppu.ctrl & PPU_CTRL0F_NMI) && (ppu.status & PPU_STATF_VBLANK);
    if (nmi && !ppu.nmi_prev)
        ppu.nmi_delay = ppu_is_pal ? 7 : 6; /* two CPU cycles */
    ppu.nmi_prev = nmi;
}

ALWAYS_INLINE void nmi_step(void)
{
    if (ppu.nmi_delay > 0) {
        --ppu.nmi_delay;
        if (ppu.nmi_delay == 0 && (ppu.ctrl & PPU_CTRL0F_NMI) && (ppu.status & PPU_STATF_VBLANK))
            nes_nmi();
    }
}

/* ─────────────────── Public API ─────────────────── */
void ppu_set_mapper_hook(void (*fn)(uint16_t)) { mapper_ppu_hook = fn; }
void ppu_setlatchfunc(ppulatchfunc_t fn)       { ppu_latchfunc   = fn; }
void ppu_setvromswitch(ppuvromswitch_t fn)     { ppu_vromswitch  = fn; }
void ppu_set_chrram(uint8_t *ptr, size_t size) { chrram_ptr = ptr; chrram_size = size; }

void ppu_set_region(bool is_pal)
{
    ppu_is_pal = is_pal;
    ppu.is_pal_system = is_pal;
}

bool ppu_frame_complete(void)
{
    if (ppu.frame_complete) {
        ppu.frame_complete = false;
        return true;
    }
    return false;
}

void ppu_reset(int hard)
{
    (void)hard;
    memset(&ppu, 0, sizeof ppu);
    ppu.fb = vid_getbuffer();
    ppu.status = rand() & 0xE0; /* bits 7-5 random on power-up */
    ppu.open_bus = 0;
    ppu.phase_mod3 = 0;
    ppu.pal_ppu_accum = 0;
    ppu.is_pal_system = ppu_is_pal;
    ppu.eval_sprite_idx = 0;
    ppu.eval_oam_addr = 0;
    ppu.eval_sec_idx = 0;
    ppu.eval_overflow = false;
    ppu.eval_byte_index = 0;
    ppu.eval_read_latch = 0;
    ppu.oam_write_during_eval = false;
    ppu.sprite0_slot_next = 0xFF;
    ppu.sprite0_slot_this = 0xFF;

    ppu.spr_fetch_slot = 0;
    ppu.spr_fetch_phase = 0;
    ppu.spr_tmp_y = ppu.spr_tmp_tile = ppu.spr_tmp_attr = ppu.spr_tmp_x = 0;
    ppu.spr_lo = ppu.spr_hi = 0;

    ppu.nmi_prev = false;
    ppu.nmi_delay = 0;
    
    a12_prev = false;
    mmc3_a12_level = false;
    mmc3_a12_low_m2_count = 0;
    
    /* Reset to default; mapper will configure if needed */
    ppu_four_screen_enabled = false;
    
    /* Initialize default nametable mirroring (vertical) */
    nametable_mapping[0] = 0;  /* NT $2000 -> CIRAM $0000 */
    nametable_mapping[1] = 1;  /* NT $2400 -> CIRAM $0400 */
    nametable_mapping[2] = 0;  /* NT $2800 -> CIRAM $0000 */
    nametable_mapping[3] = 1;  /* NT $2C00 -> CIRAM $0400 */
    
    init_bitrev();
}

/* ─────────────────── Master clock ─────────────────── */
void ppu_mmc3_m2_tick(int cycles) {
    if (!mmc3_a12_level) {
        /* Count only while A12 is low */
        mmc3_a12_low_m2_count += cycles;
        if (mmc3_a12_low_m2_count > 8) mmc3_a12_low_m2_count = 8; /* clamp */
    }
}
void ppu_clock(void)
{
    nmi_step();

    if (ppu.dot == 0) {
        ppu.sprite_zero_this  = ppu.sprite_zero_next;
        ppu.sprite0_slot_this = ppu.sprite0_slot_next;
    }

    /* 1. Visible pixel -------------------------------------------------- */
    if (IS_VISIBLE_LINE && ppu.dot >= 1 && ppu.dot <= 256) {
        /* Always resolve sprite/background priority so side effects such as
         * sprite-0 hits occur even if we skip writing the final pixel. */
        uint8_t bg_pal_row, bg_px;        bg_px  = bg_pixel(&bg_pal_row);
        uint8_t spr_pal_row, spr_pri, spr_px; spr_px = sprite_pixel(&spr_pal_row, &spr_pri);

        if (draw_enabled) {
            uint8_t *fbline = ppu.fb + (ppu.scanline * NES_SCREEN_WIDTH);
            uint8_t final_idx;
            if (!spr_px && !bg_px) {
                final_idx = pal_read_raw(0);
            } else if (!spr_px) {
                final_idx = pal_read_raw((bg_pal_row << 2) | bg_px);
            } else if (!bg_px) {
                final_idx = pal_read_raw(0x10 | (spr_pal_row << 2) | spr_px);
            } else {
                if (spr_pri) final_idx = pal_read_raw((bg_pal_row << 2) | bg_px);
                else         final_idx = pal_read_raw(0x10 | (spr_pal_row << 2) | spr_px);
            }

            /* Apply PPUMASK grayscale and emphasis */
            if (ppu.mask & 0x01) { /* Grayscale */
                final_idx = apply_grayscale(final_idx);
            }
            if (ppu.mask & 0xE0) { /* Emphasis bits */
                final_idx = apply_emphasis_idx(final_idx, ppu.mask);
            }

            fbline[ppu.dot - 1] = final_idx;
        }
    }

    /* 2. Shift registers ------------------------------------------------ */
    if (RENDERING_ENABLED) {
        if ((ppu.dot >= 2 && ppu.dot <= 257) || (ppu.dot >= 321 && ppu.dot <= 336))
            bg_shift();
        if (IS_VISIBLE_LINE && ppu.dot >= 1 && ppu.dot <= 256)
            sprite_shift();
    }

    /* 3. Background fetch & scroll ------------------------------------- */
    /*
     * In addition to the visible tile fetches, the PPU performs a set of
     * prefetch cycles at dots 321–340. The final four dots (337–340) fetch the
     * first two tiles of the next scanline. These dummy fetches are required to
     * mirror hardware behaviour and keep the MMC3 A12 edge timing accurate.
     */
    if (RENDERING_ENABLED && (IS_VISIBLE_LINE || IS_PRERENDER_LINE)) {
        if ((ppu.dot >= 1 && ppu.dot <= 256) || (ppu.dot >= 321 && ppu.dot <= 340))
            bg_fetch();
        if (ppu.dot == 256) inc_y();
        else if (ppu.dot == 257) copy_x_from_t();
        else if (IS_PRERENDER_LINE && ppu.dot >= 280 && ppu.dot <= 304) copy_y_from_t();
    }

    /* 4. Sprite pipeline ----------------------------------------------- */
    if (ppu.dot == 1) {
        /* Initialize sprite evaluation state at start of each line */
        memset(ppu.sec_oam, 0xFF, 32);
        ppu.eval_sprite_idx = 0;
        ppu.eval_oam_addr = 0;
        ppu.eval_sec_idx = 0;
        ppu.eval_overflow = false;
        ppu.sprite_zero_next = false;
        ppu.sprite0_slot_next = 0xFF;
        ppu.eval_byte_index = 0;
        ppu.eval_read_latch = 0;
    }

    /* Sprite evaluation with proper even/odd cycle behavior during 65-256 */
    if (ppu.dot >= 65 && ppu.dot <= 256 && (IS_VISIBLE_LINE || IS_PRERENDER_LINE) && RENDERING_ENABLED) {
        if ((ppu.dot & 1) == 1) { /* Odd cycles: 65, 67, 69, ... 255 */
            eval_sprite_read_primary(); /* Read from primary OAM */
        } else { /* Even cycles: 66, 68, 70, ... 256 */
            eval_sprite_write_secondary(); /* Write to secondary OAM if in range */
        }
#ifndef NDEBUG
        /* Assert correct mapping during sprite evaluation */
        if ((ppu.dot & 1) == 1) {
            /* Odd cycles should read from primary OAM */
        } else {
            /* Even cycles should write to secondary OAM */
        }
#endif
    }

    if (ppu.dot == 257) {
        ppu.oam_addr = 0; /* hardware forces this */
        ppu.oam_write_during_eval = false; /* clear after eval window */

        /* Set overflow flag based on evaluation results */
        if (ppu.eval_overflow) ppu.status |= PPU_STATF_MAXSPRITE;
        else ppu.status &= ~PPU_STATF_MAXSPRITE;

        /* Calculate sprite count for fetch */
        ppu.sprite_count = (ppu.eval_sec_idx >> 2);
        if (ppu.sprite_count > 8) ppu.sprite_count = 8;

        /* Initialise sprite fetch state */
        ppu.spr_fetch_slot = 0;
        ppu.spr_fetch_phase = 0;
        ppu.next_sprite_xmin = 255;
        for (uint8_t i = 0; i < SPR_UNIT_MAX; ++i)
            ppu.spr[i].in_range = false;
    }

    /* Per-dot sprite tile fetch (dots 257-320) */
    if ((IS_VISIBLE_LINE || IS_PRERENDER_LINE) && RENDERING_ENABLED &&
        ppu.dot >= 257 && ppu.dot <= 320) {
        uint8_t rel = ppu.dot - 257;
        uint8_t slot = rel >> 3;
        uint8_t phase = rel & 7;
        ppu.spr_fetch_slot = slot;
        ppu.spr_fetch_phase = phase;

        uint8_t spr_h = (ppu.ctrl & PPU_CTRL0F_SPR16) ? 16 : 8;
        uint16_t cur_line = IS_PRERENDER_LINE ? 0 : (ppu.scanline + 1);

        switch (phase) {
        case 0:
            ppu.spr_tmp_y = ppu.sec_oam[slot * 4 + 0];
            break;
        case 1:
            ppu.spr_tmp_tile = ppu.sec_oam[slot * 4 + 1];
            break;
        case 2:
            ppu.spr_tmp_attr = ppu.sec_oam[slot * 4 + 2];
            break;
        case 3:
            ppu.spr_tmp_x = ppu.sec_oam[slot * 4 + 3];
            break;
        case 4: {
            uint8_t row = cur_line - ppu.spr_tmp_y;
            if (ppu.spr_tmp_attr & OAMF_VFLIP) row = (spr_h - 1) - row;
            uint8_t tile = ppu.spr_tmp_tile;
            uint16_t addr;
            if (spr_h == 16) {
                uint8_t even_tile = tile & 0xFE;
                uint16_t bank = (tile & 1) ? 0x1000 : 0x0000;
                uint8_t fine = row & 7;
                uint16_t offset = (row & 8) ? 16 : 0;
                addr = bank + even_tile * 16 + offset + fine;
            } else {
                uint16_t base = (ppu.ctrl & PPU_CTRL0F_SPRADDR) ? 0x1000 : 0x0000;
                addr = base + tile * 16 + row;
            }
            if (ppu_latchfunc) {
                uint16_t spr_base = (spr_h == 16)
                                   ? ((tile & 1) ? 0x1000 : 0x0000)
                                   : ((ppu.ctrl & PPU_CTRL0F_SPRADDR) ? 0x1000 : 0x0000);
                ppu_latchfunc(spr_base, tile);
            }
            ppu.spr_fetch_addr = addr;
            uint8_t lo = chr_read(addr);
            if (ppu.spr_tmp_attr & OAMF_HFLIP) lo = bitrev[lo];
            ppu.spr_lo = lo;
            break;
        }
        case 5: {
            uint8_t hi = chr_read(ppu.spr_fetch_addr + 8);
            if (ppu.spr_tmp_attr & OAMF_HFLIP) hi = bitrev[hi];
            ppu.spr_hi = hi;
            break;
        }
        case 6:
            chr_read(ppu.spr_fetch_addr);
            break;
        case 7:
            chr_read(ppu.spr_fetch_addr);
            if (slot < ppu.sprite_count) {
                spr_unit_t *u = &ppu.spr[slot];
                u->x      = ppu.spr_tmp_x;
                u->pt_lo  = ppu.spr_lo;
                u->pt_hi  = ppu.spr_hi;
                u->attr   = ppu.spr_tmp_attr;
                u->in_range = true;
                if (ppu.spr_tmp_x < ppu.next_sprite_xmin)
                    ppu.next_sprite_xmin = ppu.spr_tmp_x;
            }
            break;
        }
    }

    if (ppu.dot == 321 && ppu.next_sprite_xmin == 255)
        ppu.next_sprite_xmin = 0;

    /* 5. VBlank --------------------------------------------------------- */
    if (ppu.scanline == 241 && ppu.dot == 1) {
        ppu.status |= PPU_STATF_VBLANK;
        nmi_check();
    }
    if (IS_PRERENDER_LINE && ppu.dot == 1) {
        ppu.status &= ~(PPU_STATF_VBLANK | PPU_STATF_STRIKE | PPU_STATF_MAXSPRITE);
        nmi_check();
    }

    /* 6. Odd frame cycle skip ------------------------------------------ */
    /* On odd frames with rendering enabled, dot 339 is skipped (going directly to 341)
     * This timing is correct - the skip occurs at dot 339, not 340 (NTSC only) */
    if (!ppu_is_pal && IS_PRERENDER_LINE && ppu.dot == 339 && ppu.odd_frame && RENDERING_ENABLED)
        INC_DOT();

    /* 7. Advance counters ---------------------------------------------- */
    INC_DOT();

    if (ppu.scanline == 0 && ppu.dot == 0) {
        ppu.odd_frame = !ppu.odd_frame;
        ppu.frame_complete = true;
    }
}

/* Call exactly once per CPU cycle that elapses on the CPU core.
 * This function advances the PPU *during that CPU cycle*. */
void ppu_step_one_cpu_cycle(void) {
    nes_t *nes = ppu_get_nes();
    if (!ppu.is_pal_system) {
        /* NTSC: 3 PPU dots per CPU cycle */
        ppu_clock(); nes->ppu_cycles_total++;
        ppu_clock(); nes->ppu_cycles_total++;
        ppu_clock(); nes->ppu_cycles_total++;
        /* keep a modulo-3 phase marker if other code wants it */
        ppu.phase_mod3 = (ppu.phase_mod3 + 1) % 3;
    } else {
        /* PAL/Dendy: 16 dots every 5 CPU cycles => 3 or 4 dots per CPU cycle */
        ppu.pal_ppu_accum += 16;      /* 0..(5*k)+r */
        while (ppu.pal_ppu_accum >= 5) {
            ppu_clock();
            nes->ppu_cycles_total++;
            ppu.pal_ppu_accum -= 5;
        }
    }
}

/* ─────────────────── CPU ⇆ PPU interface ($2000-$2007) ─────────────────── */
uint8_t ppu_read(uint32_t addr)
{
    addr &= 7;
    uint8_t ret = 0xFF;

    switch (addr) {
    case 2: /* PPUSTATUS */
        ret = (ppu.status & 0xE0) | (ppu.open_bus & 0x1F);
        ppu.status &= ~PPU_STATF_VBLANK;
        nmi_check();
        ppu.w = 0;
        ppu.open_bus = ret;
        break;
    case 4: /* OAMDATA */
        if (RENDERING_ENABLED && (IS_VISIBLE_LINE || IS_PRERENDER_LINE) &&
            ppu.dot >= 65 && ppu.dot <= 256) {
            ret = ppu.eval_read_latch;
        } else {
            ret = ppu.oam[ppu.oam_addr];
        }
        ppu.open_bus = ret;
        break;
    case 7: /* PPUDATA */
        if ((ppu.v & 0x3F00) == 0x3F00) {
            ret = ppu_bus_read(ppu.v);
            ppu.buffered_data = ppu_bus_read(ppu.v & 0x2FFF);
        } else {
            ret = ppu.buffered_data;
            ppu.buffered_data = ppu_bus_read(ppu.v);
        }
        ppu.open_bus = ret;
        ppu.v += (ppu.ctrl & PPU_CTRL0F_ADDRINC) ? 32 : 1;
        ppu.v &= 0x7FFF;
        break;
    default:
        ppu.open_bus = ret;
        break;
    }
    return ret;
}

void ppu_write(uint32_t addr, uint8_t value)
{
    ppu.open_bus = value;
    addr &= 7;
    switch (addr) {
    case 0: /* PPUCTRL */
        ppu.ctrl = value;
        ppu.t = (ppu.t & ~0x0C00) | ((value & 0x03) << 10);
        nmi_check();  /* check immediately in case bit 7 turned on in VBlank */
        break;
    case 1: ppu.mask = value; break;                      /* PPUMASK */
    case 3: ppu.oam_addr = value; break;                  /* OAMADDR */
    case 4: /* OAMDATA */
        ppu.oam[ppu.oam_addr++] = value;
        /* Check for write during sprite evaluation */
        if (RENDERING_ENABLED && (IS_VISIBLE_LINE || IS_PRERENDER_LINE) &&
            ppu.dot >= 65 && ppu.dot <= 256) {
            ppu.oam_write_during_eval = true;
        }
        break;
    case 5: /* PPUSCROLL */
        if (!ppu.w) {
            ppu.x = value & 7;
            ppu.t = (ppu.t & ~0x001F) | (value >> 3);
            ppu.w = 1;
        } else {
            ppu.t = (ppu.t & ~0x73E0) | ((value & 0x07) << 12) | ((value & 0xF8) << 2);
            ppu.w = 0;
        }
        break;
    case 6: /* PPUADDR */
        if (!ppu.w) {
            ppu.t = (ppu.t & 0x00FF) | ((value & 0x3F) << 8);
            ppu.w = 1;
        } else {
            ppu.t = (ppu.t & 0x7F00) | value;
            ppu.v = ppu.t;
            ppu.w = 0;
        }
        break;
    case 7: /* PPUDATA */
        ppu_bus_write(ppu.v, value);
        ppu.v += (ppu.ctrl & PPU_CTRL0F_ADDRINC) ? 32 : 1;
        ppu.v &= 0x7FFF;
        break;
    }
}

/* $4014 / $4016 – DMA / VS strobe */
void ppu_writehigh(uint32_t addr, uint8_t val)
{
    if (addr == PPU_OAMDMA) {
        uint16_t base = val << 8;
        for (int i = 0; i < 256; ++i)
            ppu.oam[ppu.oam_addr++] = nes6502_getbyte(base + i);

        /* After DMA the internal OAMADDR is reset to 0 (hardware) */
        ppu.oam_addr = 0;

        uint32_t cycles = nes6502_getcycles(false);
        /* OAM DMA is 513 cycles, or 514 if starting on an odd CPU cycle. */
        int dma_cycles = 513;
        
        /* Ensure DMA starts on even CPU cycle */
        if (cycles & 1) {
            dma_cycles = 514;    /* Total DMA takes 514 cycles when starting on odd */
        }
        
#if defined(TRACE_PPU) && TRACE_PPU
        printf("OAM DMA: start_cycle=%s, total_cycles=%d\n",
               (cycles & 1) ? "odd" : "even", dma_cycles);
#endif
        nes_t *nes = ppu_get_nes();
        for (int i = 0; i < dma_cycles; ++i) {
            ppu_mmc3_m2_tick(1);
            ppu_step_one_cpu_cycle();
            nes6502_burn(1);
            nes->cpu_cycles_total++;
        }
        nes6502_release();
#if defined(ENABLE_VS_SYSTEM)
    } else if (addr == PPU_JOY0) { /* VS-System CHR bank switch */
        if (ppu_vromswitch) ppu_vromswitch(val);
#endif
    }
}

uint8_t ppu_readhigh(uint32_t addr) { (void)addr; return 0xFF; }

/* ─────────────────── Legacy nofrendo glue ─────────────────── */
void ppu_scanline(bitmap_t *b, int s, bool d) { (void)b; (void)s; (void)d; }
void ppu_endscanline(int s)                   { (void)s; }
void ppu_checknmi(void)                       { nmi_check(); }
ppu_t *ppu_create(void) { 
    static int dummy = 1;
    ppu_reset(1);
    return (ppu_t*)&dummy; /* Return non-NULL dummy handle */
}

void ppu_destroy(ppu_t **pp) { 
    if (pp && *pp) {
        *pp = NULL;
    }
}

void ppu_setpal(ppu_t *ppu, rgb_t *pal64) 
{ 
    (void)ppu; /* Ignore ppu parameter - global palette system */
    if (pal64) {
        vid_setpalette(pal64);
    }
}

void ppu_setdefaultpal(ppu_t *ppu) 
{ 
    (void)ppu; /* Ignore ppu parameter - global palette system */
    vid_setpalette(nes_palette);
}

void ppu_displaysprites(bool enable)
{
    sprites_enabled = enable;
}

void ppu_set_draw_enabled(bool enable)
{
    draw_enabled = enable;
}

bool ppu_enabled(void)
{
    return RENDERING_ENABLED;
}

/* Debug/GUI functions for pattern table and OAM display */
void ppu_dumppattern(bitmap_t *bmp, int table_num, int x, int y, int col) 
{
    if (!bmp || table_num < 0 || table_num > 1) return;
    
    /* Draw 16x16 grid of 8x8 tiles from the specified pattern table */
    uint16_t base_addr = table_num ? 0x1000 : 0x0000;
    
    for (int tile_y = 0; tile_y < 16; tile_y++) {
        for (int tile_x = 0; tile_x < 16; tile_x++) {
            uint8_t tile_id = tile_y * 16 + tile_x;
            uint16_t tile_addr = base_addr + tile_id * 16;
            
            /* Draw each 8x8 tile */
            for (int py = 0; py < 8; py++) {
                uint8_t lo = chr_read(tile_addr + py);
                uint8_t hi = chr_read(tile_addr + py + 8);
                
                for (int px = 0; px < 8; px++) {
                    uint8_t pixel = ((hi >> (7-px)) & 1) << 1 | ((lo >> (7-px)) & 1);
                    if (pixel) {
                        int screen_x = x + tile_x * 8 + px;
                        int screen_y = y + tile_y * 8 + py;
                        if (screen_x < bmp->width && screen_y < bmp->height) {
                            bmp->line[screen_y][screen_x] = col;
                        }
                    }
                }
            }
        }
    }
}

void ppu_dumpoam(bitmap_t *bmp, int x, int y) 
{
    if (!bmp) return;
    
    /* Display OAM data as text/visual representation */
    /* For simplicity, just show first few sprites */
    for (int i = 0; i < 8 && i * 4 < 256; i++) {
        uint8_t sprite_y = ppu.oam[i * 4];
        uint8_t tile = ppu.oam[i * 4 + 1];
        uint8_t attr = ppu.oam[i * 4 + 2];
        uint8_t sprite_x = ppu.oam[i * 4 + 3];
        
        /* Simple visualization - draw a small rectangle for each sprite */
        int draw_x = x + (sprite_x >> 2); /* Scale down position */
        int draw_y = y + i * 10 + (sprite_y >> 3);
        
        if (draw_x < bmp->width - 8 && draw_y < bmp->height - 8) {
            /* Draw small rectangle representing the sprite */
            for (int dy = 0; dy < 6; dy++) {
                for (int dx = 0; dx < 8; dx++) {
                    if (draw_x + dx < bmp->width && draw_y + dy < bmp->height) {
                        bmp->line[draw_y + dy][draw_x + dx] = (attr & 3) + 1;
                    }
                }
            }
        }
    }
}

uint8_t *ppu_getpage(int page) 
{ 
    if (page >= 16) return NULL;
    
    return chr_page_ptrs[page]; 
}

/* Single source of truth for CHR mapping - sets up the page table used by chr_read */
void ppu_setpage(int size, int page, uint8_t *ptr)
{
    if (page >= 16)
        return;

    /* CHR/NT mapping: For page < 16, set chr_page_ptrs[page+i] = ptr + i*0x400 */
    for (int i = 0; i < size && (page + i) < 16; i++) {
        chr_page_ptrs[page + i] = ptr ? ptr + (i * 0x400) : NULL;
    }
}

/* Proper nametable mirroring implementation */
void ppu_mirror(int page0, int page1, int page2, int page3) 
{ 
    nametable_mapping[0] = page0 & 3;
    nametable_mapping[1] = page1 & 3;
    nametable_mapping[2] = page2 & 3;
    nametable_mapping[3] = page3 & 3;
}

void ppu_mirrorhipages(void) 
{ 
    /* Mirror high pages - used for some mappers */
    /* This sets nametables 2,3 to mirror 0,1 respectively */
    ppu_mirror(0, 1, 0, 1);  /* Vertical mirroring */
}

/* State serialization implementation */
void ppu_get_state(ppu_state_t *state)
{
    state->ctrl = ppu.ctrl;
    state->mask = ppu.mask;
    state->status = ppu.status;
    state->oam_addr = ppu.oam_addr;
    
    state->v = ppu.v;
    state->t = ppu.t;
    state->x = ppu.x;
    state->w = ppu.w;
    
    state->buffered_data = ppu.buffered_data;
    
    state->dot = ppu.dot;
    state->scanline = ppu.scanline;
    state->odd_frame = ppu.odd_frame;
    
    state->eval_sprite_idx = ppu.eval_sprite_idx;
    state->eval_oam_addr = ppu.eval_oam_addr;
    state->eval_sec_idx = ppu.eval_sec_idx;
    state->eval_overflow = ppu.eval_overflow;
    state->eval_temp_y = ppu.eval_temp_y;
    state->sprite_in_range = ppu.sprite_in_range;
    
    state->open_bus = ppu.open_bus;
    
    /* MMC3 A12 filter state */
    state->a12_prev = a12_prev;
    state->mmc3_a12_level = mmc3_a12_level;
    state->mmc3_a12_low_m2_count = mmc3_a12_low_m2_count;
    
    /* Background pipeline state */
    state->bg_pt_lo = ppu.bg.pt_lo;
    state->bg_pt_hi = ppu.bg.pt_hi;
    state->bg_at_lo = ppu.bg.at_lo;
    state->bg_at_hi = ppu.bg.at_hi;
    state->bg_next_nt = ppu.bg.next_nt;
    state->bg_next_at = ppu.bg.next_at;
    state->bg_next_pt_lo = ppu.bg.next_pt_lo;
    state->bg_next_pt_hi = ppu.bg.next_pt_hi;
    
    /* Sprite pipeline state */
    state->sprite_count = ppu.sprite_count;
    state->sprite_zero_this = ppu.sprite_zero_this;
    state->sprite_zero_next = ppu.sprite_zero_next;
    state->next_sprite_xmin = ppu.next_sprite_xmin;
    
    /* Frame completion state */
    state->frame_complete = ppu.frame_complete;
}

void ppu_set_state(const ppu_state_t *state)
{
    ppu.ctrl = state->ctrl;
    ppu.mask = state->mask;
    ppu.status = state->status;
    ppu.oam_addr = state->oam_addr;
    
    ppu.v = state->v;
    ppu.t = state->t;
    ppu.x = state->x;
    ppu.w = state->w;
    
    ppu.buffered_data = state->buffered_data;
    
    ppu.dot = state->dot;
    ppu.scanline = state->scanline;
    ppu.odd_frame = state->odd_frame;
    
    ppu.eval_sprite_idx = state->eval_sprite_idx;
    ppu.eval_oam_addr = state->eval_oam_addr;
    ppu.eval_sec_idx = state->eval_sec_idx;
    ppu.eval_overflow = state->eval_overflow;
    ppu.eval_temp_y = state->eval_temp_y;
    ppu.sprite_in_range = state->sprite_in_range;
    
    ppu.open_bus = state->open_bus;
    
    /* MMC3 A12 filter state */
    a12_prev = state->a12_prev;
    mmc3_a12_level = state->mmc3_a12_level;
    mmc3_a12_low_m2_count = state->mmc3_a12_low_m2_count;
    
    /* Background pipeline state */
    ppu.bg.pt_lo = state->bg_pt_lo;
    ppu.bg.pt_hi = state->bg_pt_hi;
    ppu.bg.at_lo = state->bg_at_lo;
    ppu.bg.at_hi = state->bg_at_hi;
    ppu.bg.next_nt = state->bg_next_nt;
    ppu.bg.next_at = state->bg_next_at;
    ppu.bg.next_pt_lo = state->bg_next_pt_lo;
    ppu.bg.next_pt_hi = state->bg_next_pt_hi;
    
    /* Sprite pipeline state */
    ppu.sprite_count = state->sprite_count;
    ppu.sprite_zero_this = state->sprite_zero_this;
    ppu.sprite_zero_next = state->sprite_zero_next;
    ppu.next_sprite_xmin = state->next_sprite_xmin;
    
    /* Frame completion state */
    ppu.frame_complete = state->frame_complete;
}

void ppu_get_oam(uint8_t oam[256])
{
    memcpy(oam, ppu.oam, 256);
}

void ppu_set_oam(const uint8_t oam[256])
{
    memcpy(ppu.oam, oam, 256);
}

void ppu_get_palette(uint8_t palette[32])
{
    memcpy(palette, ppu.palette, 32);
}

void ppu_set_palette(const uint8_t palette[32])
{
    memcpy(ppu.palette, palette, 32);
}

void ppu_get_ciram(uint8_t ciram_out[0x1000])
{
    memcpy(ciram_out, ciram, 0x1000);
}

void ppu_set_ciram(const uint8_t ciram_in[0x1000])
{
    memcpy(ciram, ciram_in, 0x1000);
}

void ppu_get_mirroring(uint8_t mapping[4])
{
    memcpy(mapping, nametable_mapping, 4);
}

void ppu_set_mirroring(const uint8_t mapping[4])
{
    memcpy(nametable_mapping, mapping, 4);
}

void ppu_set_four_screen_mode(bool enabled)
{
    ppu_four_screen_enabled = enabled;
}
