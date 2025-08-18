/*
 *  nes_ppu.h — public interface for the revised NES PPU core
 *
 *  This header intentionally exposes only what nes_ppu.c really exports.
 *  If you still need the old debug helpers (ppu_getcontext, ppu_mirror…)
 *  you’ll have to add them back to the C file as real code.
 */

#ifndef NES_PPU_H
#define NES_PPU_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "bitmap.h"
#include "noftypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 *  Memory-mapped PPU registers
 * ------------------------------------------------------------------------- */
#define PPU_CTRL0   0x2000
#define PPU_CTRL1   0x2001
#define PPU_STAT    0x2002
#define PPU_OAMADDR 0x2003
#define PPU_OAMDATA 0x2004
#define PPU_SCROLL  0x2005
#define PPU_VADDR   0x2006
#define PPU_VDATA   0x2007

/* High-memory I/O */
#define PPU_OAMDMA  0x4014
#define PPU_JOY0    0x4016
#define PPU_JOY1    0x4017   /* VS-System strobe (rarely used) */

/* -------------------------------------------------------------------------
 *  $2000 – PPUCTRL flags
 * ------------------------------------------------------------------------- */
#define PPU_CTRL0F_NMI       0x80
#define PPU_CTRL0F_SPR16     0x20    /* OBJ height is 8×16      */
#define PPU_CTRL0F_BGADDR    0x10    /* BG pattern table select */
#define PPU_CTRL0F_SPRADDR   0x08    /* OBJ pattern table sel.  */
#define PPU_CTRL0F_ADDRINC   0x04    /* VRAM ++ = 32 if set     */
#define PPU_CTRL0F_NAMETAB   0x03    /* Base nametable (bits 0-1) */

/* $2001 – PPUMASK flags */
#define PPU_CTRL1F_OBJON     0x10
#define PPU_CTRL1F_BGON      0x08
#define PPU_CTRL1F_OBJMASK   0x04
#define PPU_CTRL1F_BGMASK    0x02

/* $2002 – PPUSTATUS flags */
#define PPU_STATF_VBLANK     0x80
#define PPU_STATF_STRIKE     0x40    /* Sprite-0 hit */
#define PPU_STATF_MAXSPRITE  0x20    /* > 8 sprites this line   */

/* OAM attribute bits */
#define OAMF_VFLIP   0x80
#define OAMF_HFLIP   0x40
#define OAMF_BEHIND  0x20   /* OBJ behind BG            */
#define OAMF_PALETTE 0x03   /* Low 2 bits – palette idx */

/* Hardware limit */
#define PPU_MAXSPRITE 8

/* -------------------------------------------------------------------------
 *  Mapper helper callback types
 * ------------------------------------------------------------------------- */
typedef void (*ppulatchfunc_t)(uint32_t base, uint8_t tile);
typedef void (*ppuvromswitch_t)(uint8_t bank);

/* Opaque forward declaration – details are private to nes_ppu.c */
typedef struct ppu_s ppu_t;

/* -------------------------------------------------------------------------
 *  Public API
 * ------------------------------------------------------------------------- */

/* ---- Mapper integration ------------------------------------------------- */
void ppu_set_mapper_hook(void (*fn)(uint16_t addr));
void ppu_setlatchfunc(ppulatchfunc_t fn);      /* MMC-2 / MMC-4 latch */
void ppu_setvromswitch(ppuvromswitch_t fn);    /* VS-System CHR bank  */
void ppu_set_chrram(uint8_t *ptr, size_t size);/* Cartridge CHR RAM   */

/* ---- Core lifecycle ----------------------------------------------------- */
void ppu_reset(int hard);   /* hard ≠ 0 → power-on state */
void ppu_clock(void);       /* advance one master PPU cycle */
void ppu_mmc3_m2_tick(int cycles); /* advance M2-based low counter */
void ppu_set_region(bool is_pal); /* select PAL or NTSC timing */
bool ppu_frame_complete(void); /* true if frame just completed */

/* ---- CPU ⇆ PPU bus ------------------------------------------------------ */
uint8_t ppu_read (uint32_t addr);             /* $2000-$2007 mirrored */
void    ppu_write(uint32_t addr, uint8_t val);

uint8_t ppu_readhigh (uint32_t addr);         /* $4014/$4016/$4017 …  */
void    ppu_writehigh(uint32_t addr, uint8_t val);

/* ---- Legacy nofrendo glue ---------------------------------------------- */
void   ppu_scanline   (bitmap_t *bmp, int scanline, bool draw_flag);
void   ppu_endscanline(int scanline);
void   ppu_checknmi   (void);

ppu_t *ppu_create(void);          /* trivial stub returns NULL */
void   ppu_destroy(ppu_t **ppu);  /* likewise – keeps API intact */

void   ppu_setpal       (ppu_t *ppu, rgb_t *pal64);
void   ppu_setdefaultpal(ppu_t *ppu);
void   ppu_displaysprites(bool enable);
void   ppu_set_draw_enabled(bool enable); /* skip final pixel writes when false */
bool   ppu_enabled(void);

/* Debug/GUI functions */
void   ppu_dumppattern(bitmap_t *bmp, int table_num, int x, int y, int col);
void   ppu_dumpoam(bitmap_t *bmp, int x, int y);

/* Legacy memory mapping functions - stubs for compatibility */
uint8_t *ppu_getpage(int page);
void     ppu_setpage(int size, int page, uint8_t *ptr);
void     ppu_mirror(int page0, int page1, int page2, int page3);
void     ppu_mirrorhipages(void);

/* State serialization interface */
typedef struct {
    /* PPU registers */
    uint8_t ctrl, mask, status;
    uint8_t oam_addr;
    
    /* Loopy registers */
    uint16_t v, t;
    uint8_t  x, w;
    
    /* Buffered data */
    uint8_t buffered_data;
    
    /* Timing state */
    int dot, scanline;
    bool odd_frame;
    
    /* Sprite evaluation state */
    uint8_t  eval_sprite_idx;
    uint16_t eval_oam_addr;
    uint8_t  eval_sec_idx;
    bool eval_overflow;
    uint8_t eval_temp_y;
    bool sprite_in_range;
    
    /* Open bus value */
    uint8_t open_bus;
    
    /* MMC3 A12 filter state */
    bool a12_prev;
    bool mmc3_a12_level;
    int  mmc3_a12_low_m2_count;
    
    /* Background pipeline state */
    uint16_t bg_pt_lo, bg_pt_hi;
    uint16_t bg_at_lo, bg_at_hi;
    uint8_t  bg_next_nt, bg_next_at, bg_next_pt_lo, bg_next_pt_hi;
    
    /* Sprite pipeline state */
    uint8_t sprite_count;
    bool sprite_zero_this, sprite_zero_next;
    uint8_t next_sprite_xmin;
    
    /* Frame completion state */
    bool frame_complete;
} ppu_state_t;

void ppu_get_state(ppu_state_t *state);
void ppu_set_state(const ppu_state_t *state);
void ppu_get_oam(uint8_t oam[256]);
void ppu_set_oam(const uint8_t oam[256]);
void ppu_get_palette(uint8_t palette[32]);
void ppu_set_palette(const uint8_t palette[32]);
void ppu_get_ciram(uint8_t ciram[0x1000]);
void ppu_set_ciram(const uint8_t ciram[0x1000]);
void ppu_get_mirroring(uint8_t mapping[4]);
void ppu_set_mirroring(const uint8_t mapping[4]);

/* 4-screen mode control */
void ppu_set_four_screen_mode(bool enabled);

#ifdef __cplusplus
}
#endif
#endif /* NES_PPU_H */
