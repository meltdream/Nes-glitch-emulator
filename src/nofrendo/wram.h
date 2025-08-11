/*--------------------------------------------------------------------
   wram.h  –  minimal public surface for the new WRAM pager
--------------------------------------------------------------------*/
#ifndef WRAM_H
#define WRAM_H

#include <stdint.h>     /* for uint32_t / uint8_t */

/* opaque forward-decl: comes from nes.h */
struct nes_s;

/*  called once per cart, right after rom_load() in nes_insertcart()  */
void wram_init(struct nes_s *nes);

/*  Only mapper 004 (MMC3 / MMC6) needs this call:                    */
void mmc_bankwram(int size, uint32_t addr, uint8_t bank);

#endif /* WRAM_H */
