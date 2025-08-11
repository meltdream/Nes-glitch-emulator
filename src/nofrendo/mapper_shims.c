/*
 * Thin shims so the full MMC3 (mapper 4) code links on the esp_8_bit fork.
 * They keep the existing behaviour (WRAM always enabled, IRQ line cleared
 * next CPU cycle) but expose the symbols the mapper wants.
 */

#include "noftypes.h"
#include "nes.h"
#include "nes_mmc.h"


void nes_irq(void)
{
    ext_irq_line = 1;
    nes6502_irq();
}

/* ------------------------------------------------------------------ */
/*  IRQ acknowledge – desktop core de-asserts the line immediately.    */
/*  Our 6502 already drops it on the next fetch, so a stub is fine.    */
/* ------------------------------------------------------------------ */
void nes_irq_ack(void)
{
    ext_irq_line   = 0;          /* release the line          */
    nes6502_clear_pending_irq();
}
