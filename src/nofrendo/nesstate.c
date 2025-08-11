/*
** Nofrendo (c) 1998-2000 Matthew Conte (matt@conte.com)
**
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of version 2 of the GNU Library General 
** Public License as published by the Free Software Foundation.
**
** This program is distributed in the hope that it will be useful, 
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
** Library General Public License for more details.  To obtain a 
** copy of the GNU Library General Public License, write to the Free 
** Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
**
** Any permitted reproduction of these routines, in whole or in part,
** must bear this legend.
**
**
** nesstate.c
**
** state saving/loading
** $Id: nesstate.c,v 1.2 2001/04/27 14:37:11 neil Exp $
*/

#include "stdio.h"
#include "string.h"
#include "noftypes.h"
#include "nesstate.h"
#include "gui.h"
#include "nes.h"
#include "log.h"
#include "osd.h"
#include "libsnss.h"
#include "nes6502.h"

#define  FIRST_STATE_SLOT  0
#define  LAST_STATE_SLOT   9

static int state_slot = FIRST_STATE_SLOT;

/* Set the state-save slot to use (0 - 9) */
void state_setslot(int slot)
{
   /* Don't send a message if we're already at that slot */
   if (state_slot != slot && slot >= FIRST_STATE_SLOT
       && slot <= LAST_STATE_SLOT)
   {
      state_slot = slot;
      gui_sendmsg(GUI_WHITE, "State slot set to %d", slot);
   }
}

static int save_baseblock(nes_t *state, SNSS_FILE *snssFile)
{
   int i;

   ASSERT(state);

   nes6502_getcontext(state->cpu);

   snssFile->baseBlock.regA = state->cpu->a_reg;
   snssFile->baseBlock.regX = state->cpu->x_reg;
   snssFile->baseBlock.regY = state->cpu->y_reg;
   snssFile->baseBlock.regFlags = state->cpu->p_reg;
   snssFile->baseBlock.regStack = state->cpu->s_reg;
   snssFile->baseBlock.regPc = state->cpu->pc_reg;

   /* Save CPU state */
   memcpy(snssFile->baseBlock.cpuRam, state->cpu->mem_page[0], 0x800);
   
   /* Save PPU state using new cycle-accurate interface */
   ppu_state_t ppu_state;
   uint8_t mirroring[4];
   
   ppu_get_state(&ppu_state);
   ppu_get_oam(snssFile->baseBlock.spriteRam);
   ppu_get_ciram(snssFile->baseBlock.ppuRam);
   ppu_get_palette(snssFile->baseBlock.palette);
   ppu_get_mirroring(mirroring);
   
   snssFile->baseBlock.reg2000 = ppu_state.ctrl;
   snssFile->baseBlock.reg2001 = ppu_state.mask;
   snssFile->baseBlock.vramAddress = ppu_state.v;
   snssFile->baseBlock.spriteRamAddress = ppu_state.oam_addr;
   snssFile->baseBlock.tileXOffset = ppu_state.x;
   
   /* Save mirroring state */
   snssFile->baseBlock.mirrorState[0] = mirroring[0];
   snssFile->baseBlock.mirrorState[1] = mirroring[1];
   snssFile->baseBlock.mirrorState[2] = mirroring[2];
   snssFile->baseBlock.mirrorState[3] = mirroring[3];

   return 0;
}

static bool save_vramblock(nes_t *state, SNSS_FILE *snssFile)
{
   ASSERT(state);

   if (NULL == state->rominfo->vram)
      return -1;

   if (state->rominfo->vram_banks > 2)
   {
      log_printf("too many VRAM banks: %d\n", state->rominfo->vram_banks);
      return -1;
   }

   snssFile->vramBlock.vramSize = VRAM_8K * state->rominfo->vram_banks;

   memcpy(snssFile->vramBlock.vram, state->rominfo->vram, snssFile->vramBlock.vramSize);
   return 0;
}

static int save_sramblock(nes_t *state, SNSS_FILE *snssFile)
{
   int i;
   bool written = false;
   int sram_length;

   ASSERT(state);

   sram_length = state->rominfo->sram_banks * SRAM_1K;

   /* Check to see if any SRAM was written to */
   for (i = 0; i < sram_length; i++)
   {
      if (state->rominfo->sram[i])
      {
         written = true;
         break;
      }
   }

   if (false == written)
      return -1;

   if (state->rominfo->sram_banks > 8)
   {
      log_printf("Unsupported number of SRAM banks: %d\n", state->rominfo->sram_banks);
      return -1;
   }

   snssFile->sramBlock.sramSize = SRAM_1K * state->rominfo->sram_banks;

   /* TODO: this should not always be true!! */
   snssFile->sramBlock.sramEnabled = true;

   memcpy(snssFile->sramBlock.sram, state->rominfo->sram, snssFile->sramBlock.sramSize);

   return 0;
}

static int save_soundblock(nes_t *state, SNSS_FILE *snssFile)
{
   ASSERT(state);

   apu_getcontext(state->apu);

   /* rect 0 */
   snssFile->soundBlock.soundRegisters[0x00] = state->apu->rectangle[0].regs[0];
   snssFile->soundBlock.soundRegisters[0x01] = state->apu->rectangle[0].regs[1];
   snssFile->soundBlock.soundRegisters[0x02] = state->apu->rectangle[0].regs[2];
   snssFile->soundBlock.soundRegisters[0x03] = state->apu->rectangle[0].regs[3];
   /* rect 1 */
   snssFile->soundBlock.soundRegisters[0x04] = state->apu->rectangle[1].regs[0];
   snssFile->soundBlock.soundRegisters[0x05] = state->apu->rectangle[1].regs[1];
   snssFile->soundBlock.soundRegisters[0x06] = state->apu->rectangle[1].regs[2];
   snssFile->soundBlock.soundRegisters[0x07] = state->apu->rectangle[1].regs[3];
   /* triangle */
   snssFile->soundBlock.soundRegisters[0x08] = state->apu->triangle.regs[0];
   snssFile->soundBlock.soundRegisters[0x0A] = state->apu->triangle.regs[1];
   snssFile->soundBlock.soundRegisters[0x0B] = state->apu->triangle.regs[2];
   /* noise */
   snssFile->soundBlock.soundRegisters[0X0C] = state->apu->noise.regs[0];
   snssFile->soundBlock.soundRegisters[0X0E] = state->apu->noise.regs[1];
   snssFile->soundBlock.soundRegisters[0x0F] = state->apu->noise.regs[2];
   /* dmc */
   snssFile->soundBlock.soundRegisters[0x10] = state->apu->dmc.regs[0];
   snssFile->soundBlock.soundRegisters[0x11] = state->apu->dmc.regs[1];
   snssFile->soundBlock.soundRegisters[0x12] = state->apu->dmc.regs[2];
   snssFile->soundBlock.soundRegisters[0x13] = state->apu->dmc.regs[3];
   /* control */
   snssFile->soundBlock.soundRegisters[0x15] = state->apu->enable_reg;

   return 0;
}

static int save_mapperblock(nes_t *state, SNSS_FILE *snssFile)
{
   int i;
   ASSERT(state);

   mmc_getcontext(state->mmc);

   /* TODO: filthy hack in snss standard */
   /* We don't need to write mapper state for mapper 0 */
   if (0 == state->mmc->intf->number)
      return -1;

   nes6502_getcontext(state->cpu);

   /* TODO: snss spec should be updated, using 4kB ROM pages.. */
   for (i = 0; i < 4; i++)
      snssFile->mapperBlock.prgPages[i] = (state->cpu->mem_page[(i + 4) * 2] - state->rominfo->rom) >> 13;

   if (state->rominfo->vrom_banks)
   {
      for (i = 0; i < 8; i++)
         snssFile->mapperBlock.chrPages[i] = (ppu_getpage(i) - state->rominfo->vrom + (i * 0x400)) >> 10;
   }
   else
   {
      /* bleh! slight hack */
      for (i = 0; i < 8; i++)
         snssFile->mapperBlock.chrPages[i] = i;
   }

   if (state->mmc->intf->get_state)
      state->mmc->intf->get_state(&snssFile->mapperBlock);

   return 0;
}

static void load_baseblock(nes_t *state, SNSS_FILE *snssFile)
{
   int i;
   
   ASSERT(state);

   nes6502_getcontext(state->cpu);

   state->cpu->a_reg = snssFile->baseBlock.regA;
   state->cpu->x_reg = snssFile->baseBlock.regX;
   state->cpu->y_reg = snssFile->baseBlock.regY;
   state->cpu->p_reg = snssFile->baseBlock.regFlags;
   state->cpu->s_reg = snssFile->baseBlock.regStack;
   state->cpu->pc_reg = snssFile->baseBlock.regPc;

   /* Load CPU state */
   memcpy(state->cpu->mem_page[0], snssFile->baseBlock.cpuRam, 0x800);
   nes6502_setcontext(state->cpu);

   /* Load PPU state using new cycle-accurate interface */
   ppu_state_t ppu_state;
   
   ppu_state.ctrl = snssFile->baseBlock.reg2000;
   ppu_state.mask = snssFile->baseBlock.reg2001;
   ppu_state.status = 0;  /* Status register cleared on read */
   ppu_state.oam_addr = snssFile->baseBlock.spriteRamAddress;
   
   ppu_state.v = snssFile->baseBlock.vramAddress;
   ppu_state.t = 0;  /* Reset temp address register */
   ppu_state.x = snssFile->baseBlock.tileXOffset;
   ppu_state.w = 0;  /* Reset write toggle */
   
   ppu_state.buffered_data = 0;  /* Reset read buffer */
   
   /* Reset timing state to start of frame */
   ppu_state.dot = 0;
   ppu_state.scanline = 0;
   ppu_state.odd_frame = false;
   
   ppu_state.open_bus = 0;
   
   /* Restore PPU state */
   ppu_set_state(&ppu_state);
   ppu_set_oam(snssFile->baseBlock.spriteRam);
   ppu_set_ciram(snssFile->baseBlock.ppuRam);
   ppu_set_palette(snssFile->baseBlock.palette);
   ppu_set_mirroring(snssFile->baseBlock.mirrorState);
}

static void load_vramblock(nes_t *state, SNSS_FILE *snssFile)
{
   ASSERT(state);

   ASSERT(snssFile->vramBlock.vramSize <= VRAM_8K); /* can't handle more than this! */
   memcpy(state->rominfo->vram, snssFile->vramBlock.vram, snssFile->vramBlock.vramSize);
}

static void load_sramblock(nes_t *state, SNSS_FILE *snssFile)
{
   ASSERT(state);

   ASSERT(snssFile->sramBlock.sramSize <= SRAM_8K); /* can't handle more than this! */
   memcpy(state->rominfo->sram, snssFile->sramBlock.sram, snssFile->sramBlock.sramSize);
}

static void load_controllerblock(nes_t *state, SNSS_FILE *snssFile)
{
   UNUSED(state);
   UNUSED(snssFile);
}

static void load_soundblock(nes_t *state, SNSS_FILE *snssFile)
{
   int i;

   ASSERT(state);

   for (i = 0; i < 0x15; i++)
   {
      if (i != 0x13) /* do NOT trigger OAM DMA! */
         apu_write(0x4000 + i, snssFile->soundBlock.soundRegisters[i]);
   }
}

/* TODO: magic numbers galore */
static void load_mapperblock(nes_t *state, SNSS_FILE *snssFile)
{
   int i;
   
   ASSERT(state);

   mmc_getcontext(state->mmc);

   for (i = 0; i < 4; i++)
      mmc_bankrom(8, 0x8000 + (i * 0x2000), snssFile->mapperBlock.prgPages[i]);

   for (i = 0; i < 8; i++)
      mmc_bankvrom(1, i * 0x400, snssFile->mapperBlock.chrPages[i]);

   if (state->mmc->intf->set_state)
      state->mmc->intf->set_state(&snssFile->mapperBlock);

   mmc_setcontext(state->mmc);
}


int state_save(void)
{
   SNSS_FILE *snssFile;
   SNSS_RETURN_CODE status;
   char fn[PATH_MAX + 1], ext[5];
   nes_t *machine;

   /* get the pointer to our NES machine context */
   machine = nes_getcontextptr();
   ASSERT(machine);
   
   /* build our filename using the image's name and the slot number */
   strncpy(fn, machine->rominfo->filename, PATH_MAX - 4);
   
   ASSERT(state_slot >= FIRST_STATE_SLOT && state_slot <= LAST_STATE_SLOT);
   sprintf(ext, ".ss%d", state_slot);
   osd_newextension(fn, ext);

   /* open our state file for writing */
   status = SNSS_OpenFile(&snssFile, fn, SNSS_OPEN_WRITE);
   if (SNSS_OK != status)
      goto _error;

   /* now get all of our blocks */
   if (0 == save_baseblock(machine, snssFile))
   {
      status = SNSS_WriteBlock(snssFile, SNSS_BASR);
      if (SNSS_OK != status)
         goto _error;
   }

   if (0 == save_vramblock(machine, snssFile))
   {
      status = SNSS_WriteBlock(snssFile, SNSS_VRAM);
      if (SNSS_OK != status)
         goto _error;
   }

   if (0 == save_sramblock(machine, snssFile))
   {
      status = SNSS_WriteBlock(snssFile, SNSS_SRAM);
      if (SNSS_OK != status)
         goto _error;
   }

   if (0 == save_soundblock(machine, snssFile))
   {
      status = SNSS_WriteBlock(snssFile, SNSS_SOUN);
      if (SNSS_OK != status)
         goto _error;
   }

   if (0 == save_mapperblock(machine, snssFile))
   {
      status = SNSS_WriteBlock(snssFile, SNSS_MPRD);
      if (SNSS_OK != status)
         goto _error;
   }

   /* close the file, we're done */
   status = SNSS_CloseFile(&snssFile);
   if (SNSS_OK != status)
      goto _error;

   gui_sendmsg(GUI_GREEN, "State %d saved", state_slot);
   return 0;

_error:
   gui_sendmsg(GUI_RED, "error: %s", SNSS_GetErrorString(status));
   SNSS_CloseFile(&snssFile);
   return -1;
}

int state_load(void)
{
   SNSS_FILE *snssFile;
   SNSS_RETURN_CODE status;
   SNSS_BLOCK_TYPE block_type;
   char fn[PATH_MAX + 1], ext[5];
   unsigned int i;
   nes_t *machine;

   /* get our machine's context pointer */
   machine = nes_getcontextptr();
   ASSERT(machine);

   /* build the state name using the ROM's name and the slot number */
   strncpy(fn, machine->rominfo->filename, PATH_MAX - 4);

   ASSERT(state_slot >= FIRST_STATE_SLOT && state_slot <= LAST_STATE_SLOT);
   sprintf(ext, ".ss%d", state_slot);
   osd_newextension(fn, ext);
   
   /* open our file for writing */
   status = SNSS_OpenFile(&snssFile, fn, SNSS_OPEN_READ);
   if (SNSS_OK != status)
      goto _error;

   /* iterate through all present blocks */
   for (i = 0; i < snssFile->headerBlock.numberOfBlocks; i++)
   {
      status = SNSS_GetNextBlockType(&block_type, snssFile);
      if (SNSS_OK != status)
         goto _error;

      status = SNSS_ReadBlock(snssFile, block_type);
      if (SNSS_OK != status)
         goto _error;

      switch (block_type)
      {
      case SNSS_BASR:
         load_baseblock(machine, snssFile);
         break;

      case SNSS_VRAM:
         load_vramblock(machine, snssFile);
         break;

      case SNSS_SRAM:
         load_sramblock(machine, snssFile);
         break;
      
      case SNSS_MPRD:
         load_mapperblock(machine, snssFile);
         break;
      
      case SNSS_CNTR:
         load_controllerblock(machine, snssFile);
         break;
      
      case SNSS_SOUN:
         load_soundblock(machine, snssFile);
         break;
      
      case SNSS_UNKNOWN_BLOCK:
      default:
         log_printf("unknown SNSS block type\n");
         break;
      }
   }

   /* close file, we're done */
   status = SNSS_CloseFile(&snssFile);
   if (SNSS_OK != status)
      goto _error;

   gui_sendmsg(GUI_GREEN, "State %d restored", state_slot);

   return 0;

_error:
   gui_sendmsg(GUI_RED, "error: %s", SNSS_GetErrorString(status));
   SNSS_CloseFile(&snssFile);
   return -1;
}

/*
** $Log: nesstate.c,v $
** Revision 1.2  2001/04/27 14:37:11  neil
** wheeee
**
** Revision 1.1.1.1  2001/04/27 07:03:54  neil
** initial
**
** Revision 1.4  2000/11/11 14:52:33  matt
** states finally bleepin' work again
**
** Revision 1.3  2000/11/09 14:07:28  matt
** state load fixed, state save mostly fixed
**
** Revision 1.2  2000/10/25 00:23:16  matt
** makefiles updated for new directory structure
**
** Revision 1.1  2000/10/24 12:20:28  matt
** initial revision
**
*/
