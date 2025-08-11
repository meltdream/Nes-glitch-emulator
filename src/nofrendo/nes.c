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
** nes.c                      – NES hardware-level support
** (adapted to the cycle-accurate PPU, August 2025)
*/

#ifndef _WIN32
#define _POSIX_C_SOURCE 199309L
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <time.h>
#endif

#include "noftypes.h"
#include "nes6502.h"
#include "log.h"
#include "osd.h"
#include "gui.h"
#include "nes.h"
#include "nes_apu.h"
#include "new_ppu.h"
#include "nes_rom.h"
#include "nes_mmc.h"
#include "vid_drv.h"
#include "nofrendo.h"

#define  NES_CLOCK_DIVIDER    12
//#define  NES_MASTER_CLOCK     21477272.727272727272
#define  NES_MASTER_CLOCK     (236250000 / 11)
/* Legacy scanline timing constant - no longer used with cycle-accurate PPU */
#define  NES_FIQ_PERIOD       29830  /* APU frame IRQ every 29,830 CPU cycles (NTSC) */

#define  NES_RAMSIZE          0x800

#define  NES_SKIP_LIMIT       (NES_REFRESH_RATE / 5)   /* 12 or 10, depending on PAL/NTSC */

/* Error codes */
#define  NESERR_OUT_OF_MEMORY  -1
#define  NESERR_BAD_FILE       -2

/* PPU_PER_CPU constant removed - now handled by catch-up scheduler */

static nes_t nes;

int nes_isourfile(const char *filename)
{
   return rom_checkmagic(filename);
}

/* TODO: just asking for problems -- please remove */

nes_t *nes_getcontextptr(void)
{
   return &nes;
}

void nes_getcontext(nes_t *machine)
{
   apu_getcontext(nes.apu);
   nes6502_getcontext(nes.cpu);
   mmc_getcontext(nes.mmc);

   *machine = nes;
}

void nes_setcontext(nes_t *machine)
{
   ASSERT(machine);
   nes = *machine;

   apu_setcontext(nes.apu);
   nes6502_setcontext(nes.cpu);
   mmc_setcontext(nes.mmc);
}

/* Forward declarations for memory handlers */
static uint8 ram_read(uint32 address);
static void ram_write(uint32 address, uint8 value);
static uint8 io_read(uint32 address);
static void io_write(uint32 address, uint8 value);

#define LAST_MEMORY_HANDLER   { -1, -1, NULL }

static nes6502_memread default_readhandler[] =
{
   { 0x0800, 0x1FFF, ram_read     },
   { 0x2000, 0x3FFF, ppu_read     },
   { 0x4000, 0x4015, apu_read     },
   { 0x4016, 0x4017, io_read      },
   LAST_MEMORY_HANDLER
};

static nes6502_memwrite default_writehandler[] =
{
   { 0x0800, 0x1FFF, ram_write     },
   { 0x2000, 0x3FFF, ppu_write     },
   { 0x4000, 0x4013, apu_write     },
   { 0x4015, 0x4015, apu_write     },
   { 0x4014, 0x4014, ppu_writehigh },
   { 0x4016, 0x4016, io_write      },
   { 0x4017, 0x4017, apu_write     },
   LAST_MEMORY_HANDLER
};

/* this big nasty boy sets up the address handlers that the CPU uses */
static void build_address_handlers(nes_t *machine)
{
   int count, num_handlers = 0;
   mapintf_t *intf;
   
   ASSERT(machine);
   
   /* Validate machine structure pointers - comprehensive safety checks */
   if (!machine) return;
   
   /* Initialize all handler arrays to safe defaults first */
   memset(machine->readhandler, 0, sizeof(nes6502_memread) * MAX_MEM_HANDLERS);
   memset(machine->writehandler, 0, sizeof(nes6502_memwrite) * MAX_MEM_HANDLERS);
   
   /* Check for valid mmc and intf before accessing */
   if (!machine->mmc || !machine->mmc->intf) {
      /* MMC not set up yet - only set up default handlers */
      
      /* Set up default read handlers with bounds checking */
      for (count = 0; count < sizeof(default_readhandler)/sizeof(default_readhandler[0]) && num_handlers < MAX_MEM_HANDLERS - 1; count++) {
         if (NULL == default_readhandler[count].read_func)
            break;
         machine->readhandler[num_handlers] = default_readhandler[count];
         num_handlers++;
      }
      /* Write explicit terminator */
      machine->readhandler[num_handlers].min_range = 0xFFFFFFFF;
      machine->readhandler[num_handlers].max_range = 0xFFFFFFFF;
      machine->readhandler[num_handlers].read_func = NULL;

      /* Set up default write handlers with bounds checking */
      num_handlers = 0;
      for (count = 0; count < sizeof(default_writehandler)/sizeof(default_writehandler[0]) && num_handlers < MAX_MEM_HANDLERS - 1; count++) {
         if (NULL == default_writehandler[count].write_func)
            break;
         machine->writehandler[num_handlers] = default_writehandler[count];
         num_handlers++;
      }
      /* Write explicit terminator */
      machine->writehandler[num_handlers].min_range = 0xFFFFFFFF;
      machine->writehandler[num_handlers].max_range = 0xFFFFFFFF;
      machine->writehandler[num_handlers].write_func = NULL;
      return;
   }
   
   intf = machine->mmc->intf;

   /* Set up default read handlers first */
   for (count = 0; count < sizeof(default_readhandler)/sizeof(default_readhandler[0]) && num_handlers < MAX_MEM_HANDLERS - 1; count++) {
      if (NULL == default_readhandler[count].read_func)
         break;
      machine->readhandler[num_handlers] = default_readhandler[count];
      num_handlers++;
   }

   /* Add MMC-specific read handlers - with safe bounds checking */
   for (count = 0; count < MAX_MEM_HANDLERS && num_handlers < MAX_MEM_HANDLERS - 1; count++) {
      if (NULL == intf->mem_read[count].read_func)
         break;
      machine->readhandler[num_handlers].min_range = intf->mem_read[count].min_range;
      machine->readhandler[num_handlers].max_range = intf->mem_read[count].max_range;
      machine->readhandler[num_handlers].read_func = intf->mem_read[count].read_func;
      num_handlers++;
   }
   /* Terminator */
   machine->readhandler[num_handlers].min_range = 0xFFFFFFFF;
   machine->readhandler[num_handlers].max_range = 0xFFFFFFFF;
   machine->readhandler[num_handlers].read_func = NULL;

   /* Set up default write handlers */
   num_handlers = 0;
   for (count = 0; count < sizeof(default_writehandler)/sizeof(default_writehandler[0]) && num_handlers < MAX_MEM_HANDLERS - 1; count++) {
      if (NULL == default_writehandler[count].write_func)
         break;
      machine->writehandler[num_handlers] = default_writehandler[count];
      num_handlers++;
   }

   /* Add MMC-specific write handlers - with safe bounds checking */
   for (count = 0; count < MAX_MEM_HANDLERS && num_handlers < MAX_MEM_HANDLERS - 1; count++) {
      if (NULL == intf->mem_write[count].write_func)
         break;
      machine->writehandler[num_handlers].min_range = intf->mem_write[count].min_range;
      machine->writehandler[num_handlers].max_range = intf->mem_write[count].max_range;
      machine->writehandler[num_handlers].write_func = intf->mem_write[count].write_func;
      num_handlers++;
   }
   /* Terminator */
   machine->writehandler[num_handlers].min_range = 0xFFFFFFFF;
   machine->writehandler[num_handlers].max_range = 0xFFFFFFFF;
   machine->writehandler[num_handlers].write_func = NULL;
}

static uint8 ram_read(uint32 address)
{
   return nes.cpu->mem_page[0][address & (NES_RAMSIZE - 1)];
}

static void ram_write(uint32 address, uint8 value)
{
   nes.cpu->mem_page[0][address & (NES_RAMSIZE - 1)] = value;
}

static uint8 read_protect(uint32 address)
{
   UNUSED(address);
   return 0xFF;
}

static void write_protect(uint32 address, uint8 value)
{
   UNUSED(address);
   UNUSED(value);
}

/* -------- Controller I/O ($4016/$4017) -------- */
static uint8 joy_state[2] = {0, 0};   /* platform input should set these (1=pressed per bit A,B,Select,Start,Up,Down,Left,Right) */
static uint8 joy_shift[2] = {0, 0};
static uint8 joy_strobe   = 0;

/* Optional: expose a setter the platform layer can call after osd_getinput() */
void nes_set_joy_state(int port, uint8 state) {
   if ((unsigned)port < 2) joy_state[port] = state;
}

static void io_write(uint32 address, uint8 value)
{
   if (address == 0x4016) {
      uint8 new_strobe = value & 1;
      /* Rising edge or level-high keeps reloading the shift registers */
      if (new_strobe) {
         joy_shift[0] = joy_state[0];
         joy_shift[1] = joy_state[1];
      } else if (joy_strobe) {
         /* 1->0 transition: latch once */
         joy_shift[0] = joy_state[0];
         joy_shift[1] = joy_state[1];
      }
      joy_strobe = new_strobe;
   }
}

static uint8 io_read(uint32 address)
{
   uint8 ret = 0x40; /* open-bus high bits approximated; D6 often reads as open bus */
   int  port = (address == 0x4017) ? 1 : 0;

   uint8 bit;
   if (joy_strobe) {
      /* Real-time A button while strobe high */
      bit = joy_state[port] & 1;
   } else {
      bit = joy_shift[port] & 1;
      /* shift only when not strobing */
      joy_shift[port] = (joy_shift[port] >> 1) | 0x80; /* after 8 reads, returns 1s */
   }
   ret |= bit;
   return ret;
}


static uint8 nes_clearfiq(void)
{
   if (nes.fiq_occurred)
   {
      nes.fiq_occurred = false;
      return 0x40;
   }

   return 0;
}

void nes_setfiq(uint8 value)
{
   nes.fiq_state = value;
   nes.fiq_cycles = (int) NES_FIQ_PERIOD;
}

static void nes_checkfiq(int cycles_delta)
{
   while (cycles_delta > 0) {
      int cycles_to_process = (cycles_delta < nes.fiq_cycles) ? cycles_delta : nes.fiq_cycles;
      nes.fiq_cycles -= cycles_to_process;
      cycles_delta -= cycles_to_process;
      
      if (nes.fiq_cycles <= 0)
      {
         nes.fiq_cycles += (int) NES_FIQ_PERIOD;
         if (0 == (nes.fiq_state & 0xC0))
         {
            nes.fiq_occurred = true;
            nes6502_irq();
         }
      }
   }
}

static void ppu_catchup(void)
{
   uint64_t ppu_target;
   
   if (nes.is_pal_region) {
      /* Use delta-based calculation to prevent drift */
      uint64_t cpu_delta = nes.cpu_cycles_total - nes.last_catchup_cpu_cycles;
      
      /* Apply 16/5 ratio to delta only */
      uint64_t ppu_delta_base = (cpu_delta * 16) / 5;
      uint64_t remainder = (cpu_delta * 16) % 5;
      
      /* Add remainder to accumulator */
      nes.pal_fractional_acc += remainder;
      
      /* Extract whole cycles from accumulator */
      uint64_t extra_cycles = nes.pal_fractional_acc / 5;
      nes.pal_fractional_acc %= 5;
      
      ppu_target = nes.ppu_cycles_total + ppu_delta_base + extra_cycles;
      nes.last_catchup_cpu_cycles = nes.cpu_cycles_total;
   } else {
      ppu_target = nes.cpu_cycles_total * 3;
   }
   
   while (nes.ppu_cycles_total < ppu_target) {
      ppu_clock();
      nes.ppu_cycles_total++;
   }
   
   /* Debug assertion to verify timing invariants */
   #ifdef DEBUG
   /* PAL is exactly 16/5 = 3.2 PPU per CPU */
   uint64_t expected_min = nes.cpu_cycles_total * (nes.is_pal_region ? 16 : 3) / (nes.is_pal_region ? 5 : 1);
   uint64_t expected_max = expected_min + 1;
   assert(nes.ppu_cycles_total >= expected_min && nes.ppu_cycles_total <= expected_max);
   #endif
}

void nes_nmi(void)
{
   nes6502_nmi();
}

void nes_setregion(bool is_pal)
{
   nes.is_pal_region = is_pal;
}

void nes_togglepause(void)
{
   nes.pause = !nes.pause;
}

void nes_poweroff(void)
{
   nes.poweroff = true;
}

/* ──────────────────────────────────────────────────────────────
 * Catch-up scheduler frame renderer
 * ────────────────────────────────────────────────────────────── */
void nes_renderframe(bool draw_flag)
{
   /* Tell the PPU whether the results of this frame should be drawn.  The
    * PPU still executes all cycles so timing and game state remain accurate,
    * but it can skip writing pixels to the framebuffer when drawing is
    * disabled. */
   ppu_set_draw_enabled(draw_flag);

   mapintf_t *mapintf = nes.mmc->intf;
   bool frame_done = false;

   /* Run until PPU signals actual frame completion (handles variable 89341/89342 cycle frames) */
   while (!frame_done)
   {
      /* Execute CPU instruction */
      int cpu_cycles = nes6502_execute(1);
      nes.cpu_cycles_total += cpu_cycles;

      /* APU frame-IRQ advancement in CPU stepping path */
      nes_checkfiq(cpu_cycles);

      /* Run PPU catch-up scheduler */
      ppu_catchup();
      
      /* Check frame completion only once per iteration to avoid race conditions */
      frame_done = ppu_frame_complete();
   }

   /* mapper vblank callback */
   if (mapintf->vblank)
      mapintf->vblank();
}

static void system_video(bool draw)
{
   /* When draw is false we skip all video work. The emulator core still
    * advances a full frame but nothing is pushed to the display. */
   if (false == draw)
      return;

   /* overlay our GUI on top of it */
   //gui_frame(true);

   /* blit to screen */
   vid_flush();
}

/* main emulation loop */
void nes_emulate(void)
{
   int last_ticks, frames_to_render;

   osd_setsound(nes.apu->process);

   last_ticks = nofrendo_ticks;
   frames_to_render = 0;
   nes.fiq_cycles = (int) NES_FIQ_PERIOD;

   while (false == nes.poweroff)
   {
      /* Poll input once per iteration regardless of rendering state */
      osd_getinput();

      int current_ticks = nofrendo_ticks;
      if (current_ticks != last_ticks)
      {
         int tick_diff = current_ticks - last_ticks;

         frames_to_render += tick_diff;
         /* gui_tick(tick_diff); */
         last_ticks = current_ticks;
      }

      if (true == nes.pause)
      {
         /* TODO: dim the screen, and pause/silence the apu */
         system_video(true);
         frames_to_render = 0;
      }
      else if (frames_to_render > 1)
      {
         frames_to_render--;
         nes_renderframe(false);
         system_video(false);
      }
      else if ((1 == frames_to_render && true == nes.autoframeskip) ||
               false == nes.autoframeskip)
      {
         frames_to_render = 0;
         nes_renderframe(true);
         system_video(true);
      }
      else
      {
#ifdef _WIN32
         Sleep(1);
#else
         struct timespec ts;
         ts.tv_sec = 0;
         ts.tv_nsec = 1000000;
         nanosleep(&ts, NULL);
#endif
      }
   }
}

static void mem_trash(uint8 *buffer, int length)
{
   int i;

   ASSERT(buffer);

   for (i = 0; i < length; i++)
      buffer[i] = rand();
}

/* Reset NES hardware */
void nes_reset(int reset_type)
{
   if (HARD_RESET == reset_type)
   {
      memset(nes.cpu->mem_page[0], 0, NES_RAMSIZE);
      if (nes.rominfo->vram)
         mem_trash(nes.rominfo->vram, 0x2000 * nes.rominfo->vram_banks);
   }

   apu_reset();
   ppu_reset(reset_type);
   mmc_reset();
   
   /* Reset alignment: clock PPU for 7 * region_ratio CPU cycles before CPU reset */
   int ppu_cycles_before_cpu_reset = nes.is_pal_region ? (7 * 16) / 5 : 7 * 3;
   for (int i = 0; i < ppu_cycles_before_cpu_reset; i++) {
      ppu_clock();
      nes.ppu_cycles_total++;
   }
   nes.cpu_cycles_total += 7;  /* Account for the CPU cycles we're simulating */
   
   nes6502_reset();

   nes.fiq_occurred = false;
   nes.fiq_state = 0;
   nes.fiq_cycles = (int) NES_FIQ_PERIOD;
   nes.pal_fractional_acc = 0;
   /* Legacy scanline timing removed - now handled by cycle-accurate PPU */
}

static int nes_init(void)
{
   int error;

   /* allocate our main structs */
   nes.cpu = malloc(sizeof(nes6502_context));
   nes.ppu = ppu_create();
   nes.apu = malloc(sizeof(apu_t));
   nes.mmc = malloc(sizeof(mmc_t));
   if (NULL == nes.cpu || NULL == nes.apu || NULL == nes.mmc)
      return NESERR_OUT_OF_MEMORY;

   /* Initialize CPU context */
   memset(nes.cpu, 0, sizeof(nes6502_context));
   
   /* Initialize handler arrays to safe defaults BEFORE assigning to CPU */
   memset(nes.readhandler, 0, sizeof(nes.readhandler));
   memset(nes.writehandler, 0, sizeof(nes.writehandler));
   
   nes.cpu->read_handler = nes.readhandler;
   nes.cpu->write_handler = nes.writehandler;

   /* Initialize APU */
   if (0 != (error = apu_init(nes.apu, NULL)))
      return error;

   if (0 != (error = mmc_init(nes.mmc)))
      return error;

   /* setcart removed - initialization handled elsewhere */

   build_address_handlers(&nes);

   nes.poweroff = false;
   nes.pause = false;
   nes.autoframeskip = true;
   nes.scanline_cycles = 0;
   nes.scanline = 0;
   nes.fiq_occurred = false;
   nes.fiq_state = 0;
   nes.fiq_cycles = (int) NES_FIQ_PERIOD;
   
   /* Initialize catch-up scheduler */
   nes.cpu_cycles_total = 0;
   nes.ppu_cycles_total = 0;
   nes.last_catchup_cpu_cycles = 0;
   nes.is_pal_region = false;  /* Default to NTSC, can be overridden */

   return 0;
}

nes_t *nes_create(void)
{
   int error;

   if (0 != (error = nes_init()))
      return NULL;

   nes_reset(HARD_RESET);

   return &nes;
}

void nes_destroy(nes_t **machine)
{
   if (machine && *machine) {
      if ((*machine)->mmc)
         mmc_destroy((*machine)->mmc);
      if ((*machine)->apu)
         apu_destroy((*machine)->apu);
      if ((*machine)->ppu)
         ppu_destroy(&(*machine)->ppu);
      if ((*machine)->cpu)
         free((*machine)->cpu);

      if ((*machine)->rominfo)
         rom_freeinfo((*machine)->rominfo, (*machine)->ppu);

      memset(*machine, 0, sizeof(nes_t));
      *machine = NULL;
   }
}

int nes_insertcart(const char *filename, nes_t *machine)
{
   int error;
   nes_t *nes_ptr = machine ? machine : &nes;

   if (NULL != nes_ptr->rominfo)
      rom_freeinfo(nes_ptr->rominfo, nes_ptr->ppu);

   if (NULL == (nes_ptr->rominfo = rom_load(filename)))
      return NESERR_BAD_FILE;

   if (0 != (error = mmc_setcart(nes_ptr)))
      return error;

   nes_reset(HARD_RESET);

   return 0;
}

/* -------------------------------------------------------------------------
   CVS history below – kept verbatim for posterity
   ------------------------------------------------------------------------- */

/*
** $Log: nes.c,v $
** Revision 1.33  2000/08/12 22:58:15  matt
** preliminary vblank
**
** Revision 1.32  2000/07/30 04:01:37  matt
** nes_setcontext stuff
**
** Revision 1.31  2000/07/27 03:34:48  matt
** minor jugglings
**
** Revision 1.30  2000/07/27 02:55:23  matt
** nes_emulate went through detox
**
** Revision 1.27  2000/07/27 02:49:18  matt
** cleaner flow in nes_emulate
**
** Revision 1.26  2000/07/27 01:17:09  matt
** nes_insertrom -> nes_insertcart
**
** Revision 1.25  2000/07/26 21:36:14  neil
** Big honkin' change -- see the mailing list
**
** Revision 1.24  2000/07/25 02:25:53  matt
** safer xxx_destroy calls
**
** Revision 1.23  2000/07/24 04:32:40  matt
** fixed invocation order problem
**
** Revision 1.22  2000/07/21 04:15:34  matt
** minor bugs fixed
**
** Revision 1.21  2000/07/19 15:00:27  matt
** cart name part of rominfo
**
** Revision 1.20  2000/07/17 01:53:28  matt
** nes_setcontext now working
**
** Revision 1.19  2000/07/15 23:10:25  matt
** nes_reset works again, PPU fix, and some other crap
**
** Revision 1.18  2000/07/08 20:46:39  matt
** yet more housekeeping
**
** Revision 1.17  2000/07/06 15:06:16  matt
** fat, drunk, and stupid is no way to go through life
**
** Revision 1.16  2000/07/05 05:47:31  matt
** moved some stuff around
**
** Revision 1.15  2000/07/05 04:32:35  matt
** housekeeping
**
** Revision 1.14  2000/07/05 03:53:01  matt
** changed a few constants, removed direct references
**
** Revision 1.13  2000/07/05 00:09:07  matt
** termite chuck, termite chuck
**
** Revision 1.12  2000/07/04 21:08:40  matt
** cleaned up some header files
**
** Revision 1.11  2000/07/04 18:32:04  matt
** finally got one
**
** Revision 1.10  2000/07/04 17:18:31  matt
** more legal garbage
**
** Revision 1.9  2000/07/04 06:26:32  matt
** scanline emulation simplifications/timing fixes
**
** Revision 1.8  2000/07/04 04:58:29  matt
** dynamic memory range handlers
**
** Revision 1.7  2000/06/26 04:58:51  matt
** minor bugfix
**
** Revision 1.6  2000/06/20 20:42:12  matt
** fixed some NULL pointer problems
**
** Revision 1.5  2000/06/09 15:12:26  matt
** initial revision
**
*/
