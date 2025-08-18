/* vim: set tabstop=3 expandtab:
**
** This file is in the public domain.
**
** osd.c
**
** $Id: osd.c,v 1.2 2001/04/27 14:37:11 neil Exp $
**
*/

#include "errno.h"
#include "fcntl.h"
#include "limits.h"
#include "signal.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "sys/time.h"
#include "sys/stat.h"
#include "sys/types.h"
#include "unistd.h"
       
#include "noftypes.h"
#include "nofconfig.h"
#include "log.h"
#include "osd.h"
#include "nofrendo.h"
#include "event.h"

#include "version.h"
#include "nes.h"

// TODO. this is really ugly. need to resolve with emu_nofrendo

#define  NES_CLOCK_DIVIDER    12
//#define  NES_MASTER_CLOCK     21477272.727272727272
#define  NES_MASTER_CLOCK     (236250000 / 11)
#define  NES_SCANLINE_CYCLES  (1364.0 / NES_CLOCK_DIVIDER)
#define  NES_FIQ_PERIOD       (NES_MASTER_CLOCK / NES_CLOCK_DIVIDER / 60)

char configfilename[]="na";

/* This is os-specific part of main() */
int osd_main(int argc, char *argv[])
{
   config.filename = configfilename;
   return main_loop("rom", system_autodetect);
}

/* File system interface */
void osd_fullname(char *fullname, const char *shortname)
{
   strncpy(fullname, shortname, PATH_MAX);
}

/* This gives filenames for storage of saves */
char *osd_newextension(char *string, char *ext)
{
   return string;
}

/* This gives filenames for storage of PCX snapshots */
int osd_makesnapname(char *filename, int len)
{
   return -1;
}

//=======

//#define  NES_VISIBLE_HEIGHT   224

#define  DEFAULT_SAMPLERATE   22100
#define  DEFAULT_FRAGSIZE     128

#define  DEFAULT_WIDTH        256
#define  DEFAULT_HEIGHT       240
//#define  DEFAULT_HEIGHT       NES_VISIBLE_HEIGHT

static int init(int width, int height) { return 0; };
static void shutdown(void) {};
static int set_mode(int width, int height) { return 0; };

uint32 nes_pal[256];
static void set_palette(rgb_t *pal)
{
    for (int i = 0; i < 256; i++)
        nes_pal[i] = (pal[i].r << 16) | (pal[i].g << 8) | pal[i].b;
};
static void clear(uint8 color) {};


/*
** NES video output
**
** The original implementation used a dummy framebuffer of size 1 which
** resulted in the PPU writing far beyond the allocated memory and corrupting
** the heap.  Replace this with a statically allocated framebuffer matching the
** NES resolution (256x240) and back it with a bitmap structure that contains
** the necessary line pointers.  This mirrors the approach used by the SMS
** emulator and avoids repeated heap allocations that could lead to
** fragmentation.
*/

/* Framebuffer backing store */
static uint8 fb[DEFAULT_WIDTH * DEFAULT_HEIGHT];

/* Bitmap wrapper providing line pointers for each scanline */
typedef struct
{
    bitmap_t bmp;
    uint8 *lines[DEFAULT_HEIGHT];
} nes_bitmap_t;

static nes_bitmap_t nes_screen;

/* acquire the directbuffer for writing */
static bitmap_t *lock_write(void)
{
    nes_screen.bmp.width = DEFAULT_WIDTH;
    nes_screen.bmp.height = DEFAULT_HEIGHT;
    nes_screen.bmp.pitch = DEFAULT_WIDTH;
    nes_screen.bmp.hardware = true;
    nes_screen.bmp.data = fb;

    for (int y = 0; y < DEFAULT_HEIGHT; y++)
        nes_screen.lines[y] = fb + (y * DEFAULT_WIDTH);

    return &nes_screen.bmp;
}

/* release the resource */
static void free_write(int num_dirties, rect_t *dirty_rects)
{
    (void)num_dirties;
    (void)dirty_rects;
    /* nothing to free; framebuffer is static */
}

/* blit the rendered frame into the framebuffer */
static void custom_blit(bitmap_t *bmp, int num_dirties, rect_t *dirty_rects)
{
    (void)num_dirties;
    (void)dirty_rects;

    int h = bmp->height < DEFAULT_HEIGHT ? bmp->height : DEFAULT_HEIGHT;
    int w = bmp->width < DEFAULT_WIDTH ? bmp->width : DEFAULT_WIDTH;

    for (int y = 0; y < h; y++)
        memcpy(fb + y * DEFAULT_WIDTH, bmp->line[y], w);
}

viddriver_t sdlDriver =
{
   "Simple DirectMedia Layer",         /* name */
   init,          /* init */
   shutdown,      /* shutdown */
   set_mode,      /* set_mode */
   set_palette,   /* set_palette */
   clear,         /* clear */
   lock_write,    /* lock_write */
   free_write,    /* free_write */
   custom_blit,   /* custom_blit */
   false          /* invalidate flag */
};

void osd_getvideoinfo(vidinfo_t *info)
{
   info->default_width = DEFAULT_WIDTH;
   info->default_height = DEFAULT_HEIGHT;
   info->driver = &sdlDriver;
}

/* flip between full screen and windowed */
void osd_togglefullscreen(int code)
{
}

//
int _input_mask;
void input_key(int k, int down)
{
    int b = -1;
    switch (k) {
        case 0x400000E2: b = 0; break; // option->a
        case 0x400000E1: b = 1; break; // shift->b
        case 0x0D:       b = 2; break; // return->start
        case 0x09:       b = 3; break; // tab->select
        case 0x40000052: b = 4; break; // up
        case 0x40000051: b = 5; break; // down
        case 0x40000050: b = 6; break; // left
        case 0x4000004F: b = 7; break; // right

        case 'r': b = 8; break; // soft reset
        case 't': b = 9; break; // hard reset
        default:
            return;
    }
    b = 1 << b;
    if (down)
        _input_mask |= b;
    else
        _input_mask &= ~b;
}

void osd_getinput(void)
{
    const int ev[16]= {
        event_joypad1_a,
        event_joypad1_b,
        event_joypad1_start,
        event_joypad1_select,

        event_joypad1_up,
        event_joypad1_down,
        event_joypad1_left,
        event_joypad1_right,

        event_soft_reset,
        event_hard_reset
    };

    static int oldb=0xffff;
    int b =_input_mask;
    int chg = b^oldb;
    int x;
    oldb=b;
    event_t evh;
    for (x=0; x<16; x++) {
        if (chg&1) {
            evh=event_get(ev[x]);
            if (evh) evh(b & 1);
        }
        chg>>=1;
        b>>=1;
    }
}

void osd_getmouse(int *x, int *y, int *button)
{
}

//---------------------------------------------------------------------------
// Timer installation
//
// osd_installtimer installs a periodic timer that invokes the callback
// function at the requested frequency.  The callback is expected to update
// |nofrendo_ticks| (typically by incrementing it) so that the rest of the
// emulator can track the passage of time.  A non‑zero value is returned if
// the timer could not be created.
//
// This implementation uses POSIX interval timers via setitimer().  The
// callback is dispatched from a signal handler registered on SIGALRM.
//---------------------------------------------------------------------------
static void (*timer_callback)(void) = NULL;

static void timer_handler(int signum)
{
    (void)signum;            /* unused */
    if (NULL != timer_callback)
        timer_callback();
}

int osd_installtimer(int frequency, void *func, int funcsize, void *counter, int countersize)
{
    (void)funcsize;          /* parameters are kept for API compatibility */
    (void)counter;
    (void)countersize;

    if (frequency <= 0 || NULL == func)
        return -1;

    timer_callback = (void (*)(void))func;

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = timer_handler;
    if (sigaction(SIGALRM, &sa, NULL) < 0)
        return -1;

    struct itimerval timer;
    memset(&timer, 0, sizeof(timer));
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = 1000000 / frequency;
    timer.it_interval = timer.it_value;

    if (setitimer(ITIMER_REAL, &timer, NULL) < 0)
        return -1;

    return 0;
}

int osd_init()
{
    return 0;
}

nes_t* _nes_p = 0;
int nes_emulate_init(const char* path, int width, int height)
{
    if (!_nes_p) {
        if (log_init())
           return -1;
        if (vid_init(width,height,&sdlDriver))
            return -1;
        if (vid_setmode(NES_SCREEN_WIDTH, 240))
            return -1;
        event_init();
        event_set_system(system_nes);
        _nes_p = nes_create();
        if (!_nes_p)
            return -1;
    }
    if (nes_insertcart(path,_nes_p))
        return -1;

    osd_setsound(_nes_p->apu->process);
    _nes_p->scanline_cycles = 0;
    _nes_p->fiq_cycles = (int) NES_FIQ_PERIOD;
    return 0;
}

void nes_renderframe(bool draw_flag);
extern bitmap_t *primary_buffer; //, *back_buffer = NULL;

// emulate a frame, return
uint8** nes_emulate_frame(bool draw_flag)
{
    nes_renderframe(draw_flag);

    /* Only push the rendered frame to the display and poll input when
     * drawing is requested.  When draw_flag is false the emulation still
     * advances internally but no visible frame is produced. */
    if (draw_flag) {
        vid_flush();
        osd_getinput();
    }
    if (primary_buffer)
        return primary_buffer->line;
    return NULL;
}
