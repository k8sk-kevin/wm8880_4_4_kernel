/*
 * Copyright (c) 2008-2013 WonderMedia Technologies, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * WonderMedia Technologies, Inc.
 * 4F, 533, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C
 */

#include <asm/cacheflush.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <mach/hardware.h>
#include "ge_accel.h"

int flipcnt;
int flipreq;
int vbl;
int vsync = 1;
int debug;

module_param(flipcnt, int, S_IRUSR | S_IWUSR | S_IWGRP | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(flipcnt, "Flip count");

module_param(flipreq, int, S_IRUSR | S_IWUSR | S_IWGRP | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(flipreq, "Flip request count");

module_param(vbl, int, S_IRUSR | S_IWUSR | S_IWGRP | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(vbl, "Wait vsync for each frame (0)");

module_param(vsync, int, S_IRUSR | S_IWUSR | S_IWGRP | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(vsync, "Can use vsync (1)");

module_param(debug, int, S_IRUSR | S_IWUSR | S_IWGRP | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(debug, "Show debug information");

/**************************
 *    Export functions    *
 **************************/

#define M(x) ((x)<<20)

unsigned int phy_mem_end(void)
{
	unsigned int memsize = (num_physpages << PAGE_SHIFT);
	unsigned int n = 32 << 20;

	while (n <= memsize)
		n <<= 1;

	memsize = n;
	printk(KERN_DEBUG "Detected RAM size %d MB\n", memsize>>20);

	return memsize;
}

unsigned int phy_mem_end_sub(u32 size)
{
	return phy_mem_end() - M(size);
}
EXPORT_SYMBOL(phy_mem_end_sub);

/* ge_vo_functions depends vpu to work */

void ge_vo_get_default_var(struct fb_var_screeninfo *var)
{
#ifdef HAVE_VPP
	vpp_get_info(0, var);
#endif
}

void ge_vo_wait_vsync(void)
{
#ifdef HAVE_VPP
	if (vsync)
		vpp_wait_vsync(0, 1);
#endif
}

static int ge_vo_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
#ifdef HAVE_VPP
	wmtfb_set_mutex(info, 1);
	vpp_pan_display(var, info);
	wmtfb_set_mutex(info, 0);
#endif
	flipcnt++;

	return 0;
}

struct ge_var {
	struct fb_info *info;
	struct fb_var_screeninfo var[1];
	struct fb_var_screeninfo new_var[1];
	struct timeval most_recent_flip_time;
	int dirty;
	int force_sync;
	spinlock_t lock[1];
	void (*start)(struct ge_var *ge_var);
	void (*stop)(struct ge_var *ge_var);
	void (*get)(struct ge_var *ge_var, struct fb_var_screeninfo *var);
	void (*set)(struct ge_var *ge_var, struct fb_var_screeninfo *var);
	void (*sync)(struct ge_var *ge_var, int wait);
};

static struct ge_var *ge_var_s;

static void ge_var_start(struct ge_var *ge_var);
static void ge_var_stop(struct ge_var *ge_var);
static void ge_var_get(struct ge_var *ge_var, struct fb_var_screeninfo *var);
static void ge_var_set(struct ge_var *ge_var, struct fb_var_screeninfo *var);
static void ge_var_sync(struct ge_var *ge_var, int wait);

static struct ge_var *create_ge_var(struct fb_info *info)
{
	struct ge_var *ge_var;

	ge_var = (struct ge_var *)
		kcalloc(1, sizeof(struct ge_var), GFP_KERNEL);

	ge_var->info = info;
	ge_var->start = &ge_var_start;
	ge_var->stop = &ge_var_stop;
	ge_var->get = &ge_var_get;
	ge_var->set = &ge_var_set;
	ge_var->sync = &ge_var_sync;

	do_gettimeofday(&ge_var->most_recent_flip_time);

	ge_var->start(ge_var);

	return ge_var;
}

static void release_ge_var(struct ge_var *ge_var)
{
	if (ge_var) {
		ge_var->stop(ge_var);
		kfree(ge_var);
	}
}

static void ge_var_start(struct ge_var *ge_var)
{
	spin_lock_init(ge_var->lock);
}

static void ge_var_stop(struct ge_var *ge_var)
{
}

static void ge_var_get(struct ge_var *ge_var, struct fb_var_screeninfo *var)
{
	spin_lock(ge_var->lock);
	memcpy(var, ge_var->var, sizeof(struct fb_var_screeninfo));
	spin_unlock(ge_var->lock);
}

static void ge_var_set(struct ge_var *ge_var, struct fb_var_screeninfo *var)
{
	int wait;

	spin_lock(ge_var->lock);
	if (memcmp(ge_var->new_var, var, sizeof(struct fb_var_screeninfo))) {
		memcpy(ge_var->new_var, var, sizeof(struct fb_var_screeninfo));
		ge_var->dirty++;
	}
	spin_unlock(ge_var->lock);

	wait = vbl || (var->activate & FB_ACTIVATE_VBL);
	ge_var->sync(ge_var, wait);
}

static void ge_var_sync(struct ge_var *ge_var, int wait)
{
	struct timeval t;
	struct timeval *mrft;
	unsigned long us;

	spin_lock(ge_var->lock);

	if (ge_var->dirty == 0) {
		spin_unlock(ge_var->lock);
		return;
	}

	memcpy(ge_var->var, ge_var->new_var, sizeof(struct fb_var_screeninfo));

	mrft = &ge_var->most_recent_flip_time;
	do_gettimeofday(&t);

	us = (t.tv_sec - mrft->tv_sec) * 1000000 +
		(t.tv_usec - mrft->tv_usec);

	spin_unlock(ge_var->lock);

	ge_vo_pan_display(ge_var->var, ge_var->info);
	ge_var->dirty = 0;

	/* 60 fps */
	if (wait && us < 16667) {
		if (debug) {
			struct timeval t1;
			struct timeval t2;
			int ms;
			do_gettimeofday(&t1);
			ge_vo_wait_vsync();
			do_gettimeofday(&t2);
			ms = (t2.tv_sec - t1.tv_sec) * 1000 +
			     (t2.tv_usec - t1.tv_usec) / 1000;
			printk(KERN_DEBUG "ge: wait vsync for %d ms\n", ms);
		} else
			ge_vo_wait_vsync();
	}

	do_gettimeofday(&ge_var->most_recent_flip_time);
}

static int fb_var_cmp(struct fb_var_screeninfo *var1,
		      struct fb_var_screeninfo *var2)
{
	/* Compare from xres to bits_per_pixel should be enough */
	return memcmp(var1, var2, 28);
}

/**
 * ge_init - Initial and display framebuffer.
 *
 * Fill the framebuffer with a default color, back.
 * Display the framebuffer using GE AMX.
 *
 * Although VQ is supported in design, I just can't find any benefit
 * from VQ. It wastes extra continuous physical memory, and runs much
 * slower than direct register access. Moreover, the source code
 * becomes more complex and is hard to maintain. Accessing VQ from
 * the user space is also a nightmare. In brief, the overhead of VQ makes
 * it useless. In order to gain the maximum performance
 * from GE and to keep the driver simple, I'm going to stop using VQ.
 * I will use VQ only when it is necessary.
 *
 * @info is the fb_info provided by framebuffer driver.
 * @return zero on success.
 */
int ge_init(struct fb_info *info)
{
	static int boot_init; /* boot_init = 0 */
	/*
	 * Booting time initialization
	 */
	if (!boot_init) {
		ge_var_s = create_ge_var(info);
		boot_init = 1;
	}

	return 0;
}

/**
 * ge_exit - Disable GE.
 *
 * No memory needs to be released here.
 * Turn off the AMX to stop displaying framebuffer.
 * Update the index of MMU.
 *
 * @info is fb_info from fbdev.
 * @return zero on success.
 */
int ge_exit(struct fb_info *info)
{
	release_ge_var(ge_var_s);
	release_mem_region(info->fix.mmio_start, info->fix.mmio_len);

	return 0;
}

int ge_release(struct fb_info *info)
{
	struct ge_var *ge_var = ge_var_s;

	ge_var->sync(ge_var, 0); /* apply pending changes */

	return 0;
}

/**
 * ge_pan_display - Pans the display.
 *
 * Pan (or wrap, depending on the `vmode' field) the display using the
 * `xoffset' and `yoffset' fields of the `var' structure.
 * If the values don't fit, return -EINVAL.
 *
 * @var: frame buffer variable screen structure
 * @info: frame buffer structure that represents a single frame buffer
 *
 * Returns negative errno on error, or zero on success.
 */
int ge_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct ge_var *ge_var;

	ge_var = ge_var_s;

	if (debug)
		printk(KERN_DEBUG "pan_display\n");
	/*
	printk(KERN_DEBUG "%s: xoff = %d, yoff = %d, xres = %d, yres = %d\n",
	       __func__, var->xoffset, var->yoffset,
	       info->var.xres, info->var.yres);
	*/
	flipreq++;

	if ((var->xoffset + info->var.xres > info->var.xres_virtual) ||
	    (var->yoffset + info->var.yres > info->var.yres_virtual)) {
		/* Y-pan is used in most case.
		 * So please make sure that yres_virtual is
		 * greater than (yres + yoffset).
		 */
		printk(KERN_ERR "%s: out of range\n", __func__);
		return -EINVAL;
	}

	if ((var->activate & FB_ACTIVATE_MASK) == FB_ACTIVATE_NOW &&
	    fb_var_cmp(ge_var->new_var, var))
		ge_var->set(ge_var, var);

	return 0;
}

int ge_sync(struct fb_info *info)
{
	return 0;
}

int ge_blank(int mode, struct fb_info *info)
{
#ifdef HAVE_VPP
	return vpp_set_blank(info, mode);
#else
	return 0;
#endif
}

int ge_suspend(struct fb_info *info)
{
	return 0;
}

int ge_resume(struct fb_info *info)
{
	return 0;
}
