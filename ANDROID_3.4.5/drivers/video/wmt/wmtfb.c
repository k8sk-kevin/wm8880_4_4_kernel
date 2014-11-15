/*++
 * linux/drivers/video/wmt/wmtfb.c
 * WonderMedia frame buffer driver
 *
 * Copyright c 2014  WonderMedia  Technologies, Inc.
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
--*/

#define WMTFB_C
/* #define DEBUG */
/* #define DEBUG_DETAIL */

/*----------------------- DEPENDENCE -----------------------------------------*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <asm/page.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>

#include "vpp.h"

/*----------------------- PRIVATE MACRO --------------------------------------*/

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define FBUT_XXXX    1     *//*Example*/

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx fbut_xxx_t; *//*Example*/

/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  fbut_xxx;        *//*Example*/

static struct fb_fix_screeninfo wmtfb_fix = {
	.id             = "wmtfbx",
	.smem_start     = 0,
	.smem_len       = 0,
	.type           = FB_TYPE_PACKED_PIXELS,
	.type_aux       = 0,
	.visual         = FB_VISUAL_TRUECOLOR,
	.xpanstep       = 1,
	.ypanstep       = 1,
	.ywrapstep      = 1,
	.line_length    = 0,
	.mmio_start     = 0,
	.mmio_len       = 0,
	.accel          = FB_ACCEL_NONE
};

static struct fb_var_screeninfo wmtfb_var = {
	.xres           = 0,
	.yres           = 0,
	.xres_virtual   = 0,
	.yres_virtual   = 0,
#if 0
	.bits_per_pixel = 32,
	.red            = {16, 8, 0},
	.green          = {8, 8, 0},
	.blue           = {0, 8, 0},
	.transp         = {24, 8, 0},
#else
	.bits_per_pixel = 16,
	.red            = {11, 5, 0},
	.green          = {5, 6, 0},
	.blue           = {0, 5, 0},
	.transp         = {0, 0, 0},
#endif
	.activate       = FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE,
	.height         = -1,
	.width          = -1,
	.pixclock       = 0,
	.left_margin    = 40,
	.right_margin   = 24,
	.upper_margin   = 32,
	.lower_margin   = 11,
	.hsync_len      = 96,
	.vsync_len      = 2,
	.vmode          = FB_VMODE_NONINTERLACED
};

int wmtfb_probe_ready;
/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void fbut_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
/*----------------------- Linux Proc --------------------------------------*/
#ifdef DEBUG
void wmtfb_show_var(char *str, struct fb_var_screeninfo *var)
{
	int pixclk;

	pixclk = (var->pixclock) ? (PICOS2KHZ(var->pixclock) * 1000) : 0;
	DPRINT("----- %s ------------------------\n", str);
	DPRINT("res(%d,%d),vir(%d,%d),offset(%d,%d)\n",
		var->xres, var->yres, var->xres_virtual,
		var->yres_virtual, var->xoffset, var->yoffset);
	DPRINT("pixclk %d(%d),hsync %d,vsync %d\n", var->pixclock, pixclk,
		var->hsync_len, var->vsync_len);
	DPRINT("left %d,right %d,upper %d,lower %d\n", var->left_margin,
		var->right_margin, var->upper_margin, var->lower_margin);
	DPRINT("bpp %d, grayscale %d\n", var->bits_per_pixel, var->grayscale);
	DPRINT("nonstd %d, activate %d, height %d, width %d\n",
		var->nonstd, var->activate, var->height, var->width);
	DPRINT("vmode 0x%x,sync 0x%x,rotate %d,accel %d\n",
		var->vmode, var->sync, var->rotate, var->accel_flags);
	DPRINT("-----------------------------\n");
	return;
}
#endif

static DEFINE_SEMAPHORE(vpp_sem);
static DEFINE_SEMAPHORE(vpp_sem2);
void wmtfb_set_mutex(struct fb_info *info, int lock)
{
	struct vout_info_t *par = (struct vout_info_t *) info->par;

	if (lock)
		down(&par->sem);
	else
		up(&par->sem);
}

int vpp_set_blank(struct fb_info *info, int blank)
{
	struct vout_info_t *par = (struct vout_info_t *) info->par;
	int i;

	DBG_MSG("(%d,%d)\n", info->node, blank);

	for (i = 0; i < VPP_VOUT_NUM; i++) {
		if (par->vout[i] == 0)
			break;
		vout_set_blank(par->vout[i]->num, blank);
	}
	return 0;
}

void vpp_set_lvds_blank(int blank)
{
	if (lcd_get_lvds_id() != LCD_LVDS_1024x600)
		return;

	if (blank == 0) {
		outl(inl(GPIO_BASE_ADDR + 0xC0) | BIT10, GPIO_BASE_ADDR + 0xC0);
		outl(inl(GPIO_BASE_ADDR + 0x80) | BIT10, GPIO_BASE_ADDR + 0x80);
		mdelay(6);
	}

	if (blank)
		msleep(50);

	vout_set_blank(VPP_VOUT_NUM_LVDS,
		(blank) ? VOUT_BLANK_POWERDOWN : VOUT_BLANK_UNBLANK);
	lvds_set_power_down((blank) ? 1 : 0);

	if (blank) {
		mdelay(6);
		/* GPIO10 off  8ms -> clock -> off */
		outl(inl(GPIO_BASE_ADDR + 0xC0) & ~BIT10,
			GPIO_BASE_ADDR + 0xC0);
	}
}

void vpp_var_to_fb(struct fb_var_screeninfo *var,
				struct fb_info *info, vdo_framebuf_t *fb)
{
	unsigned int addr;
	int y_bpp, c_bpp;

	if (var) {
		fb->col_fmt = WMT_FB_COLFMT(var->nonstd);
		if (!fb->col_fmt) {
			switch (var->bits_per_pixel) {
			case 16:
				if ((info->var.red.length == 5) &&
					(info->var.green.length == 6) &&
					(info->var.blue.length == 5)) {
					fb->col_fmt = VDO_COL_FMT_RGB_565;
				} else if ((info->var.red.length == 5) &&
					(info->var.green.length == 5) &&
					(info->var.blue.length == 5)) {
					fb->col_fmt = VDO_COL_FMT_RGB_1555;
				} else {
					fb->col_fmt = VDO_COL_FMT_RGB_5551;
				}
				break;
			case 32:
				fb->col_fmt = VDO_COL_FMT_ARGB;
				break;
			default:
				fb->col_fmt = VDO_COL_FMT_RGB_565;
				break;
			}
			y_bpp = var->bits_per_pixel;
			c_bpp = 0;
		} else {
			y_bpp = 8;
			c_bpp = 8;
		}

		fb->img_w = var->xres;
		fb->img_h = var->yres;
		fb->fb_w = var->xres_virtual;
		fb->fb_h = var->yres_virtual;
		fb->h_crop = 0;
		fb->v_crop = 0;
		fb->flag = 0;
		fb->bpp = var->bits_per_pixel;

		addr = info->fix.smem_start +
		(var->yoffset * var->xres_virtual * ((y_bpp + c_bpp) >> 3));
		addr += var->xoffset * ((y_bpp) >> 3);
		fb->y_addr = addr;
		fb->y_size = var->xres_virtual * var->yres * (y_bpp >> 3);
		fb->c_addr = fb->y_addr + fb->y_size;
		fb->c_size = var->xres_virtual * var->yres * (c_bpp >> 3);
	}
}

void vpp_pan_display_bitblit(struct vout_info_t *par)
{
	vdo_framebuf_t src, dst;
	struct fb_videomode vmode;
	struct govrh_mod_t *govr;
	struct fb_info *dfb_info;
	struct vout_info_t *d_info;
	int fb_no;

	/* DBG_MSG("fb0 bitblit\n"); */
	src = par->fb;
	for (fb_no = 1; ; fb_no++) {
		d_info = vout_info_get_entry(fb_no);
		if (!d_info)
			break;

		govr = vout_info_get_govr(fb_no);
		if (govr == 0)
			break;
		govrh_get_framebuffer(govr, &dst);
		govrh_get_videomode(govr, &vmode);
		dst.img_w = vmode.xres;
		dst.fb_w = vpp_calc_fb_width(dst.col_fmt, dst.img_w);
		dst.img_h = vmode.yres;
		dst.fb_h = vmode.yres;
		dst.y_size = dst.fb_w * dst.img_h * (dst.bpp >> 3);

		dfb_info = (struct fb_info *) d_info->fb_info_p;
		if (dfb_info) {
			dfb_info->var.yoffset =
				(dfb_info->var.yoffset) ? 0 : dst.img_h;
			dst.y_addr = dfb_info->fix.smem_start +
				(dst.fb_w * dfb_info->var.yoffset *
				(dst.bpp >> 3));
			dst.c_addr = 0;
		} else {
			govrh_get_fb_addr(govr, &dst.y_addr, &dst.c_addr);
		}

		if (d_info->alloc_mode == VOUT_ALLOC_GE_OVERSCAN) {
			if (!d_info->mb) {
				int size = dst.y_size * VPP_MB_ALLOC_NUM;

				d_info->mb = mb_alloc(size);
				if (d_info->mb) {
					MSG("mb alloc 0x%x,%d\n",
						d_info->mb, size);
				} else {
					DBG_ERR("alloc fail\n");
					return;
				}
			}
			dst.y_addr = d_info->mb +
				(dst.fb_w * dfb_info->var.yoffset *
				(dst.bpp >> 3));
		}

		if (dst.y_addr) {
			p_scl->scale_sync = 1;
			vpp_set_recursive_scale(&src, &dst);
			vout_set_framebuffer(d_info, &dst);
		}
	}
}

int vpp_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct vout_info_t *par = (struct vout_info_t *) info->par;

	if (g_vpp.hdmi_certify_flag)
		return 0;

	DBG_DETAIL("fb %d\n", (info) ? info->node : 0);
	vpp_var_to_fb(var, info, &par->fb);
	if (wmtfb_probe_ready && g_vpp.fb0_bitblit && (info->node == 0))
		vpp_pan_display_bitblit(par);
	else
		vout_set_framebuffer(par, &par->fb);

#ifdef CONFIG_VPP_STREAM_CAPTURE
	if (g_vpp.stream_enable && (par->hwc_mode == VOUT_HWC_VIRTUAL)) {
		g_vpp.stream_mb_index = var->yoffset / var->yres;
		vpp_dbg_show_val1(VPP_DBGLVL_STREAM, 0,
			"stream pan disp", g_vpp.stream_mb_index);
	}
#endif
	if (vpp_check_dbg_level(VPP_DBGLVL_DISPFB)) {
		char buf[50];
		unsigned int yaddr = 0, caddr = 0;
		struct govrh_mod_t *govr;

		govr = vout_info_get_govr(info->node);
		if (govr)
			govrh_get_fb_addr(govr, &yaddr, &caddr);
		sprintf(buf, "pan_display %d,0x%x", par->num, yaddr);
		vpp_dbg_show(VPP_DBGLVL_DISPFB, par->num + 1, buf);
	}

	if (vpp_check_dbg_level(VPP_DBGLVL_FPS)) {
		char buf[10];

		sprintf(buf, "fb%d", par->num);
		vpp_dbg_timer(&par->pandisp_timer, buf, 2);
	}
	return 0;
}

int vpp_set_par(struct fb_info *info)
{
	struct vout_info_t *par = (struct vout_info_t *) info->par;
	vdo_framebuf_t fb;
	struct fb_videomode var, cur;
	struct govrh_mod_t *govr;

	if (g_vpp.hdmi_certify_flag)
		return 0;

	govr = vout_info_get_govr(info->node);
	if (!govr)
		return 0;

	wmtfb_set_mutex(info, 1);

	/* check frame buffer */
	vpp_var_to_fb(&info->var, info, &fb);
	par->fb.fb_h = fb.fb_h;
	if (memcmp(&fb.img_w, &par->fb.img_w, 32)) {
		if ((wmtfb_probe_ready == 0) && g_vpp.govrh_preinit) {
			MSG("[uboot logo] fb%d\n", info->node);
			govrh_get_framebuffer(govr, &par->fb);
			vpp_set_recursive_scale(&par->fb, &fb);
		}
#ifdef DEBUG
		MSG("set_par %d : set framebuf\n",
			info->node);
		vpp_show_framebuf("cur", &par->fb);
		vpp_show_framebuf("new", &fb);
#endif
		par->fb = fb;
	} else {
		fb.img_w = 0;
	}

	/* check timing */
	fb_var_to_videomode(&var, &info->var);
	govrh_get_videomode(govr, &cur);
	if ((cur.xres == var.xres) && (cur.yres == var.yres)) {
		unsigned int cur_pixclk, new_pixclk;

		/* diff less than 500K */
		cur_pixclk = PICOS2KHZ(cur.pixclock);
		new_pixclk = PICOS2KHZ(var.pixclock);
		if (abs(new_pixclk - cur_pixclk) < 500) {
			var.pixclock = cur.pixclock;
			var.refresh = cur.refresh;
		}
		/* diff less than 2 */
		if (abs(var.refresh - cur.refresh) <= 2)
			var.refresh = cur.refresh;
	}

	//var.sync &= (FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT);
    var.sync = cur.sync;
	if (memcmp(&var, &cur, sizeof(struct fb_videomode))) {
#ifdef DEBUG
		DPRINT("[wmtfb] set_par %d: set timing\n", info->node);
		vpp_show_timing("cur", &cur, 0);
		vpp_show_timing("new", &var, 0);
#endif
	} else {
		var.xres = 0;
	}
	vout_config(par, (var.xres) ? &var : 0, (fb.img_w) ? &fb : 0);
	wmtfb_set_mutex(info, 0);
	return 0;
}

int vpp_get_info(int fbn, struct fb_var_screeninfo *var)
{
	struct vout_info_t *par;

	par = vout_info_get_entry(fbn);
	if (!par)
		return -1;

	var->xres = par->resx;
	var->yres = par->resy;
	var->xres_virtual = par->resx_virtual;
	var->yres_virtual = var->yres * VPP_MB_ALLOC_NUM;
	var->bits_per_pixel = (g_vpp.mb_colfmt == VDO_COL_FMT_ARGB) ? 32 : 16;
	var->pixclock = par->resx * par->resy * par->fps;
	var->pixclock = (var->pixclock) ? (var->pixclock / 1000) : 0;
	var->pixclock = (var->pixclock) ? KHZ2PICOS(var->pixclock) : 0;
	var->left_margin = 0;
	var->right_margin = 0;
	var->upper_margin = 0;
	var->lower_margin = 0;
	var->hsync_len = 0;
	var->vsync_len = 0;
        if (par->option & VPP_OPT_INTERLACE) {
                var->vmode |= FB_VMODE_INTERLACED;
                var->pixclock *= 2;
        }
#ifdef DEBUG
	MSG("[get_info] fb%d\n", fbn);
	wmtfb_show_var("get_info", var);
#endif
	return 0;
}

int wmtfb_alloc(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct vout_info_t *par = (struct vout_info_t *) info->par;
	unsigned int size;
	int no_alloc = 0;
	vdo_color_fmt colfmt;
	int y_bpp, c_bpp;
	unsigned int mb_resx, mb_resy, fb_num;
	int i;

	colfmt = (var->nonstd) ? WMT_FB_COLFMT(var->nonstd) :
		((var->bits_per_pixel == 16) ?
		VDO_COL_FMT_RGB_565 : VDO_COL_FMT_ARGB);
	vpp_get_colfmt_bpp(colfmt, &y_bpp, &c_bpp);

	switch (par->alloc_mode) {
	case VOUT_ALLOC_FIX_MB:
		if (info->fix.smem_start) {
			no_alloc = 1;
			break;
		}

		{
		char buf[100];
		int varlen = 100;

		if (wmt_getsyspara("wmt.display.mb",
			(unsigned char *)buf, &varlen) == 0) {
			unsigned int parm[10];

			vpp_parse_param(buf, (unsigned int *)parm, 4, 0);
			MSG("boot parm mb (%d,%d),bpp %d,fb %d\n",
					parm[0], parm[1], parm[2], parm[3]);
			mb_resx = parm[0];
			mb_resy = parm[1];
			y_bpp = parm[2] * 8;
			c_bpp = 0;
			fb_num = parm[3];
		} else {
			mb_resx = VPP_HD_MAX_RESX;
			mb_resy = VPP_HD_MAX_RESY;
			fb_num = VPP_MB_ALLOC_NUM;
		}
		}
		break;
	case VOUT_ALLOC_DYNAMIC_MB:
		if ((par->hwc_mode == VOUT_HWC_VIRTUAL)
			&& !wmtfb_probe_ready) {
			no_alloc = 1;
			break;
		}
		mb_resx = var->xres_virtual;
		mb_resy = var->yres;
		fb_num = VPP_MB_ALLOC_NUM;
		break;
	case VOUT_ALLOC_GE_OVERSCAN:
		if (!info->fix.smem_start) {
			struct vout_info_t *fb0_par;
			struct fb_info *fb0_info;

			fb0_par = vout_info_get_entry(0);
			fb0_info = (struct fb_info *) fb0_par->fb_info_p;
			info->fix.smem_start = fb0_info->fix.smem_start;
			info->fix.smem_len = fb0_info->fix.smem_len;
			info->screen_base = fb0_info->screen_base;
			info->screen_size = fb0_info->screen_size;
		}
	default:
		no_alloc = 1;
		break;
	}
#if 0
	DBG_MSG("fb%d,mode %d,size %d,len %d,%dx%d,no alloc %d\n",
		info->node, par->alloc_mode, size, info->fix.smem_len,
		var->xres, var->yres, no_alloc);
#endif
	if (no_alloc)
		return 0; /* don't need */

	size = mb_resx * mb_resy * ((y_bpp + c_bpp) / 8) * fb_num ;
	if (size == info->fix.smem_len)
		return 0;

	/* free pre mb */
	if (info->fix.smem_start) {
		mb_free(info->fix.smem_start);
		info->fix.smem_start = 0;
		info->fix.smem_len = 0;
		info->screen_base = 0;
		info->screen_size = 0;
		MSG("[wmtfb] fb%d free mb\n", info->node);
	}

	if ((var->xres == 0) || (var->yres == 0))
		return -1;

	info->fix.smem_start = mb_alloc(size);
	if (!info->fix.smem_start) {
		DBG_ERR("fb%d alloc mb fail %d\n", info->node, size);
		return -1;
	}
	info->fix.smem_len = size;
	info->screen_base = mb_phys_to_virt(info->fix.smem_start);
	info->screen_size = size;
	MSG("[wmtfb] fb%d mb 0x%x,len %d,base 0x%x(%d,%d,%d)\n", info->node,
		(int) info->fix.smem_start, info->fix.smem_len,
		(int) info->screen_base, mb_resx, mb_resy, fb_num);

	if (wmtfb_probe_ready) {
		int ysize, csize;

		size = size / fb_num;
		ysize = mb_resx * mb_resy * y_bpp / 8;
		csize = mb_resx * mb_resy * c_bpp / 8;
		for (i = 0; i < fb_num; i++) {
			memset(info->screen_base + i * size, 0x0, ysize);
			if (c_bpp)
				memset(info->screen_base + i * size + ysize,
					0x80, csize);
		}
	}

	if (par->hwc_mode == VOUT_HWC_VIRTUAL) {
		vpp_lock();
		g_vpp.stream_mb_cnt = VPP_MB_ALLOC_NUM;
		size = var->xres_virtual * var->yres * ((y_bpp + c_bpp) / 8);
		for (i = 0; i < g_vpp.stream_mb_cnt; i++)
			g_vpp.stream_mb[i] = info->fix.smem_start + size * i;
		vpp_unlock();
	}
	return 0;
}

static int wmtfb_open
(
	struct fb_info *info,   /*!<; // a pointer point to struct fb_info */
	int user                /*!<; // user space mode */
)
{
	DBG_MSG("Enter wmtfb_open\n");
	return 0;
}

static int wmtfb_release
(
	struct fb_info *info,   /*!<; // a pointer point to struct fb_info */
	int user                /*!<; // user space mode */
)
{
	DBG_MSG("Enter wmtfb_release\n");
	return 0;
}

int wmtfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct vout_info_t *par;
	int temp;
	int force = 0;

#ifdef DEBUG_DETAIL
	DMSG("Enter %d\n", info->node);
	wmtfb_show_var("[check_var beg] cur var", &info->var);
	wmtfb_show_var("[check_var beg] new var", var);
#endif
	if (!info->par) { /* link fb & vout info */
		par = vout_info_get_entry(info->node);
		info->par = (void *) par;
		par->fb_info_p = (void *) info;
	}

	par = (struct vout_info_t *) info->par;
	if (par->alloc_mode == VOUT_ALLOC_GE_OVERSCAN) {
		struct vout_info_t *fb0_par;

		fb0_par = vout_info_get_entry(0);
		var->xres_virtual = fb0_par->resx_virtual;
		var->yres_virtual = fb0_par->resy_virtual * VPP_MB_ALLOC_NUM;
	} else {
		var->xres_virtual = vpp_calc_fb_width(
			(var->bits_per_pixel == 16) ?
			VDO_COL_FMT_RGB_565 : VDO_COL_FMT_ARGB,
			(var->xres_virtual < var->xres) ?
			var->xres : var->xres_virtual);
	}

	if (wmtfb_alloc(var, info))
		return -ENOMEM;

	if ((var->xres == 0) || (var->yres == 0))
		return -1;

	temp = var->xres_virtual * (var->bits_per_pixel >> 3);
	temp = (temp) ? (info->fix.smem_len / temp) : 0;
	if (temp < var->yres_virtual) {
		var->yres_virtual = temp;
	}

	/* more than 1M is khz not picos (for ut_vpp) */
	if (var->pixclock > 1000000) {
		temp = var->pixclock / 1000;
		temp = (temp) ? KHZ2PICOS(temp) : 0;
		DBG_MSG("pixclock patch(>1000000)%d-->%d\n",
			var->pixclock, temp);
		var->pixclock = temp;
	}

	/* less than 100 is fps not picos (for ut_vpp) */
	if ((var->pixclock > 0) && (var->pixclock < 100)) {
		unsigned int htotal, vtotal;

		htotal = var->xres + var->right_margin + var->hsync_len +
			var->left_margin;
		vtotal = var->yres + var->lower_margin + var->vsync_len +
			var->upper_margin;
		temp = (htotal * vtotal * var->pixclock) / 1000;
		temp = (temp) ? KHZ2PICOS(temp) : 0;
		DBG_MSG("pixclock patch(<100)%d-->%d\n", var->pixclock, temp);
		var->pixclock = temp;
	}
#ifdef DEBUG_DETAIL
	wmtfb_show_var("cur var", &info->var);
	wmtfb_show_var("new var", var);
#endif
	switch (var->bits_per_pixel) {
	case 1:
	case 8:
		if (var->red.offset > 8) {
			var->red.offset = 0;
			var->red.length = 8;
			var->green.offset = 0;
			var->green.length = 8;
			var->blue.offset = 0;
			var->blue.length = 8;
			var->transp.offset = 0;
			var->transp.length = 0;
		}
		break;
	case 16: /* ARGB 1555 */
		if (var->transp.length) {
			var->red.offset = 10;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 5;
			var->blue.offset = 0;
			var->blue.length = 5;
			var->transp.offset = 15;
			var->transp.length = 1;
		} else { /* RGB 565 */
			var->red.offset = 11;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 0;
			var->blue.length = 5;
			var->transp.offset = 0;
			var->transp.length = 0;
		}
		break;
	case 24: /* RGB 888 */
	case 32: /* ARGB 8888 */
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	}

	if (g_vpp.fb_manual)
		return 0;

	if (!wmtfb_probe_ready)
		force = 1;

	if (g_vpp.fb_recheck & (0x1 << info->node)) {
		force = 1;
		g_vpp.fb_recheck &= ~(0x1 << info->node);
	}

	if ((var->xres != info->var.xres) || (var->yres != info->var.yres) ||
		memcmp(&info->var.pixclock, &var->pixclock, 4 * 9) || force) {
		struct fb_videomode varfbmode;
		unsigned int yres_virtual;
		unsigned int xres_virtual;

		DPRINT("[wmtfb_check_var] fb%d res(%d,%d)->(%d,%d),force %d\n",
			info->node, info->var.xres, info->var.yres,
			var->xres, var->yres, force);
#ifdef DEBUG
		wmtfb_show_var("cur var", &info->var);
		wmtfb_show_var("new var", var);
#endif
		yres_virtual = var->yres_virtual;
		xres_virtual = var->xres_virtual;
		fb_var_to_videomode(&varfbmode, var);
#ifdef DEBUG
		DPRINT("new fps %d\n", varfbmode.refresh);
#endif
		if (vout_find_match_mode(info->node, &varfbmode, 1)) {
			DPRINT("[wmtfb] not support\n");
			return -1;
		}
		fb_videomode_to_var(var, &varfbmode);
		var->yres_virtual = yres_virtual;
		var->xres_virtual = xres_virtual;
#ifdef DEBUG
		wmtfb_show_var("[wmtfb] time change", var);
#endif
		vout_get_width_height(info->node, &var->width, &var->height);
	}
	return 0;
}

static int wmtfb_set_par
(
	struct fb_info *info /*!<; // a pointer point to struct fb_info */
)
{
	struct fb_var_screeninfo *var = &info->var;

	DBG_DETAIL("Enter fb%d(%dx%d)\n", info->node, var->xres, var->yres);

	/* init your hardware here */
	/* ... */
	if (var->bits_per_pixel == 8)
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else
		info->fix.visual = FB_VISUAL_TRUECOLOR;
	vpp_set_par(info);
	info->fix.line_length = var->xres_virtual * var->bits_per_pixel / 8;
	return 0;
}

static int wmtfb_setcolreg
(
	unsigned regno,         /*!<; // register no */
	unsigned red,           /*!<; // red color map */
	unsigned green,         /*!<; // green color map */
	unsigned blue,          /*!<; // blue color map */
	unsigned transp,        /*!<; // transp map */
	struct fb_info *info    /*!<; // a pointer point to struct fb_info */
)
{
	return 0;

}

static int wmtfb_pan_display
(
	struct fb_var_screeninfo *var,  /*!<; // a pointer fb_var_screeninfo */
	struct fb_info *info            /*!<; // a pointer fb_info */
)
{
	static struct timeval tv1 = {0, 0};

	DBG_DETAIL("Enter wmtfb_pan_display\n");

	wmtfb_set_mutex(info, 1);
	if (var->activate & FB_ACTIVATE_VBL) {
		struct timeval tv2;

		do_gettimeofday(&tv2);
		if (tv1.tv_sec) {
			int us;

			us = (tv2.tv_sec == tv1.tv_sec) ?
				(tv2.tv_usec - tv1.tv_usec) :
				(1000000 + tv2.tv_usec - tv1.tv_usec);
			if (us < 16667)
				vpp_wait_vsync(1, 1);
		}
	}
	vpp_pan_display(var, info);
	do_gettimeofday(&tv1);
	wmtfb_set_mutex(info, 0);

	DBG_DETAIL("Exit wmtfb_pan_display\n");
	return 0;
}

#define UMP_INVALID_SECURE_ID    ((unsigned int)-1)
#define GET_UMP_SECURE_ID        _IOWR('m', 310, unsigned int)
#define GET_UMP_SECURE_ID_BUF1   _IOWR('m', 311, unsigned int)
#define GET_UMP_SECURE_ID_BUF2   _IOWR('m', 312, unsigned int)
int wmtfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	int retval = 0;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		vpp_wait_vsync(info->node, 1);
		break;
	case GET_UMP_SECURE_ID:
	case GET_UMP_SECURE_ID_BUF1:
	case GET_UMP_SECURE_ID_BUF2:
		{
		unsigned int ump_id;
		extern unsigned int (*mali_get_ump_secure_id)
			(unsigned int addr, unsigned int size);
		if (mali_get_ump_secure_id)
			ump_id = (*mali_get_ump_secure_id)(info->fix.smem_start,
							   info->fix.smem_len);
		else
			ump_id = UMP_INVALID_SECURE_ID;
		printk("[wmtfb] ump_id %d,0x%x,len %d\n", ump_id,
			(int)info->fix.smem_start, info->fix.smem_len);
		return put_user((unsigned int) ump_id,
				(unsigned int __user *) arg);
		}
		break;
	default:
		break;
	}
	return retval;
}

static int wmtfb_mmap
(
	struct fb_info *info,           /*!<; // a pointer fb_info */
	struct vm_area_struct *vma      /*!<; // a pointer vm_area_struct */
)
{
	unsigned long off;
	unsigned long start;
	u32 len;

	DBGMSG("Enter wmtfb_mmap\n");

	/* frame buffer memory */
	start = info->fix.smem_start;
	off = vma->vm_pgoff << PAGE_SHIFT;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.smem_len);
	if (off >= len) {
		/* memory mapped io */
		off -= len;
		if (info->var.accel_flags)
			return -EINVAL;
		start = info->fix.mmio_start;
		len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.mmio_len);
	}

	start &= PAGE_MASK;
	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;
	DBGMSG("Exit wmtfb_mmap\n");
	return 0;
}

int wmtfb_hw_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	return 0;
}

int wmtfb_blank(int blank, struct fb_info *info)
{
	DBGMSG("(%d,%d)\n", info->node, blank);
	vpp_set_blank(info, blank);
	return 0;
}

/***************************************************************************
	driver file operations struct define
****************************************************************************/
static struct fb_ops wmtfb_ops = {
	.owner          = THIS_MODULE,
	.fb_open        = wmtfb_open,
	.fb_release     = wmtfb_release,
#if 0
	.fb_read      = wmtfb_read,
	.fb_write     = wmtfb_write,
#endif
	.fb_check_var   = wmtfb_check_var,
	.fb_set_par     = wmtfb_set_par,
	.fb_setcolreg   = wmtfb_setcolreg,
	.fb_pan_display = wmtfb_pan_display,
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea    = cfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
	.fb_blank       = wmtfb_blank,
	.fb_cursor      = wmtfb_hw_cursor,
	.fb_ioctl       = wmtfb_ioctl,
	.fb_mmap        = wmtfb_mmap,
};

static int __init wmtfb_probe
(
	struct platform_device *dev /*!<; // a pointer point to struct device */
)
{
	struct fb_info *info;
	struct fb_var_screeninfo var;
	int i;

	DBG_MSG("Enter\n");

	for (i = 1; ; i++) {
		memcpy(&var, &wmtfb_var, sizeof(struct fb_var_screeninfo));
		if (vpp_get_info(i, &var))
			break;
		info = framebuffer_alloc(0, &dev->dev);
		if (!info)
			return -ENOMEM;
		info->fbops = &wmtfb_ops;
		memcpy(&info->fix, &wmtfb_fix,
			sizeof(struct fb_fix_screeninfo));
		info->fix.id[5] = '0' + i;
		info->flags = FBINFO_DEFAULT;
		if (register_framebuffer(info) < 0)
			return -EINVAL;

		MSG(KERN_INFO "fb%d: %s frame buffer device\n",
			info->node, info->fix.id);

		wmtfb_check_var(&var, info);
		memcpy(&info->var, &var, sizeof(struct fb_var_screeninfo));
		wmtfb_set_par(info);
		wmtfb_pan_display(&info->var, info);

		if (info->node == 1) {
			info->dev->power.async_suspend = 1;
			dev_set_drvdata(&dev->dev, info);
		}
	}

	for (i = 0; i < VPP_VOUT_NUM; i++) {
		int blank;
		struct vout_t *vout;

		vout = vout_get_entry(i);
		if (!vout)
			continue;
		blank = (vout->status & VPP_VOUT_STS_ACTIVE) ?
			VOUT_BLANK_UNBLANK : VOUT_BLANK_POWERDOWN;
		blank = (vout->status & VPP_VOUT_STS_BLANK) ?
			VOUT_BLANK_NORMAL : blank;
		vout_set_blank(i, blank);
	}
	DBG_MSG("Leave\n");
	wmtfb_probe_ready = 1;
	g_vpp.govrh_preinit = 0;
	return 0;
}

static int wmtfb_remove
(
	struct platform_device *dev /*!<; // a pointer point to struct device */
)
{
	struct fb_info *info = dev_get_drvdata(&dev->dev);

	if (info) {
		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
	}
	return 0;
}

#ifdef CONFIG_PM
static int wmtfb_suspend
(
	struct platform_device *pDev,   /*!<; // a pointer struct device */
	pm_message_t state		/*!<; // suspend state */
)
{
	return 0;
}

static int wmtfb_resume
(
	struct platform_device *pDev	/*!<; // a pointer struct device */
)
{
	return 0;
}
#else
#define wmtfb_suspend NULL
#define wmtfb_resume NULL
#endif

/***************************************************************************
	device driver struct define
****************************************************************************/
static struct platform_driver wmtfb_driver = {
	.driver.name	= "wmtfb",
	.driver.bus	= &platform_bus_type,
	.probe		= wmtfb_probe,
	.remove		= wmtfb_remove,
	.suspend	= wmtfb_suspend,
	.resume		= wmtfb_resume,
};

/***************************************************************************
	platform device struct define
****************************************************************************/
static u64 wmtfb_dma_mask = 0xffffffffUL;
static struct platform_device wmtfb_device = {
	.name   = "wmtfb",
	.dev    = {
		.dma_mask = &wmtfb_dma_mask,
		.coherent_dma_mask = ~0,
		.power.async_suspend = 1,
	},

#if 0
	.id     = 0,
	.dev    = {
		.release = wmtfb_platform_release,
	},
	.num_resources  = 0,    /* ARRAY_SIZE(wmtfb_resources), */
	.resource       = NULL, /* wmtfb_resources, */
#endif
};

static int __init wmtfb_init(void)
{
	int ret;

	/*
	 *  For kernel boot options (in 'video=wmtfb:<options>' format)
	 */
	ret = platform_driver_register(&wmtfb_driver);
	if (!ret) {
		ret = platform_device_register(&wmtfb_device);
		if (ret)
			platform_driver_unregister(&wmtfb_driver);
	}
	return ret;

}
module_init(wmtfb_init);

static void __exit wmtfb_exit(void)
{
	printk(KERN_ALERT "Enter wmtfb_exit\n");

	platform_driver_unregister(&wmtfb_driver);
	platform_device_unregister(&wmtfb_device);
	return;
}
module_exit(wmtfb_exit);

MODULE_AUTHOR("WonderMedia SW Team");
MODULE_DESCRIPTION("wmtfb device driver");
MODULE_LICENSE("GPL");
/*--------------------End of Function Body -----------------------------------*/
#undef WMTFB_C
