/*++
 * linux/drivers/video/wmt/vout.c
 * WonderMedia video post processor (VPP) driver
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

#define VOUT_C
#undef DEBUG
/* #define DEBUG */
/* #define DEBUG_DETAIL */
/*----------------------- DEPENDENCE -----------------------------------------*/
#include "vout.h"

/*----------------------- PRIVATE MACRO --------------------------------------*/
/* #define  VO_XXXX  xxxx    *//*Example*/

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define VO_XXXX    1     *//*Example*/

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx vo_xxx_t; *//*Example*/

/*----------EXPORTED PRIVATE VARIABLES are defined in vout.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  vo_xxx;        *//*Example*/
struct vout_t *vout_array[VPP_VOUT_NUM];
struct vout_inf_t *vout_inf_array[VOUT_INF_MODE_MAX];
struct vout_dev_t *vout_dev_list;

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void vo_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
/*----------------------- vout API --------------------------------------*/
void vout_register(int no, struct vout_t *vo)
{
	if (no >= VPP_VOUT_NUM)
		return;

	vo->num = no;
	vo->govr = (void *) p_govrh;
	vo->status = VPP_VOUT_STS_REGISTER;
	vo->info = 0;
	vout_array[no] = vo;
}

struct vout_t *vout_get_entry(int no)
{
	if (no >= VPP_VOUT_NUM)
		return 0;
	return vout_array[no];
}
EXPORT_SYMBOL(vout_get_entry);

struct vout_info_t *vout_get_info_entry(int no)
{
	struct vout_info_t *info;
	struct vout_t *vout;
	int i, j;

	if (no >= VPP_VOUT_NUM)
		return 0;

	for (i = 0; i < VPP_VOUT_INFO_NUM; i++) {
		info = vout_info[i];
		if (!info)
			break;
		for (j = 0; j < VPP_VOUT_NUM; j++) {
			vout = info->vout[j];
			if (vout == 0)
				break;
			if (vout->num == no)
				return info;
		}
	}
	return 0;
}

void vout_change_status(struct vout_t *vo, int mask, int sts)
{
	DBG_DETAIL("(0x%x,%d)\n", mask, sts);
	if (sts)
		vo->status |= mask;
	else
		vo->status &= ~mask;

	switch (mask) {
	case VPP_VOUT_STS_PLUGIN:
		if (sts == 0) {
			vo->status &= ~(VPP_VOUT_STS_EDID +
				VPP_VOUT_STS_CONTENT_PROTECT);
			vo->edid_info.option = 0;
#ifdef __KERNEL__
			vpp_netlink_notify_cp(0);
#endif
		}
		break;
	default:
		break;
	}
}

int vout_query_inf_support(int no, enum vout_inf_mode_t mode)
{
	struct vout_t *vo;

	if (no >= VPP_VOUT_NUM)
		return 0;

	if (mode >= VOUT_INF_MODE_MAX)
		return 0;

	vo = vout_get_entry(no);
	return (vo->fix_cap & BIT(mode)) ? 1 : 0;
}

/*----------------------- vout interface API --------------------------------*/
int vout_inf_register(enum vout_inf_mode_t mode, struct vout_inf_t *inf)
{
	if (mode >= VOUT_INF_MODE_MAX) {
		DBG_ERR("vout interface mode invalid %d\n", mode);
		return -1;
	}

	if (vout_inf_array[mode])
		DBG_ERR("vout interface register again %d\n", mode);

	vout_inf_array[mode] = inf;
	return 0;
} /* End of vout_register */

struct vout_inf_t *vout_inf_get_entry(enum vout_inf_mode_t mode)
{
	if (mode >= VOUT_INF_MODE_MAX) {
		DBG_ERR("vout interface mode invalid %d\n", mode);
		return 0;
	}
	return vout_inf_array[mode];
}

/*----------------------- vout device API -----------------------------------*/
int vout_device_register(struct vout_dev_t *ops)
{
	struct vout_dev_t *list;

	if (vout_dev_list == 0) {
		vout_dev_list = ops;
		list = ops;
	} else {
		list = vout_dev_list;
		while (list->next != 0)
			list = list->next;
		list->next = ops;
	}
	ops->next = 0;
	return 0;
}

struct vout_dev_t *vout_get_device(struct vout_dev_t *ops)
{
	if (ops == 0)
		return vout_dev_list;
	return ops->next;
}

struct vout_t *vout_get_entry_adapter(enum vout_mode_t mode)
{
	int no;

	switch (mode) {
	case VOUT_SD_DIGITAL:
	case VOUT_DVI:
	case VOUT_LCD:
	case VOUT_DVO2HDMI:
	case VOUT_SD_ANALOG:
	case VOUT_VGA:
		no = VPP_VOUT_NUM_DVI;
		break;
	case VOUT_HDMI:
		no = VPP_VOUT_NUM_HDMI;
		break;
	case VOUT_LVDS:
		no = VPP_VOUT_NUM_LVDS;
		break;
	default:
		no = VPP_VOUT_NUM;
		break;
	}
	return vout_get_entry(no);
}

struct vout_inf_t *vout_get_inf_entry_adapter(enum vout_mode_t mode)
{
	int no;

	switch (mode) {
	case VOUT_SD_DIGITAL:
	case VOUT_SD_ANALOG:
	case VOUT_DVI:
	case VOUT_LCD:
	case VOUT_DVO2HDMI:
	case VOUT_VGA:
		no = VOUT_INF_DVI;
		break;
	case VOUT_HDMI:
		no = VOUT_INF_HDMI;
		break;
	case VOUT_LVDS:
		no = VOUT_INF_LVDS;
		break;
	default:
		no = VOUT_INF_MODE_MAX;
		return 0;
	}
	return vout_inf_get_entry(no);
}

enum vpp_vout_s vout_get_mode_adapter(struct vout_t *vout)
{
	enum vpp_vout_s mode;

	switch (vout->inf->mode) {
	case VOUT_INF_DVI:
		mode = VPP_VOUT_DVI;
		if (vout->dev && (strcmp("LCD", vout->dev->name) == 0))
			mode = VPP_VOUT_LCD;
		break;
	case VOUT_INF_HDMI:
		mode = VPP_VOUT_HDMI;
		break;
	case VOUT_INF_LVDS:
		mode = VPP_VOUT_LVDS;
		break;
	default:
		mode = VPP_VOUT_NONE;
		break;
	}
	return mode;
}

int vout_info_add_entry(int no, struct vout_t *vo)
{
	struct vout_info_t *info;
	int i = 0;

	if ((vo == 0) || (vo->info))
		return 0;

	info = vout_info[no];
	info->num = no;
	if (vo->num < VPP_VOUT_NUM) { /* not virtual vout */
		for (i = 0; i < VPP_VOUT_NUM; i++) {
			if (info->vout[i] == 0) {
				info->vout[i] = vo;
				vo->info = info;
				break;
			} else {
				if (info->vout[i] == vo) /* exist */
					break;
			}
		}
	}

	if (i == 0) { /* new */
		info->resx = vo->resx;
		info->resy = vo->resy;
		info->resx_virtual = vpp_calc_align(info->resx, 4);
		info->resy_virtual = info->resy;
		info->fps = (int) vo->fps;
		DBG_MSG("new %dx%d@%d\n", info->resx, info->resy, info->fps);
	}

	DBG_MSG("info %d,%dx%d@%d\n", no, info->resx, info->resy, info->fps);
	return no;
}

struct vout_info_t *vout_info_get_entry(int no)
{
	if (no >= VPP_VOUT_INFO_NUM)
		return 0;
	return vout_info[no];
}

void vout_info_set_fixed_timing(int no, struct fb_videomode *vmode)
{
	struct vout_info_t *info;

	DBG_MSG("(%d)\n", no);

	info = vout_info_get_entry(no);
	if (!info->fixed_vmode) {
		info->fixed_vmode =
			kmalloc(sizeof(struct fb_videomode), GFP_KERNEL);
		if (!info->fixed_vmode)
			return;
	}
	memcpy(info->fixed_vmode, vmode, sizeof(struct fb_videomode));
}

struct govrh_mod_t *vout_info_get_govr(int no)
{
	struct vout_info_t *info;
	struct govrh_mod_t *govr = 0;
	int i;

	info = vout_info[no];
	if (!info)
		return 0;

	for (i = 0; i < VPP_VOUT_NUM; i++) {
		if (info->vout[i]) {
			if (govr == 0)
				govr = info->vout[i]->govr;
			if (info->vout[i]->status & VPP_VOUT_STS_ACTIVE) {
				govr = info->vout[i]->govr;
				break;
			}
		}
	}
	return govr;
}

int vout_check_ratio_16_9(unsigned int resx, unsigned int resy)
{
	int val;

	val = (resy) ? ((resx * 10) / resy) : 0;
	return (val >= 15) ? 1 : 0;
}

struct fb_videomode *vout_get_video_mode(int vout_num,
	struct fb_videomode *vmode, int option)
{
	int i;
	struct fb_videomode *p, *best = NULL;
	unsigned int diff = -1, diff_refresh = -1;
	int d;
	int resx, resy, fps;
	unsigned int pixel_clock, diff_pixel_clock = -1;
	struct vout_t *vo = 0;
	char *edid = 0;

	resx = vmode->xres;
	resy = vmode->yres;
	fps = vmode->refresh;
#ifdef DEBUG
	pixel_clock = (vmode->pixclock) ? PICOS2KHZ(vmode->pixclock) * 1000 : 0;
	DBG_MSG("%d,%dx%d@%d,%d,0x%x\n", vout_num, resx, resy, fps,
		pixel_clock, option);
#endif
	pixel_clock = vmode->pixclock;

	/* EDID detail timing */
	if (option & VOUT_MODE_OPTION_EDID) {
		unsigned int opt;
		struct fb_videomode *edid_vmode;

		vo = vout_get_entry(vout_num);
		if (vo == 0)
			return 0;

		edid = vout_get_edid(vout_num);
		if (edid_parse(edid, &vo->edid_info)) {
			opt = fps | ((option &
				VOUT_MODE_OPTION_INTERLACE) ?
				EDID_TMR_INTERLACE : 0);
			if (edid_find_support(&vo->edid_info,
				resx, resy, opt, &edid_vmode)) {
				if (edid_vmode) {
					DBG_MSG("EDID detail timing\n");
					return edid_vmode;
				}
			}
		}
	}

	/* video mode table */
	for (i = 0; ; i++) {
		p = (struct fb_videomode *) &vpp_videomode[i];
		if (p->pixclock == 0)
			break;

		if (option & VOUT_MODE_OPTION_LESS) {
			if ((p->xres > resx) || (p->yres > resy))
				continue;
		}
		if (option & VOUT_MODE_OPTION_GREATER) {
			if ((p->xres < resx) || (p->yres < resy))
				continue;
		}
		if (option & VOUT_MODE_OPTION_INTERLACE) {
			if (!(p->vmode & FB_VMODE_INTERLACED))
				continue;
		}
		if (option & VOUT_MODE_OPTION_PROGRESS) {
			if ((p->vmode & FB_VMODE_INTERLACED))
				continue;
		}
		if ((option & VOUT_MODE_OPTION_EDID) &&
			(edid_parse(edid, &vo->edid_info))) {
			unsigned int opt;
			struct fb_videomode *edid_vmode;

			opt = p->refresh | ((option &
				VOUT_MODE_OPTION_INTERLACE) ?
				EDID_TMR_INTERLACE : 0);
			if (edid_find_support(&vo->edid_info,
				p->xres, p->yres, opt, &edid_vmode)) {
				if (edid_vmode)
					p = edid_vmode;
			} else {
				continue;
			}
		}
		d = abs(p->xres - resx) + abs(p->yres - resy);
		d = abs(d);
		if (diff > d) {
			diff = d;
			diff_refresh = abs(p->refresh - fps);
			diff_pixel_clock = abs(p->pixclock - pixel_clock);
			best = p;
		} else if (diff == d) {
			d = abs(p->refresh - fps);
			if (diff_refresh > d) {
				diff_refresh = d;
				diff_pixel_clock =
					abs(p->pixclock - pixel_clock);
				best = p;
			} else if (diff_refresh == d) {
				d = abs(p->pixclock - pixel_clock);
				if (diff_pixel_clock > d) {
					diff_pixel_clock = d;
					best = p;
				}
			}
		}
	}
	if (best)
		DBG_MSG("%dx%d@%d\n", best->xres, best->yres, best->refresh);
	return best;
}

int vout_get_width_height(int fbnum, int *width, int *height)
{
	struct vout_info_t *info;
	int i;

	info = vout_info_get_entry(fbnum);
	*width = 0;
	*height = 0;
	for (i = 0; i < VPP_VOUT_NUM; i++) {
		struct vout_t *vout;

		vout = info->vout[i];
		if (vout) {
			if ((vout->inf->mode == VOUT_INF_DVI) && p_lcd) {
				if (info->fixed_width != 0 && info->fixed_height != 0) {
					*width = info->fixed_width;
					*height = info->fixed_height;
				} else {
					*width = p_lcd->width;
					*height = p_lcd->height;
				}
				break;
			}
			if (vout_chkplug(vout->num)) {
				if (vout_get_edid_option(vout->num)
					& EDID_OPT_VALID) {
					*width = vout->edid_info.width;
					*height = vout->edid_info.height;
					break;
				}
			}
		}
	}
	return 0;
}

#ifndef CONFIG_VPOST
int vout_find_match_mode(int fbnum,
				struct fb_videomode *vmode, int match)
{
	struct vout_info_t *info;
	struct fb_videomode *p;
	int no = VPP_VOUT_NUM;
	unsigned int option;
	int i;

	DBG_DETAIL("(%d)\n", fbnum);

	info = vout_info_get_entry(fbnum);
	if (vmode->refresh == 59)
		vmode->refresh = 60;

	/* fixed timing */
	if (info->fixed_vmode) {
		if (info->fixed_vmode->xres != vmode->xres)
			return -1;
		if (info->fixed_vmode->yres != vmode->yres)
			return -1;
		if (info->fixed_vmode->refresh != vmode->refresh)
			return -1;
		p = info->fixed_vmode;
		goto label_find_match;
	}
	for (i = 0; i < VPP_VOUT_NUM; i++) {
		if (info->vout[i]) {
			int vout_no = info->vout[i]->num;

			if (no == VPP_VOUT_NUM)
				no = vout_no; /* get first vo */
			if (vout_chkplug(vout_no)) {
				no = vout_no;
				break;
			}
		}
	}
	/* resolution match and interlace match */
	option = VOUT_MODE_OPTION_GREATER + VOUT_MODE_OPTION_LESS;
	option |= (no == VPP_VOUT_NUM) ? 0 : VOUT_MODE_OPTION_EDID;
	option |= (vmode->vmode & FB_VMODE_INTERLACED) ?
		VOUT_MODE_OPTION_INTERLACE : VOUT_MODE_OPTION_PROGRESS;
	p = vout_get_video_mode(no, vmode, option);
	if (p)
		goto label_find_match;

	/* resolution match but interlace not match */
	option = VOUT_MODE_OPTION_GREATER + VOUT_MODE_OPTION_LESS;
	option |= (no == VPP_VOUT_NUM) ? 0 : VOUT_MODE_OPTION_EDID;
	option |= (vmode->vmode & FB_VMODE_INTERLACED) ?
		VOUT_MODE_OPTION_PROGRESS : VOUT_MODE_OPTION_INTERLACE;
	p = vout_get_video_mode(no, vmode, option);
	if (p)
		goto label_find_match;

/*	if( !match ){ */
		/* resolution less best mode */
		option = VOUT_MODE_OPTION_LESS;
		option |= (no == VPP_VOUT_NUM) ? 0 : VOUT_MODE_OPTION_EDID;
		p = vout_get_video_mode(no, vmode, option);
		if (p)
			goto label_find_match;
/*	} */
	DBG_ERR("no support mode\n");
	return -1;
label_find_match:
	*vmode = *p;
#ifdef CONFIG_UBOOT
	info->p_vmode = p;
#endif
	return 0;
}
#endif

int vout_find_edid_support_mode(
	struct edid_info_t *info,
	unsigned int *resx,
	unsigned int *resy,
	unsigned int *fps,
	int r_16_9
)
{
	/* check the EDID to find one that not large and same ratio mode*/
#ifdef CONFIG_WMT_EDID
	int i, cnt;
	struct fb_videomode *p;
	unsigned int w, h, f, option;

	if ((*resx == 720) && (*resy == 480) && (*fps == 50))
		*fps = 60;

	for (i = 0, cnt = 0; ; i++) {
		if (vpp_videomode[i].pixclock == 0)
			break;
		cnt++;
	}

	for (i = cnt - 1; i >= 0; i--) {
		p = (struct fb_videomode *) &vpp_videomode[i];
		h = p->yres;
		if (h > *resy)
			continue;

		w = p->xres;
		if (w > *resx)
			continue;

		f = p->refresh;
		if (f > *fps)
			continue;

		if (r_16_9 != vout_check_ratio_16_9(w, h))
			continue;

		option = f & EDID_TMR_FREQ;
		option |= (p->vmode & FB_VMODE_INTERLACED) ?
			EDID_TMR_INTERLACE : 0;

		if (edid_find_support(info, w, h, option, &p)) {
			*resx = w;
			*resy = h;
			*fps = f;
			DBG_MSG("(%dx%d@%d)\n", w, h, f);
			return 1;
		}
	}
#endif
	return 0;
}

/*----------------------- vout control API ----------------------------------*/
void vout_set_framebuffer(struct vout_info_t *info, vdo_framebuf_t *fb)
{
	int i;
	struct vout_t *vo;

	if (!info)
		return;

	for (i = 0; i < VPP_VOUT_NUM; i++) {
		vo = info->vout[i];
		if (vo == 0)
			break;
		if (vo->govr)
			vo->govr->fb_p->set_framebuf(fb);
	}
}

int vout_set_blank(int no, enum vout_blank_t arg)
{
	struct vout_t *vo;

	DBG_DETAIL("(%d,%d)\n", no, arg);

	vo = vout_get_entry(no);
	if (vo && (vo->inf)) {
		vout_change_status(vo, VPP_VOUT_STS_BLANK, arg);
		vo->inf->blank(vo, arg);
		if (vo->dev && vo->dev->set_power_down)
			vo->dev->set_power_down(
				(arg == VOUT_BLANK_POWERDOWN) ? 1 : 0);
		if (vo->govr)
			govrh_set_MIF_enable(vo->govr, (arg) ? 0 : 1);
	}
	return 0;
}

int vout_set_mode(int no, enum vout_inf_mode_t mode)
{
	struct vout_t *vo;

	DBG_DETAIL("(%d,%d)\n", no, mode);

	if (vout_query_inf_support(no, mode) == 0) {
		DBG_ERR("not support this interface(%d,%d)\n", no, mode);
		return -1;
	}

	vo = vout_get_entry(no);
	if (vo->inf) {
		if (vo->inf->mode == mode)
			return 0;
		vo->inf->uninit(vo, 0);
		vout_change_status(vo, VPP_VOUT_STS_ACTIVE, 0);
		if (vo->dev)
			vo->dev->set_power_down(1);
	}

	vo->inf = vout_inf_get_entry(mode);
	vo->inf->init(vo, 0);
	vout_change_status(vo, VPP_VOUT_STS_ACTIVE, 1);
	return 0;
}

int vout_config(struct vout_info_t *info, struct fb_videomode *vmode,
	vdo_framebuf_t *fb)
{
	struct vout_t *vo;
	int i;

	DBG_DETAIL("\n");

	if (!vmode && !fb)
		return 0;

	if (vmode) {
		/* option for interface & device config */
		info->resx = vmode->xres;
		info->resy = vmode->yres;
		info->fps = (vmode->refresh == 59) ? 60 : vmode->refresh;
		info->option = (vmode->vmode & FB_VMODE_INTERLACED) ?
						VPP_OPT_INTERLACE : 0;
		info->option |= (vmode->sync & FB_SYNC_HOR_HIGH_ACT) ?
						VPP_DVO_SYNC_POLAR_HI : 0;
		info->option |= (vmode->sync & FB_SYNC_VERT_HIGH_ACT) ?
						VPP_DVO_VSYNC_POLAR_HI : 0;
	}

	for (i = 0; i < VPP_VOUT_NUM; i++) {
		vo = info->vout[i];
		if (vo == 0)
			break;

		if (vo->govr == 0)
			continue;

		if (vmode) {
			govrh_set_videomode(vo->govr, vmode);

			if (vo->inf) {
				vo->inf->config(vo, (int)info);
				if (vo->dev)
					vo->dev->config(info);
			}
		}

		if (fb)
			govrh_set_framebuffer(vo->govr, fb);
	}
	return 0;
}

int vout_chkplug(int no)
{
	struct vout_t *vo;
	struct vout_inf_t *inf;
	int ret = 0;

	DBG_DETAIL("(%d)\n", no);

	vo = vout_get_entry(no);
	if (vo == 0)
		return 0;

	if (vo->inf == 0)
		return 0;

	inf = vout_inf_get_entry(vo->inf->mode);
	if (inf == 0)
		return 0;

	if (vo->dev && vo->dev->check_plugin)
		ret = vo->dev->check_plugin(0);
	else
		ret = inf->chkplug(vo, 0);
	vout_change_status(vo, VPP_VOUT_STS_PLUGIN, ret);
	return ret;
}

int vout_inf_chkplug(int no, enum vout_inf_mode_t mode)
{
	struct vout_t *vo;
	struct vout_inf_t *inf;
	int plugin = 0;

	DBG_MSG("(%d,%d)\n", no, mode);
	if (vout_query_inf_support(no, mode) == 0)
		return 0;

	vo = vout_get_entry(no);
	inf = vout_inf_get_entry(mode);
	if (inf) {
		if (inf->chkplug)
			plugin = inf->chkplug(vo, 0);
	}
	return plugin;
}

char *vout_get_edid(int no)
{
	struct vout_t *vo;
	int ret;

	DBG_DETAIL("(%d)\n", no);

	if (edid_disable)
		return 0;
	vo = vout_get_entry(no);
	if (vo == 0)
		return 0;

	if (vo->status & VPP_VOUT_STS_EDID) {
		DBG_MSG("edid exist\n");
		return vo->edid;
	}

	vout_change_status(vo, VPP_VOUT_STS_EDID, 0);
#ifdef CONFIG_VOUT_EDID_ALLOC
	if (vo->edid == 0) {
		vo->edid = kmalloc(128 * EDID_BLOCK_MAX, GFP_KERNEL);
		if (!vo->edid) {
			DBG_ERR("edid buf alloc\n");
			return 0;
		}
	}
#endif

	ret = 1;
	if (vo->dev && vo->dev->get_edid) {
		ret = vo->dev->get_edid(vo->edid);
	} else {
		if (vo->inf->get_edid)
			ret = vo->inf->get_edid(vo, (int)vo->edid);
	}

	if (ret == 0) {
		DBG_DETAIL("edid read\n");
		vout_change_status(vo, VPP_VOUT_STS_EDID, 1);
		return vo->edid;
	} else {
		DBG_MSG("read edid fail\n");
	}

#ifdef CONFIG_VOUT_EDID_ALLOC
	kfree(vo->edid);
	vo->edid = 0;
#endif
	return 0;
}

int vout_get_edid_option(int no)
{
	struct vout_t *vo;

	DBG_DETAIL("(%d)\n", no);

	vo = vout_get_entry(no);
	if (vo == 0)
		return 0;

	if (vo->edid_info.option)
		return vo->edid_info.option;

	if (vout_get_edid(no) == 0) {
		if (no == VPP_VOUT_NUM_HDMI) { /* HDMI wo EDID still can work */
			vo->edid_info.option = (EDID_OPT_HDMI + EDID_OPT_AUDIO);
			return vo->edid_info.option;
		}
		return 0;
	}

	edid_parse(vo->edid, &vo->edid_info);
	return vo->edid_info.option;
}

unsigned int vout_get_mask(struct vout_info_t *vo_info)
{
	unsigned int mask;
	int i;

	if (g_vpp.virtual_display) {
		if (vo_info->num == 0)
			return 0;
		return VPP_VOUT_ALL;
	}

	mask = 0;
	for (i = 0; i <= VPP_VOUT_NUM; i++) {
		if (vo_info->vout[i] == 0)
			break;
		mask |= (0x1 << vo_info->vout[i]->num);
	}
	return mask;
}

int vout_check_plugin(int clr_sts)
{
	struct vout_t *vo;
	int i;
	int plugin = 0;

	for (i = 0; i <= VPP_VOUT_NUM; i++) {
		vo = vout_get_entry(i);
		if (vo == 0)
			continue;
		if (vo->inf == 0)
			continue;

		if (vo->dev) {
			if (!(vo->dev->capability & VOUT_DEV_CAP_FIX_PLUG)) {
				if (vout_chkplug(i)) {
					plugin = 1;
					if (clr_sts)
						vout_change_status(vo,
							VPP_VOUT_STS_PLUGIN, 0);
					DBG_MSG("[VPP] ext dev plugin\n");
				}
			}
		} else {
			if (!(vo->inf->capability & VOUT_INF_CAP_FIX_PLUG)) {
				if (vout_chkplug(i)) {
					plugin = 1;
					if (clr_sts)
						vout_change_status(vo,
							VPP_VOUT_STS_PLUGIN, 0);
					DBG_MSG("[VPP] inf dev plugin\n");
				}
			}
		}
	}
	return plugin;
}

enum vout_tvformat_t vout_get_tvformat(void)
{
	char buf[40] = {0};
	int varlen = 40;
	enum vout_tvformat_t s_tvformat = TV_MAX;

	if (wmt_getsyspara("wmt.display.tvformat", buf, &varlen) == 0) {
		if (!strnicmp(buf, "PAL", 3))
			s_tvformat = TV_PAL;
		else if (!strnicmp(buf, "NTSC", 4))
			s_tvformat = TV_NTSC;
		else
			s_tvformat = TV_UNDEFINED;
	} else
		s_tvformat = TV_UNDEFINED;

	return s_tvformat;
}

/*--------------------End of Function Body -----------------------------------*/
#undef VOUT_C
