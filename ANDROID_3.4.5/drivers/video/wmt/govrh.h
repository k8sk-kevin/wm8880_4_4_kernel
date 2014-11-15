/*++
 * linux/drivers/video/wmt/govrh.h
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

#include "vpp.h"

#ifdef WMT_FTBLK_GOVRH

#ifndef GOVRH_H
#define GOVRH_H

struct govrh_mod_t {
	VPP_MOD_BASE;

	unsigned int *reg_bk2;
	unsigned int pm_enable;
	unsigned int pm_tg;
	unsigned int vo_clock;
	unsigned int underrun_cnt;
	unsigned int csc_mode_force;
};

#ifdef WMT_FTBLK_GOVRH_CURSOR
#define GOVRH_CURSOR_HIDE_TIME	15
struct govrh_cursor_mod_t {
	VPP_MOD_BASE;

	unsigned int posx;
	unsigned int posy;
	unsigned int hotspot_x;
	unsigned int hotspot_y;
	int chg_flag;
	vdo_color_fmt colfmt;
	unsigned int cursor_addr1;
	unsigned int cursor_addr2;
	int enable;
	int hide_cnt;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef GOVRH_C
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN struct govrh_mod_t *p_govrh;
#ifdef WMT_FTBLK_GOVRH_CURSOR
EXTERN struct govrh_cursor_mod_t *p_cursor;
#endif
#ifdef WMT_FTBLK_GOVRH2
EXTERN struct govrh_mod_t *p_govrh2;
#endif

EXTERN void govrh_set_tg_enable(struct govrh_mod_t *base,
					vpp_flag_t enable);
EXTERN int govrh_get_tg_mode(struct govrh_mod_t *base);
EXTERN int govrh_get_hscale_up(struct govrh_mod_t *base);
EXTERN void govrh_set_direct_path(struct govrh_mod_t *base, int enable);
EXTERN int govrh_get_dvo_enable(struct govrh_mod_t *base);
EXTERN void govrh_set_dvo_enable(struct govrh_mod_t *base,
	vpp_flag_t enable);
EXTERN void govrh_set_dvo_sync_polar(struct govrh_mod_t *base,
	vpp_flag_t hsync, vpp_flag_t vsync);
EXTERN void govrh_set_dvo_outdatw(struct govrh_mod_t *base,
	vpp_datawidht_t width);
EXTERN void govrh_set_dvo_clock_delay(struct govrh_mod_t *base,
	int inverse, int delay);
EXTERN void govrh_set_colorbar(struct govrh_mod_t *base,
	vpp_flag_t enable, int mode, int inv);
EXTERN void govrh_set_contrast(struct govrh_mod_t *base,
	unsigned int value);
EXTERN unsigned int govrh_get_contrast(struct govrh_mod_t *base);
EXTERN void govrh_set_brightness(struct govrh_mod_t *base,
	unsigned int value);
EXTERN unsigned int govrh_get_brightness(struct govrh_mod_t *base);
EXTERN void govrh_set_saturation(struct govrh_mod_t *base, unsigned int value);
EXTERN unsigned int govrh_get_saturation(struct govrh_mod_t *base);
EXTERN void govrh_set_MIF_enable(struct govrh_mod_t *base, vpp_flag_t enable);
EXTERN int govrh_get_MIF_enable(struct govrh_mod_t *base);
EXTERN void govrh_set_color_format(struct govrh_mod_t *base,
	vdo_color_fmt format);
EXTERN vdo_color_fmt govrh_get_color_format(struct govrh_mod_t *base);
EXTERN void govrh_set_source_format(struct govrh_mod_t *base,
	vpp_display_format_t format);
EXTERN void govrh_set_output_format(struct govrh_mod_t *base,
	vpp_display_format_t field);
EXTERN void govrh_set_fb_addr(struct govrh_mod_t *base,
	unsigned int y_addr, unsigned int c_addr);
EXTERN void govrh_get_fb_addr(struct govrh_mod_t *base,
	unsigned int *y_addr, unsigned int *c_addr);
EXTERN void govrh_set_fb_width(struct govrh_mod_t *base,
	unsigned int width);
EXTERN void govrh_set_fb_info(struct govrh_mod_t *base,
	unsigned int width, unsigned int act_width,
	unsigned int x_offset, unsigned int y_offset);
EXTERN void govrh_get_fb_info(struct govrh_mod_t *base,
	unsigned int *width, unsigned int *act_width,
	unsigned int *x_offset, unsigned int *y_offset);
EXTERN void govrh_set_fifo_index(struct govrh_mod_t *base,
	unsigned int index);
EXTERN void govrh_set_reg_level(struct govrh_mod_t *base,
	vpp_reglevel_t level);
EXTERN void govrh_set_reg_update(struct govrh_mod_t *base,
	vpp_flag_t enable);
EXTERN void govrh_set_csc_mode(struct govrh_mod_t *base,
	vpp_csc_t mode);
EXTERN void govrh_set_framebuffer(struct govrh_mod_t *base,
	vdo_framebuf_t *inbuf);
EXTERN void govrh_get_framebuffer(struct govrh_mod_t *base,
	vdo_framebuf_t *fb);
EXTERN vdo_color_fmt govrh_get_dvo_color_format(
	struct govrh_mod_t *base);
EXTERN void govrh_set_dvo_color_format(struct govrh_mod_t *base,
	vdo_color_fmt fmt);
EXTERN enum vpp_int_err_t govrh_get_int_status(struct govrh_mod_t *base);
EXTERN void govrh_clean_int_status(struct govrh_mod_t *base,
	enum vpp_int_err_t int_sts);
EXTERN unsigned int govrh_set_clock(struct govrh_mod_t *base,
	unsigned int pixel_clock);
EXTERN void govrh_set_videomode(struct govrh_mod_t *base,
	struct fb_videomode *vmode);
EXTERN void govrh_get_tg(struct govrh_mod_t *base, vpp_clock_t *tmr);
EXTERN void govrh_get_videomode(struct govrh_mod_t *base,
	struct fb_videomode *vmode);
EXTERN void govrh_HDMI_set_blank_value(struct govrh_mod_t *base,
	unsigned int val);
EXTERN void govrh_HDMI_set_3D_mode(struct govrh_mod_t *base,
	int mode);
EXTERN int govrh_is_top_field(struct govrh_mod_t *base);
EXTERN void govrh_IGS_set_mode(struct govrh_mod_t *base,
	int no, int mode_18bit, int msb);
EXTERN void govrh_IGS_set_RGB_swap(struct govrh_mod_t *base,
	int mode);
EXTERN int govrh_mod_init(void);

#ifdef WMT_FTBLK_GOVRH_CURSOR
EXTERN void govrh_CUR_set_enable(struct govrh_cursor_mod_t *base,
	vpp_flag_t enable);
EXTERN void govrh_CUR_set_framebuffer(struct govrh_cursor_mod_t *base,
	vdo_framebuf_t *fb);
EXTERN void govrh_CUR_set_coordinate(struct govrh_cursor_mod_t *base,
	unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
EXTERN void govrh_CUR_set_position(struct govrh_cursor_mod_t *base,
	unsigned int x, unsigned int y);
EXTERN void govrh_CUR_set_color_key_mode(struct govrh_cursor_mod_t *base,
	int alpha, int enable, int mode);
EXTERN void govrh_CUR_set_color_key(struct govrh_cursor_mod_t *base,
	int enable, int alpha, unsigned int colkey);
EXTERN void govrh_CUR_set_colfmt(struct govrh_cursor_mod_t *base,
	vdo_color_fmt colfmt);
EXTERN void govrh_CUR_irqproc(int arg);
#endif

#undef EXTERN

#ifdef __cplusplus
}
#endif
#endif /* GOVRH_H */
#endif /* WMT_FTBLK_GOVRH */
