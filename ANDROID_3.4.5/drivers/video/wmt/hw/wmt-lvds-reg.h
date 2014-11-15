/*++
 * linux/drivers/video/wmt/hw/wmt-lvds-reg.h
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

#ifndef WMT_LVDS_REG_H
#define WMT_LVDS_REG_H

#define WMT_FTBLK_LVDS

struct lvds_base_regs {
	union {
		unsigned int val;
		struct {
			unsigned int inv_clk:1;
			unsigned int _01_03:3;
			unsigned int dual_channel:1;
			unsigned int _05_07:3;
			unsigned int test:4;
		} b;
	} status; /* 0x00 */

	union {
		unsigned int val;
		struct {
			unsigned int drv_pdmode:1;
			unsigned int _01:1;
			unsigned int vbg_sel:2;
			unsigned int _04_07:4;
			unsigned int pd:1;
			unsigned int tre_en:2;
			unsigned int _11:1;
			unsigned int pllck_dly:3;
			unsigned int _15:1;
			unsigned int pll_cpset:2;
			unsigned int pll_r_f:1;
		} b;
	} test; /* 0x04 */

	union {
		unsigned int val;
		struct {
			unsigned int update:1;
			unsigned int _01_07:7;
			unsigned int level:1;
		} b;
	} level; /* 0x08 */

	union {
		unsigned int val;
		struct {
			unsigned int bpp_type:3; /* 0-888,1-555,2-666,3-565 */
			unsigned int _03_07:5;
			unsigned int ldi_shift_left:1; /* 0-shift right,1-left*/
		} b;
	} igs; /* 0x0c */

	union {
		unsigned int val;
		struct {
			unsigned int out_data_12:1; /* 0-24bit,1-12bit */
			unsigned int hsync_polar_lo:1; /* 0-active hi,1-low */
			unsigned int dvo_enable:1;
			unsigned int vsync_polar_lo:1; /* 0-active hi,1-low */
		} b;
	} set; /* 0x10 */

	union {
		unsigned int val;
		struct {
			unsigned int colfmt:2; /* 0-YUV444,1/3-RGB,2-YUV422 */
		} b;
	} set2; /* 0x14 */

	union {
		unsigned int val;
		struct {
			unsigned int pll_ready:1;
			unsigned int _01_07:7;
			unsigned int rsen:1;
		} b;
	} detect; /* 0x18 */

	union {
		unsigned int val;
		struct {
			unsigned int pll_tsync:1;
			unsigned int tp2s_type:1;
			unsigned int div_sel:2;
			unsigned int pd_v2i:1;
			unsigned int vco_sx:1;
			unsigned int vco_mode:1;
			unsigned int _07:1;
			unsigned int vsref_sel:2;
			unsigned int mode:1;
			unsigned int pd_l2ha:1;
			unsigned int pd_l2hb:1;
			unsigned int l2ha_hsen:1;
			unsigned int resa_en:1;
			unsigned int resa_s:1;
			unsigned int pll_lpfs:2;
		} b;
	} test2; /* 0x1c */
};

#define REG_LVDS_BEGIN	(LVDS_BASE_ADDR + 0x00)
#define REG_LVDS_END	(LVDS_BASE_ADDR + 0x1C)
#ifndef LVDS_C
extern struct lvds_base_regs *lvds_regs;
#endif
#endif /* WMT_LVDS_REG_H */
