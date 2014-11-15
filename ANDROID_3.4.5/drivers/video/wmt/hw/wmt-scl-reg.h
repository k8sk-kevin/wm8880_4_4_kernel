/*++
 * linux/drivers/video/wmt/hw/wmt-scl-reg.h
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

#ifndef WMT_SCL_REG_H
#define WMT_SCL_REG_H

/* feature */
#define WMT_FTBLK_SCL

/* constant */
#define WMT_SCL_RCYC_MIN	0	/* 1T */
#define WMT_SCL_H_DIV_MAX	8192
#define WMT_SCL_V_DIV_MAX	8192
#define WMT_SCL_FB_WIDTH_MAX	8192

#define WMT_SCL_SCALE_DST_H_MAX	1920	/* bypass no limit */

struct scl_base1_regs {
	union {
		unsigned int val;
		struct {
			unsigned int alu_enable:1;
		} b;
	} en; /* 0x0 */

	union {
		unsigned int val;
		struct {
			unsigned int reg_update:1;
		} b;
	} upd; /* 0x04 */

	union {
		unsigned int val;
		struct {
			unsigned int reg_level:1;
		} b;
	} sel; /* 0x08 */

	unsigned int _0c_38[12];

	union {
		unsigned int val;
		struct {
			unsigned int hxwidth:13;
		} b;
	} hxwidth; /* 0x3c */

	union {
		unsigned int val;
		struct {
			unsigned int mif_en:1;
			unsigned int _01_03:3;
			unsigned int rgb_mode:2; /* 0-YUV,1-RGB565,3-RGB32 */
			unsigned int _06_07:2;
			unsigned int _420c_fmt:1; /* 0-frame,1-field */
			unsigned int vfmt:3; /* 0-YUV422,1-YUV420,
							2-YUV444,4-RGB32 */
			unsigned int h264_fmt:1; /* 0-MPEG,1-H264 */
			unsigned int _13_15:3;
			unsigned int iofmt:1; /* 0-frame,1-field */
			unsigned int _17_23:7;
			unsigned int color_en:1; /* 0-disable,1-enable */
			unsigned int color_wide:1; /* 0-Normal,1-Wider */
			unsigned int color_inv:1; /* 0-Normal,1-Opposite color*/
		} b;
	} r2_ctl; /* 0x40 */

	unsigned int r2_ysa; /* 0x44 */
	unsigned int r2_csa; /* 0x48 */

	union {
		unsigned int val;
		struct {
			unsigned int fbw:13; /* frame buffer width pixel */
			unsigned int _13_15:3;
			unsigned int lnsize:13; /* line width pixel */
		} b;
	} r2_h_size; /* 0x4c */

	union {
		unsigned int val;
		struct {
			unsigned int hcrop:13;
			unsigned int _13_15:3;
			unsigned int vcrop:13;
		} b;
	} r2_crop; /* 0x50 */

	union {
		unsigned int val;
		struct {
			unsigned int src:2; /* 0-RMIF1,1-RMIF2,2-Fixed ALPHA */
			unsigned int _02_07:6;
			unsigned int dst:2; /* 0-RMIF1,1-RMIF2,2-Fixed ALPHA */
			unsigned int _09_15:6;
			unsigned int swap:1; /* 0-(alpha,1-a),1:(1-a,alpha) */
		} b;
	} alpha_md; /* 0x54 */

	union {
		unsigned int val;
		struct {
			unsigned int src_fixed:8;
			unsigned int dst_fixed:8;
		} b;
	} alpha_fxd; /* 0x58 */

	union {
		unsigned int val;
		struct {
			unsigned int enable:1;
			unsigned int _01_07:7;
			unsigned int from:1; /* 0-RMIF1,1-RMIF2 */
			unsigned int _09_15:7;
			unsigned int comp:2; /* 0-888,1-777,2-666,3-555 */
			unsigned int _17_23:7;
			unsigned int mode:3; /* (Non-Hit,Hit):0/1-(alpha,alpha),
				2-(alpha,pix1),3-(pix1,alpha),4-(alpha,pix2),
				5-(pix2,alpha),6-(pix1,pix2),7-(pix2,pix1) */
		} b;
	} alpha_colorkey; /* 0x5c */

	union {
		unsigned int val;
		struct {
			unsigned int r:8;
			unsigned int g:8;
			unsigned int b:8;
		} b;
	} alpha_colorkey_rgb; /* 0x60 */

	unsigned int _64_6c[3];

	union {
		unsigned int val;
		struct {
			unsigned int vxwidth:13;
			unsigned int _13_15:3;
			unsigned int dst_vxwidth:13;
		} b;
	} vxwidth; /* 0x70 */

	union {
		unsigned int val;
		struct {
			unsigned int h:1;
			unsigned int _01_15:15;
			unsigned int v:1;
		} b;
	} sclup_en; /* 0x74 */

	union {
		unsigned int val;
		struct {
			unsigned int thr:13;
			unsigned int _13_15:3;
			unsigned int substep:13;
		} b;
	} vscale1; /* 0x78 */

	union {
		unsigned int val;
		struct {
			unsigned int substepcnt:13;
			unsigned int _13_15:3;
			unsigned int step:13;
		} b;
	} vscale2; /* 0x7c */

	union {
		unsigned int val;
		struct {
			unsigned int stepcnt:17;
		} b;
	} vscale3; /* 0x80 */

	union {
		unsigned int val;
		struct {
			unsigned int thr:13;
			unsigned int _13_15:3;
			unsigned int substep:13;
		} b;
	} hscale1; /* 0x84 */

	union {
		unsigned int val;
		struct {
			unsigned int substepcnt:13;
			unsigned int _13_15:3;
			unsigned int step:13;
		} b;
	} hscale2; /* 0x88 */

	union {
		unsigned int val;
		struct {
			unsigned int stepcnt:17;
		} b;
	} hscale3; /* 0x8c */

	union {
		unsigned int val;
		struct {
			unsigned int y_req_num:8;
			unsigned int c_req_num:8;
		} b;
	} r_req_num; /* 0x90 */

	unsigned int scldw; /* 0x94 */ /* (VPU path, scale dn)
					0 - bilinear mode, quality better */
	unsigned int sw_426; /* 0x98 */ /* 1-follow 426, 0-437 */
	unsigned int vbypass; /* 0x9c */

	union {
		unsigned int val;
		struct {
			unsigned int enable:1;
			unsigned int _1_3:3;
			unsigned int err_off:1; /*disable TG_EN in tg timeout*/
			unsigned int _5_7:3;
			unsigned int watchdog_enable:1;
			unsigned int _9_15:7;
			unsigned int rdcyc:8;
			unsigned int oneshot:1; /* sacling complete will set
							SCL tg enable to 0 */
		} b;
	} tg_ctl; /* 0xa0 */

	union {
		unsigned int val;
		struct {
			unsigned int h_allpixel:13;
			unsigned int _13_15:3;
			unsigned int v_allline:13;
		} b;
	} tg_total; /* 0xa4 */

	union {
		unsigned int val;
		struct {
			unsigned int v_actbg:8;
			unsigned int _8_15:8;
			unsigned int v_actend:13;
		} b;
	} tg_v_active; /* 0xa8 */

	union {
		unsigned int val;
		struct {
			unsigned int h_actbg:10;
			unsigned int _10_15:6;
			unsigned int h_actend:13;
		} b;
	} tg_h_active; /* 0xac */

	union {
		unsigned int val;
		struct {
			unsigned int vbie:7;
			unsigned int _7:1;
			unsigned int pvbi:5;
		} b;
	} tg_vbi; /* 0xb0 */

	unsigned int tg_watchdog; /* 0xb4 */

	union {
		unsigned int val;
		struct {
			unsigned int tgerr:1;
		} b;
	} tg_sts; /* 0xb8 */

	union {
		unsigned int val;
		struct {
			unsigned int enable:1;
		} b;
	} tg_govw; /* 0xbc */

	union {
		unsigned int val;
		struct {
			unsigned int mif_enable:1; /*0:Disable, 1:Enable */
			unsigned int _1_3:3;
			unsigned int rgb_mode:2; /*0:YC,1:RGB565,3:RGB32 */
			unsigned int _6_7:2;
			unsigned int src_disp_fmt:1; /*420C 0:Frame, 1:Field */
			unsigned int yuv:2; /*0:422,1:420,2:444*/
			unsigned int rgb:1; /*0:YCbCr, 1:RGB32 */
			unsigned int h264:1; /*0:MPEG, 1:H264 */
			unsigned int _13_15:3;
			unsigned int field:1; /*0:Frame, 1:Field */
			unsigned int _17_23:7;
			unsigned int colorbar_enable:1;
			unsigned int colorbar_mode:1;
			unsigned int colorbar_inv:1;
		} b;
	} r_ctl; /* 0xc0 */

	unsigned int r_ysa; /* 0xc4 */
	unsigned int r_csa; /* 0xc8 */

	union {
		unsigned int val;
		struct {
			unsigned int fb_w:13;
			unsigned int _13_15:3;
			unsigned int pix_w:13;
		} b;
	} r_h_size; /* 0xcc */

	union {
		unsigned int val;
		struct {
			unsigned int hcrop:13;
			unsigned int _13_15:3;
			unsigned int vcrop:13;
		} b;
	} r_crop; /* 0xd0 */

	union {
		unsigned int val;
		struct {
			unsigned int thr:4;
			unsigned int _4_7:4;
			unsigned int r1_mif_err:1;
			unsigned int r2_mif_err:1;
		} b;
	} r_fifo_ctl; /* 0xd4 */

	unsigned int _d8_dc[2];

	union {
		unsigned int val;
		struct {
			unsigned int mif_enable:1;
			unsigned int _1_7:7;
			unsigned int yuv:1; /* 0-444,1-422 */
			unsigned int rgb:1; /* 0-YC,1-RGB32 */
		} b;
	} w_ctl; /* 0xe0 */

	unsigned int w_ysa; /* 0xe4 */
	unsigned int w_csa; /* 0xe8 */

	union {
		unsigned int val;
		struct {
			unsigned int fb_w:13;
			unsigned int _13_15:3;
			unsigned int pxl_w:13;
		} b;
	} w_y_time; /* 0xec */

	union {
		unsigned int val;
		struct {
			unsigned int fb_w:13;
			unsigned int _13_15:3;
			unsigned int pxl_w:12;
		} b;
	} w_c_time; /* 0xf0 */

	union {
		unsigned int val;
		struct {
			unsigned int mif_c_err:1;
			unsigned int _1_7:7;
			unsigned int mif_y_err:1;
			unsigned int _9_15:7;
			unsigned int mif_rgb_err:1;
		} b;
	} w_ff_ctl; /* 0xf4 */

	union {
		unsigned int val;
		struct {
			unsigned int mif_c_err:1;
			unsigned int mif_y_err:1;
			unsigned int mif_rgb_err:1;
			unsigned int _3_7:5;
			unsigned int r2_mif_enable:1;
			unsigned int r1_mif_enable:1;
			unsigned int _10_15:6;
			unsigned int tg_err:1;
		} b;
	} w_int_en; /* 0xf8 */

	union {
		unsigned int val;
		struct {
			unsigned int h:1;
			unsigned int _1_7:7;
			unsigned int v:1;
		} b;
	} true_bilinear; /* 0xfc */
};

struct scl_base2_regs {
	union {
		unsigned int val;
		struct {
			unsigned int mode:1; /* 0-RGB2YC,1-YC2RGB */
			unsigned int _01_07:7;
			unsigned int clamp_enable:1; /* clamp to 16-235 */
			unsigned int _09_15:7;
			unsigned int enable:1;
		} b;
	} csc_ctl; /* 0x0 */

	unsigned int csc1; /* 0x4 */
	unsigned int csc2; /* 0x8 */
	unsigned int csc3; /* 0xc */
	unsigned int csc4; /* 0x10 */
	unsigned int csc5; /* 0x14 */
	unsigned int csc6; /* 0x18 */

	union {
		unsigned int val;
		struct {
			unsigned int enable:1;
			unsigned int _01_07:7;
			unsigned int data:8;
		} b;
	} argb_alpha; /* 0x1c */

	union {
		unsigned int val;
		struct {
			unsigned int mode:2; /* 0-888,1-555,2-666,3-565 */
		} b;
	} igs; /* 0x20 */

	union {
		unsigned int val;
		struct {
			unsigned int mode:1; /* 0-CCIR/ITU-601 */
			unsigned int _01_07:7;
			unsigned int clamp:1; /* 0-direct,1-16-235 */
			unsigned int _09_15:7;
			unsigned int enable:1;
		} b;
	} r2_csc; /* 0x24 */

	unsigned int r2_csc1; /* 0x28 */
	unsigned int r2_csc2; /* 0x2c */
	unsigned int r2_csc3; /* 0x30 */
	unsigned int r2_csc4; /* 0x34 */
	unsigned int r2_csc5; /* 0x38 */
	unsigned int r2_csc6; /* 0x3c */
	unsigned int _40_9c[24];

	union {
		unsigned int val;
		struct {
			unsigned int h:1;
			unsigned int _01_07:7;
			unsigned int v:1;
		} b;
	} recursive_mode; /* 0xa0 */

	unsigned int _a4_bc[7];

	union {
		unsigned int val;
		struct {
			unsigned int deblock:1;
			unsigned int field_deflicker:1;
			unsigned int frame_deflicker:1;
		} b;
	} field_mode; /* 0xc0 */

	union {
		unsigned int val;
		struct {
			unsigned int layer1_boundary:8;
			unsigned int layer2_boundary:8;
		} b;
	} dblk_threshold; /* 0xc4 */

	union {
		unsigned int val;
		struct {
			unsigned int condition:1; /* 0-up or down,1-up & down */
			unsigned int _01_07:7;
			unsigned int y_thd:8;
			unsigned int c_thd:8;
		} b;
	} field_flicker; /* 0xc8 */
	union {
		unsigned int val;
		struct {
			unsigned int rgb:1; /* 0-Y,1-RGB */
			unsigned int _01_07:7;
			unsigned int sampler:5; /* 2^x */
			unsigned int _13_15:3;
			unsigned int scene_chg_thd:8;
		} b;
	} frame_flicker; /* 0xcc */

	union {
		unsigned int val;
		struct {
			unsigned int rdcyc_1t:1;
		} b;
	} readcyc_1t; /* 0xd0 */

	unsigned int _d4_e0[4];
};

#define REG_SCL_BASE1_BEGIN	(SCL_BASE_ADDR + 0x00)
#define REG_SCL_BASE1_END	(SCL_BASE_ADDR + 0xFC)
#define REG_SCL_BASE2_BEGIN	(SCL_BASE2_ADDR + 0x00)
#define REG_SCL_BASE2_END	(SCL_BASE2_ADDR + 0xE0)

#ifndef SCL_C
extern HW_REG struct scl_base1_regs *scl_regs1;
extern HW_REG struct scl_base2_regs *scl_regs2;
#endif
#endif /* WMT_SCL_REG_H */
