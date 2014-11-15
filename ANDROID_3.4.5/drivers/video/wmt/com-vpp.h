/*++
 * linux/drivers/video/wmt/com-vpp.h
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

#ifndef COM_VPP_H
#define COM_VPP_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/fb.h>
#include <asm/io.h>
#include <mach/common_def.h>
#include <mach/com-video.h>
#else
#include "com-video.h"
#ifdef CFG_LOADER 
#include "com-fb.h"
#endif
#endif

#define VPP_NEW_FBUF_MANAGER

#define VPP_AHB_CLK			250000000

#ifdef CONFIG_MAX_RESX
	#define VPP_HD_MAX_RESX		CONFIG_MAX_RESX
#else
	#define VPP_HD_MAX_RESX		1920
#endif

#ifdef CONFIG_MAX_RESY
	#define VPP_HD_MAX_RESY		CONFIG_MAX_RESY
#else
	#define VPP_HD_MAX_RESY		1200
#endif

#ifdef CONFIG_DEFAULT_RESX
	#define VPP_HD_DISP_RESX	CONFIG_DEFAULT_RESX
#else
	#define VPP_HD_DISP_RESX	1024
#endif

#ifdef CONFIG_DEFAULT_RESY
	#define VPP_HD_DISP_RESY	CONFIG_DEFAULT_RESY
#else
	#define VPP_HD_DISP_RESY	768
#endif

#ifdef CONFIG_DEFAULT_FPS
	#define VPP_HD_DISP_FPS		CONFIG_DEFAULT_FPS
#else
	#define VPP_HD_DISP_FPS		60
#endif

#define VPP_HD_DISP_PIXCLK	65000000

#define BIT0	0x00000001
#define BIT1	0x00000002
#define BIT2	0x00000004
#define BIT3	0x00000008
#define BIT4	0x00000010
#define BIT5	0x00000020
#define BIT6	0x00000040
#define BIT7	0x00000080
#define BIT8	0x00000100
#define BIT9	0x00000200
#define BIT10	0x00000400
#define BIT11	0x00000800
#define BIT12	0x00001000
#define BIT13	0x00002000
#define BIT14	0x00004000
#define BIT15	0x00008000
#define BIT16	0x00010000
#define BIT17	0x00020000
#define BIT18	0x00040000
#define BIT19	0x00080000
#define BIT20	0x00100000
#define BIT21	0x00200000
#define BIT22	0x00400000
#define BIT23	0x00800000
#define BIT24	0x01000000
#define BIT25	0x02000000
#define BIT26	0x04000000
#define BIT27	0x08000000
#define BIT28	0x10000000
#define BIT29	0x20000000
#define BIT30	0x40000000
#define BIT31	0x80000000

#define VPP_YUV_BLACK		0x008080	/* Y, Cr, Cb */
#define VPP_YUV_WHITE		0xff8080
#define VPP_YUV_RED		0x51f05a
#define VPP_YUV_GREEN		0x902235
#define VPP_YUV_BLUE		0x286df0
#define VPP_RGB32_BLACK		0x00000000

#define VPP_COL_RGB32_BLACK	0x000000
#define VPP_COL_RGB32_WHITE	0xFFFFFF
#define VPP_COL_RGB32_RED	0xFF0000
#define VPP_COL_RGB32_GREEN	0x00FF00
#define VPP_COL_RGB32_BLUE	0x0000FF

#define VPP_COL_BLACK		0x008080	/* Y, Cr, Cb */
#define VPP_COL_WHITE		0xff8080
#define VPP_COL_RED		0x41d464
#define VPP_COL_GREEN		0x902235
#define VPP_COL_BLUE		0x2372d4

#define VPP_MAGNUM(s, e)	((2^((s)-(e)+1))-1)

#define WMT_FB_COLFMT(a)	(a & 0xFF)

enum vpp_fbuf_s {
	VPP_FBUF_GOVR_1,
	VPP_FBUF_GOVR_2,
	VPP_FBUF_SCLR_1,
	VPP_FBUF_SCLR_2,
	VPP_FBUF_MAX
};
#define vpp_fbuf_t enum vpp_fbuf_s

enum vpp_flag_s {
	VPP_FLAG_NULL = 0,
	VPP_FLAG_ENABLE = 1,
	VPP_FLAG_DISABLE = 0,
	VPP_FLAG_TRUE = 1,
	VPP_FLAG_FALSE = 0,
	VPP_FLAG_ZERO = 0,
	VPP_FLAG_ONE = 1,
	VPP_FLAG_SUCCESS = 1,
	VPP_FLAG_ERROR = 0,
	VPP_FLAG_RD = 1,
	VPP_FLAG_WR = 0,
};
#define vpp_flag_t enum vpp_flag_s

enum vpp_mod_s {
	VPP_MOD_GOVRH2,
	VPP_MOD_GOVRH,
	VPP_MOD_DISP,
	VPP_MOD_GOVW,
	VPP_MOD_GOVM,
	VPP_MOD_SCL,
	VPP_MOD_SCLW,
	VPP_MOD_VPU,
	VPP_MOD_VPUW,
	VPP_MOD_PIP,
	VPP_MOD_VPPM,
	VPP_MOD_LCDC,
	VPP_MOD_CURSOR,
	VPP_MOD_MAX
};
#define vpp_mod_t enum vpp_mod_s

enum vpp_vout_s {
	VPP_VOUT_NONE = 0,
	VPP_VOUT_SDA = 1,
	VPP_VOUT_LCD = 2,
	VPP_VOUT_DVI = 3,
	VPP_VOUT_HDMI = 4,
	VPP_VOUT_DVO2HDMI = 5,
	VPP_VOUT_LVDS = 6,
	VPP_VOUT_VGA = 7,
	VPP_VOUT_FBDEV = 8,
	VPP_VOUT_VIRDISP = 9,
	VPP_VOUT_MAX
};
#define vpp_vout_t enum vpp_vout_s

enum vpp_display_format_s {
	VPP_DISP_FMT_FRAME,	/* Progressive */
	VPP_DISP_FMT_FIELD,	/* Interlace */
	VPP_DISP_FMT_MAX,
};
#define vpp_display_format_t enum vpp_display_format_s

enum vpp_media_format_s {
	VPP_MEDIA_FMT_MPEG,
	VPP_MEDIA_FMT_H264,
	VPP_MEDIA_FMT_MAX,
};
#define vpp_media_format_t enum vpp_media_format_s

enum vpp_csc_s { /* don't change this order */
	VPP_CSC_YUV2RGB2_MIN,
	VPP_CSC_YUV2RGB_SDTV_0_255 = VPP_CSC_YUV2RGB2_MIN,
	VPP_CSC_YUV2RGB_SDTV_16_235,
	VPP_CSC_YUV2RGB_HDTV_0_255,
	VPP_CSC_YUV2RGB_HDTV_16_235,
	VPP_CSC_YUV2RGB_JFIF_0_255,
	VPP_CSC_YUV2RGB_SMPTE170M,
	VPP_CSC_YUV2RGB_SMPTE240M,
	VPP_CSC_RGB2YUV_MIN,
	VPP_CSC_RGB2YUV_SDTV_0_255 = VPP_CSC_RGB2YUV_MIN,
	VPP_CSC_RGB2YUV_SDTV_16_235,
	VPP_CSC_RGB2YUV_HDTV_0_255,
	VPP_CSC_RGB2YUV_HDTV_16_235,
	VPP_CSC_RGB2YUV_JFIF_0_255,
	VPP_CSC_RGB2YUV_SMPTE170M,
	VPP_CSC_RGB2YUV_SMPTE240M,
	VPP_CSC_RGB2YUV_JFIF_VT1625,
	VPP_CSC_MAX,
	VPP_CSC_BYPASS
};
#define vpp_csc_t enum vpp_csc_s

enum vpp_reglevel_s {
	VPP_REG_LEVEL_1,
	VPP_REG_LEVEL_2,
	VPP_REG_LEVEL_MAX,
};
#define vpp_reglevel_t enum vpp_reglevel_s

enum vpp_datawidht_s {
	VPP_DATAWIDHT_12,
	VPP_DATAWIDHT_24,
	VPP_DATAWIDHT_MAX,
};
#define vpp_datawidht_t enum vpp_datawidht_s

enum vpp_tvsys_s {
	VPP_TVSYS_NTSC,
	VPP_TVSYS_NTSCJ,
	VPP_TVSYS_NTSC443,
	VPP_TVSYS_PAL,
	VPP_TVSYS_PALM,
	VPP_TVSYS_PAL60,
	VPP_TVSYS_PALN,
	VPP_TVSYS_PALNC,
	VPP_TVSYS_720P,
	VPP_TVSYS_1080I,
	VPP_TVSYS_1080P,
	VPP_TVSYS_MAX
};
#define vpp_tvsys_t enum vpp_tvsys_s

enum vpp_tvconn_s {
	VPP_TVCONN_YCBCR,
	VPP_TVCONN_SCART,
	VPP_TVCONN_YPBPR,
	VPP_TVCONN_VGA,
	VPP_TVCONN_SVIDEO,
	VPP_TVCONN_CVBS,
	VPP_TVCONN_MAX
};
#define vpp_tvconn_t enum vpp_tvconn_s

#define VPP_OPT_INTERLACE		0x01
#define VPP_DVO_SYNC_POLAR_HI		0x08
#define VPP_DVO_VSYNC_POLAR_HI		0x10

struct vpp_clock_s {
	int read_cycle;

	unsigned int total_pixel_of_line;
	unsigned int begin_pixel_of_active;
	unsigned int end_pixel_of_active;

	unsigned int total_line_of_frame;
	unsigned int begin_line_of_active;
	unsigned int end_line_of_active;

	unsigned int hsync;
	unsigned int vsync;
	unsigned int line_number_between_VBIS_VBIE;
	unsigned int line_number_between_PVBI_VBIS;
};
#define vpp_clock_t struct vpp_clock_s

struct vpp_scale_s {
	vdo_framebuf_t src_fb;
	vdo_framebuf_t dst_fb;
};
#define vpp_scale_t struct vpp_scale_s

struct vpp_scale_overlap_s {
	int mode;
	vdo_framebuf_t src_fb;
	vdo_framebuf_t src2_fb;
	vdo_framebuf_t dst_fb;
};
#define vpp_scale_overlap_t struct vpp_scale_overlap_s

struct vpp_overlap_s {
	unsigned int alpha_src_type:2; /* 0-mif1,1-mif2,2-fix */
	unsigned int alpha_src:8;
	unsigned int alpha_dst_type:2; /* 0-mif1,1-mif2,2-fix */
	unsigned int alpha_dst:8;
	unsigned int alpha_swap:1; /* 0-(alpha,1-alpha),1:(1-alpha,alpha) */
	unsigned int color_key_from:2; /* 0-RMIF1,1-RMIF2 */
	unsigned int color_key_comp:2; /* 0-888,1-777,2-666,3-555 */
	unsigned int color_key_mode:3; /* (Non-Hit,Hit):0/1-(alpha,alpha),
				2-(alpha,pix1), 3-(pix1,alpha),4-(alpha,pix2),
				5-(pix2,alpha),6-(pix1,pix2),7-(pix2,pix1) */
	unsigned int reserved:4;
	unsigned int color_key; /* ARGB */
};
#define vpp_overlap_t struct vpp_overlap_s

#define VPP_VOUT_STS_REGISTER	0x01
#define VPP_VOUT_STS_ACTIVE	0x02
#define VPP_VOUT_STS_PLUGIN	0x04
#define VPP_VOUT_STS_EDID	0x08
#define VPP_VOUT_STS_BLANK	0x10
#define VPP_VOUT_STS_POWERDN	0x20
#define VPP_VOUT_STS_CONTENT_PROTECT 0x40
#define VPP_VOUT_STS_FB		0xF00
#define VPP_VOUT_ARG_NUM 5
struct vpp_vout_info_s {
	enum vpp_vout_s mode[VPP_VOUT_ARG_NUM]; 
	unsigned int status[VPP_VOUT_ARG_NUM];
};
#define vpp_vout_info_t struct vpp_vout_info_s

struct vpp_vout_parm_s {
	int num;
	int arg;
};
#define vpp_vout_parm_t struct vpp_vout_parm_s

#define VOUT_OPT_BLANK		BIT(8)

#define VPP_CAP_DUAL_DISPLAY	BIT(0)
struct vpp_cap_s {
	unsigned int chip_id;
	unsigned int version;
	unsigned int resx_max;
	unsigned int resy_max;
	unsigned int pixel_clk;
	unsigned int module;
	unsigned int option;
};
#define vpp_cap_t struct vpp_cap_s

struct vpp_i2c_s {
	unsigned int addr;
	unsigned int index;
	unsigned int val;
};
#define vpp_i2c_t struct vpp_i2c_s

struct vpp_mod_fbinfo_s {
	vpp_mod_t mod;
	int read;
	vdo_framebuf_t fb;
};
#define vpp_mod_fbinfo_t struct vpp_mod_fbinfo_s

struct vpp_vmode_parm_s {
	unsigned int resx;
	unsigned int resy;
	unsigned int fps;
	unsigned int option;
};
#define vpp_vmode_parm_t struct vpp_vmode_parm_s

#define VPP_VOUT_VMODE_NUM	20
struct vpp_vout_vmode_s {
	vpp_vout_t mode;
	int num;
	vpp_vmode_parm_t parm[VPP_VOUT_VMODE_NUM];
};
#define vpp_vout_vmode_t struct vpp_vout_vmode_s

struct vpp_vout_edid_s {
	vpp_vout_t mode;
	int size;
	char *buf;
};
#define vpp_vout_edid_t struct vpp_vout_edid_s

struct vpp_vout_cp_info_s {
	int num;
	unsigned int bksv[2];
};
#define vpp_vout_cp_info_t struct vpp_vout_cp_info_s

#define VPP_VOUT_CP_NUM		336
struct vpp_vout_cp_key_s {
	char key[VPP_VOUT_CP_NUM];
};
#define vpp_vout_cp_key_t struct vpp_vout_cp_key_s

struct vpp_mod_parm_t {
	vpp_mod_t mod;
	int read;
	unsigned int parm;
};

#define VPPIO_MAGIC		'f'

/* VPP common ioctl command */
#define VPPIO_VPP_BASE		0x0
#define VPPIO_VPPGET_INFO _IOR(VPPIO_MAGIC, VPPIO_VPP_BASE + 0, vpp_cap_t)
#define VPPIO_VPPSET_INFO _IOW(VPPIO_MAGIC, VPPIO_VPP_BASE + 0, vpp_cap_t)
#define VPPIO_I2CSET_BYTE _IOW(VPPIO_MAGIC, VPPIO_VPP_BASE + 1, vpp_i2c_t)
#define VPPIO_I2CGET_BYTE _IOR(VPPIO_MAGIC, VPPIO_VPP_BASE + 1, vpp_i2c_t)
#define VPPIO_MODSET_CSC \
	_IOW(VPPIO_MAGIC, VPPIO_VPP_BASE + 2, struct vpp_mod_parm_t)
#define VPPIO_MODULE_FBINFO \
	_IOWR(VPPIO_MAGIC, VPPIO_VPP_BASE + 6, vpp_mod_fbinfo_t)
#define VPPIO_STREAM_ENABLE	_IO(VPPIO_MAGIC, VPPIO_VPP_BASE + 12)
#define VPPIO_STREAM_GETFB \
	_IOR(VPPIO_MAGIC, VPPIO_VPP_BASE + 13, vdo_framebuf_t)
#define VPPIO_STREAM_PUTFB \
	_IOW(VPPIO_MAGIC, VPPIO_VPP_BASE + 13, vdo_framebuf_t)
#define VPPIO_MULTIVD_ENABLE _IO(VPPIO_MAGIC, VPPIO_VPP_BASE + 15)

/* VOUT ioctl command */
#define VPPIO_VOUT_BASE		0x10
#define VPPIO_VOGET_INFO _IOR(VPPIO_MAGIC, VPPIO_VOUT_BASE + 0, vpp_vout_info_t)
#define VPPIO_VOGET_HDMI_3D \
	_IOR(VPPIO_MAGIC, VPPIO_VOUT_BASE + 1, struct vpp_vout_parm_s)
#define VPPIO_VOUT_VMODE \
	_IOWR(VPPIO_MAGIC, VPPIO_VOUT_BASE + 7, vpp_vout_vmode_t)
#define VPPIO_VOGET_EDID _IOR(VPPIO_MAGIC, VPPIO_VOUT_BASE + 8, vpp_vout_edid_t)
#define VPPIO_VOGET_CP_INFO \
	_IOR(VPPIO_MAGIC, VPPIO_VOUT_BASE + 9, vpp_vout_cp_info_t)
#define VPPIO_VOSET_CP_KEY \
	_IOW(VPPIO_MAGIC, VPPIO_VOUT_BASE + 10, vpp_vout_cp_key_t)
#define VPPIO_VOSET_AUDIO_PASSTHRU	_IO(VPPIO_MAGIC, VPPIO_VOUT_BASE + 11)
#define VPPIO_VOSET_VIRTUAL_FBDEV	_IO(VPPIO_MAGIC, VPPIO_VOUT_BASE + 13)

/* SCL ioctl command */
#define VPPIO_SCL_BASE		0x60
#define VPPIO_SCL_SCALE	_IOWR(VPPIO_MAGIC, VPPIO_SCL_BASE + 0, vpp_scale_t)
#define VPPIO_SCL_SCALE_ASYNC \
	_IOWR(VPPIO_MAGIC, VPPIO_SCL_BASE + 2, vpp_scale_t)
#define VPPIO_SCL_SCALE_FINISH	_IO(VPPIO_MAGIC, VPPIO_SCL_BASE + 3)
#define VPPIO_SCLSET_OVERLAP \
	_IOW(VPPIO_MAGIC, VPPIO_SCL_BASE + 4, vpp_overlap_t)
#define VPPIO_SCL_SCALE_OVERLAP	\
	_IOWR(VPPIO_MAGIC, VPPIO_SCL_BASE + 5, vpp_scale_overlap_t)

#define VPPIO_MAX		0x70
#endif /* COM_VPP_H */
