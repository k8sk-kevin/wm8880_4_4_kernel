/*++
 * linux/drivers/video/wmt/vout.h
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

#ifndef VOUT_H
/* To assert that only one occurrence is included */
#define VOUT_H
/*-------------------- MODULE DEPENDENCY -------------------------------------*/
#include "vpp.h"
#include "sw_i2c.h"
#include "edid.h"

/*	following is the C++ header	*/
#ifdef	__cplusplus
extern	"C" {
#endif

/*-------------------- EXPORTED PRIVATE CONSTANTS ----------------------------*/
/* #define  VO_XXXX  1    *//*Example*/
/* #define CONFIG_VOUT_EDID_ALLOC */
#define CONFIG_VOUT_REFACTORY

#define VOUT_INFO_DEFAULT_RESX	1024
#define VOUT_INFO_DEFAULT_RESY	768
#define VOUT_INFO_DEFAULT_FPS	60

/*-------------------- EXPORTED PRIVATE TYPES---------------------------------*/
/* typedef  void  vo_xxx_t;  *//*Example*/

enum vout_mode_t {
	VOUT_SD_ANALOG,
	VOUT_SD_DIGITAL,
	VOUT_LCD,
	VOUT_DVI,
	VOUT_HDMI,
	VOUT_DVO2HDMI,
	VOUT_LVDS,
	VOUT_VGA,
	VOUT_BOOT,
	VOUT_MODE_MAX,
	VOUT_MODE_ALL = VOUT_MODE_MAX
};

enum vout_dev_mode_t {
	VOUT_DEV_VGA,
	VOUT_DEV_DVI,
	VOUT_DEV_LCD,
	VOUT_DEV_HDMI,
	VOUT_DEV_SDD,
	VOUT_DEV_LVDS,
	VOUT_DEV_MODE_MAX
};

enum vout_inf_mode_t {
	VOUT_INF_DVI,
	VOUT_INF_HDMI,
	VOUT_INF_LVDS,
	VOUT_INF_MODE_MAX
};

enum vout_tvformat_t {
	TV_PAL,
	TV_NTSC,
	TV_UNDEFINED,
	TV_MAX
};

/* wmt.display.fb0 - op:[type:op1:op2:resx:resy:fps]:[...]:[...]
<op>:[<type>:<op1>:<op2>:<resx>:<resy>:<fps>]:[...]...
[H] <op>:=
<op>.Bit[3:0] : fb type (0-GE,1-alloc & fix,2-dynamic alloc,3-GE overscan)
<op>.Bit[7:4] : hwc type (0-normal,1-scale,2-overscan,3-virtual)
<op>.Bit[8] : multi display
<op>.Bit[9] : color format valid flag
<op>.Bit[23:16] : color format

[H] <type>:=
1: SDA
[H]<op1>:= mode (0-YCbCr,1-VGA,2-YPbPr,4-SVideo,5-CVBS)
[H]<op2>
2: LCD
[H]<op1>:= lcd id (0-auto or OEM,1-Chilin,2-Innolux,3-AUO,4-Eking,5-Hannstar)
[H]<op2>:=
<op2>.Bit[7:0] : bit per pixel
<op2>.Bit[10:8] : rgb mode (0-888,1-555,2-666,3-565)
<op2>.Bit[11] : msb(0-lsb,1-msb)
<op2>.Bit[13:12] : swap (0-RGB [7-0], 1-RGB [0-7], 2-BGR [7-0], 3-BGR [0-7])
3: DVI
[H]<op1> :=
<op1>.Bit[7:0] : color format(6-ARGB)
<op1>.Bit[11:8] : dvi i2c bus id
<op1>.Bit[15:12] : dvi interrupt gpio no
[H]<op2> :=
<op2>.Bit[0] : (0-12bit,1-24bit)
<op2>.Bit[1] : (1:interlace)
<op2>.Bit[2] : disable external device
<op2>.Bit[3] : disable external device interrupt
<op2>.Bit[4] : dvi i2c bus id valid
<op2>.Bit[5] : dvi interrupt gpio no valid
<op2>.Bit[10:8] : rgb mode (0-888,1-555,2-666,3-565)
<op2>.Bit[11] : (0-lsb,1-msb)
<op2>.Bit[13:12] : swap (0-RGB [7-0], 1-RGB [0-7], 2-BGR [7-0], 3-BGR [0-7])
4: HDMI
[H]<op1> (1-422,3-444,6-ARGB)
[H]<op2> :=
<op2>.Bit[0] : (0-12bit,1-24bit)
<op2>.Bit[1] : (1:interlace)
<op2>.Bit[2] : (1:CEC)
<op2>.Bit[3] : (1:disable)
<op2>.Bit[4] : (1:sp mode)
6:LVDS
same as LCD
8: virtual frame buffer
[H]<op1> (0-disable,1-enable)

[D] <resx> := horizontal resolution
[D] <resy> := vertical resolution
[D] <fps> := frame per second
*/

#define WMT_DISP_FB_MULTI BIT(8)
#define WMT_DISP_FB_COLFMT BIT(9)
#define WMT_DISP_FB_COLFMT_MASK	0xFF0000

#define WMT_DISP_FB_GET_BPP(a) (a & 0xFF)
#define WMT_DISP_FB_GET_RGB_MODE(a) ((a & 0x700) >> 8)
#define WMT_DISP_FB_MSB	BIT(11)
#define WMT_DISP_FB_RGB_SWAP(a) ((a & 0x3000) >> 12)

#define WMT_DISP_FB_DVI_24BIT BIT(0)
#define WMT_DISP_FB_INTERLACE BIT(1)
#define WMT_DISP_FB_DISABLE_EXTDEV BIT(2)
#define WMT_DISP_FB_DISBALE_DVI_INT BIT(3)
#define WMT_DISP_FB_DVI_I2C BIT(4)
#define WMT_DISP_FB_DVI_INT BIT(5)

#define WMT_DISP_FB_HDMI_CEC BIT(2)
#define WMT_DISP_FB_HDMI_DISABLE BIT(3)
#define WMT_DISP_FB_HDMI_SP_MODE BIT(4)

enum vout_alloc_mode_t {
	VOUT_ALLOC_GE,
	VOUT_ALLOC_FIX_MB,
	VOUT_ALLOC_DYNAMIC_MB,
	VOUT_ALLOC_GE_OVERSCAN,
	VOUT_ALLOC_MODE_MAX
};

enum vout_hwc_mode_t {
	VOUT_HWC_NORMAL,
	VOUT_HWC_SCALE,
	VOUT_HWC_OVERSCAN,
	VOUT_HWC_VIRTUAL,
	VOUT_HWC_MODE_MAX
};

struct vout_info_t {
	int num;
	struct vout_t *vout[VPP_VOUT_NUM + 1];
	int multi; /* multi display in same time */

	/* frame buffer alloc */
	enum vout_alloc_mode_t alloc_mode;
	enum vout_hwc_mode_t hwc_mode;
	void *fb_info_p; /* fb info pointer */
#ifdef CONFIG_KERNEL
	struct semaphore sem;
#endif

	int resx;
	int resy;
	int resx_virtual;
	int resy_virtual;
	int bpp;
	int fps;
	unsigned int pixclk;
	unsigned int option;

	struct fb_videomode *fixed_vmode;
	int fixed_width;
	int fixed_height;
	vdo_framebuf_t fb;
#ifdef CONFIG_UBOOT
	struct fb_videomode *p_vmode;
#endif
	struct vpp_dbg_timer_t pandisp_timer;
	unsigned int mb;
	unsigned int mb_size;
};

struct vout_audio_t {
	int fmt; /* sample bits */
	int sample_rate; /* sample rate */
	int channel; /* channel count */
};

struct vout_t;
struct vout_inf_t;

#define VOUT_DEV_CAP_FIX_RES		0x1
#define VOUT_DEV_CAP_EDID		0x2
#define VOUT_DEV_CAP_AUDIO		0x4
#define VOUT_DEV_CAP_FIX_PLUG		0x8

struct vout_dev_t {
	struct vout_dev_t *next;
	char name[10];
	enum vout_inf_mode_t mode;
	struct vout_t *vout;
	unsigned int capability;

	int (*init)(struct vout_t *vo);
	void (*set_power_down)(int enable);
	int (*set_mode)(unsigned int *option);
	int (*config)(struct vout_info_t *info);
	int (*check_plugin)(int hotplug);
	int (*get_edid)(char *buf);
	int (*set_audio)(struct vout_audio_t *arg);
	int (*interrupt)(void);
	void (*poll)(void);
	int (*suspend)(void);
	int (*resume)(void);
};

enum vout_blank_t {
	VOUT_BLANK_UNBLANK, /* screen: unblanked, hsync: on,  vsync: on */
	VOUT_BLANK_NORMAL,  /* screen: blanked,   hsync: on,  vsync: on */
	VOUT_BLANK_VSYNC_SUSPEND,/* screen: blanked,   hsync: on,  vsync: off */
	VOUT_BLANK_HSYNC_SUSPEND,/* screen: blanked,   hsync: off, vsync: on */
	VOUT_BLANK_POWERDOWN /* screen: blanked,   hsync: off, vsync: off */
};

#define VOUT_CAP_INTERFACE	0x000000FF
#define VOUT_CAP_BUS		0x00000F00
#define VOUT_CAP_GOVR		0x0000F000
#define VOUT_CAP_EXT_DEV	0x00010000
#define VOUT_CAP_FIX_PLUG	0x00020000
#define VOUT_CAP_AUDIO		0x00040000
#define VOUT_CAP_EDID		0x00080000

struct vout_t {
	int num;
	unsigned int fix_cap;
	struct vout_info_t *info;	
	struct vout_inf_t *inf;	/* interface ops */
	struct vout_dev_t *dev; /* device ops */
	struct govrh_mod_t *govr;
	int resx;
	int resy;
	int fps;
	int pixclk;
	unsigned int status;
#ifdef CONFIG_VOUT_EDID_ALLOC
	char *edid;
#else
	char edid[128*EDID_BLOCK_MAX];
#endif
	struct edid_info_t edid_info;
	unsigned int option[3];
	enum vout_blank_t pre_blank;
	int disable;
};

#define VOUT_INF_CAP_FIX_PLUG	BIT(0)
struct vout_inf_t {
	enum vout_inf_mode_t mode;
	unsigned int capability;

	/* function */
	int (*init)(struct vout_t *vo, int arg);
	int (*uninit)(struct vout_t *vo, int arg);
	int (*blank)(struct vout_t *vo, enum vout_blank_t arg);
	int (*config)(struct vout_t *vo, int arg);
	int (*chkplug)(struct vout_t *vo, int arg);
	int (*get_edid)(struct vout_t *vo, int arg);
/*	int (*ioctl)(struct vout_t *vo,int arg); */
};

/*-------------------- EXPORTED PRIVATE VARIABLES ---------------------------*/
#ifdef VOUT_C /* allocate memory for variables only in vout.c */
#define EXTERN

const char *vout_inf_str[] = {"DVI", "HDMI", "LVDS", "VGA", "SDA", "SDD"};
const char *vout_adpt_str[] = {"SD_DIGITAL", "SD_DIGITAL", "LCD", "DVI",
	"HDMI", "DVO2HDMI", "LVDS", "VGA", "BOOT"};

#else
#define	EXTERN	extern

extern const char *vout_inf_str[];
extern const char *vout_adpt_str[];

#endif /* ifdef VOUT_C */

EXTERN struct vout_info_t *vout_info[VPP_VOUT_INFO_NUM];

/* EXTERN int      vo_xxx; *//*Example*/
EXTERN int (*vout_board_info)(int arg);

#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -----------------------------*/
/* #define VO_XXX_YYY   xxxx *//*Example*/
/*--------------------- EXPORTED PRIVATE FUNCTIONS  -------------------------*/
/* extern void  vo_xxx(void); *//*Example*/

void vout_register(int no, struct vout_t *vo);
struct vout_t *vout_get_entry(int no);
struct vout_info_t *vout_get_info_entry(int no);
void vout_change_status(struct vout_t *vo, int mask, int sts);
int vout_query_inf_support(int no, enum vout_inf_mode_t mode);

int vout_inf_register(enum vout_inf_mode_t mode, struct vout_inf_t *inf);
struct vout_inf_t *vout_inf_get_entry(enum vout_inf_mode_t mode);

int vout_device_register(struct vout_dev_t *ops);
struct vout_dev_t *vout_get_device(struct vout_dev_t *ops);

struct vout_t *vout_get_entry_adapter(enum vout_mode_t mode);
struct vout_inf_t *vout_get_inf_entry_adapter(enum vout_mode_t mode);
int vout_info_add_entry(int no, struct vout_t *vo);
struct vout_info_t *vout_info_get_entry(int no);
void vout_info_set_fixed_timing(int no, struct fb_videomode *vmode);
struct govrh_mod_t *vout_info_get_govr(int no);
enum vpp_vout_s vout_get_mode_adapter(struct vout_t *vout);

int vout_config(struct vout_info_t *info, struct fb_videomode *vmode,
	vdo_framebuf_t *fb);
int vout_set_mode(int no, enum vout_inf_mode_t mode);
int vout_set_blank(int no, enum vout_blank_t blank);
void vout_set_framebuffer(struct vout_info_t *info, vdo_framebuf_t *fb);
int vout_chkplug(int no);
void vout_set_int_type(int type);
char *vout_get_edid(int no);
int vout_get_edid_option(int no);
int vout_check_plugin(int clr_sts);
void vout_print_entry(struct vout_t *vo);

int vout_init(void);
int vout_exit(void);
int vo_i2c_proc(int id, unsigned int addr, unsigned int index,
	char *pdata, int len);
int vout_set_audio(struct vout_audio_t *arg);
int vout_find_edid_support_mode(struct edid_info_t *info,
	unsigned int *resx, unsigned int *resy, unsigned int *fps, int r_16_9);
int vout_check_ratio_16_9(unsigned int resx, unsigned int resy);
unsigned int vout_get_mask(struct vout_info_t *vo_info);
void vout_set_int_enable(int enable);
int vout_get_clr_int(void);
void vo_hdmi_set_clock(int enable);
enum vout_tvformat_t vout_get_tvformat(void);
#ifndef CONFIG_VPOST
int vout_find_match_mode(int fbnum,
		struct fb_videomode *vmode, int match);
#endif
#define VOUT_MODE_OPTION_LESS		BIT0
#define VOUT_MODE_OPTION_GREATER	BIT1
#define VOUT_MODE_OPTION_EDID		BIT2
#define VOUT_MODE_OPTION_INTERLACE	BIT3
#define VOUT_MODE_OPTION_PROGRESS	BIT4
struct fb_videomode *vout_get_video_mode(int vout_num,
			struct fb_videomode *vmode, int option);
int vout_get_width_height(int fbnum, int *width, int *height);
void vo_hdmi_cp_set_enable_tmr(int sec);
struct vout_dev_t *lcd_get_dev(void);
#ifdef	__cplusplus
}
#endif

#endif /* ifndef VOUT_H */

/*=== END vout.h ==========================================================*/
