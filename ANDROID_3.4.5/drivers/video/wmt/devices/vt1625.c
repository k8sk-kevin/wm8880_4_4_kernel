/*++
 * linux/drivers/video/wmt/vt1625.c
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

#define VT1625_C
/* #define DEBUG */
/* #define DEBUG_DETAIL */
/*----------------------- DEPENDENCE -----------------------------------------*/
#include "../vout.h"

#ifdef CONFIG_KERNEL
#include <linux/workqueue.h>
#endif

/*----------------------- PRIVATE MACRO --------------------------------------*/
/* #define  VT1625_XXXX  xxxx    *//*Example*/
#ifdef CONFIG_KERNEL
#define CONFIG_VT1625_INTERRUPT
#endif
#define CONFIG_VT1625_POWER
#define CONFIG_VM700

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define VT1625_XXXX    1     *//*Example*/
#define VT1625_ADDR 0x40

enum {
	VT1625_INPUT_SELECTION = 0x00,
	VT1625_SYNC_SELECTION_0 = 0x01,
	VT1625_SYNC_SELECTION_1 = 0x02,
	VT1625_FILTER_SELECTION = 0x03,
	VT1625_OUTPUT_MODE = 0x04,
	VT1625_CLOCK_CONTROL = 0x05,

	/* start position & overflow */
	VT1625_OVERFLOW = 0x06,
	VT1625_START_ACTIVE_VIDEO = 0x07,
	VT1625_START_HORIZONTAL_POSITION = 0x08,
	VT1625_START_VERTICAL_POSITION = 0x09,

	/* amplitude factor */
	VT1625_CR_AMPLITUDE_FACTOR = 0x0A,
	VT1625_BLACK_LEVEL = 0x0B,
	VT1625_Y_AMPLITUDE_FACTOR = 0x0C,
	VT1625_CB_AMPLITUDE_FACTOR = 0x0D,

	VT1625_POWER_MANAGEMENT = 0x0E,
	VT1625_STATUS = 0x0F,

	/* Hue */
	VT1625_HUE_ADJUSTMENT = 0x10,
	VT1625_OVERFLOW_MISC = 0x11,

	/* PLL */
	VT1625_PLL_P2 = 0x12,
	VT1625_PLL_D = 0x13,
	VT1625_PLL_N = 0x14,
	VT1625_PLL_OVERFLOW = 0x15,

	/* Sub Carrier */
	VT1625_SUBCARRIER_VALUE_0 = 0x16,
	VT1625_SUBCARRIER_VALUE_1 = 0x17,
	VT1625_SUBCARRIER_VALUE_2 = 0x18,
	VT1625_SUBCARRIER_VALUE_3 = 0x19,

	VT1625_VERSION_ID = 0x1B,
	VT1625_DAC_OVERFLOW = 0x1C,

	/* test */
	VT1625_TEST_0 = 0x1D,
	VT1625_TEST_1 = 0x1E,

	VT1625_FILTER_SWITCH = 0x1F,
	VT1625_TV_SYNC_STEP = 0x20,
	VT1625_TV_BURST_ENVELOPE_STEP = 0x21,
	VT1625_TV_SUB_CARRIER_PHASE_ADJUST = 0x22,
	VT1625_TV_BLANK_LEVEL = 0x23,
	VT1625_TV_SIGNAL_OVERFLOW = 0x24,

	/* DAC & GPO */
	VT1625_DAC_SELECTION_0 = 0x4A,
	VT1625_DAC_SELECTION_1 = 0x4B,
	VT1625_GPO = 0x4C,

	VT1625_COLBAR_LUMA_DELAY = 0x4D,
	VT1625_UV_DELAY = 0x4E,
	VT1625_BURST_MAX_AMPLITUDE = 0x4F,

	/* Graphic timing */
	VT1625_GRAPHIC_H_TOTAL = 0x50,
	VT1625_GRAPHIC_H_ACTIVE = 0x51,
	VT1625_GRAPHIC_H_OVERFLOW = 0x52,
	VT1625_GRAPHIC_V_TOTAL = 0x53,
	VT1625_GRAPHIC_V_OVERFLOW = 0x54,

	/* TV timing */
	VT1625_TV_H_TOTAL = 0x55,
	VT1625_TV_H_ACTIVE = 0x56,
	VT1625_TV_H_SYNC_WIDTH = 0x57,
	VT1625_TV_H_OVERFLOW = 0x58,
	VT1625_TV_BURST_START = 0x59,
	VT1625_TV_BURST_END = 0x5A,
	VT1625_TV_VIDEO_START = 0x5B,
	VT1625_TV_VIDEO_END = 0x5C,
	VT1625_TV_VIDEO_OVERFLOW = 0x5D,

	/* scale factor */
	VT1625_V_SCALE_FACTOR = 0x5E,
	VT1625_H_SCALE_FACTOR = 0x5F,
	VT1625_SCALE_OVERFLOW = 0x60,
	VT1625_H_BLUR_SCALE_OVERFLOW = 0x61,
	VT1625_ADAPTIVE_DEFLICKER_THR = 0x62,
	VT1625_SCALE_H_TOTAL = 0x63,
	VT1625_SCALE_H_TOTAL_OVERFLOW = 0x64,

	/* Amplitude factor */
	VT1625_PY_AMP_FACTOR = 0x65,
	VT1625_PB_AMP_FACTOR = 0x66,
	VT1625_PR_AMP_FACTOR = 0x67,

	VT1625_POST_ADJUST = 0x68,
	VT1625_AUTO_CORRECT_SENSE = 0x69,

	/* WSS 0x6A - 0x73 */
	VT1625_INT_WSS_2 = 0x71,

	/* Close Caption 0x74 - 0x7A */

	/* Signature Value 0x7B - 0x82 */
} vt1625_reg_t;

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx vt1625_xxx_t; *//*Example*/

/*----------EXPORTED PRIVATE VARIABLES are defined in vt1625.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  vt1625_xxx;        *//*Example*/

static char vt1625_ntsc_param[] = {
	0x03, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x82, /* 00 - 07 */
#ifdef CONFIG_VM700
         0x14, 0x05, 0x6E, 0x15, 0x51, 0x50, 0x37, 0xB7, /* 08 - 0f */
         0x00, 0x80, 0x04, 0x08, 0x08, 0x90, 0xD6, 0x7B, /* 10 - 17 */
#else
	0x14, 0x05, 0x6E, 0x15, 0x52, 0x4E, 0x37, 0xB7, /* 08 - 0f */
	0x08, 0x80, 0x04, 0x08, 0x08, 0x90, 0xD6, 0x7B, /* 10 - 17 */
#endif
         0xF0, 0x21, 0x02, 0x50, 0x43, 0x80, 0x00, 0xFC, /* 18 - 1f */
         0x16, 0x08, 0xDC, 0x7D, 0x02, 0x56, 0x33, 0x8F, /* 20 - 27 */
         0x58, 0x00, 0x00, 0xA6, 0x29, 0xD4, 0x81, 0x00, /* 28 - 2f */
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 30 - 37 */
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 38 - 3f */
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 40 - 47 */
#ifdef CONFIG_VM700
         0x00, 0x00, 0xC5, 0x0F, 0x08, 0x01, 0x01, 0x43, /* 48 - 4f */
#else
	0x00, 0x00, 0xC5, 0x0F, 0x00, 0x01, 0x10, 0x44, /* 48 - 4f */
#endif
         0x59, 0xCF, 0x23, 0x0C, 0x02, 0x59, 0xCF, 0x7F, /* 50 - 57 */
         0x23, 0x94, 0xD6, 0x00, 0x9C, 0x06, 0x00, 0x00, /* 58 - 5f */
         0x80, 0x28, 0xFF, 0x59, 0x03, 0x55, 0x56, 0x56, /* 60 - 67 */
         0x00, 0x90, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, /* 68 - 6f */
         0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 70 - 77 */
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 78 - 7f */
         0x00, 0x00, 0x00
};

static char vt1625_pal_param[] = {
	0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x8C, /* 00 - 07 */
#ifdef CONFIG_VM700
         0x0E, 0x01, 0x6E, 0x00, 0x51, 0x50, 0x37, 0xB7, /* 08 - 0f */
         0x00, 0x80, 0x04, 0x08, 0x08, 0x90, 0xCB, 0x8A, /* 10 - 17 */
#else
	0x0E, 0x01, 0x7a, 0x00, 0x55, 0x58, 0x37, 0xB7, /* 08 - 0f */
	0xff, 0x87, 0x04, 0x08, 0x08, 0x90, 0xCB, 0x8A, /* 10 - 17 */
#endif
         0x09, 0x2A, 0x06, 0x50, 0x41, 0x80, 0x00, 0xFC, /* 18 - 1f */
         0x17, 0x0C, 0x4E, 0x76, 0x02, 0x5F, 0x34, 0x8C, /* 20 - 27 */
         0x4F, 0x5E, 0x15, 0xA2, 0x22, 0x80, 0xD3, 0x10, /* 28 - 2f */
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 30 - 37 */
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 38 - 3f */
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 40 - 47 */
#ifdef CONFIG_VM700
         0x00, 0x00, 0xC5, 0x0F, 0x08, 0x02, 0x01, 0x43, /* 48 - 4f */
#else
	0x00, 0x00, 0xC5, 0x0F, 0x00, 0x02, 0x10, 0x4C, /* 48 - 4f */
#endif
         0x5f, 0xCF, 0x23, 0x70, 0x02, 0x5F, 0xD0, 0x7F, /* 50 - 57 */
         0x23, 0x92, 0xCE, 0xDF, 0xA0, 0x06, 0x00, 0x00, /* 58 - 5f */
         0x80, 0x20, 0xFF, 0x5F, 0x03, 0x5f, 0x00, 0x00, /* 60 - 67 */
         0x00, 0x90, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, /* 68 - 6f */
         0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 70 - 77 */
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 78 - 7f */
         0x00, 0x00, 0x00
};

enum vt1625_out_t {
	VT1625_OUT_CVBS,
	VT1625_OUT_YUV,
	VT1625_OUT_VGA,
	VT1625_OUT_MAX
};

enum vt1625_out_t vt1625_out_mode;

int vt1625_tv_mode;
vdo_color_fmt vt1625_colfmt;

static int pre_plugin;

/*
* VT1625 U-Boot Env to Set Register
*/
typedef struct {
	unsigned char offset;
	unsigned char value;
} vt1625_reg_env_t;

#define VT1625_REG_MAX_OFFSET  0x82  /* Register Offset: 0x00 ~ 0x82 */
/*
* setenv wmt.vt1625.pal.reg regOffset1=regValue1,regOffset2=regValue2,...
* for example:
*    setenv wmt.vt1625.pal.reg  0a=75,0c=53,23=7a,4f=48
*    setenv wmt.vt1625.ntsc.reg 0a=75,0c=53,23=7a,4f=48
*/
#define VT1625_PAL_REG_ENV  "wmt.vt1625.pal.reg"
#define VT1625_NTSC_REG_ENV "wmt.vt1625.ntsc.reg"

/*
* setenv wmt.vt1625.cvbs.always.turnon 1
*     The cvbs is always turned on event if the av line is pluged out
* setenv wmt.vt1625.cvbs.always.turnon 0
*     The cvbs is turned on if the av line is pluged in.
*     And the cvbs is turned off if the av line is pluged out
*/
#define VT1625_CVBS_ALWAYS_TURNON  "wmt.vt1625.cvbs.always.turnon"

static int vt1625_cvbs_always_turnon;

#ifdef CONFIG_KERNEL
/*
* VT1625 Timer to Monitor Register
*/
/*
* Monitor the vt1625 register for avoiding the register is cleared.
* 	setenv wmt.vt1625.reg.monitor 1
*
* If the wmt.vt1625.reg.monitor is Not set or set to 0,
* it will not monitor the register
*
*/
#define VT1625_REG_MONITOR_ENV "wmt.vt1625.reg.monitor"

#define VT1625_TIMER_INTERVAL   1000   // 1000 ms

static struct timer_list vt1625_timer;
static struct work_struct vt1625_work;

static void vt1625_set_tv_mode(int ntsc);

#endif

#ifdef CONFIG_UBOOT
#define msleep  mdelay
#endif

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void vt1625_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
/*
* Function: register_is_right()
* Parametr:
*   ntsc = 0: PAL
*   ntsc = 1: NTSC
* Return  :
*   0: the register's values is wrong
*   1: the register's values is right
*/
static int register_is_right(int ntsc)
{
	int i;
	char buf[32];

    vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR,
        VT1625_INPUT_SELECTION, buf, 32);

    for(i = 0; i < 32; i++) {
        /*
        * reg 0x0E is power management register. Skip it
        * reg 0x0F is status register. it is volatile. Skip it
        */
        if(i == 14 || i == 15)
            continue;


        if(i == 0) {
            if(vt1625_colfmt == VDO_COL_FMT_YUV444) {
                if(buf[0] != 0x3A)
                    break;
            } else {
                if(buf[0] != 0x03)
                    break;
            }
        } else {
            if(ntsc) {
                /*
                * NTSC
                */
                if(buf[i] != vt1625_ntsc_param[i])
                    break;
            } else {
                /*
                * PAL
                */
                if(buf[i] != vt1625_pal_param[i])
                    break;
            }
        }
    }

    if(i != 32)
        return 0;
    else
        return 1;
}

#ifdef CONFIG_KERNEL
static void vt1625_reconfig(struct work_struct *work)
{
    int right;

	if(vt1625_tv_mode) {
        right = register_is_right((vt1625_tv_mode == 1) ? 1 : 0);
		if(right == 0) {
			DBG_ERR("VT1625 Reg Error, re-init the register\n");

			vt1625_set_tv_mode((vt1625_tv_mode == 1) ? 1 : 0);
		}
	}

	mod_timer(&vt1625_timer, jiffies + msecs_to_jiffies(VT1625_TIMER_INTERVAL));
}

static DECLARE_WORK(vt1625_work, vt1625_reconfig);

static void vt1625_config_timer(unsigned long fcontext)
{
    schedule_work(&vt1625_work);
}

static void init_vt1625_timer(void)
{
	char buf[40];
	int varlen = 40;
	char *endp;
	int value, reg_monitor;

	reg_monitor = 0;

	if (wmt_getsyspara(VT1625_REG_MONITOR_ENV, buf, &varlen) == 0) {
		value = simple_strtoul(buf, &endp, 0);
		if( value != 0)
			reg_monitor = 1;
	}

	if(reg_monitor) {
		init_timer(&vt1625_timer);
		vt1625_timer.function = vt1625_config_timer;
		vt1625_timer.data = 0;
	} else
		vt1625_timer.function = NULL;

}

static void start_vt1625_timer(void)
{
	if(vt1625_timer.function)
		mod_timer(&vt1625_timer, jiffies + msecs_to_jiffies(VT1625_TIMER_INTERVAL));
}

static void stop_vt1625_timer(void)
{
	if(vt1625_timer.function)
		del_timer_sync(&vt1625_timer);
}
#endif

/*
* Function : vt1625_parse_reg_env
* Parameter:
*    p_env   : env name
*    p_reg   : store the vt1625 register offset and value
*    p_regnum: register number
* Return:
*    0  : the env is set and the env's value is available
*   -1  : the env is Not set or the env's value is wrong
*   -12 : no memory for parsing register env
*/
static int vt1625_parse_reg_env(char *p_env,
		vt1625_reg_env_t *p_reg, int *p_regnum)
{
	int i;
	char *buf;
	int buflen = 1024;
	unsigned int value;
	const char *p;
	char *endp;

	buf = kmalloc(buflen, GFP_KERNEL);
	if(buf == NULL) {
		DBG_ERR("kzalloc fail\n");
		return -12;
	}

	if(wmt_getsyspara(p_env, buf, &buflen) != 0) {
		kfree(buf);
		return -1;
	}

	*p_regnum = 0;
	p = buf;

	for(i = 0; i <= VT1625_REG_MAX_OFFSET; i++) {
		value = simple_strtoul(p, &endp, 16);
		if(value > VT1625_REG_MAX_OFFSET) {
			DBG_ERR("wrong register offset\n");
			kfree(buf);
			return -1;
		}
		(p_reg + i)->offset = value;
		/*
		* reg_offset must be followed reg_value
		* If reg_offset is NOT followed any reg_value, It is wrong format
		*/
		if(*endp == '\0'|| *(endp + 1) == '\0') {
			DBG_ERR("wrong env(%s) format\n", p_env);
			kfree(buf);
			return -1;
		}

		p = endp + 1;

		value = simple_strtoul(p, &endp, 16);
		if(value > 0xFF) {
			DBG_ERR("wrong register value\n");
			kfree(buf);
			return -1;
		}
		(p_reg + i)->value = value;
		*p_regnum = *p_regnum + 1;

		if(*endp == '\0')
			break;

		p = endp + 1;
	}

	kfree(buf);
	return 0;
}

/*the define and struct i2c_msg were declared int linux/i2c.h*/
void vt1625_reg_dump(void)
{
	int i;
	char buf[256];

	vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR, VT1625_INPUT_SELECTION,
		buf, 128);
	for (i = 0; i < 128; i += 8) {
		MSG("0x%02X : 0x%02X,0x%02X,0x%02X,0x%02X",
			i, buf[i], buf[i + 1], buf[i + 2], buf[i + 3]);
		MSG(",0x%02X,0x%02X,0x%02X,0x%02X\n",
			buf[i + 4], buf[i + 5], buf[i + 6], buf[i + 7]);
	}
}

static char vt1625_get_dac_val(enum vt1625_out_t mode)
{
	char ret;

	switch (mode) {
	case VT1625_OUT_CVBS:
		ret = 0x37;
		break;
	case VT1625_OUT_VGA:
		ret = 0x38;
		break;
	case VT1625_OUT_YUV:
	default:
		ret = 0x0;
		break;
	}
	return ret;
}

static void vt1625_set_tv_mode(int ntsc)
{
	char *p;
	char buf[10];

/*
	vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_INPUT_SELECTION, buf, 5);
	DBG_MSG("ntsc %d, 0x%x, 0x%x\n", ntsc, buf[0], buf[4]);
	vt1625_tv_mode = (ntsc) ? 1 : 2;
#ifdef CONFIG_KERNEL
	if (buf[0] && (vt1625_out_mode != VT1625_OUT_MAX)) {
		if (ntsc && !(buf[4] & BIT0))
			return;
		if (!ntsc && (buf[4] & BIT0))
			return;
	}
#endif
*/
	vt1625_tv_mode = (ntsc) ? 1 : 2;
	if(register_is_right(ntsc))
		return;

	DBG_MSG("tv %s,mode %d\n", (ntsc) ? "NTSC" : "PAL", vt1625_out_mode);

	p = (char *)((ntsc) ?  vt1625_ntsc_param : vt1625_pal_param);
	vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR, VT1625_INPUT_SELECTION,
				&p[VT1625_INPUT_SELECTION], 0x71);
	if (vt1625_out_mode == VT1625_OUT_MAX) { /* not stable so no use */
		buf[0] = 0x0;
		vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_POWER_MANAGEMENT, buf, 1);
		mdelay(10);
		vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_STATUS, buf, 1);
		vt1625_out_mode = (buf[0] & 0x7) ?
			VT1625_OUT_CVBS : VT1625_OUT_VGA;
		DBG_MSG("get out mode %d, 0x%x\n", vt1625_out_mode, buf[0]);
	}

	if (vt1625_out_mode == VT1625_OUT_VGA) {
		vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_SYNC_SELECTION_1, buf, 1);
		buf[0] |= 0xA0;
		vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_SYNC_SELECTION_1, buf, 1);

		vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_DAC_OVERFLOW, buf, 1);
		buf[0] |= 0x20;
		vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_DAC_OVERFLOW, buf, 1);

		vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_TEST_1, buf, 1);
		buf[0] |= 0x40;
		vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_TEST_1, buf, 1);
	} else {
#ifdef CONFIG_VT1625_INTERRUPT
		/* interrupt (VGA no work) */
		vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_INT_WSS_2, buf, 1);
		buf[0] |= 0xA0; /* enable sense interrupt */
		vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_INT_WSS_2, buf, 1);
#endif
	}

	if (vt1625_colfmt == VDO_COL_FMT_YUV444) {
		/*
		* Force write reg0x00 and reg0x4C
		*/
		buf[0] = 0x3A;
		vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_INPUT_SELECTION, buf, 1);
		buf[0] = 0x08;
		vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_GPO, buf, 1);
	}

#ifdef CONFIG_VT1625_POWER
	buf[0] = vt1625_get_dac_val(vt1625_out_mode);
	vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
		VT1625_POWER_MANAGEMENT, buf, 1);
#endif
}

static int vt1625_check_plugin(int hotplug)
{
	char buf[2];
	char cur[1];
	int plugin;

	/*
	* Enable VT1625 Power First
	*/
	vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR,
		VT1625_POWER_MANAGEMENT, cur, 1);

	buf[0] = vt1625_get_dac_val(vt1625_out_mode);

	if(cur[0] != buf[0]) {
		vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_POWER_MANAGEMENT, buf, 1);

		msleep(10);
	}

	if((vt1625_out_mode == VT1625_OUT_CVBS) && vt1625_cvbs_always_turnon)
		return 1;

	vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR, VT1625_POWER_MANAGEMENT,
		buf, 2);
	plugin = ~buf[1] & (~buf[0] & 0x3F);
	DBG_MSG("[VT1625] DAC A %d, B %d, C %d, D %d, E %d, F %d\n",
		(plugin & 0x20) ? 1 : 0, (plugin & 0x10) ? 1 : 0,
		(plugin & 0x08) ? 1 : 0, (plugin & 0x04) ? 1 : 0,
		(plugin & 0x02) ? 1 : 0, (plugin & 0x01) ? 1 : 0);
	return (plugin) ? 1 : 0;
}

static int vt1625_init(struct vout_t *vo)
{
	char buf[40];
	int varlen = 40;
	char *endp;
	unsigned int value;

	DBG_MSG("\n");
	if (vt1625_tv_mode) { /* resume reinit */
		MSG("[VT1625] DVI reinit\n");
		vt1625_set_tv_mode((vt1625_tv_mode == 1) ? 1 : 0);
		if (govrh_get_dvo_enable(p_govrh2) == 0)
			govrh_set_dvo_enable(p_govrh2, VPP_FLAG_ENABLE);
		pre_plugin = 0;
		return 0;
	}

	vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR, VT1625_VERSION_ID, buf, 1);
	if (buf[0] != 0x50) /* check version id */
		return -1;

	if (wmt_getsyspara("wmt.display.vt1625.mode", buf, &varlen) == 0) {
		if (memcmp(buf, "yuv", 3) == 0)
			vt1625_out_mode = VT1625_OUT_YUV;
		else if (memcmp(buf, "vga", 3) == 0)
			vt1625_out_mode = VT1625_OUT_VGA;
		else
			vt1625_out_mode = VT1625_OUT_CVBS;
		DPRINT("[VT1625] mode %d\n", vt1625_out_mode);
	} else
		vt1625_out_mode = VT1625_OUT_CVBS; /* VT1625_OUT_MAX; */

#ifdef CONFIG_VM700
	vt1625_colfmt = VDO_COL_FMT_YUV444;
#else
	vt1625_colfmt = VDO_COL_FMT_ARGB;
#endif
	if (wmt_getsyspara("wmt.display.vt1625.colfmt", buf, &varlen) == 0) {
		if (memcmp(buf, "yuv", 3) == 0)
			vt1625_colfmt = VDO_COL_FMT_YUV444;
		else if (memcmp(buf, "rgb", 3) == 0)
			vt1625_colfmt = VDO_COL_FMT_ARGB;
	}

	vo->option[0] = (unsigned int) vt1625_colfmt;
	vo->option[1] = (unsigned int) VPP_DATAWIDHT_12;

	vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_INPUT_SELECTION, buf, 5);
	if (buf[0])
		vt1625_tv_mode = (buf[4]) ? 2 : 1;

    p_govrh2->fb_p->csc_mode = VPP_CSC_RGB2YUV_SDTV_0_255;

	if (wmt_getsyspara(VT1625_CVBS_ALWAYS_TURNON, buf, &varlen) == 0) {
		value = simple_strtoul(buf, &endp, 0);
		if(value != 0)
			vt1625_cvbs_always_turnon = 1;
		else
			vt1625_cvbs_always_turnon = 0;
	} else
		vt1625_cvbs_always_turnon = 0;

#ifdef CONFIG_KERNEL
    vt1625_set_tv_mode((vt1625_tv_mode == 1) ? 1 : 0);

	start_vt1625_timer();
#endif

	MSG("[VT1625] DVI ext device\n");
	return 0;
}

static int vt1625_set_mode(unsigned int *option)
{
#ifdef CONFIG_VT1625_INTERRUPT
	char buf[1];
#endif

	DBG_MSG("\n");
#ifdef CONFIG_VT1625_INTERRUPT
	if (!g_vpp.dvi_int_disable) {
	vout_set_int_type(1);
	vout_set_int_enable(1);

	vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR,
		VT1625_INT_WSS_2, buf, 1);
	buf[0] |= 0xA0; /* enable sense interrupt */
	vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
		VT1625_INT_WSS_2, buf, 1);
	}
#endif
	return 0;
}

static void vt1625_set_power_down(int enable)
{
	struct vout_t *vo;
	char buf[1];
	char cur[1];

	/*
	bit 0-2 : DAC D/E/F - VGA
	bit 3 : DAC C - CVBS
	bit 3-5 : DAC A/B/C - YPbPr
	bit 6 : PLL
	bit 7 : IO pad
	*/
	vo = vout_get_entry(VPP_VOUT_NUM_DVI);
	if (vo->status & (VPP_VOUT_STS_BLANK + VPP_VOUT_STS_POWERDN))
		enable = 1;

	/* power down for not support resolution */
//#ifndef CONFIG_VT1625_INTERRUPT
	if ((vt1625_tv_mode != 0) && enable && g_vpp.dvi_int_disable)
		buf[0] = 0xFF;
	else
//#endif
		buf[0] = vt1625_get_dac_val(vt1625_out_mode);

	vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR,
		VT1625_POWER_MANAGEMENT, cur, 1);

	if (cur[0] == buf[0])
		return;

	DBG_MSG("enable %d,cur 0x%x,new 0x%x\n", enable, cur[0], buf[0]);
#if 1
	if (enable == 0) {
		cur[0] &= ~0x40; /* turn on PLL */
		vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_POWER_MANAGEMENT, cur, 1);
		mdelay(3);

		cur[0] &= ~0x80; /* turn on IO pad */
		vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_POWER_MANAGEMENT, cur, 1);
		mdelay(3);
	}
#endif
#ifdef CONFIG_VT1625_POWER
	vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
		VT1625_POWER_MANAGEMENT, buf, 1);
#endif
}

static int vt1625_config(struct vout_info_t *info)
{
	int ntsc = -1;

	DBG_MSG("%d,%d\n", info->resx, info->resy);
	if (info->resx == 720) {
		switch (info->resy) {
		case 480:
			ntsc = 1;
			break;
		case 576:
			ntsc = 0;
			break;
		default:
			break;
		}
	}

	if (ntsc != -1)
		vt1625_set_tv_mode(ntsc);
	else
		vt1625_tv_mode = 0;
	DBG_MSG("end\n");
	return 0;
}

static int vt1625_get_edid(char *buf)
{
	return 0;
}

#ifdef CONFIG_VT1625_INTERRUPT
static int vt1625_interrupt(void)
{
	char buf[1];

	vppif_reg32_write(GPIO_BASE_ADDR + 0x4c0, 0x1 << VPP_VOINT_NO,
			VPP_VOINT_NO, 0x0); /* GPIO pull-up */
	/* interrupt */
	vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR, VT1625_INT_WSS_2, buf, 1);
	DBG_MSG("0x%x\n", buf[0]);
	buf[0] &= ~0x40; /* clear interrupt */
	vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR, VT1625_INT_WSS_2, buf, 1);
	return vt1625_check_plugin(1);
}
#endif

static void vt1625_poll(void)
{
	int plugin;
	char buf[1];
	char cur[1];

	if (govrh_get_dvo_enable(p_govrh2) == 0)
		govrh_set_dvo_enable(p_govrh2, VPP_FLAG_ENABLE);

	plugin = vt1625_check_plugin(0);
	if (plugin != pre_plugin) {
		struct vout_t *vo;

		vo = vout_get_entry(VPP_VOUT_NUM_DVI);
		vout_change_status(vo, VPP_VOUT_STS_PLUGIN, plugin);
#ifdef CONFIG_KERNEL
		vpp_netlink_notify_plug(VPP_VOUT_NUM_DVI, plugin);
#endif
		pre_plugin = plugin;
		DMSG("%d\n", plugin);
	}

	/*
	* Disable VT1625 Power if CVBS Not plugin
	*/
	if(plugin == 0) {
		vpp_i2c_read(VPP_DVI_I2C_ID, VT1625_ADDR,
			VT1625_POWER_MANAGEMENT, cur, 1);

		buf[0] = 0xFF;

		if(cur[0] != buf[0]) {
			vpp_i2c_write(VPP_DVI_I2C_ID, VT1625_ADDR,
				VT1625_POWER_MANAGEMENT, buf, 1);
		}
	}
}

static int vt1625_suspend(void)
{
	DMSG("\n");

#ifdef CONFIG_KERNEL
	stop_vt1625_timer();
#endif

	return 0;
}

static int vt1625_resume(void)
{
	DMSG("\n");

#ifdef CONFIG_KERNEL
	start_vt1625_timer();
#endif
	return 0;
}

/*----------------------- vout device plugin ---------------------------------*/
struct vout_dev_t vt1625_vout_dev_ops = {
	.name = "VT1625",
	.mode = VOUT_INF_DVI,

	.init = vt1625_init,
	.set_power_down = vt1625_set_power_down,
	.set_mode = vt1625_set_mode,
	.config = vt1625_config,
	.check_plugin = vt1625_check_plugin,
	.get_edid = vt1625_get_edid,
#ifdef CONFIG_VT1625_INTERRUPT
	.interrupt = vt1625_interrupt,
#endif
	.poll = vt1625_poll,
	.suspend = vt1625_suspend,
	.resume = vt1625_resume,
};

int vt1625_module_init(void)
{
	vt1625_reg_env_t *p_reg;
	int i, ret, regnum;

	p_reg = kmalloc((VT1625_REG_MAX_OFFSET + 1) * sizeof(vt1625_reg_env_t),
		GFP_KERNEL);
	if(p_reg) {
		ret = vt1625_parse_reg_env(VT1625_PAL_REG_ENV, p_reg, &regnum);
		if(ret == 0) {
			for(i = 0; i < regnum; i++)
				vt1625_pal_param[(p_reg + i)->offset] =
					(p_reg + i)->value;
		}

		ret = vt1625_parse_reg_env(VT1625_NTSC_REG_ENV, p_reg, &regnum);
		if(ret == 0) {
			for(i = 0; i < regnum; i++)
				vt1625_ntsc_param[(p_reg + i)->offset] =
					(p_reg + i)->value;
		}

		kfree(p_reg);
	} else
		DBG_ERR("kzalloc fail\n");

#ifdef CONFIG_KERNEL
	init_vt1625_timer();
#endif

	vout_device_register(&vt1625_vout_dev_ops);

	return 0;
}
module_init(vt1625_module_init);
/*--------------------End of Function Body -----------------------------------*/
#undef VT1625_C
