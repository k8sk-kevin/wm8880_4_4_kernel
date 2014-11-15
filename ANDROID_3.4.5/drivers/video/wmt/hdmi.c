/*++
 * linux/drivers/video/wmt/hdmi.c
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

#define HDMI_C
#undef DEBUG
/* #define DEBUG */
/*----------------------- DEPENDENCE -----------------------------------------*/
#include "hdmi.h"
#include "vout.h"
#ifdef __KERNEL__
#include <asm/div64.h>
#endif

/*----------------------- PRIVATE MACRO --------------------------------------*/

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define HDMI_XXXX    1     *//*Example*/
/* #define CONFIG_HDMI_INFOFRAME_DISABLE */
/* #define CONFIG_HDMI_EDID_DISABLE */

#define HDMI_I2C_FREQ	80000
/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx hdmi_xxx_t; *//*Example*/
enum hdmi_fifo_slot_t {
	HDMI_FIFO_SLOT_AVI = 0,
	HDMI_FIFO_SLOT_VENDOR = 1,
	HDMI_FIFO_SLOT_AUDIO = 2,
	HDMI_FIFO_SLOT_CONTROL = 3,
	HDMI_FIFO_SLOT_MAX = 15
};

/*----------EXPORTED PRIVATE VARIABLES are defined in hdmi.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  hdmi_xxx;        *//*Example*/

HW_REG struct hdmi_base1_regs *hdmi_regs1 = (void *) (HDMI_BASE_ADDR + 0x100);
HW_REG struct hdmi_base2_regs *hdmi_regs2 = (void *) HDMI_BASE2_ADDR;

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void hdmi_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
/*---------------------------- HDMI COMMON API -------------------------------*/
unsigned char hdmi_ecc(unsigned char *buf, int bit_cnt)
{
	#define HDMI_CRC_LEN	9

	int crc[HDMI_CRC_LEN], crc_o[HDMI_CRC_LEN];
	int i, j;
	int input, result, result_rev = 0;

	for (i = 0; i < HDMI_CRC_LEN; i++)
		crc[i] = 0;

	for (i = 0; i < bit_cnt; i++) {
		for (j = 0; j < HDMI_CRC_LEN; j++)
			crc_o[j] = crc[j];
		input = (buf[i/8] & (1<<(i%8))) ? 1 : 0;
		crc[0] = crc_o[7] ^ input;
		crc[1] = crc_o[0];
		crc[2] = crc_o[1];
		crc[3] = crc_o[2];
		crc[4] = crc_o[3];
		crc[5] = crc_o[4];
		crc[6] = crc_o[5] ^ crc_o[7] ^ input;
		crc[7] = crc_o[6] ^ crc_o[7] ^ input;
		crc[8] = crc_o[7];

		result     = 0;
		result_rev = 0;
		for (j = 0; j < HDMI_CRC_LEN - 1; j++) {
			result     += (crc[j] << j);
			result_rev += (crc[j] << (HDMI_CRC_LEN - 2 - j));
		}
	}

/*	DPRINT("[HDMI] crc 0x%x, %x %x %x %x %x %x %x\n",result_rev,
			buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6]); */
	return result_rev;
}

unsigned char hdmi_checksum(unsigned char *header,
					unsigned char *buf, int cnt)
{
	unsigned char sum;
	int i;

	for (i = 0, sum = 0; i < cnt; i++)
		sum += buf[i];
	for (i = 0; i < 3; i++)
		sum += header[i];
	return 0 - sum;
}

#ifdef WMT_FTBLK_HDMI
/*---------------------------- HDMI HAL --------------------------------------*/
void hdmi_set_power_down(int pwrdn)
{
	DBG_DETAIL("(%d)\n", pwrdn);

	if ((hdmi_regs2->test.b.pd == 0) && (pwrdn == 0))
		return; /* avoid HDMI reset */

	hdmi_regs2->status.b.internal_ldo = (pwrdn) ? 0 : 1;
	hdmi_regs2->test.b.pd = pwrdn;
	if (!pwrdn) {
		hdmi_regs2->dftset2.b.reset_pll = 1;
		mdelay(1);
		hdmi_regs2->dftset2.b.reset_pll = 0;
	}
	mdelay(1);
	hdmi_regs2->test2.b.pd_l2ha = pwrdn;
}

int hdmi_get_power_down(void)
{
	return hdmi_regs2->test.b.pd;
}

void hdmi_set_enable(vpp_flag_t enable)
{
	hdmi_regs1->general_ctrl.b.enable = enable;
	hdmi_regs1->general_ctrl.b.vsync_enable = 1; /* for write only */
	hdmi_regs2->test2.b.mode = (enable) ? 0 : 1;
}

void hdmi_set_avmute(vpp_flag_t mute)
{
	hdmi_regs1->aud_insert_ctrl.b.avmute_set_enable = mute;
}

void hdmi_set_dvi_enable(vpp_flag_t enable)
{
	hdmi_regs1->general_ctrl.b.dvi_mode_enable = enable;
	hdmi_regs1->general_ctrl.b.vsync_enable = 1; /* for write only */
}

void hdmi_set_sync_low_active(vpp_flag_t hsync, vpp_flag_t vsync)
{
	hdmi_regs1->general_ctrl.b.hsync_low_active = hsync;
	hdmi_regs1->general_ctrl.b.vsync_low_active = vsync;
	hdmi_regs1->general_ctrl.b.vsync_enable = 1; /* for write only */
}

void hdmi_get_sync_polar(int *hsync_hi, int *vsync_hi)
{
	*hsync_hi = (hdmi_regs1->general_ctrl.b.hsync_low_active) ? 0 : 1;
	*vsync_hi = (hdmi_regs1->general_ctrl.b.vsync_low_active) ? 0 : 1;
}

void hdmi_set_output_colfmt(vdo_color_fmt colfmt)
{
	unsigned int val;

	switch (colfmt) {
	default:
	case VDO_COL_FMT_ARGB:
		val = 0;
		break;
	case VDO_COL_FMT_YUV444:
		val = 1;
		break;
	case VDO_COL_FMT_YUV422H:
	case VDO_COL_FMT_YUV422V:
		val = 2;
		break;
	}
	hdmi_regs1->general_ctrl.b.convert_yuv422 = (val == 2) ? 1 : 0;
	hdmi_regs1->general_ctrl.b.output_format = val;
	hdmi_regs1->general_ctrl.b.vsync_enable = 1; /* for write only */
}

vdo_color_fmt hdmi_get_output_colfmt(void)
{
	unsigned int val;

	val = hdmi_regs1->general_ctrl.b.output_format;
	switch (val) {
	default:
	case 0:
		return VDO_COL_FMT_ARGB;
	case 1:
		return VDO_COL_FMT_YUV444;
	case 2:
		return VDO_COL_FMT_YUV422H;
	}
	return VDO_COL_FMT_ARGB;
}

int hdmi_get_plugin(void)
{
	int plugin;

	if (hdmi_regs1->hotplug_detect.b.in_enable) {
		plugin = hdmi_regs1->hotplug_detect.b.sts;
	} else {
		int tre_en;

		tre_en = hdmi_regs2->test.b.tre_en;
		hdmi_regs2->test.b.tre_en = 0;
		plugin = hdmi_regs2->detect.b.rsen;
		hdmi_regs2->test.b.tre_en = tre_en;
	}
	return plugin;
}

int hdmi_get_plug_status(void)
{
	int reg;

	reg = hdmi_regs1->hotplug_detect.val;
	return reg & 0x3000000;
}

void hdmi_clear_plug_status(void)
{
	hdmi_regs1->hotplug_detect.b.in_sts = 1;
	hdmi_regs1->hotplug_detect.b.out_sts = 1;
}

void hdmi_enable_plugin(int enable)
{
	hdmi_regs1->hotplug_detect.b.out_enable = enable;
	hdmi_regs1->hotplug_detect.b.in_enable = enable;
}

void hdmi_write_fifo(enum hdmi_fifo_slot_t no, unsigned int *buf, int cnt)
{
	int i;

	if (no > HDMI_FIFO_SLOT_MAX)
		return;
#ifdef DEBUG
{
	char *ptr;

	DPRINT("[HDMI] wr fifo %d,cnt %d", no, cnt);
	ptr = (char *) buf;
	for (i = 0; i < cnt; i++) {
		if ((i % 4) == 0)
			DPRINT("\n %02d :", i);
		DPRINT(" 0x%02x", ptr[i]);
	}
	DPRINT("\n[HDMI] AVI info package end\n");
}
#endif
	hdmi_regs1->fifo_ctrl.val = (no << 8);
	cnt = (cnt + 3) / 4;
	for (i = 0; i < cnt; i++)
		hdmi_regs1->wr_fifo_addr[i] = buf[i];
	hdmi_regs1->fifo_ctrl.b.wr_strobe = 1;
}

void hdmi_read_fifo(enum hdmi_fifo_slot_t no, unsigned int *buf, int cnt)
{
	int i;
	int rdy;

	if (no > HDMI_FIFO_SLOT_MAX)
		return;

	rdy = hdmi_regs1->infoframe_ctrl.b.fifo1_rdy;
	hdmi_regs1->infoframe_ctrl.b.fifo1_rdy = 0;

	no = no - 1;
	hdmi_regs1->fifo_ctrl.val = (no << 8);
	hdmi_regs1->fifo_ctrl.b.rd_strobe = 1;
	cnt = (cnt + 3) / 4;
	for (i = 0; i < cnt; i++)
		buf[i] = hdmi_regs1->rd_fifo_addr[i];
	hdmi_regs1->infoframe_ctrl.b.fifo1_rdy = rdy;
#ifdef DEBUG
{
	char *ptr;

	cnt *= 4;
	DPRINT("[HDMI] rd fifo %d,cnt %d", no, cnt);
	ptr = (char *) buf;
	for (i = 0; i < cnt; i++) {
		if ((i % 4) == 0)
			DPRINT("\n %02d :", i);
		DPRINT(" 0x%02x", ptr[i]);
	}
	DPRINT("\n[HDMI] AVI info package end\n");
}
#endif
}

#if 1
int hdmi_ddc_delay_us = 5;
int hdmi_ddc_ctrl_delay_us = 5;

#define HDMI_DDC_OUT
#define HDMI_DDC_DELAY		hdmi_ddc_delay_us
#define HDMI_DDC_CHK_DELAY	1
#define HDMI_DDC_CTRL_DELAY	hdmi_ddc_ctrl_delay_us
#else
#define HDMI_DDC_OUT
#define HDMI_DDC_DELAY		1
#define HDMI_DDC_CHK_DELAY	1
#define HDMI_DDC_CTRL_DELAY	1
#endif

#define HDMI_STATUS_START	BIT16
#define HDMI_STATUS_STOP	BIT17
#define HDMI_STATUS_WR_AVAIL	BIT18
#define HDMI_STATUS_CP_USE	BIT19
#define HDMI_STATUS_SW_READ	BIT25
int hdmi_DDC_check_status(unsigned int checkbits, int condition)
{
	int status = 1;
	unsigned int i = 0, maxloop;

	maxloop = 500 / HDMI_DDC_CHK_DELAY;
	udelay(HDMI_DDC_DELAY);

	if (condition) { /* wait 1 --> 0 */
		while ((hdmi_regs1->i2c_ctrl2.val & checkbits)
			&& (i < maxloop)) {
			udelay(HDMI_DDC_CHK_DELAY);
			if (++i == maxloop)
				status = 0;
		}
	} else { /* wait 0 --> 1 */
		while (!(hdmi_regs1->i2c_ctrl2.val & checkbits)
			&& (i < maxloop)) {
			udelay(HDMI_DDC_CHK_DELAY);
			if (++i == maxloop)
				status = 0;
		}
	}

	if ((status == 0) && (checkbits != HDMI_STATUS_SW_READ)) {
		unsigned int reg;

		reg = hdmi_regs1->i2c_ctrl2.val;
		DBG_DETAIL("[HDMI] status timeout check 0x%x,wait to %s\n",
			checkbits, (condition) ? "0" : "1");
		DBG_DETAIL("[HDMI] 0x%x,sta %d,stop %d,wr %d,rd %d,cp %d\n",
			reg, (reg & HDMI_STATUS_START) ? 1 : 0,
			(reg & HDMI_STATUS_STOP) ? 1 : 0,
			(reg & HDMI_STATUS_WR_AVAIL) ? 1 : 0,
			(reg & HDMI_STATUS_SW_READ) ? 1 : 0,
			(reg & HDMI_STATUS_CP_USE) ? 1 : 0);
	}
	return status;
}

void hdmi_DDC_set_freq(unsigned int hz)
{
	unsigned int clock;
	unsigned int div;

	clock = 25000000*15/100;	/* RTC clock source */
	div = clock / hz;

	hdmi_regs1->i2c_ctrl.b.i2c_clk_divider = div;
	DBG_DETAIL("[HDMI] set freq(%d,clk %d,div %d)\n", hz, clock, div);
}

void hdmi_DDC_reset(void)
{
	hdmi_regs1->i2c_ctrl.b.i2c_sw_reset = 1;
	udelay(1);
	hdmi_regs1->i2c_ctrl.b.i2c_sw_reset = 0;
}

int hdmi_DDC_read_func(char addr, int index, char *buf, int length)
{
	int status = 1;
	unsigned int i = 0;
	int err_cnt = 0;

	DBG_DETAIL("[HDMI] read DDC(index 0x%x,len %d),reg 0x%x\n",
		index, length, hdmi_regs1->i2c_ctrl2.val);

#ifdef CONFIG_HDMI_EDID_DISABLE
	return status;
#endif

	hdmi_DDC_set_freq(g_vpp.hdmi_i2c_freq);
	/* enhanced DDC read */
	if (index >= 256) {
		/* sw start, write data avail */
		vppif_reg32_write((unsigned int) &hdmi_regs1->i2c_ctrl2,
					BIT18 + BIT16, 16, 0x5);
		udelay(HDMI_DDC_CTRL_DELAY);
		/* wait start & wr data avail */
		status = hdmi_DDC_check_status(HDMI_STATUS_START +
					HDMI_STATUS_WR_AVAIL, 1);
		if (status == 0) {
			DBGMSG("[HDMI] *E* start\n");
			err_cnt++;
			goto ddc_read_fail;
		}

		/* Slave address */
		hdmi_regs1->i2c_ctrl2.b.wr_data = 0x60;
		hdmi_regs1->i2c_ctrl2.b.wr_data_avail = 1;
		udelay(HDMI_DDC_CTRL_DELAY);
		/* wait wr data avail */
		status = hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);
		if (status == 0) {
			DBGMSG("[HDMI] *E* slave addr 0x%x\n", addr);
			err_cnt++;
			goto ddc_read_fail;
		}

		/* Offset */
		hdmi_regs1->i2c_ctrl2.b.wr_data = 0x1;
		hdmi_regs1->i2c_ctrl2.b.wr_data_avail = 1;
		udelay(HDMI_DDC_CTRL_DELAY);
		/* wait wr data avail */
		status = hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);
		if (status == 0) {
			DBGMSG("[HDMI] *E* index 0x%x\n", index);
			err_cnt++;
			goto ddc_read_fail;
		}
		index -= 256;
	}

	/* START */
	hdmi_regs1->i2c_ctrl2.val = 0x50000; /* start & data avail */
	udelay(HDMI_DDC_CTRL_DELAY);
	/* wait start & wr data avail */
	status = hdmi_DDC_check_status(HDMI_STATUS_START +
					HDMI_STATUS_WR_AVAIL, 1);
	if (status == 0) {
		DBGMSG("[HDMI] *E* start\n");
		err_cnt++;
		goto ddc_read_fail;
	}

	/* Slave address */
	hdmi_regs1->i2c_ctrl2.val = 0x400A0; /* addr & data avail */
	udelay(HDMI_DDC_CTRL_DELAY);
	/* wait wr data avail */
	status = hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);
	if (status == 0) {
		DBGMSG("[HDMI] *E* slave addr 0x%x\n", addr);
		err_cnt++;
		goto ddc_read_fail;
	}

	/* Offset */
	hdmi_regs1->i2c_ctrl2.val = (0x40000 + index); /* index & data avail */
	udelay(HDMI_DDC_CTRL_DELAY);
	/* wait wr data avail */
	status = hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);
	if (status == 0) {
		DBGMSG("[HDMI] *E* index 0x%x\n", index);
		err_cnt++;
		goto ddc_read_fail;
	}

	/* START */
	hdmi_regs1->i2c_ctrl2.val = 0x50000; /* start & data avail */
	udelay(HDMI_DDC_CTRL_DELAY);
	/* wait start & wr data avail */
	status = hdmi_DDC_check_status(HDMI_STATUS_START +
					HDMI_STATUS_WR_AVAIL, 1);
	if (status == 0) {
		DBGMSG("[HDMI] *E* restart\n");
		err_cnt++;
		goto ddc_read_fail;
	}

	/* Slave Address + 1 */
	hdmi_regs1->i2c_ctrl2.val = 0x400A1; /* addr(rd) & data avail */
	udelay(HDMI_DDC_CTRL_DELAY);
	/* wait wr data avail */
	status = hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);
	if (status == 0) {
		DBGMSG("[HDMI] *E* slave addr 0x%x\n", addr + 1);
		err_cnt++;
		goto ddc_read_fail;
	}

	/* Read Data */
	for (i = 0; i < length; i++) {
		hdmi_regs1->i2c_ctrl2.val = 0x40000; /* data avail */
		udelay(HDMI_DDC_CTRL_DELAY);
		/* wait wr data avail */
		status = hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);
		if (status == 0) {
			DBGMSG("[HDMI] *E* wr ACK(%d)\n", i);
			err_cnt++;
			goto ddc_read_fail;
			/* break; */
		}

		/* wait sw read not set */
		status = hdmi_DDC_check_status(HDMI_STATUS_SW_READ, 0);
		if (status == 0) {
			DBGMSG("[HDMI] *E* read avail(%d)\n", i);
			if (i == 0) {
				err_cnt++;
				/* goto ddc_read_fail; */
			} else {
				/* g_vpp.dbg_hdmi_ddc_read_err++; */
			}
			goto ddc_read_fail;
			/* break; */
		}

		*buf++ = hdmi_regs1->i2c_ctrl2.b.rd_data;
		udelay(HDMI_DDC_DELAY);
		hdmi_regs1->i2c_ctrl2.val = 0x2000000; /* clr sw read */
		udelay(HDMI_DDC_DELAY);
	}

	/* STOP */
	/* sw stop, write data avail */
	vppif_reg32_write((unsigned int) &hdmi_regs1->i2c_ctrl2.val,
					BIT18 + BIT17, 17, 3);
	udelay(HDMI_DDC_CTRL_DELAY);
	/* wait start & wr data avail */
	status = hdmi_DDC_check_status(HDMI_STATUS_STOP +
				HDMI_STATUS_WR_AVAIL + HDMI_STATUS_CP_USE, 1);
	if (status == 0) {
		DBGMSG("[HDMI] *E* stop\n");
		err_cnt++;
		goto ddc_read_fail;
	}
	udelay(HDMI_DDC_DELAY);

ddc_read_fail:
	if (err_cnt)
		DBGMSG("[HDMI] *E* read DDC %d\n", err_cnt);
	return (err_cnt) ? 1 : 0;
}

int hdmi_DDC_read(char addr, int index, char *buf, int length)
{
	int retry = 3;
	int ret;

	DBG_MSG("(0x%x,0x%x,%d)\n", addr, index, length);

	do {
		ret = hdmi_DDC_read_func(addr, index, buf, length);
		if (ret == 0)
			break;
		hdmi_DDC_reset();
		DPRINT("[HDMI] *W* DDC reset %d\n", ret);
		retry--;
	} while (retry);

	return (retry == 0) ? 1 : 0;
}

void hdmi_audio_enable(vpp_flag_t enable)
{
	if (hdmi_regs1->aud_enable != enable) {
		if (!enable) {
#ifdef CONFIG_KERNEL
			msleep(5);
#endif
			if (g_vpp.hdmi_ch_change)
				REG32_VAL(I2S_BASE_ADDR + 0x188) = 0;
		}
		hdmi_regs1->aud_enable = (enable) ? 1 : 0;
	}
}

void hdmi_audio_mute(vpp_flag_t enable)
{
	hdmi_regs1->aud_ratio.b.mute = enable;
}

/*----------------------- HDMI API --------------------------------------*/
void hdmi_write_packet(unsigned int header, unsigned char *packet,
				int cnt)
{
	unsigned char buf[36];
	int i;
	enum hdmi_fifo_slot_t no;

#ifdef CONFIG_HDMI_INFOFRAME_DISABLE
	return;
#endif
	memcpy(&buf[0], &header, 3);
	buf[3] = hdmi_ecc((unsigned char *)&header, 24);
	for (i = 0; i < cnt / 7; i++) {
		memcpy(&buf[4+8*i], &packet[7*i], 7);
		buf[11+8*i] = hdmi_ecc(&packet[7*i], 56);
	}

	switch (header & 0xFF) {
	case HDMI_PACKET_INFOFRAME_AVI:
		no = HDMI_FIFO_SLOT_AVI;
		break;
	case HDMI_PACKET_INFOFRAME_AUDIO:
		no = HDMI_FIFO_SLOT_AUDIO;
		break;
	case HDMI_PACKET_INFOFRAME_VENDOR:
		no = HDMI_FIFO_SLOT_VENDOR;
		break;
	default:
		no = HDMI_FIFO_SLOT_CONTROL;
		break;
	}
	hdmi_write_fifo(no, (unsigned int *)buf, (4 + 8 * (cnt / 7)));
}

void hdmi_tx_null_packet(void)
{
	hdmi_write_packet(HDMI_PACKET_NULL, 0, 0);
}

void hdmi_tx_general_control_packet(int mute)
{
	unsigned char buf[7];
	memset(buf, 0x0, 7);
	buf[0] = (mute) ? 0x01 : 0x10;
	buf[1] = HDMI_COLOR_DEPTH_24 | (HDMI_PHASE_4 << 4);
	hdmi_write_packet(HDMI_PACKET_GENERAL_CTRL, buf, 7);
}

int hdmi_get_pic_aspect(enum hdmi_video_code_t vic)
{
	switch (vic) {
	case HDMI_640x480p60_4x3:
	case HDMI_720x480p60_4x3:
	case HDMI_1440x480i60_4x3:
	case HDMI_1440x240p60_4x3:
	case HDMI_2880x480i60_4x3:
	case HDMI_2880x240p60_4x3:
	case HDMI_1440x480p60_4x3:
	case HDMI_720x576p50_4x3:
	case HDMI_1440x576i50_4x3:
	case HDMI_1440x288p50_4x3:
	case HDMI_2880x576i50_4x3:
	case HDMI_2880x288p50_4x3:
	case HDMI_1440x576p50_4x3:
		return HDMI_PIC_ASPECT_4_3;
	default:
		break;
	}
	return HDMI_PIC_ASPECT_16_9;
}

int hdmi_get_vic(int resx, int resy, int fps, int interlace)
{
	struct hdmi_vic_t info;
	int i;

	info.resx = resx;
	info.resy = resy;
	info.freq = fps;
	info.option = (interlace) ? HDMI_VIC_INTERLACE : HDMI_VIC_PROGRESS;
	info.option |= (vout_check_ratio_16_9(resx, resy)) ?
				HDMI_VIC_16x9 : HDMI_VIC_4x3;
	for (i = 0; i < HDMI_VIDEO_CODE_MAX; i++) {
		if (memcmp(&hdmi_vic_info[i], &info,
			sizeof(struct hdmi_vic_t)) == 0)
			return i;
	}
	return HDMI_UNKNOW;
}

void hdmi_tx_avi_infoframe_packet(vdo_color_fmt colfmt,
						enum hdmi_video_code_t vic)
{
	unsigned int header;
	unsigned char buf[28];
	unsigned char temp;

	memset(buf, 0x0, 28);
	header = HDMI_PACKET_INFOFRAME_AVI + (0x2 << 8) + (0x0d << 16);
	buf[1] = HDMI_SI_NO_DATA + (HDMI_BI_V_H_VALID << 2) +
				(HDMI_AF_INFO_NO_DATA << 4);
	switch (colfmt) {
	case VDO_COL_FMT_YUV422H:
	case VDO_COL_FMT_YUV422V:
		temp = HDMI_OUTPUT_YUV422;
		break;
	case VDO_COL_FMT_YUV444:
		temp = HDMI_OUTPUT_YUV444;
		break;
	case VDO_COL_FMT_ARGB:
	default:
		temp = HDMI_OUTPUT_RGB;
		break;
	}
	buf[1] += (temp << 5);
	buf[2] = HDMI_ASPECT_RATIO_PIC + (hdmi_get_pic_aspect(vic) << 4) +
		(HDMI_COLORIMETRY_ITU709 << 6);
	buf[3] = 0x84;
	buf[4] = vic;
	switch (vic) {
	case HDMI_1440x480i60_16x9:
	case HDMI_1440x576i50_16x9:
		buf[5] = HDMI_PIXEL_REP_2;
		break;
	default:
		buf[5] = HDMI_PIXEL_REP_NO;
		break;
	}
	buf[0] = hdmi_checksum((unsigned char *)&header, buf, 28);
	hdmi_write_packet(header, buf, 28);
}

void hdmi_tx_audio_infoframe_packet(int channel, int freq)
{
	unsigned int header;
	unsigned char buf[28];

	memset(buf, 0x0, 28);
	header = HDMI_PACKET_INFOFRAME_AUDIO + (0x1 << 8) + (0x0a << 16);
	buf[1] = (channel - 1) + (HDMI_AUD_TYPE_REF_STM << 4);
	buf[2] = 0x0; /* HDMI_AUD_SAMPLE_24 + (freq << 2); */
	buf[3] = 0x00;
	/* 0x13: RRC RLC RR RL FC LFE FR FL
	    0x1F: FRC FLC RR RL FC LFE FR FL */
	buf[4] = (channel == 8) ? 0x13 : 0;
	buf[5] = 0x0; /* 0 db */
	buf[0] = hdmi_checksum((unsigned char *)&header, buf, 28);
	hdmi_write_packet(header, buf, 28);
}

void hdmi_tx_vendor_specific_infoframe_packet(void)
{
	unsigned int header;
	unsigned char buf[28];
	unsigned char structure_3d, meta_present;
	unsigned char hdmi_video_format;

	/* 0-No,1-1 byte param,2-3D format */
	hdmi_video_format = (g_vpp.hdmi_3d_type) ? 2 : 0;
	/* HDMI_3D_STRUCTURE_XXX; */
	structure_3d = (g_vpp.hdmi_3d_type == 1) ? 0 : g_vpp.hdmi_3d_type;
	meta_present = 0;

	memset(buf, 0x0, 28);
	header = HDMI_PACKET_INFOFRAME_VENDOR + (0x1 << 8) + (0xa << 16);
	buf[1] = 0x3;
	buf[2] = 0xC;
	buf[3] = 0x0;
	buf[4] = (hdmi_video_format << 5);
	buf[5] = (structure_3d << 4) + ((meta_present) ? 0x8 : 0x0);
	buf[6] = 0x0;	/* 3D_Ext_Data */
#if 0	/* metadata present */
	buf[7] = 0x0;	/* 3D_Metadata_type,3D_Metadata_Length(N) */
	buf[8] = 0x0;	/* 3D Metadata 1_N */
#endif
	buf[0] = hdmi_checksum((unsigned char *)&header, buf, 28);
	hdmi_write_packet(header, buf, 28);
}

#define HDMI_N_CTS_USE_TABLE
#ifdef HDMI_N_CTS_USE_TABLE
struct hdmi_n_cts_s {
	unsigned int n;
	unsigned int cts;
};

struct hdmi_n_cts_s hdmi_n_cts_table[7][11] = {
	/* 32kHz */
	{{ 9152, 84375 }, {4096, 37800}, {4096, 40500}, {8192, 81081},
	{ 4096, 81000 }, {4096, 81081}, {11648, 316406}, {4096, 111375},
	{ 11648, 632812}, {4096, 222750}, {4096, 0}},
	/* 44.1kHz */
	{{7007, 46875}, {6272, 42000}, {6272, 45000}, {6272, 45045},
	{6272, 90000}, {6272, 90090}, {17836, 351562}, {6272, 123750},
	{17836, 703125}, {6272, 247500}, {6272, 0}},
	/* 88.2kHz */
	{{14014, 46875}, {12544, 42000}, {12544, 45000}, {12544, 45045},
	{12544, 90000}, {12544, 90090}, {35672, 351562}, {12544, 123750},
	{35672, 703125}, {12544, 247500}, {12544, 0}},
	/* 176.4kHz */
	{{28028, 46875}, {25088, 42000}, {25088, 45000}, {25088, 45045},
	{25088, 90000}, {25088, 90090}, {71344, 351562}, {25088, 123750},
	{71344, 703125}, {25088, 247500}, {25088, 0}},
	/* 48kHz */
	{{9152, 56250}, {6144, 37800}, {6144, 40500}, {8192, 54054},
	{6144, 81000}, {6144, 81081}, {11648, 210937}, {6144, 111375},
	{11648, 421875}, {6144, 222750}, {6144, 0}},
	/* 96kHz */
	{{18304, 56250}, {12288, 37800}, {12288, 40500}, {16384, 54054},
	{12288, 81000}, {12288,81081}, {23296, 210937}, {12288, 111375},
	{23296, 421875}, {12288, 222750}, {12288, 0}},
	/* 192kHz */
	{{36608, 56250}, {24576, 37800}, {24576, 40500}, {32768, 54054},
	{24576, 81000}, {24576, 81081}, {46592, 210937}, {24576, 111375},
	{46592, 421875}, {24576, 222750}, {24576, 0}}
};

struct hdmi_n_cts_s *hdmi_get_n_cts(unsigned int tmds_clk,
					unsigned int freq)
{
	int i, j;

	switch (freq) {
	case 32000:
		i = 0;
		break;
	case 44100:
		i = 1;
		break;
	case 88200:
		i = 2;
		break;
	case 176400:
		i = 3;
		break;
	case 48000:
		i = 4;
		break;
	case 96000:
		i = 5;
		break;
	case 192000:
		i = 6;
		break;
	default:
		return 0;
	}

	switch (tmds_clk) {
	case 25174825:
		j = 0;
		break;
	case 25200000:
		j = 1;
		break;
	case 27000000:
		j = 2;
		break;
	case 27027000:
		j = 3;
		break;
	case 54000000:
		j = 4;
		break;
	case 54054000:
		j = 5;
		break;
	case 74175824:
		j = 6;
		break;
	case 74250000:
		j = 7;
		break;
	case 148351648:
		j = 8;
		break;
	case 148500000:
		j = 9;
		break;
	default:
		j = 10;
		break;
	}
	return &hdmi_n_cts_table[i][j];
}
#endif

void hdmi_set_audio_n_cts(unsigned int freq)
{
	unsigned int n = 0, cts = 0;

#ifdef HDMI_N_CTS_USE_TABLE
	struct hdmi_n_cts_s *p;

	p = hdmi_get_n_cts(g_vpp.hdmi_pixel_clock, freq);
	if (p) {
		n = p->n;
		cts = p->cts;
		MSG("[HDMI] use table n %d, cts %d\n", n, cts);
	}
#endif

	if (n == 0)
		n = 128 * freq / 1000;

	if (cts == 0) {
#ifdef __KERNEL__
		unsigned int tmp;
		unsigned int pll_clk;

		pll_clk = auto_pll_divisor(DEV_I2S, GET_FREQ, 0, 0);
		tmp = (inl(AUDREGF_BASE_ADDR + 0x70) & 0xF);

		switch (tmp) {
		case 0 ... 4:
			tmp = 0x01 << tmp;
			break;
		case 9 ... 12:
			tmp = 3 * (0x1 << (tmp-9));
			break;
		default:
			tmp = 1;
			break;
		}
		{
		unsigned long long tmp2;
		unsigned long long div2;
		unsigned long mod;

		tmp2 = g_vpp.hdmi_pixel_clock;
		tmp2 = tmp2 * n * tmp;
		div2 = pll_clk;
		mod = do_div(tmp2, div2);
		cts = tmp2;
		}
		DBGMSG("[HDMI] i2s %d,cts %d,reg 0x%x\n", pll_clk, cts,
			vppif_reg32_in(AUDREGF_BASE_ADDR + 0x70));
#else
		cts = (g_vpp.hdmi_pixel_clock / 1000) - 1;
#endif
	}
	hdmi_regs1->aud_sample_rate1.b.n_20bits = n;
	hdmi_regs1->aud_ratio.b.acr_ratio = cts - 1;
	hdmi_regs1->aud_sample_rate2.b.cts_select = 0;
	cts = 0;
	hdmi_regs1->aud_sample_rate1.b.cts_low_12bits = cts & 0xFFF;
	hdmi_regs1->aud_sample_rate2.b.cts_hi_8bits = (cts & 0xFF000) >> 12;
	DBGMSG("[HDMI] set audio freq %d,n %d,cts %d,tmds %d\n",
				freq, n, cts, g_vpp.hdmi_pixel_clock);
}

void hdmi_config_audio(struct vout_audio_t *info)
{
	unsigned int freq;

	g_vpp.hdmi_audio_channel = info->channel;
	g_vpp.hdmi_audio_freq = info->sample_rate;

	/* enable ARF & ARFP clock */
	REG32_VAL(PM_CTRL_BASE_ADDR + 0x254) |= (BIT4 | BIT3);
	hdmi_tx_audio_infoframe_packet(info->channel, info->sample_rate);
	hdmi_audio_enable(VPP_FLAG_DISABLE);
	hdmi_regs1->aud_mode.b.layout = (info->channel > 2) ? 1 : 0;
	hdmi_regs1->aud_mode.b._2ch_eco = (info->channel > 2) ? 0 : 1;
	switch (info->sample_rate) {
	case 32000:
		freq = 0x3;
		break;
	case 44100:
		freq = 0x0;
		break;
	case 88200:
		freq = 0x8;
		break;
	case 176400:
		freq = 0xC;
		break;
	default:
	case 48000:
		freq = 0x2;
		break;
	case 96000:
		freq = 0xA;
		break;
	case 192000:
		freq = 0xE;
		break;
	case 768000:
		freq = 0x9;
		break;
	}
	hdmi_regs1->aud_chan_status0 = (freq << 24) + 0x4;
	hdmi_regs1->aud_chan_status1 = 0x0;
	hdmi_regs1->aud_chan_status2 = 0xb;
	hdmi_regs1->aud_chan_status3 = 0x0;
	hdmi_regs1->aud_chan_status4 = 0x0;
	hdmi_regs1->aud_chan_status5 = 0x0;

	hdmi_set_audio_n_cts(info->sample_rate);
	hdmi_regs1->aud_ratio.b.acr_enable = 1;
	hdmi_regs1->aud_sample_rate2.b.aipclk_rate = 0;
	hdmi_audio_enable(VPP_FLAG_ENABLE);
}

void hdmi_config_video(struct hdmi_info_t *info)
{
	hdmi_set_output_colfmt(info->outfmt);
	hdmi_tx_avi_infoframe_packet(info->outfmt, info->vic);
	hdmi_tx_vendor_specific_infoframe_packet();
}

void hdmi_set_option(unsigned int option)
{
	vdo_color_fmt colfmt;
	int temp;

	hdmi_set_dvi_enable((option & EDID_OPT_HDMI) ?
		VPP_FLAG_DISABLE : VPP_FLAG_ENABLE);
	hdmi_audio_enable((option & EDID_OPT_AUDIO) ?
		VPP_FLAG_ENABLE : VPP_FLAG_DISABLE);

	colfmt = hdmi_get_output_colfmt();
	switch (colfmt) {
	case VDO_COL_FMT_YUV422H:
		temp = option & EDID_OPT_YUV422;
		break;
	case VDO_COL_FMT_YUV444:
		temp = option & EDID_OPT_YUV444;
		break;
	default:
		temp = 1;
		break;
	}
	if (temp == 0) {
		hdmi_set_output_colfmt(VDO_COL_FMT_ARGB);
		DBG_MSG("[HDMI] TV not support %s,use default RGB\n",
			vpp_colfmt_str[colfmt]);
	}
	DBG_MSG("[HDMI] set option(8-HDMI,6-AUDIO) 0x%x\n", option);
}

void hdmi_config(struct hdmi_info_t *info)
{
	struct vout_audio_t audio_info;
	int h_porch;
	int delay_cfg;
	vpp_clock_t clock;

	hdmi_regs1->ctrl.b.hden = 0;
	hdmi_regs1->infoframe_ctrl.b.select = 0;
	hdmi_regs1->infoframe_ctrl.b.fifo1_rdy = 0;
	hdmi_config_video(info);

	govrh_get_tg(p_govrh, &clock);
	h_porch = clock.total_pixel_of_line - clock.end_pixel_of_active; /*fp*/
	delay_cfg = 47 - h_porch;
	if (delay_cfg <= 0)
		delay_cfg = 1;
	h_porch = clock.begin_pixel_of_active;	/* bp */
	h_porch = (h_porch - (delay_cfg + 1) - 26) / 32;
	if (h_porch <= 0)
		h_porch = 1;
	if (h_porch >= 8)
		h_porch = 0;

	hdmi_regs1->general_ctrl.b.cp_delay = delay_cfg;
	hdmi_regs1->general_ctrl.b.vsync_enable = 1; /* for write only */
	hdmi_regs1->infoframe_ctrl.b.horiz_blank_max_pck = h_porch;
	DBGMSG("[HDMI] H blank max pck %d,delay %d\n", h_porch, delay_cfg);

	audio_info.fmt = 16;
	audio_info.channel = info->channel;
	audio_info.sample_rate = info->freq;
	hdmi_config_audio(&audio_info);

	hdmi_regs1->infoframe_ctrl.b.fifo1_addr = 0;
	hdmi_regs1->infoframe_ctrl.b.fifo1_len = 2;
	hdmi_regs1->infoframe_ctrl.b.fifo1_rdy = 1;
	hdmi_set_option(info->option);
	hdmi_regs2->test.b.tre_en =
		(g_vpp.hdmi_pixel_clock < 40000000) ? 3 : 2;
}

/*----------------------- Module API --------------------------------------*/
void hdmi_set_cp_enable(vpp_flag_t enable)
{
	if (!g_vpp.hdmi_cp_enable)
		enable = 0;

	if (hdmi_cp)
		hdmi_cp->enable(enable);

#ifdef __KERNEL__
	if (hdmi_cp && hdmi_cp->poll) {
		vpp_irqproc_del_work(VPP_INT_GOVRH_VBIS, (void *)hdmi_cp->poll);
		if (enable)
			vpp_irqproc_work(VPP_INT_GOVRH_VBIS,
				(void *)hdmi_cp->poll, 0, 0, 0);
	}
#endif
}

int hdmi_check_cp_int(void)
{
	int ret = 0;

	if (hdmi_cp)
		ret = hdmi_cp->interrupt();
	return ret;
}

void hdmi_get_bksv(unsigned int *bksv)
{
	if (hdmi_cp)
		hdmi_cp->get_bksv(bksv);
}

#ifdef __KERNEL__
void hdmi_hotplug_notify(int plug_status)
{
	if (g_vpp.hdmi_disable)
		return;
	vpp_netlink_notify_plug(VPP_VOUT_NUM_HDMI, plug_status);
}
#else
#define hdmi_hotplug_notify
#endif

int hdmi_check_plugin(int hotplug)
{
	static int last_plugin = -1;
	int plugin;
	int flag;

	if (g_vpp.hdmi_disable)
		return 0;

	plugin = hdmi_get_plugin();
	hdmi_clear_plug_status();
#ifdef __KERNEL__
	/* disable HDMI before change clock */
	if (plugin == 0) {
		hdmi_set_enable(0);
		hdmi_set_power_down(1);
	}
	vpp_set_clock_enable(DEV_HDMII2C, plugin, 1);
	vpp_set_clock_enable(DEV_HDCE, plugin, 1);

	/* slow down clock for plugout */
	flag = (auto_pll_divisor(DEV_HDMILVDS, GET_FREQ, 0, 0)
			== 8000000) ? 0 : 1;
	if ((plugin != flag) && !g_vpp.virtual_display) {
		int pixclk;

		pixclk = (plugin) ? g_vpp.hdmi_pixel_clock : 8000000;
		auto_pll_divisor(DEV_HDMILVDS, SET_PLLDIV, 0, pixclk);
	}
#endif
	if (last_plugin != plugin) {
		DPRINT("[HDMI] HDMI plug%s,hotplug %d\n", (plugin) ?
						"in" : "out", hotplug);
		last_plugin = plugin;
	}
#if 0	/* Denzel test */
	if (plugin == 0)
		hdmi_set_dvi_enable(VPP_FLAG_ENABLE);
#endif
	return plugin;
}

void hdmi_reg_dump(void)
{
	DPRINT("========== HDMI register dump ==========\n");
	vpp_reg_dump(REG_HDMI_BEGIN, REG_HDMI_END - REG_HDMI_BEGIN);
	vpp_reg_dump(REG_HDMI2_BEGIN, REG_HDMI2_END - REG_HDMI2_BEGIN);

	DPRINT("---------- HDMI common ----------\n");
	DPRINT("enable %d,hden %d,reset %d,dvi %d\n",
		hdmi_regs1->general_ctrl.b.enable,
		hdmi_regs1->ctrl.b.hden,
		hdmi_regs1->general_ctrl.b.reset,
		hdmi_regs1->general_ctrl.b.dvi_mode_enable);
	DPRINT("colfmt %d,conv 422 %d,hsync low %d,vsync low %d\n",
		hdmi_regs1->general_ctrl.b.output_format,
		hdmi_regs1->general_ctrl.b.convert_yuv422,
		hdmi_regs1->general_ctrl.b.hsync_low_active,
		hdmi_regs1->general_ctrl.b.vsync_low_active);
	DPRINT("dbg bus sel %d,state mach %d\n",
		hdmi_regs1->general_ctrl.b.dbg_bus_select,
		hdmi_regs1->general_ctrl.b.state_machine_status);
	DPRINT("eep reset %d,encode %d,eess %d\n",
		hdmi_regs1->ctrl.b.eeprom_reset,
		hdmi_regs1->ctrl.b.encode_enable,
		hdmi_regs1->ctrl.b.eess_enable);
	DPRINT("verify pj %d,auth test %d,cipher %d\n",
		hdmi_regs1->ctrl.b.verify_pj_enable,
		hdmi_regs1->ctrl.b.auth_test_key,
		hdmi_regs1->ctrl.b.cipher_1_1);
	DPRINT("preamble %d\n", hdmi_regs1->ctrl.b.preamble);

	DPRINT("---------- HDMI hotplug ----------\n");
	DPRINT("plug %s\n", (hdmi_regs1->hotplug_detect.b.sts) ? "in" : "out");
	DPRINT("plug in enable %d, status %d\n",
		hdmi_regs1->hotplug_detect.b.in_enable,
		hdmi_regs1->hotplug_detect.b.in_sts);
	DPRINT("plug out enable %d, status %d\n",
		hdmi_regs1->hotplug_detect.b.out_enable,
		hdmi_regs1->hotplug_detect.b.out_sts);
	DPRINT("debounce detect %d,sample %d\n",
		hdmi_regs1->hotplug_debounce.b.detect,
		hdmi_regs1->hotplug_debounce.b.sample);

	DPRINT("---------- I2C ----------\n");
	DPRINT("enable %d,exit FSM %d,key read %d\n",
		hdmi_regs1->ctrl.b.i2c_enable,
		hdmi_regs1->i2c_ctrl.b.force_exit_fsm,
		hdmi_regs1->i2c_ctrl.b.key_read_word);
	DPRINT("clk divid %d,rd data 0x%x,wr data 0x%x\n",
		hdmi_regs1->i2c_ctrl.b.i2c_clk_divider,
		hdmi_regs1->i2c_ctrl2.b.rd_data,
		hdmi_regs1->i2c_ctrl2.b.wr_data);
	DPRINT("start %d,stop %d,wr avail %d\n",
		hdmi_regs1->i2c_ctrl2.b.sw_start_req,
		hdmi_regs1->i2c_ctrl2.b.sw_stop_req,
		hdmi_regs1->i2c_ctrl2.b.wr_data_avail);
	DPRINT("status %d,sw read %d,sw i2c req %d\n",
		hdmi_regs1->i2c_ctrl2.b.i2c_status,
		hdmi_regs1->i2c_ctrl2.b.sw_read,
		hdmi_regs1->i2c_ctrl2.b.sw_i2c_req);

	DPRINT("---------- AUDIO ----------\n");
	DPRINT("enable %d,sub pck %d,spflat %d\n",
		hdmi_regs1->aud_enable,
		hdmi_regs1->aud_mode.b.sub_packet,
		hdmi_regs1->aud_mode.b.spflat);
	DPRINT("aud pck insert reset %d,enable %d,delay %d\n",
		hdmi_regs1->aud_insert_ctrl.b.pck_insert_reset,
		hdmi_regs1->aud_insert_ctrl.b.pck_insert_enable,
		hdmi_regs1->aud_insert_ctrl.b.insert_delay);
	DPRINT("avmute set %d,clr %d,pixel repete %d\n",
		hdmi_regs1->aud_insert_ctrl.b.avmute_set_enable,
		hdmi_regs1->aud_insert_ctrl.b.avmute_clr_enable,
		hdmi_regs1->aud_insert_ctrl.b.pixel_repetition);
	DPRINT("acr ratio %d,acr enable %d,mute %d\n",
		hdmi_regs1->aud_ratio.b.acr_ratio,
		hdmi_regs1->aud_ratio.b.acr_enable,
		hdmi_regs1->aud_ratio.b.mute);
	DPRINT("layout %d,pwr save %d,n 20bits %d\n",
		hdmi_regs1->aud_mode.b.layout,
		hdmi_regs1->aud_mode.b.pwr_saving,
		hdmi_regs1->aud_sample_rate1.b.n_20bits);
	DPRINT("cts low 12 %d,hi 8 %d,cts sel %d\n",
		hdmi_regs1->aud_sample_rate1.b.cts_low_12bits,
		hdmi_regs1->aud_sample_rate2.b.cts_hi_8bits,
		hdmi_regs1->aud_sample_rate2.b.cts_select);
	DPRINT("aipclk rate %d\n", hdmi_regs1->aud_sample_rate2.b.aipclk_rate);

	DPRINT("---------- INFOFRAME ----------\n");
	DPRINT("sel %d,hor blank pck %d\n",
		hdmi_regs1->infoframe_ctrl.b.select,
		hdmi_regs1->infoframe_ctrl.b.horiz_blank_max_pck);
	DPRINT("fifo1 ready %d,addr 0x%x,len %d\n",
		hdmi_regs1->infoframe_ctrl.b.fifo1_rdy,
		hdmi_regs1->infoframe_ctrl.b.fifo1_addr,
		hdmi_regs1->infoframe_ctrl.b.fifo1_len);
	DPRINT("fifo2 ready %d,addr 0x%x,len %d\n",
		hdmi_regs1->infoframe_ctrl.b.fifo2_rdy,
		hdmi_regs1->infoframe_ctrl.b.fifo2_addr,
		hdmi_regs1->infoframe_ctrl.b.fifo2_len);
	DPRINT("wr strobe %d,rd strobe %d,fifo addr %d\n",
		hdmi_regs1->fifo_ctrl.b.wr_strobe,
		hdmi_regs1->fifo_ctrl.b.rd_strobe,
		hdmi_regs1->fifo_ctrl.b.addr);

	{
	int i;
	unsigned int buf[32];

	for (i = 0; i <= hdmi_regs1->infoframe_ctrl.b.fifo1_len; i++) {
		DPRINT("----- infoframe %d -----\n", i);
		hdmi_read_fifo(i, buf, 32);
		vpp_reg_dump((unsigned int) buf, 32);
	}
	}

	DPRINT("---------- HDMI test ----------\n");
	DPRINT("ch0 enable %d, data 0x%x\n",
		hdmi_regs1->channel_test.b.ch0_enable,
		hdmi_regs1->channel_test.b.ch0_data);
	DPRINT("ch1 enable %d, data 0x%x\n",
		hdmi_regs1->channel_test.b.ch1_enable,
		hdmi_regs1->channel_test.b.ch1_data);
	DPRINT("ch2 enable %d, data 0x%x\n",
		hdmi_regs1->hotplug_detect.b.ch2_enable,
		hdmi_regs1->hotplug_detect.b.ch2_data);
	if (hdmi_cp)
		hdmi_cp->dump();
}

#ifdef CONFIG_PM
static unsigned int *hdmi_pm_bk;
static unsigned int *hdmi_pm_bk2;
static unsigned int hdmi_pm_enable;
static unsigned int hdmi_pm_enable2;
static int hdmi_plug_enable = 0xFF;
static int hdmi_resume_plug_cnt;
#define HDMI_RESUME_PLUG_MS	50
#define HDMI_RESUME_PLUG_CNT	20
static void hdmi_do_resume_plug(struct work_struct *ptr)
{
	struct vout_t *vo;
	int plugin;
	struct delayed_work *dwork = to_delayed_work(ptr);

	plugin = hdmi_check_plugin(0);
	vo = vout_get_entry(VPP_VOUT_NUM_HDMI);
	vout_change_status(vo, VPP_VOUT_STS_PLUGIN, plugin);
	if (plugin)
		hdmi_hotplug_notify(1);
	hdmi_resume_plug_cnt--;
	if (hdmi_resume_plug_cnt && (vpp_sdev.state == 0))
		schedule_delayed_work(dwork,
			msecs_to_jiffies(HDMI_RESUME_PLUG_MS));
}

DECLARE_DELAYED_WORK(hdmi_resume_work, hdmi_do_resume_plug);

void hdmi_suspend(int sts)
{
	vo_hdmi_set_clock(1);
	switch (sts) {
	case 0:	/* disable module */
		cancel_delayed_work_sync(&hdmi_resume_work);
		hdmi_pm_enable = hdmi_regs1->general_ctrl.b.enable;
		hdmi_regs1->general_ctrl.b.enable = 0;
		hdmi_regs1->general_ctrl.b.vsync_enable = 1; /* for wr only */
		hdmi_pm_enable2 = hdmi_regs1->ctrl.b.hden;
		hdmi_regs1->ctrl.b.hden = 0;
		if (hdmi_plug_enable == 0xFF)
			hdmi_plug_enable =
				hdmi_regs1->hotplug_detect.b.out_enable;
		hdmi_enable_plugin(0);
		break;
	case 1: /* disable tg */
		break;
	case 2:	/* backup register */
		hdmi_pm_bk = vpp_backup_reg(REG_HDMI_BEGIN,
			(REG_HDMI_END - REG_HDMI_BEGIN));
		hdmi_pm_bk2 = vpp_backup_reg(REG_HDMI2_BEGIN,
			(REG_HDMI2_END - REG_HDMI2_BEGIN));
		hdmi_resume_plug_cnt = 20;
		break;
	default:
		break;
	}
	vo_hdmi_set_clock(0);
}

void hdmi_resume(int sts)
{
	vo_hdmi_set_clock(1);
	switch (sts) {
	case 0:	/* restore register */
		switch_set_state(&vpp_sdev, 0);		
		vpp_restore_reg(REG_HDMI_BEGIN,
			(REG_HDMI_END - REG_HDMI_BEGIN), hdmi_pm_bk);
		vpp_restore_reg(REG_HDMI2_BEGIN,
			(REG_HDMI2_END - REG_HDMI2_BEGIN), hdmi_pm_bk2);
		hdmi_pm_bk = 0;
		hdmi_pm_bk2 = 0;
		hdmi_config(&hdmi_info); /* re-config HDMI info frame */
		if (g_vpp.hdmi_cp_p && hdmi_cp)
			hdmi_cp->init();
		break;
	case 1:	/* enable module */
		hdmi_regs1->general_ctrl.b.enable = hdmi_pm_enable;
		hdmi_regs1->general_ctrl.b.vsync_enable = 1; /* for wr only */
		hdmi_regs1->ctrl.b.hden = hdmi_pm_enable2;
		break;
	case 2: /* enable tg */
		hdmi_check_plugin(0);
		hdmi_clear_plug_status();
		hdmi_enable_plugin(hdmi_plug_enable);
		hdmi_plug_enable = 0xFF;
		if (vpp_sdev.state == 0) {
			hdmi_resume_plug_cnt = HDMI_RESUME_PLUG_CNT;
			schedule_delayed_work(&hdmi_resume_work,
				msecs_to_jiffies(HDMI_RESUME_PLUG_MS));
		}
		break;
	default:
		break;
	}
	vo_hdmi_set_clock(0);
}
#else
#define hdmi_suspend NULL
#define hdmi_resume NULL
#endif

void hdmi_init(void)
{
	struct fb_videomode vmode;

	g_vpp.hdmi_pixel_clock = vpp_get_base_clock(VPP_MOD_GOVRH);
	g_vpp.hdmi_i2c_freq = HDMI_I2C_FREQ;
	g_vpp.hdmi_i2c_udelay = 0;
	g_vpp.hdmi_ctrl = 0x1000000;
	g_vpp.hdmi_audio_pb4 = 0x0;
	g_vpp.hdmi_audio_pb1 = 0x0;

	hdmi_info.outfmt = hdmi_get_output_colfmt();
	govrh_get_videomode(p_govrh, &vmode);
	hdmi_info.vic = hdmi_get_vic(vmode.xres, vmode.yres, vmode.refresh,
		(vmode.vmode & FB_VMODE_INTERLACED) ? 1 : 0);
	hdmi_info.channel = 2;
	hdmi_info.freq = 48000;
	hdmi_info.option = EDID_OPT_AUDIO + EDID_OPT_HDMI;

	hdmi_enable_plugin(0);

	if (g_vpp.govrh_preinit) {
		DBGMSG("[HDMI] hdmi_init for uboot logo\n");
	} else {
		/* bit8-HDMI SDA,bit9-HDMI SCL,bit10-Hotplug,bit26-CEC */
		/* GPIO disable GPIO function */
		vppif_reg32_write(GPIO_BASE_ADDR+0x54, 0x4000700, 0, 0);
		/* GPIO4 disable GPIO out */
		vppif_reg32_write(GPIO_BASE_ADDR+0x494, 0x4000700, 0, 0);
#if 0
		/* Suspend GPIO output enable */
		vppif_reg32_write(GPIO_BASE_ADDR+0x80, BIT23, 23, 1);
		/* Suspend GPIO output high */
		vppif_reg32_write(GPIO_BASE_ADDR+0xC0, BIT23, 23, 1);
		/* Wake3 disable pull ctrl */
		vppif_reg32_write(GPIO_BASE_ADDR+0x480, BIT19, 19, 0);
#endif
		hdmi_regs2->level.b.level = 1;
		hdmi_regs2->level.b.update = 1;
		hdmi_regs2->igs.b.ldi_shift_left = 1;
		hdmi_regs2->status.val = 0x0008c000;
		hdmi_regs2->test.val = 0x00450409;
		hdmi_regs2->test2.val = 0x00005022;
		hdmi_regs2->test3 = (g_vpp.hdmi_sp_mode) ?
					0x00010100 : 0x00000100;
		hdmi_set_enable(VPP_FLAG_DISABLE);
		hdmi_set_dvi_enable(VPP_FLAG_DISABLE);
		hdmi_regs1->ctrl.b.cipher_1_1 = 0;

		hdmi_regs1->tmds_ctrl.b.infoframe_sram_enable = 1;
		hdmi_regs1->infoframe_ctrl.b.select = 0;
		hdmi_regs1->infoframe_ctrl.b.fifo1_rdy = 0;

		hdmi_regs1->hotplug_detect.val = 0x0;
		hdmi_regs1->channel_test.val = 0x1;

		hdmi_DDC_reset();
		hdmi_DDC_set_freq(g_vpp.hdmi_i2c_freq);
		hdmi_regs1->ctrl.b.i2c_enable = 1;
	}
	g_vpp.hdmi_init = 1;
	if (hdmi_cp)
		hdmi_cp->init();
}
#endif /* WMT_FTBLK_HDMI */
