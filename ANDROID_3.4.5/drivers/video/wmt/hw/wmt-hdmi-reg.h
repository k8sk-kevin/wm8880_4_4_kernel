/*++
 * linux/drivers/video/wmt/hw/wmt-hdmi-reg.h
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

#ifndef WMT_HDMI_REG_H
#define WMT_HDMI_REG_H

#define WMT_FTBLK_HDMI

#define HDMI_BASE_ADDR	(HDMI_TRANSMITTE_BASE_ADDR + 0xC000)

struct hdmi_base1_regs {
	unsigned int _100_11c[8];

	union {
		unsigned int val;
		struct {
			unsigned int eeprom_reset:1;
			unsigned int encode_enable:1;
			unsigned int hden:1;
			unsigned int eess_enable:1;
			unsigned int verify_pj_enable:1;
			unsigned int i2c_enable:1;
			unsigned int auth_test_key:1;
			unsigned int _7:1;
			unsigned int cipher_1_1:1;
			unsigned int _9_11:3;
			unsigned int preamble:4;
			unsigned int _16_19:4;
			unsigned int encode_window:3;
		} b;
	} ctrl; /* 0x120 */

	union {
		unsigned int val;
		struct {
			unsigned int _0_6:7;
			unsigned int force_exit_fsm:1;
			unsigned int key_read_word:7;
			unsigned int i2c_sw_reset:1;
			unsigned int i2c_clk_divider:16;
		} b;
	} i2c_ctrl; /* 0x124 */

	union {
		unsigned int val;
		struct {
			unsigned int wr_data:8;
			unsigned int rd_data:8;
			unsigned int sw_start_req:1;
			unsigned int sw_stop_req:1;
			unsigned int wr_data_avail:1;
			unsigned int i2c_status:1; /* 0-not using,1-in using */
			unsigned int cp_key_req:1;
			unsigned int cp_key_read:1;
			unsigned int cp_key_last:1;
			unsigned int _23:1;
			unsigned int cp_src_sel:1;
			unsigned int sw_read:1;
			unsigned int sw_i2c_req:1;
			unsigned int ksv_list_avail:1;
			unsigned int ksv_verify_done:1;
		} b;
	} i2c_ctrl2; /* 0x128 */

	unsigned int _12c_27c[85];

	union {
		unsigned int val;
		struct {
			unsigned int reset:1;
			unsigned int enable:1;
			unsigned int _2_5:4;
			unsigned int dvi_mode_enable:1;
			unsigned int output_format:2; /* 0-RGB,
							1-YUV444,2-YUV422 */
			unsigned int convert_yuv422:1;
			unsigned int hsync_low_active:1; /* 0-active hi,1-lo */
			unsigned int dbg_bus_select:1; /* 0-before,1-after */
			unsigned int _12:1;
			unsigned int vsync_low_active:1; /* 0-active hi,1-lo */
			unsigned int _14_15:2;
			unsigned int cp_delay:7;
			unsigned int _23:1;
			unsigned int vsync_enable:3; /* write only */
			unsigned int state_machine_status:5;
		} b;
	} general_ctrl; /* 0x280 */

	union {
		unsigned int val;
		struct {
			unsigned int select:1; /* 0-fifo1,1-fifo2 */
			unsigned int fifo1_rdy:1; /* Info frame FIFO 1 ready */
			unsigned int fifo2_rdy:1; /* Info frame FIFO 2 ready */
			unsigned int _3:1;
			unsigned int fifo1_addr:4; /* FIFO 1 start address */
			unsigned int fifo1_len:5; /* FIFO 1 length */
			unsigned int _13_15:3;
			unsigned int fifo2_addr:4; /* FIFO 2 start address */
			unsigned int fifo2_len:5; /* FIFO 2 length */
			unsigned int _25_27:3;
			unsigned int horiz_blank_max_pck:3; /* Max packets
						that insert during HSYNC */
		} b;
	} infoframe_ctrl; /* 0x284 */
	unsigned int _288_290[3];

	union {
		unsigned int val;
		struct {
			unsigned int pck_insert_reset:1;
			unsigned int pck_insert_enable:1;
			unsigned int avmute_set_enable:1;
			unsigned int avmute_clr_enable:1;
			unsigned int insert_delay:12;
			unsigned int _16_29:14;
			unsigned int pixel_repetition:2; /* 0-none,1-2x,2-4x */
		} b;
	} aud_insert_ctrl; /* 0x294 */

	unsigned int _298;

	union {
		unsigned int val;
		struct {
			unsigned int _0_7:8;
			unsigned int acr_ratio:20;
			unsigned int acr_enable:1;
			unsigned int mute:1;
		} b;
	} aud_ratio; /* 0x29c */

	unsigned int aud_enable; /* 0x2a0 */
	unsigned int _2a4_2a8[2];

	union {
		unsigned int val;
		struct {
			unsigned int sub_packet:4;
			unsigned int spflat:4;
			unsigned int _2ch_eco:1;
			unsigned int _9:1;
			unsigned int layout:1; /* 0-2 channel,1-8 channel */
			unsigned int pwr_saving:1; /* 0-normal,1-power saving */
		} b;
	} aud_mode; /* 0x2ac */

	unsigned int _2b0_38c[56];
	unsigned int aud_chan_status0; /* 0x390 */
	unsigned int aud_chan_status1; /* 0x394 */
	unsigned int aud_chan_status2; /* 0x398 */
	unsigned int aud_chan_status3; /* 0x39c */
	unsigned int aud_chan_status4; /* 0x3a0 */
	unsigned int aud_chan_status5; /* 0x3a4 */

	union {
		unsigned int val;
		struct {
			unsigned int n_20bits:20;
			unsigned int cts_low_12bits:12;
		} b;
	} aud_sample_rate1; /* 0x3a8 */

	union {
		unsigned int val;
		struct {
			unsigned int cts_hi_8bits:8;
			unsigned int _8_27:20;
			unsigned int aipclk_rate:2; /* 0-N/2,1-N,2-N/4,3-N*2 */
			unsigned int cts_select:1; /* 0-auto,1-fixed from reg */
		} b;
	} aud_sample_rate2; /* 0x3ac */

	unsigned int _3b0_3bc[4];
	unsigned int wr_fifo_addr[9]; /* 0x3c0 - 0x3e0 */

	union {
		unsigned int val;
		struct {
			unsigned int wr_strobe:1;
			unsigned int rd_strobe:1;
			unsigned int _2_7:6;
			unsigned int addr:8;
		} b;
	} fifo_ctrl; /* 0x3e4 */

	union {
		unsigned int val;
		struct {
			unsigned int ch0_data:10;
			unsigned int ch0_enable:1;
			unsigned int _11_15:5;
			unsigned int ch1_data:10;
			unsigned int ch1_enable:1;
		} b;
	} channel_test; /* 0x3e8 */

	union {
		unsigned int val;
		struct {
			unsigned int ch2_data:10;
			unsigned int ch2_enable:1;
			unsigned int _11_15:5;
			unsigned int in_enable:1;
			unsigned int out_enable:1;
			unsigned int _18_23:6;
			unsigned int in_sts:1;
			unsigned int out_sts:1;
			unsigned int _26_30:5;
			unsigned int sts:1; /* 0-plug out,1-plug in */
		} b;
	} hotplug_detect; /* 0x3ec */

	union {
		unsigned int val;
		struct {
			unsigned int sample:8;
			unsigned int _8_15:8;
			unsigned int detect:9;
		} b;
	} hotplug_debounce; /* 0x3f0 */

	unsigned int _3f4;

	union {
		unsigned int val;
		struct {
			unsigned int test_enable:1;
			unsigned int test_format:1;
			unsigned int _2_9:8;
			unsigned int infoframe_sram_enable:1;
			unsigned int _11_15:5;
			unsigned int clock_select:1; /* 0-clk 1x, 1-clk 2x */
		} b;
	} tmds_ctrl; /* 0x3f8 */

	unsigned int _3fc;
	unsigned int rd_fifo_addr[9]; /* 0x400 - 0x420 */
};

struct hdmi_base2_regs {
	union {
		unsigned int val;
		struct {
			unsigned int inv_clk:1;
			unsigned int _1_3:3;
			unsigned int dual_channel:1;
			unsigned int _5_7:3;
			unsigned int test:4;
			unsigned int _12_18:7;
			unsigned int internal_ldo:1;
		} b;
	} status; /* 0x00 */

	union {
		unsigned int val;
		struct {
			unsigned int drv_pdmode:1;
			unsigned int _1:1;
			unsigned int vbg_sel:2;
			unsigned int _4_7:4;
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
			unsigned int _1_7:7;
			unsigned int level:1;
		} b;
	} level; /* 0x08 */

	union {
		unsigned int val;
		struct {
			unsigned int bpp_type:3; /* 0-888,1-555,2-666,3-565 */
			unsigned int _3_7:5;
			unsigned int ldi_shift_left:1; /* 0-right,1-left */
		} b;
	} igs; /* 0x0c */

	union {
		unsigned int val;
		struct {
			unsigned int out_data_12:1; /* 0-24bit,1-12bit */
			unsigned int hsync_polar_lo:1; /* 0-act hi,1-act low */
			unsigned int dvo_enable:1;
			unsigned int vsync_polar_lo:1; /* 0-act hi,1-act low */
		} b;
	} set; /* 0x10 */

	union {
		unsigned int val;
		struct {
			unsigned int colfmt_rgb:1;/* 0-RGB or YUV444,1-YUV422 */
			unsigned int colfmt_yuv422:1;
		} b;
	} set2; /* 0x14 */

	union {
		unsigned int val;
		struct {
			unsigned int pll_ready:1;
			unsigned int _1_7:7;
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
			unsigned int _7:1;
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

	unsigned int test3; /* 0x20 */

	union {
		unsigned int val;
		struct {
			unsigned int _0_15:16;
			unsigned int reset_pll:1;
		} b;
	} dftset2; /* 0x24 */
};

#define REG_HDMI_BEGIN	(HDMI_BASE_ADDR + 0x100)
#define REG_HDMI_END	(HDMI_BASE_ADDR + 0x420)
#define REG_HDMI2_BEGIN	(HDMI_BASE2_ADDR + 0x00)
#define REG_HDMI2_END	(HDMI_BASE2_ADDR + 0x28)

#ifndef HDMI_C
extern HW_REG struct hdmi_base1_regs *hdmi_regs1;
extern HW_REG struct hdmi_base2_regs *hdmi_regs2;
#endif
#endif /* WMT_HDMI_REG_H */

