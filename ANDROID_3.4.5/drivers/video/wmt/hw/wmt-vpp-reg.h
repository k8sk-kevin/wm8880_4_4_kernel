/*++
 * linux/drivers/video/wmt/hw/wmt-vpp-reg.h
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

#ifndef WMT_VPP_REG_H
#define WMT_VPP_REG_H

#define VPP_DAC_SEL_TV		1
#define VPP_DAC_SEL_VGA		0

struct vppm_base_regs {
	unsigned int _00; /* 0x00 */

	union {
		unsigned int val;
		struct {
			unsigned int _0_7:8;
			unsigned int govrh_pvbi:1;
			unsigned int govrh_vbis:1;
			unsigned int govrh_vbie:1;
			unsigned int _11:1;
			unsigned int govrh2_pvbi:1;
			unsigned int govrh2_vbis:1;
			unsigned int govrh2_vbie:1;
			unsigned int _15:1;
			unsigned int scl_pvbi:1;
			unsigned int scl_vbis:1;
			unsigned int scl_vbie:1;
			unsigned int _19:1;
			unsigned int ge_tg:1;
		} b;
	} int_sts; /* 0x4 */

	union {
		unsigned int val;
		struct {
			unsigned int _0_7:8;
			unsigned int govrh_pvbi:1;
			unsigned int govrh_vbis:1;
			unsigned int govrh_vbie:1;
			unsigned int _11:1;
			unsigned int govrh2_pvbi:1;
			unsigned int govrh2_vbis:1;
			unsigned int govrh2_vbie:1;
			unsigned int _15:1;
			unsigned int scl_pvbi:1;
			unsigned int scl_vbis:1;
			unsigned int scl_vbie:1;
			unsigned int _19:1;
			unsigned int ge_tg:1;
		} b;
	} int_en; /* 0x8 */

	unsigned int watch_sel; /* 0x0C */

	union {
		unsigned int val;
		struct {
			unsigned int scl:1;
			unsigned int _1_7:7;
			unsigned int vid:1;
			unsigned int _9_15:7;
			unsigned int ge:1;
		} b;
	} sw_reset1; /* 0x10 */

	union {
		unsigned int val;
		struct {
			unsigned int govrh:1;
			unsigned int _1_3:3;
			unsigned int lvds:1;
			unsigned int _5_7:3;
			unsigned int dvo:1;
			unsigned int dvo2:1;
			unsigned int _10_11:2;
			unsigned int cec:1;
		} b;
	} sw_reset2; /* 0x14 */

	unsigned int dac_sel; /* 0x18 */

	union {
		unsigned int val;
		struct {
			unsigned int hdmi:1;
			unsigned int _1_7:7;
			unsigned int ddc:1;
			unsigned int _9_15:7;
			unsigned int hdmi2:1;
		} b;
	} sw_reset3; /* 0x1C */

	union {
		unsigned int val;
		struct {
			unsigned int disable:1;
			unsigned int _1_7:7;
			unsigned int csi_act_lane_sel:1; /*0-lane 0/1,1-2/3*/
		} b;
	} sscg; /* 0x20 */
};

#define REG_VPP_BEGIN		(VPP_BASE_ADDR + 0x00)
#define REG_VPP_END		(VPP_BASE_ADDR + 0x28)

#ifndef VPPM_C
extern HW_REG struct vppm_base_regs *vppm_regs;
#endif


#endif /* WMT_VPP_REG_H */

