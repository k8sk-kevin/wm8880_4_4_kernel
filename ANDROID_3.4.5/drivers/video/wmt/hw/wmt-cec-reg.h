/*++
 * linux/drivers/video/wmt/hw/wmt-cec-reg.h
 * WonderMedia video post processor (VPP) driver
 *
 * Copyright c 2013  WonderMedia  Technologies, Inc.
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

#ifndef WMT_CEC_REG_H
#define WMT_CEC_REG_H

#define WMT_FTBLK_CEC

#define CEC_BASE_ADDR			(LVDS_BASE_ADDR + 0x100)
#define CEC_BASE2_ADDR			(LVDS_BASE_ADDR + 0x200)
struct cec_base_regs {
	union {
		unsigned int val;
		struct {
			unsigned int wr_start:1;
		} b;
	} enable; /* 0x0 */

	union {
		unsigned int val;
		struct {
			unsigned int wr_num:8;
		} b;
	} encode_number; /* 0x04 */

	union {
		unsigned int val;
		struct {
			unsigned int wr_data_ack:1;
			unsigned int wr_data_eom:1;
			unsigned int _02_03:2;
			unsigned int wr_data:8;
		} b;
	} encode_data[16]; /* 0x08 header,0x0c - 0x44 */

	union {
		unsigned int val;
		struct {
			unsigned int finish_reset:1;
		} b;
	} decode_reset; /* 0x48 */

	union {
		unsigned int val;
		struct {
			unsigned int rd_start:1;
			unsigned int rd_all_ack:1;
			unsigned int rd_finish:1;
		} b;
	} decode_start; /* 0x4c */

	union {
		unsigned int val;
		struct {
			unsigned int rd_data_ack:1;
			unsigned int rd_data_eom:1;
			unsigned int _02_03:2;
			unsigned int rd_data:8;
		} b;
	} decode_data[16]; /* 0x50 header, 0x54 - 0x8c */

	unsigned int wr_start_set0; /* 0x90 */
	unsigned int wr_start_set1; /* 0x94 */
	unsigned int wr_logic0_set0; /* 0x98 */
	unsigned int wr_logic0_set1; /* 0x9c */
	unsigned int wr_logic1_set0; /* 0xa0 */
	unsigned int wr_logic1_set1; /* 0xa4 */
	unsigned int rd_start_l_set0; /* 0xa8 */
	unsigned int rd_start_r_set0; /* 0xac */
	unsigned int rd_start_l_set1; /* 0xb0 */
	unsigned int rd_start_r_set1; /* 0xb4 */
	unsigned int rd_logic0_l_set0; /* 0xb8 */
	unsigned int rd_logic0_r_set0; /* 0xbc */
	unsigned int rd_logic0_l_set1; /* 0xc0 */
	unsigned int rd_logic0_r_set1; /* 0xc4 */
	unsigned int rd_logic1_l_set0; /* 0xc8 */
	unsigned int rd_logic1_r_set0; /* 0xcc */
	unsigned int rd_logic1_l_set1; /* 0xd0 */
	unsigned int rd_logic1_r_set1; /* 0xd4 */
	unsigned int physical_addr; /* 0xd8 */

	union {
		unsigned int val;
		struct {
			unsigned int addr1:4;
			unsigned int addr2:4;
			unsigned int addr3:4;
			unsigned int addr4:4;
			unsigned int addr5:4;
			unsigned int _20_23:4;
			unsigned int valid1:1;
			unsigned int valid2:1;
			unsigned int valid3:1;
			unsigned int valid4:1;
			unsigned int valid5:1;
		} b;
	} logical_addr; /* 0xdc */

	union {
		unsigned int val;
		struct {
			unsigned int retry:4;
		} b;
	} wr_retry; /* 0xe0 */

	union {
		unsigned int val;
		struct {
			unsigned int free_3x:4;
			unsigned int _04_07:4;
			unsigned int free_5x:4;
			unsigned int _12_15:4;
			unsigned int free_7x:4;
		} b;
	} free_3x; /* 0xe4 */

	unsigned int wr_set0_error; /* 0xe8 */
	unsigned int wr_set1_error; /* 0xec */

	union {
		unsigned int val;
		struct {
			unsigned int next_decode:1; /*read enable*/
		} b;
	} reject; /* 0xf0 */

	unsigned int rd_l_set0_error; /* 0xf4 */
	unsigned int rd_r_set1_error; /* 0xf8 */
	unsigned int rd_l_error; /* 0xfc */

	unsigned int rx_trig_range; /* 0x100 */
	unsigned int rx_sample_l_range; /* 0x104 */
	unsigned int rx_sample_r_range; /* 0x108 */

	union {
		unsigned int val;
		struct {
			unsigned int disable:1;
		} b;
	} comp; /* 0x10c */

	union {
		unsigned int val;
		struct {
			unsigned int err:1;
			unsigned int no_ack:1;
		} b;
	} handle_disable; /* 0x110 */

	union {
		unsigned int val;
		struct {
			unsigned int r1_encode_ok:1; /* write finish */
			unsigned int r1_decode_ok:1; /* read finish */
			unsigned int r1_error:1; /* read error */
			unsigned int r1_arb_fail:1; /* wr arb fail */
			unsigned int r1_no_ack:1; /* wr no ack */
		} b;
	} status; /* 0x114 */

	unsigned int int_enable; /* 0x118 */

	union {
		unsigned int val;
		struct {
			unsigned int disable:1;
		} b;
	} decode_full; /* 0x11c */

	union {
		unsigned int val;
		struct {
			unsigned int start:1;
			unsigned int logic0:1;
			unsigned int logic1:1;
		} b;
	} status4_disable; /* 0x120 */

	union {
		unsigned int val;
		struct {
			unsigned int enable:1; /*1:rd self wr & all dest data */
		} b;
	} rd_encode; /* 0x124 */

	union {
		unsigned int val;
		struct {
			unsigned int disable:1; /* 1 : disable arb check */
		} b;
	} arb_check; /* 0x128 */
};

#define REG_CEC_BEGIN	(CEC_BASE_ADDR + 0x0)
#define REG_CEC_END	(CEC_BASE2_ADDR + 0x28)
#ifndef CEC_C
extern struct cec_base_regs *cec_regs;
#endif
#endif /* WMT_CEC_REG_H */

