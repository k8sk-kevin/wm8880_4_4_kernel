/*++
 * linux/drivers/video/wmt/sw_i2c.h
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

#ifndef _SWI2C_H_
#define _SWI2C_H_

struct swi2c_reg_s {
	unsigned int bit_mask;
	unsigned int gpio_en;
	unsigned int out_en;
	unsigned int data_in;
	unsigned int data_out;
	unsigned int pull_en_bit_mask;
	unsigned int pull_en;
};
#define swi2c_reg_t struct swi2c_reg_s

struct swi2c_handle_s {
	struct swi2c_reg_s *scl_reg;
	struct swi2c_reg_s *sda_reg;
};
#define swi2c_handle_t struct swi2c_handle_s

int wmt_swi2c_read(
	struct swi2c_handle_s *handle,
	char addr,
	char index,
	char *buf,
	int cnt
);

int wmt_swi2c_write(
	struct swi2c_handle_s *handle,
	char addr,
	char index,
	char *buf,
	int cnt
);

int wmt_swi2c_check(struct swi2c_handle_s *handle);

#endif

