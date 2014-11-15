/*++
 * linux/drivers/video/wmt/cec.c
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

#define CEC_C
/* #define DEBUG */
/*----------------------- DEPENDENCE -----------------------------------------*/
#include "cec.h"

/*----------------------- PRIVATE MACRO --------------------------------------*/

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define CEC_XXXX    1     *//*Example*/

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx cec_xxx_t; *//*Example*/

/*----------EXPORTED PRIVATE VARIABLES are defined in cec.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  cec_xxx;        *//*Example*/
int cec_logical_addr;
int cec_physical_addr;

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void cec_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
/*---------------------------- CEC COMMON API -------------------------------*/

#ifdef WMT_FTBLK_CEC

struct cec_base_regs *cec_regs = (void *) CEC_BASE_ADDR;
/*---------------------------- CEC HAL --------------------------------------*/
void wmt_cec_tx_data(char *buf, int size)
{
	int i;
	unsigned int reg;
	int wait_idle;

#ifdef DEBUG
	DPRINT("[CEC] tx data(%d):", size);
	for (i = 0; i < size; i++)
		DPRINT(" 0x%x", buf[i]);
	DPRINT("\n");
#endif

	if (size > 16) {
		DPRINT("[CEC] *W* size max 16\n");
		return;
	}

	wait_idle = 0;
	while (cec_regs->enable.b.wr_start) {
		if (wait_idle >= 10) {
			DPRINT("[CEC] wait idle timeout\n");
			return;
		}
		wait_idle++;
		mdelay(10);
		return;
	}

	for (i = 0; i < size; i++) {
		reg = (buf[i] << 4) + 0x1;
		if (i == (size - 1))
			reg |= BIT1;
		cec_regs->encode_data[i].val = reg;
	}
	cec_regs->encode_number.b.wr_num = (size * 10) + 0x1;
	cec_regs->enable.b.wr_start = 1;
}

int wmt_cec_rx_data(char *buf)
{
	int i, size;
	unsigned int reg;

	for (i = 0; i < 16; i++) {
		reg = cec_regs->decode_data[i].val;
		buf[i] = (reg & 0xFF0) >> 4;
		if (reg & BIT1) /* EOM */
			break;
	}
	cec_regs->decode_reset.b.finish_reset = 1;
	mdelay(1);
	cec_regs->decode_reset.b.finish_reset = 0;
	size = i + 1;
#ifdef DEBUG
	DPRINT("[CEC] rx data(%d):\n", size);
	for (i = 0; i < size; i++)
		DPRINT(" 0x%x", buf[i]);
	DPRINT("\n");
#endif
	return size;
}

void wmt_cec_set_clock(void)
{
	#define CEC_CLOCK (1000000 / 7984)

	cec_regs->wr_start_set0 = (370 * CEC_CLOCK); /* 3.7 ms */
	cec_regs->wr_start_set1 = (450 * CEC_CLOCK); /* 4.5 ms */
	cec_regs->wr_logic0_set0 = (150 * CEC_CLOCK); /* 1.5 ms */
	cec_regs->wr_logic0_set1 = (240 * CEC_CLOCK); /* 2.4 ms */
	cec_regs->wr_logic1_set0 = (60 * CEC_CLOCK); /* 0.6 ms */
	cec_regs->wr_logic1_set1 = (240 * CEC_CLOCK); /* 2.4 ms */
	cec_regs->rd_start_l_set0 = (350 * CEC_CLOCK); /* 3.5 ms */
	cec_regs->rd_start_r_set0 = (390 * CEC_CLOCK); /* 3.9 ms */
	cec_regs->rd_start_l_set1 = (430 * CEC_CLOCK); /* 4.3 ms */
	cec_regs->rd_start_r_set1 = (470 * CEC_CLOCK); /* 4.7 ms */
	cec_regs->rd_logic0_l_set0 = (130 * CEC_CLOCK); /* 1.3 ms */
	cec_regs->rd_logic0_r_set0 = (170 * CEC_CLOCK); /* 1.7 ms */
	cec_regs->rd_logic0_l_set1 = (205 * CEC_CLOCK); /* 2.05 ms*/
	cec_regs->rd_logic0_r_set1 = (275 * CEC_CLOCK); /* 2.75 ms*/
	cec_regs->rd_logic1_l_set0 = (40 * CEC_CLOCK); /* 0.4 ms */
	cec_regs->rd_logic1_r_set0 = (80 * CEC_CLOCK); /* 0.8 ms */
	cec_regs->rd_logic1_l_set1 = (205 * CEC_CLOCK); /* 2.05 ms*/
	cec_regs->rd_logic1_r_set1 = (275 * CEC_CLOCK); /* 2.75 ms*/
	cec_regs->rd_l_set0_error = (182 * CEC_CLOCK); /* 1.82 ms */
	cec_regs->rd_r_set1_error = (238 * CEC_CLOCK); /* 2.38 ms */
	cec_regs->rd_l_error = (287 * CEC_CLOCK); /* 2.87 ms */
	cec_regs->rx_sample_l_range = (85 * CEC_CLOCK); /* 0.85 ms*/
	cec_regs->rx_sample_r_range = (125 * CEC_CLOCK); /*1.25 ms*/
	cec_regs->wr_set0_error = (225 * CEC_CLOCK); /* 2.25 ms */
	cec_regs->wr_set1_error = (225 * CEC_CLOCK); /* 2.25 ms ?*/
}

void wmt_cec_set_logical_addr(int no, char addr, int enable)
{
	switch (no) {
	case 0:
		cec_regs->logical_addr.b.addr1 = addr;
		cec_regs->logical_addr.b.valid1 = addr;
		break;
	case 1:
		cec_regs->logical_addr.b.addr2 = addr;
		cec_regs->logical_addr.b.valid2 = addr;
		break;
	case 2:
		cec_regs->logical_addr.b.addr3 = addr;
		cec_regs->logical_addr.b.valid3 = addr;
		break;
	case 3:
		cec_regs->logical_addr.b.addr4 = addr;
		cec_regs->logical_addr.b.valid4 = addr;
		break;
	case 4:
		cec_regs->logical_addr.b.addr5 = addr;
		cec_regs->logical_addr.b.valid5 = addr;
		break;
	default:
		DPRINT("[CEC] *W* invalid %d\n", no);
		break;
	}
	DBGMSG("[CEC] set logical addr %d,0x%x\n", no, addr);
}

void wmt_cec_rx_enable(int enable)
{
	cec_regs->reject.b.next_decode = (enable) ? 0 : 1;
	/* GPIO4 disable GPIO function */
	vppif_reg32_write(GPIO_BASE_ADDR + 0x40, BIT4, 4, (enable) ? 0 : 1);
}

void wmt_cec_enable_int(int no, int enable)
{
	if (enable)
		cec_regs->int_enable |= (0x1 << no);
	else
		cec_regs->int_enable &= ~(0x1 << no);
}

void wmt_cec_clr_int(int sts)
{
	cec_regs->status.val = sts;
}

int wmt_cec_get_int(void)
{
	int reg;
	reg = cec_regs->status.val;
	return reg;
}

void wmt_cec_enable_loopback(int enable)
{
	/* 1 : read self write and all dest data */
	cec_regs->rd_encode.b.enable = enable;
}

void wmt_cec_init_hw(void)
{
	wmt_cec_set_clock();
	cec_regs->wr_retry.b.retry = 3;
	cec_regs->rx_trig_range = 2;

	cec_regs->free_3x.b.free_3x = 3;
	cec_regs->free_3x.b.free_5x = 5;
	cec_regs->free_3x.b.free_7x = 7;

	cec_regs->comp.b.disable = 1;
	cec_regs->handle_disable.b.err = 0;
	cec_regs->handle_disable.b.no_ack = 0;
	cec_regs->decode_full.b.disable = 0;
	cec_regs->status4_disable.b.start = 1;
	cec_regs->status4_disable.b.logic0 = 1;
	cec_regs->status4_disable.b.logic1 = 1;
	cec_regs->rd_encode.b.enable = 0;
}

/*---------------------------- CEC API --------------------------------------*/
void wmt_cec_reg_dump(void)
{
	DPRINT("========== CEC register dump ==========\n");
	vpp_reg_dump(REG_CEC_BEGIN, REG_CEC_END - REG_CEC_BEGIN);

	DPRINT("---------- CEC Tx ----------\n");
	DPRINT("wr start %d,wr num %d\n",
		cec_regs->enable.b.wr_start, cec_regs->encode_number.b.wr_num);
	DPRINT("wr header ack %d,EOM %d,data 0x%x\n",
		cec_regs->encode_data[0].b.wr_data_ack,
		cec_regs->encode_data[0].b.wr_data_eom,
		cec_regs->encode_data[0].b.wr_data);
	DPRINT("wr data ack %d,EOM %d,data 0x%x\n",
		cec_regs->encode_data[1].b.wr_data_ack,
		cec_regs->encode_data[1].b.wr_data_eom,
		cec_regs->encode_data[1].b.wr_data);
	DPRINT("finish reset %d,wr retry %d\n",
		cec_regs->decode_reset.b.finish_reset,
		cec_regs->wr_retry.b.retry);
	DPRINT("---------- CEC Rx ----------\n");
	DPRINT("rd start %d,all ack %d,finish %d\n",
		cec_regs->decode_start.b.rd_start,
		cec_regs->decode_start.b.rd_all_ack,
		cec_regs->decode_start.b.rd_finish);
	DPRINT("rd header ack %d,EOM %d,data 0x%x\n",
		cec_regs->decode_data[0].b.rd_data_ack,
		cec_regs->decode_data[0].b.rd_data_eom,
		cec_regs->decode_data[0].b.rd_data);
	DPRINT("rd data ack %d,EOM %d,data 0x%x\n",
		cec_regs->decode_data[1].b.rd_data_ack,
		cec_regs->decode_data[1].b.rd_data_eom,
		cec_regs->decode_data[1].b.rd_data);

	DPRINT("---------- Logical addr ----------\n");
	DPRINT("addr1 0x%x,valid %d\n",
		cec_regs->logical_addr.b.addr1,
		cec_regs->logical_addr.b.valid1);
	DPRINT("addr2 0x%x,valid %d\n",
		cec_regs->logical_addr.b.addr2,
		cec_regs->logical_addr.b.valid2);
	DPRINT("addr3 0x%x,valid %d\n",
		cec_regs->logical_addr.b.addr3,
		cec_regs->logical_addr.b.valid3);
	DPRINT("addr4 0x%x,valid %d\n",
		cec_regs->logical_addr.b.addr4,
		cec_regs->logical_addr.b.valid4);
	DPRINT("addr5 0x%x,valid %d\n",
		cec_regs->logical_addr.b.addr5,
		cec_regs->logical_addr.b.valid5);

	DPRINT("---------- Misc ----------\n");
	DPRINT("free 3x %d,5x %d,7x %d\n",
		cec_regs->free_3x.b.free_3x, cec_regs->free_3x.b.free_5x,
		cec_regs->free_3x.b.free_7x);
	DPRINT("reject next decode %d,comp disable %d\n",
		cec_regs->reject.b.next_decode, cec_regs->comp.b.disable);
	DPRINT("err handle disable %d,no ack disable %d\n",
		cec_regs->handle_disable.b.err,
		cec_regs->handle_disable.b.no_ack);
	DPRINT("r1 enc ok %d,r1 dec ok %d,r1 err %d\n",
		cec_regs->status.b.r1_encode_ok,
		cec_regs->status.b.r1_decode_ok,
		cec_regs->status.b.r1_error);
	DPRINT("r1 arb fail %d,r1 no ack %d\n",
		cec_regs->status.b.r1_arb_fail,
		cec_regs->status.b.r1_no_ack);
	DPRINT("dec full disable %d,self rd enable %d\n",
		cec_regs->decode_full.b.disable,
		cec_regs->rd_encode.b.enable);
}

#ifdef CONFIG_PM
static unsigned int *wmt_cec_pm_bk;
void wmt_cec_do_suspend(void)
{
	/* Suspend GPIO output high */
	vppif_reg32_write(GPIO_BASE_ADDR+0xC0, BIT23, 23, 0);
	wmt_cec_pm_bk = vpp_backup_reg(REG_CEC_BEGIN,
		(REG_CEC_END - REG_CEC_BEGIN));
}

void wmt_cec_do_resume(void)
{
	vppm_regs->sw_reset2.val = 0x1011111;
	/* disable GPIO function */
	vppif_reg32_write(GPIO_BASE_ADDR + 0x40, BIT4, 4, 0);
	/* GPIO4 disable GPIO out */
	vppif_reg32_write(GPIO_BASE_ADDR + 0x80, BIT4, 4, 0);
	/* GPIO4 disable pull ctrl */
	vppif_reg32_write(GPIO_BASE_ADDR + 0x480, BIT4, 4, 0);
	/* Suspend GPIO output enable */
	vppif_reg32_write(GPIO_BASE_ADDR + 0x80, BIT23, 23, 1);
	/* Suspend GPIO output high */
	vppif_reg32_write(GPIO_BASE_ADDR + 0xC0, BIT23, 23, 1);
	/* Wake3 disable pull ctrl */
	vppif_reg32_write(GPIO_BASE_ADDR + 0x480, BIT19, 19, 0);
	vpp_restore_reg(REG_CEC_BEGIN,
		(REG_CEC_END - REG_CEC_BEGIN), wmt_cec_pm_bk);
	wmt_cec_pm_bk = 0;
}
#endif
#endif /* WMT_FTBLK_CEC */

