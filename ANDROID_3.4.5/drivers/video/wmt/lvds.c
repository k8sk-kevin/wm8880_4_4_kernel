/*++
 * linux/drivers/video/wmt/lvds.c
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

#define LVDS_C
#undef DEBUG
/* #define DEBUG */
/* #define DEBUG_DETAIL */
/*----------------------- DEPENDENCE -----------------------------------------*/
#include "lvds.h"

#ifdef WMT_FTBLK_LVDS
/*----------------------- PRIVATE MACRO --------------------------------------*/

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define LVDS_XXXX    1     *//*Example*/

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx lvds_xxx_t; *//*Example*/

/*----------EXPORTED PRIVATE VARIABLES are defined in lvds.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  lvds_xxx;        *//*Example*/
struct lvds_base_regs *lvds_regs = (void *) LVDS_BASE_ADDR;

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void lvds_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
void lvds_set_power_down(int pwrdn)
{
	DBG_DETAIL("(%d)\n", pwrdn);

	lvds_regs->test.b.pd = pwrdn;
	mdelay(1);
	lvds_regs->test2.b.pd_l2ha = pwrdn;
}

void lvds_set_enable(vpp_flag_t enable)
{
	DBG_DETAIL("(%d)\n", enable);
	lvds_regs->test.b.tre_en = (enable) ? 0 : 1;
	lvds_regs->test2.b.mode = (enable) ? 1 : 0;
	lvds_regs->test2.b.resa_en = (enable) ? 0 : 1;
#ifdef CONFIG_UBOOT
	if ((enable) && (lcd_get_lvds_id() == LCD_LVDS_1024x600)) {
		/* GPIO10 VDD_EN->CLK delay 16->38ms */
		outl(inl(GPIO_BASE_ADDR + 0x80) | BIT10, GPIO_BASE_ADDR + 0x80);
		outl(inl(GPIO_BASE_ADDR + 0xC0) | BIT10, GPIO_BASE_ADDR + 0xC0);
		mdelay(16);
	}
#endif
}

int lvds_get_enable(void)
{
	return lvds_regs->test2.b.mode;
}

void lvds_set_rgb_type(int bpp)
{
	int mode;
	int mode_change = 0x2;

	DBG_DETAIL("(%d)\n", bpp);

	/* 0:888, 1-555, 2-666, 3-565 */
	switch (bpp) {
	case 15:
		mode = 1;
		break;
	case 16:
		mode = 3;
		break;
	case 18:
		mode = 2;
		break;
	case 24:
	default:
		mode = 0;
		mode_change = 0x0;
		break;
	}
#if 1 /* IGS default */
	mode = 4;
#endif
	lvds_regs->status.b.test = mode_change;
	lvds_regs->igs.b.bpp_type = mode;
}

vdo_color_fmt lvds_get_colfmt(void)
{
	return VDO_COL_FMT_ARGB;
}

void lvds_set_sync_polar(int h_lo, int v_lo)
{
	DBG_DETAIL("(%d,%d)\n", h_lo, v_lo);
	lvds_regs->set.b.hsync_polar_lo = h_lo;
	lvds_regs->set.b.vsync_polar_lo = v_lo;
}

void lvds_get_sync_polar(int *hsync_hi, int *vsync_hi)
{
	*hsync_hi = (lvds_regs->set.b.hsync_polar_lo) ? 0 : 1;
	*vsync_hi = (lvds_regs->set.b.vsync_polar_lo) ? 0 : 1;
}

/*----------------------- Module API --------------------------------------*/
void lvds_reg_dump(void)
{
	DPRINT("========== LVDS register dump ==========\n");
	vpp_reg_dump(REG_LVDS_BEGIN, REG_LVDS_END-REG_LVDS_BEGIN);

	DPRINT("---------- LVDS common ----------\n");
	DPRINT("test %d,dual chan %d,inv clk %d\n", lvds_regs->status.b.test,
		lvds_regs->status.b.dual_channel, lvds_regs->status.b.inv_clk);
	DPRINT("ldi shift left %d,IGS bpp type %d\n",
		lvds_regs->igs.b.ldi_shift_left, lvds_regs->igs.b.bpp_type);
	DPRINT("rsen %d,pll ready %d\n", lvds_regs->detect.b.rsen,
		lvds_regs->detect.b.pll_ready);
	DPRINT("pwr dn %d\n", lvds_regs->test.b.pd);
}

#ifdef CONFIG_PM
static unsigned int *lvds_pm_bk;
static unsigned int lvds_pd_bk;
void lvds_suspend(int sts)
{
	switch (sts) {
	case 0:	/* disable module */
		break;
	case 1: /* disable tg */
		break;
	case 2:	/* backup register */
		lvds_pd_bk = lvds_regs->test.b.pd;
		lvds_set_power_down(1);
		lvds_pm_bk = vpp_backup_reg(REG_LVDS_BEGIN,
			(REG_LVDS_END-REG_LVDS_BEGIN));
		break;
	default:
		break;
	}
}

void lvds_resume(int sts)
{
	switch (sts) {
	case 0:	/* restore register */
		vpp_restore_reg(REG_LVDS_BEGIN,
			(REG_LVDS_END-REG_LVDS_BEGIN), lvds_pm_bk);
		lvds_pm_bk = 0;
		if (lcd_get_lvds_id() != LCD_LVDS_1024x600)
			lvds_set_power_down(lvds_pd_bk);
		break;
	case 1:	/* enable module */
		break;
	case 2: /* enable tg */
		break;
	default:
		break;
	}
}
#else
#define lvds_suspend NULL
#define lvds_resume NULL
#endif

void lvds_init(void)
{
	lvds_regs->level.b.level = 1;
	lvds_regs->level.b.update = 1;
	lvds_regs->test.b.pll_r_f = 1;
	lvds_regs->test.b.pll_cpset = 1;
	lvds_regs->test.b.tre_en = 0;
	lvds_regs->test.b.vbg_sel = 2;
	lvds_regs->test.b.drv_pdmode = 0;
	lvds_regs->igs.b.ldi_shift_left = 1;
	lvds_regs->test2.val = 0x31432;
}

#endif /* WMT_FTBLK_LVDS */

