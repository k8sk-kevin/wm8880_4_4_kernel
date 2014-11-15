/*++
 * linux/drivers/video/wmt/lcd-INNOLUX-AT070TN83.c
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

#define LCD_INNOLUX_AT070TN83_C
/* #define DEBUG */
/*----------------------- DEPENDENCE -----------------------------------------*/
#include "../lcd.h"

/*----------------------- PRIVATE MACRO --------------------------------------*/
/* #define  LCD_AT070TN83_XXXX  xxxx    *//*Example*/

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define LCD_AT070TN83_XXXX    1     *//*Example*/

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx lcd_xxx_t; *//*Example*/

/*----------EXPORTED PRIVATE VARIABLES are defined in lcd.h  -------------*/
static void lcd_at070tn83_initial(void);
static void lcd_at070tn83_uninitial(void);

/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  lcd_xxx;        *//*Example*/
struct lcd_parm_t lcd_at070tn83_parm = {
	.bits_per_pixel = 18,
	.capability = LCD_CAP_CLK_HI,
	.vmode = {
	.name = "INNOLUX AT707TN83",
	.refresh = 60,
	.xres = 800,
	.yres = 480,
	.pixclock = KHZ2PICOS(33333),
	.left_margin = 45,
	.right_margin = 210,
	.upper_margin = 22,
	.lower_margin = 22,
	.hsync_len = 1,
	.vsync_len = 1,
	.sync = 0,
	.vmode = 0,
	.flag = 0,
	},
	.width = 154,
	.height = 85,
	.initial = lcd_at070tn83_initial,
	.uninitial = lcd_at070tn83_uninitial,
};

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void lcd_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
static void lcd_at070tn83_initial(void)
{
#if 0
	outl(inl(GPIO_BASE_ADDR + 0x80) | BIT0, GPIO_BASE_ADDR + 0x80);
	outl(inl(GPIO_BASE_ADDR + 0x4C) | BIT28, GPIO_BASE_ADDR + 0x4C);
	outl(inl(GPIO_BASE_ADDR + 0x8C) | BIT28, GPIO_BASE_ADDR + 0x8C);
	/* DVDD */
	/* T2 > 0ms */ /* AVDD/VCOM(NANDQS) */
	outl(inl(GPIO_BASE_ADDR + 0xCC) | BIT28, GPIO_BASE_ADDR + 0xCC);
	/* T4 > 0ms */
	/* VGH */
	/* 0 < T6 <= 10ms */
	lcd_enable_signal(1); /* signal, DVO enable */
	lcd_oem_enable_backlight(200); /* T12 > 200ms, BL(bit0) */
#endif
}

static void lcd_at070tn83_uninitial(void)
{
#if 0
	/* BL(bit0) */
	outl(inl(GPIO_BASE_ADDR + 0xC0) & ~BIT0, GPIO_BASE_ADDR + 0xC0);
	mdelay(200); /* T12 > 200ms */
	lcd_enable_signal(0); /* singal, DVO enable */
	/* AVDD/VCOM(NANDQS) */
	outl(inl(GPIO_BASE_ADDR + 0xCC) & ~BIT28, GPIO_BASE_ADDR + 0xCC);
#endif
}

struct lcd_parm_t *lcd_at070tn83_get_parm(int arg)
{
	return &lcd_at070tn83_parm;
}

int lcd_at070tn83_init(void)
{
	int ret;

	ret = lcd_panel_register(LCD_INNOLUX_AT070TN83,
		(void *) lcd_at070tn83_get_parm);
	return ret;
} /* End of lcd_oem_init */
module_init(lcd_at070tn83_init);

/*--------------------End of Function Body -----------------------------------*/
#undef LCD_INNOLUX_AT070TN83_C
