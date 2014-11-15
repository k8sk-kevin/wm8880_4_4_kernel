/*++
 * linux/drivers/video/wmt/lcd-oem.c
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

#define LCD_OEM_C
/* #define DEBUG */
/*----------------------- DEPENDENCE -----------------------------------------*/
#include "../lcd.h"

/*----------------------- PRIVATE MACRO --------------------------------------*/
/* #define  LCD_OEM_XXXX  xxxx    *//*Example*/

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define LCD_OEM_XXXX    1     *//*Example*/

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx lcd_xxx_t; *//*Example*/

/*----------EXPORTED PRIVATE VARIABLES are defined in lcd.h  -------------*/

/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  lcd_xxx;        *//*Example*/

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void lcd_xxx(void); *//*Example*/
#ifdef CONFIG_UBOOT
int lcd_bl_time;
void lcd_uboot_set_backlight(void)
{
	int cur;
	do {
		wmt_read_ostc(&cur);
	} while (cur < lcd_bl_time);
	REG32_VAL(GPIO_BASE_ADDR + 0xC0) |= 0x800; /* BL( bit 11 ) */
}

#endif

void lcd_oem_enable_backlight(int wait_ms)
{
#ifdef CONFIG_UBOOT
	wmt_read_ostc(&lcd_bl_time);
	lcd_bl_time += (wait_ms * 1000);
#else
	mdelay(wait_ms);
	REG32_VAL(GPIO_BASE_ADDR + 0xC0) |= 0x800;
#endif
}

#ifndef CONFIG_VPP_SHENZHEN
static void lcd_oem_initial(void)
{
	outl(inl(GPIO_BASE_ADDR + 0x80) | 0x801, GPIO_BASE_ADDR + 0x80);
	outl(inl(GPIO_BASE_ADDR + 0xC0) | 0x801, GPIO_BASE_ADDR + 0xC0);
	lcd_enable_signal(1);
}

static void lcd_oem_uninitial(void)
{
	outl(inl(GPIO_BASE_ADDR + 0xC0) & ~0x801, GPIO_BASE_ADDR + 0xC0);
	lcd_enable_signal(0);
}
#endif

struct lcd_parm_t lcd_oem_parm = {
	.bits_per_pixel = 24,
	.capability = LCD_CAP_VSYNC_HI,
	.vmode = {
		.name = "WonderMedia OEM LCD (VGA 1024x768)",
		.refresh = 60,
		.xres = 1024,
		.yres = 768,
		.pixclock = KHZ2PICOS(63500),
		.left_margin = 152,
		.right_margin = 48,
		.upper_margin = 23,
		.lower_margin = 3,
		.hsync_len = 104,
		.vsync_len = 4,
		.sync = FB_SYNC_VERT_HIGH_ACT,
		.vmode = 0,
		.flag = 0,
	},
	.width = 222,
	.height = 125,
#ifndef CONFIG_VPP_SHENZHEN
	.initial = lcd_oem_initial,
	.uninitial = lcd_oem_uninitial
#endif
};

#ifndef CONFIG_VPP_SHENZHEN
static void lcd_oem_1024x600_initial(void)
{
	outl(inl(GPIO_BASE_ADDR + 0x80) | 0x801, GPIO_BASE_ADDR + 0x80);

	/* DVDD */
	/* T2 > 0ms */ /* AVDD/VCOM( bit 0 ) */
	outl(inl(GPIO_BASE_ADDR + 0xC0) | 0x01, GPIO_BASE_ADDR + 0xC0);
	/* T4 > 0ms */
	/* VGH */
	/* 0 < T6 <= 10ms */
	lcd_enable_signal(1); /* singal, DVO enable */
	mdelay(200); /* T12 > 200ms */
	outl(inl(GPIO_BASE_ADDR + 0xC0) | 0x800, GPIO_BASE_ADDR + 0xC0);
}

static void lcd_oem_1024x600_uninitial(void)
{
	/* BL( bit 11 ) */
	outl(inl(GPIO_BASE_ADDR + 0xC0) & ~0x800, GPIO_BASE_ADDR + 0xC0);
	mdelay(200); /* T12 > 200ms */
	lcd_enable_signal(0); /* singal, DVO enable */
	/* AVDD/VCOM( bit 0 ) */
	outl(inl(GPIO_BASE_ADDR + 0xC0) & ~0x01, GPIO_BASE_ADDR + 0xC0);
}
#endif

struct lcd_parm_t lcd_oem_parm_1024x600 = {
	.bits_per_pixel = 24,
	.capability = LCD_CAP_VSYNC_HI,
	.vmode = {
#if 1 /* 7" HHX070ML208CP21A */
		.name = "HHX070ML208CP21A",
		.refresh = 60,
		.xres = 1024,
		.yres = 600,
		.pixclock = KHZ2PICOS(51200),
		.left_margin = 140,
		.right_margin = 160,
		.upper_margin = 20,
		.lower_margin = 12,
		.hsync_len = 20,
		.vsync_len = 3,
		.sync = FB_SYNC_VERT_HIGH_ACT,
		.vmode = 0,
		.flag = 0,
#else
		.name = "ePAD 1024x600", /* HannStar HSD070PFW3 */
		.refresh = 60,
		.xres = 1024,
		.yres = 600,
		.pixclock = KHZ2PICOS(45000),
		.left_margin = 50,
		.right_margin = 50,
		.upper_margin = 10,
		.lower_margin = 10,
		.hsync_len = 4,
		.vsync_len = 4,
		.sync = FB_SYNC_VERT_HIGH_ACT,
		.vmode = 0,
		.flag = 0,
#endif
	},
#ifndef CONFIG_VPP_SHENZHEN
	.initial = lcd_oem_1024x600_initial,
	.uninitial = lcd_oem_1024x600_uninitial,
#endif
};

struct lcd_parm_t lcd_oem_parm_1024x768 = {
	.bits_per_pixel = 24,
	.capability = LCD_CAP_VSYNC_HI,
	.vmode = {
		.name = "OEM 1024x768", /* VGA 1024x768 */
		.refresh = 60,
		.xres = 1024,
		.yres = 768,
		.pixclock = KHZ2PICOS(63500),
		.left_margin = 152,
		.right_margin = 48,
		.upper_margin = 23,
		.lower_margin = 3,
		.hsync_len = 104,
		.vsync_len = 4,
		.sync = FB_SYNC_VERT_HIGH_ACT,
		.vmode = 0,
		.flag = 0,
	},
	.width = 222,
	.height = 125,
#ifndef CONFIG_VPP_SHENZHEN
	.initial = lcd_oem_initial,
	.uninitial = lcd_oem_uninitial
#endif
};

struct lcd_parm_t lcd_oem_parm_1366x768 = {
	.bits_per_pixel = 18,
	.capability = LCD_CAP_CLK_HI,
	.vmode = {
		.name = "OEM 1366X768",
		.refresh = 60,
		.xres = 1366,
		.yres = 768,
		.pixclock = KHZ2PICOS(75440),
		.left_margin = 98,
		.right_margin = 31,
		.upper_margin = 22,
		.lower_margin = 4,
		.hsync_len = 65,
		.vsync_len = 12,
		.sync = 0,
		.vmode = 0,
		.flag = 0,
	},
	.width = 293,
	.height = 164,
#ifndef CONFIG_VPP_SHENZHEN
	.initial = lcd_oem_initial,
	.uninitial = lcd_oem_uninitial
#endif
};

struct lcd_parm_t lcd_oem_parm_480x800 = {
	.bits_per_pixel = 18,
	.capability = LCD_CAP_CLK_HI,
	.vmode = {
		.name = "OEM 480x800",
		.refresh = 60,
		.xres = 480,
		.yres = 800,
		.pixclock = KHZ2PICOS(27000),
		.left_margin = 78,
		.right_margin = 78,
		.upper_margin = 60,
		.lower_margin = 60,
		.hsync_len = 4,
		.vsync_len = 4,
		.sync = 0,
		.vmode = 0,
		.flag = 0,
	},
#ifndef CONFIG_VPP_SHENZHEN
	.initial = lcd_oem_initial,
	.uninitial = lcd_oem_uninitial
#endif
};

struct lcd_parm_t lcd_oem_parm_800x480 = {
	.bits_per_pixel = 18,
	.capability = LCD_CAP_CLK_HI,
	.vmode = {
		.name = "OEM 800x480",
		.refresh = 48,
		.xres = 800,
		.yres = 480,
		.pixclock = KHZ2PICOS(27000),
		.left_margin = 50,
		.right_margin = 50,
		.upper_margin = 17,
		.lower_margin = 16,
		.hsync_len = 10,
		.vsync_len = 5,
		.sync = 0,
		.vmode = 0,
		.flag = 0,
	},
	.width = 154,
	.height = 85,
#ifndef CONFIG_VPP_SHENZHEN
	.initial = lcd_oem_initial,
	.uninitial = lcd_oem_uninitial
#endif
};

#ifndef CONFIG_VPP_SHENZHEN
static void lcd_oem_1280x800_initial(void)
{
	DBG_MSG("lcd 10 power sequence\n");
	outl(inl(GPIO_BASE_ADDR + 0x80) | 0x801, GPIO_BASE_ADDR + 0x80);

	/* VDD on */
	/* 0 < T < 50ms */
	lcd_enable_signal(1); /* singal on */
	/* VGH,VGL low */ /* AVDD/VCOM( bit 0 ) */
	outl(inl(GPIO_BASE_ADDR + 0xC0) | 0x01, GPIO_BASE_ADDR + 0xC0);
	mdelay(150); /* T5 > 120ms */
	outl(inl(GPIO_BASE_ADDR + 0xC0) | 0x800, GPIO_BASE_ADDR + 0xC0);
}

static void lcd_oem_1280x800_uninitial(void)
{
	/* turn off backlight */
	outl(inl(GPIO_BASE_ADDR + 0xC0) & ~0x800, GPIO_BASE_ADDR + 0xC0);
	mdelay(150);
	/* turn off LCD */
	outl(inl(GPIO_BASE_ADDR + 0xC0) & ~0x01, GPIO_BASE_ADDR + 0xC0);
	lcd_enable_signal(0); /* turn off singal */
}
#endif

struct lcd_parm_t lcd_oem_parm_800x1280 = {
	.bits_per_pixel = 24,
	.capability = LCD_CAP_CLK_HI,
	.vmode = {
		.name = "WY101ML369IN30A",
		.refresh = 60,
		.xres = 800,
		.yres = 1280,
		.pixclock = KHZ2PICOS(71100),
		.left_margin = 70,
		.right_margin = 80,
		.upper_margin = 10,
		.lower_margin = 10,
		.hsync_len = 10,
		.vsync_len = 3,
		.sync = 0,
		.vmode = 0,
		.flag = 0,
	},
	.width = 135,
	.height = 217,
#ifndef CONFIG_VPP_SHENZHEN
	.initial = lcd_oem_1280x800_initial,
	.uninitial = lcd_oem_1280x800_uninitial,
#endif
};

struct lcd_parm_t lcd_oem_parm_1280x800 = {
	.bits_per_pixel = 24,
	.capability = LCD_CAP_CLK_HI,
	.vmode = {
		.name = "WY101ML369IN30A",
		.refresh = 60,
		.xres = 1280,
		.yres = 800,
		.pixclock = KHZ2PICOS(71100),
		.left_margin = 70,
		.right_margin = 80,
		.upper_margin = 10,
		.lower_margin = 10,
		.hsync_len = 10,
		.vsync_len = 3,
		.sync = 0,
		.vmode = 0,
		.flag = 0,
	},
	.width = 217,
	.height = 135,
#ifndef CONFIG_VPP_SHENZHEN
	.initial = lcd_oem_1280x800_initial,
	.uninitial = lcd_oem_1280x800_uninitial,
#endif
};

struct lcd_parm_t lcd_oem_parm_768x1024 = {
	.bits_per_pixel = 24,
	.capability = LCD_CAP_CLK_HI,
	.vmode = {
		.name = "oem_768x1024",
		.refresh = 60,
		.xres = 768,
		.yres = 1024,
		.pixclock = KHZ2PICOS(59300),
		.left_margin = 80,
		.right_margin = 80,
		.upper_margin = 23,
		.lower_margin = 18,
		.hsync_len = 7,
		.vsync_len = 5,
		.sync = 0,
		.vmode = 0,
		.flag = 0,
	},
};

/*----------------------- Function Body --------------------------------------*/
struct lcd_parm_t *lcd_oem_get_parm(int arg)
{
	return &lcd_oem_parm;
}

int lcd_oem_init(void)
{
	int ret;

	ret = lcd_panel_register(LCD_WMT_OEM, (void *)lcd_oem_get_parm);
	return ret;
} /* End of lcd_oem_init */
module_init(lcd_oem_init);

struct lcd_parm_t *lcd_get_oem_parm(int resx, int resy)
{
	struct lcd_parm_t *oem_parm[] = {
		&lcd_oem_parm_480x800,
		&lcd_oem_parm_1024x600,
		&lcd_oem_parm_1024x768,
		&lcd_oem_parm_1366x768,
		&lcd_oem_parm_800x480,
		&lcd_oem_parm_800x1280,
		&lcd_oem_parm_1280x800,
		&lcd_oem_parm_768x1024,
		0
	};
	struct lcd_parm_t *p;
	int i;

	for (i = 0; ; i++) {
		p = oem_parm[i];
		if (p == 0) {
			p = oem_parm[0];
			break;
		}
		if ((resx == p->vmode.xres) && (resy == p->vmode.yres))
			break;
	}
	return p;
}
/*--------------------End of Function Body -----------------------------------*/
#undef LCD_OEM_C
