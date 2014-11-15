/*++
 * linux/drivers/video/wmt/lcd-b079xan01.c
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

#include <linux/delay.h>

#include <mach/hardware.h>
#include <linux/spi/spi.h>
#include <mach/wmt-spi.h>

#include <linux/gpio.h>
#include <mach/wmt_iomux.h>

#include "../lcd.h"

#define DRIVERNAME	"ssd2828"

static struct lcd_parm_t lcd_b079xan01_parm = {
	.bits_per_pixel = 24,
	.capability = 0,
	.vmode = {
		.name = "B079XAN01",
		.refresh = 60,
		.xres = 768,
		.yres = 1024,
		.pixclock = KHZ2PICOS(64800),
		.left_margin = 56,
		.right_margin = 60,
		.upper_margin = 30,
		.lower_margin = 36,
		.hsync_len = 64,
		.vsync_len = 50,
		.sync = 0, /* FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT, */
		.vmode = 0,
		.flag = 0,
	},
	.width = 120,
	.height = 160,
#if 0
	.initial = lcd_power_on,
	.uninitial = lcd_power_off,
#endif
};

static struct lcd_parm_t *lcd_b079xan01_get_parm(int arg)
{
	return &lcd_b079xan01_parm;
}

static int lcd_b079xan01_init(void)
{
	return lcd_panel_register(LCD_B079XAN01,
		(void *) lcd_b079xan01_get_parm);
}

struct ssd2828_chip {
	struct spi_device *spi;
	int gpio_reset;
};

static int ssd2828_read(struct spi_device *spi, uint8_t reg)
{
	int ret;
	uint8_t buf1[3] = { 0x70, 0x00, 0x00 };
	uint8_t buf2[3] = { 0x73, 0x00, 0x00 };

	buf1[2] = reg;

	ret = spi_write(spi, buf1, 3);
	if (ret) {
		pr_err("spi_write ret=%d\n", ret);
		return -EIO;
	}

	ret = spi_w8r16(spi, buf2[0]);
	if (ret < 0) {
		pr_err("spi_write ret=%d\n", ret);
		return ret;
	}

	return ret;
}

#if 0
static int ssd2828_write(struct spi_device *spi, uint8_t reg, uint16_t data)
{
	int ret;
	uint8_t buf_reg[3] = { 0x70, 0x00, 0x00 };
	uint8_t buf_data[3] = { 0x72, 0x00, 0x00 };

	buf_reg[2] = reg;

	buf_data[1] = (data >> 8) & 0xff;
	buf_data[2] = data & 0xff;

	ret = spi_write(spi, buf_reg, 3);
	if (ret) {
		pr_err("spi_write ret=%d,w cmd=0x%06x\n", ret, data);
		return ret;
	}

	ret = spi_write(spi, buf_data, 3);
	if (ret)
		pr_err("spi_write ret=%d,w cmd=0x%06x\n", ret, data);

	return ret;
}
#endif

static inline int spi_write_24bit(struct spi_device *spi, uint32_t data)
{
	int ret;
	uint8_t buf[3];

	buf[0] = (data >> 16) & 0xff;
	buf[1] = (data >> 8) & 0xff;
	buf[2] = data & 0xff;

	ret = spi_write(spi, buf, 3);
	if (ret)
		pr_err("spi_write ret=%d,w cmd=0x%06x\n", ret, data);

	return ret;
}

static const uint32_t ssd2828_init_sequence[] = {
	0x7000B1, 0x723240, /* VSA=50, HAS=64 */
	0x7000B2, 0x725078, /* VBP=30+50, HBP=56+64 */
	0x7000B3, 0x72243C, /* VFP=36, HFP=60 */
	0x7000B4, 0x720300, /* HACT=768 */
	0x7000B5, 0x720400, /* VACT=1024 */
	0x7000B6, 0x72000b, /* burst mode, 24bpp loosely packed */
	0x7000DE, 0x720003, /* no of lane=4 */
	0x7000D6, 0x720005, /* RGB order and packet number in blanking period */
	0x7000B9, 0x720000, /* disable PLL */

	/* lane speed=576 (24MHz * 24 = 576) */
	/* may modify according to requirement, 500Mbps to 560Mbps */
	/* LP clock : 576 / 9 / 8 = 8 MHz */
	0x7000BA, 0x728018,
	0x7000BB, 0x720008,

	0x7000B9, 0x720001, /* enable PPL */
	0x7000C4, 0x720001, /* enable BTA */
	0x7000B7, 0x720342, /* enter LP mode */
	0x7000B8, 0x720000, /* VC */
	0x7000BC, 0x720000, /* set packet size */

	0x700011, 0xff0000 + 200, /* sleep out cmd */
	0x700029, 0xff0000 + 200, /* display on */
	0x7000B7, 0x72030b, /* video mode on */
};

static int ssd2828_hw_reset(struct ssd2828_chip *chip)
{
	lcd_power_on(1);
	msleep(10);

	gpio_direction_output(chip->gpio_reset, 1);
	msleep(10);
	gpio_direction_output(chip->gpio_reset, 0);
	msleep(20);
	gpio_direction_output(chip->gpio_reset, 1);
	msleep(20);
	return 0;
}

static int ssd2828_hw_init(struct ssd2828_chip *chip)
{
	int ret = 0;
	int i;

	ssd2828_hw_reset(chip);

	ret = ssd2828_read(chip->spi, 0xB0);
	if (ret < 0 || ret != 0x2828) {
		pr_err("Error: SSD2828 not found!\n");
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(ssd2828_init_sequence); i++) {
		if (ssd2828_init_sequence[i] & 0xff000000) {
			msleep(ssd2828_init_sequence[i] & 0xff);
			continue;
		}
		ret = spi_write_24bit(chip->spi, ssd2828_init_sequence[i]);
		if (ret)
			break;
	}

	return ret;
}

static ssize_t option_port_testmode_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	char *s = buf;
	int ret;
	int reg;

	struct spi_device *spi = container_of(dev, struct spi_device, dev);

	s += sprintf(s, "register  value\n");

	for (reg = 0xb0; reg <= 0xff; reg++) {
		ret = ssd2828_read(spi, (uint8_t)reg);
		if (ret < 0)
			goto out;
		s += sprintf(s, "reg 0x%02X : 0x%04x\n", reg, ret);
	}

	s += sprintf(s, "=========\n");

out:
	return s - buf;
}

static ssize_t option_port_testmode_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	return 0;
}

static DEVICE_ATTR(testmode, S_IRUGO,
		   option_port_testmode_show,
		   option_port_testmode_store);

static int __devinit ssd2828_probe(struct spi_device *spi)
{
	struct ssd2828_chip *chip;
	int gpio = WMT_PIN_GP0_GPIO0;
	int ret;

	ret = gpio_request(gpio, "SSD2828 Reset");
	if (ret) {
		dev_err(&spi->dev, "can not open GPIO %d\n", gpio);
		return ret;
	}

	ret = ssd2828_read(spi, 0xB0);
	if (ret < 0 || ret != 0x2828) {
		pr_err("Error: SSD2828 not found!\n");
		return -ENODEV;
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->spi = spi;
	chip->gpio_reset = gpio;
	spi_set_drvdata(spi, chip);

	ret = sysfs_create_file(&spi->dev.kobj, &dev_attr_testmode.attr);
	if (unlikely(ret))
		pr_err("ssd2828 sysfs_create_file failed\n");

	return ret;
}

static int ssd2828_spi_resume(struct spi_device *spi)
{
	struct ssd2828_chip *chip = dev_get_drvdata(&spi->dev);
	return ssd2828_hw_init(chip);
}

static struct spi_driver ssd2828_driver = {
	.driver = {
		.name	= DRIVERNAME,
		.owner	= THIS_MODULE,
	},
	.probe		= ssd2828_probe,
	.resume		= ssd2828_spi_resume,
#if 0
	.remove		= __devexit_p(ssd2828_remove),
	.shutdown	= ssd2828_shutdown,
#endif
};

static struct spi_board_info ssd2828_spi_info[] __initdata = {
	{
		.modalias	= DRIVERNAME,
		.bus_num	= 1,
		.chip_select	= 0,
		.max_speed_hz	= 12000000,
		.irq		= -1,
		.mode		= SPI_CLK_MODE3,
	},
};

static int wmt_lcd_panel(void)
{
	char buf[96];
	int len = sizeof(buf);
	int type, id = 0;

	if (wmt_getsyspara("wmt.display.param", buf, &len))
		return -ENODEV;

	sscanf(buf, "%d:%d", &type, &id);
	return id;
}

static int __init b079xan01_init(void)
{
	int ret;

	if (wmt_lcd_panel() != LCD_B079XAN01) {
		pr_err("LCD B079XAN01 not found\n");
		return -EINVAL;
	}

	ret = spi_register_board_info(ssd2828_spi_info,
				      ARRAY_SIZE(ssd2828_spi_info));
	if (ret) {
		pr_err("spi_register_board_info failed\n");
		return ret;
	}

	ret = spi_register_driver(&ssd2828_driver);
	if (ret) {
		pr_err("spi_register_driver failed\n");
		return -EIO;
	}

	if (lcd_b079xan01_init()) {
		spi_unregister_driver(&ssd2828_driver);
		return -ENODEV;
	}

	pr_info("spi %s register success\n", DRIVERNAME);

	return 0;
}

static void b079xan01_exit(void)
{
	spi_unregister_driver(&ssd2828_driver);
}

module_init(b079xan01_init);
module_exit(b079xan01_exit);
