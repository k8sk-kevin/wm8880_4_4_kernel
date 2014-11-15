/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>
#include <linux/irq.h>

#include <mach/hardware.h>

#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/wmt_iomux.h>

int pwm_or_gpio=1;
extern int wmt_getsyspara(char *varname, unsigned char *varval, int *varlen);


struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		invert;
	unsigned int		period;
	unsigned int		lth_ns;
	unsigned int		hth_ns;
	int			(*notify)(struct device *,
					  int brightness);
	void			(*notify_after)(struct device *,
					int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
};

static inline void __pwm0_gpio_setup(void)
{
	/* select pwm function */
	REG8_VAL(GPIO_CTRL_GP20_PWM0_BYTE_ADDR) &= ~0x1;

	/* pwm0 output enable */
	REG8_VAL(GPIO_OD_GP20_PWM0_BYTE_ADDR) &= ~0x01;

	/* share pin */
	REG32_VAL(PIN_SHARING_SEL_4BYTE_ADDR) &= ~0x1000;
}

static int pre_brightness_value;
void sgm3727_bl_brightness(int brightness_value)
{
    int i = 0;
	int pulse_num = 0;
    brightness_value= (brightness_value-10)*31/245;
    brightness_value = 31 - brightness_value;

    if (brightness_value >= 32)
    {
        brightness_value = 31;
    }
    if (brightness_value < 0)
    {
        brightness_value = 0;
    }
	//printk("%s,brightness_value=%d,%d\n",__FUNCTION__,pre_brightness_value,brightness_value);
	if(pre_brightness_value == brightness_value)
	{
		return;
	}
	else if(pre_brightness_value == 0xFF)
	{
		pulse_num = brightness_value;
	}
	else if(brightness_value>pre_brightness_value)
	{
		pulse_num = brightness_value - pre_brightness_value;
	}
	else
	{
		pulse_num = (brightness_value + 32 ) - pre_brightness_value;
	}

	//printk("%s,pulse_num=%d\n",__FUNCTION__,pulse_num);
    if (pulse_num < 32)
    {
		local_irq_disable();
		if(pre_brightness_value == 0xFF)
		{
	        gpio_direction_output(WMT_PIN_GP0_GPIO0,0);
	        mdelay(5);//5
	        gpio_direction_output(WMT_PIN_GP0_GPIO0,1);
	        udelay(50);
		}
        for (i=0; i < pulse_num; i++)
        {
            gpio_direction_output(WMT_PIN_GP0_GPIO0,0);
            udelay(10);//50
            gpio_direction_output(WMT_PIN_GP0_GPIO0,1);
            udelay(10);//50
        }
		local_irq_enable();
    }

	pre_brightness_value = brightness_value;
}

static int cw500_backlight_update_status(struct backlight_device *bl)
{
    struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
    int brightness = bl->props.brightness;
    int max = bl->props.max_brightness;

    if(!brightness)
    {
        gpio_direction_output(WMT_PIN_GP0_GPIO0,0);
		pre_brightness_value = 0xFF;
		mdelay(10);
        return;
    }
    else
    {
        sgm3727_bl_brightness(brightness);
    }

    return 0;
}

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;

//	if (bl->props.power != FB_BLANK_UNBLANK)
//		brightness = 0;
//
//	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
//		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	if (brightness == 0) {
		if (pb->invert) {
			pwm_config(pb->pwm, pb->period, pb->period);
		} else {
			pwm_config(pb->pwm, 0, pb->period);
			pwm_disable(pb->pwm);
		}
	} else {
		brightness = pb->lth_ns +
			(brightness * (pb->period - pb->lth_ns - pb->hth_ns) / max);
		if (pb->invert)
			brightness = pb->period - brightness;
		pwm_config(pb->pwm, brightness, pb->period);
		pwm_enable(pb->pwm);
	}

	if (pb->notify_after)
		pb->notify_after(pb->dev, brightness);

	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

backlight_update_status_select(struct backlight_device *bl)
{
	if(pwm_or_gpio){
		//printk("pwm branch\n");
		pwm_backlight_update_status(bl);
	}else{
		printk("gpio branch\n");
		cw500_backlight_update_status(bl);
	}

}

static const struct backlight_ops pwm_backlight_ops = {
	//.update_status	= pwm_backlight_update_status,//TODO: add uboot control rubbit
	//.update_status	= cw500_backlight_update_status,
	.update_status	= backlight_update_status_select,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
};
/*
*    wmt.bl_select:
*    0----->gpio control back light
*    1----->pwm control back light
*    default pwm control back light
*/
static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	int ret;
	unsigned char buf[64];
	int varlen = sizeof(buf);

	memset(buf, 0x0, sizeof(buf));
	ret = wmt_getsyspara("wmt.bl_select", buf, &varlen);
	if (ret == 0) {
		sscanf(buf, "%d", &pwm_or_gpio);
	}
	else {
		pwm_or_gpio = 1;
		printk("default use pwm to control backlight,pwm_or_gpio:%d\n",pwm_or_gpio);
	}
	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}
	if(pwm_or_gpio){
		__pwm0_gpio_setup();
	}else{
		pre_brightness_value = 0;
		//ret = gpio_request(WMT_PIN_GP0_GPIO0,"gpio lcd back light");
		//if (ret){
		//	printk("gpio_request gpio lcd back light  failed\n");
		//	goto err_alloc;
		//}
	}
	pb->invert = data->invert;
	pb->period = data->pwm_period_ns;
	pb->notify = data->notify;
	pb->notify_after = data->notify_after;
	pb->check_fb = data->check_fb;
	pb->lth_ns = data->lth_brightness * (data->pwm_period_ns / data->max_brightness);
	pb->hth_ns = (data->max_brightness - data->hth_brightness) *
			(data->pwm_period_ns / data->max_brightness);
	pb->dev = &pdev->dev;

	pb->pwm = pwm_request(data->pwm_id, "backlight");
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for backlight\n");
		ret = PTR_ERR(pb->pwm);
		goto err_alloc;
	} else
		dev_dbg(&pdev->dev, "got pwm for backlight\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.brightness = data->dft_brightness;
	//backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
	pwm_free(pb->pwm);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	pwm_free(pb->pwm);
	if (data->exit)
		data->exit(&pdev->dev);
	if(!pwm_or_gpio)
		gpio_free(WMT_PIN_GP0_GPIO0);
	return 0;
}

#ifdef CONFIG_PM
static int pwm_backlight_suspend(struct device *dev)
{
	int ret=0;
	struct backlight_device *bl = dev_get_drvdata(dev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	printk("%s\n",__func__);
	if (pb->notify)
		pb->notify(pb->dev, 0);
	if(pwm_or_gpio){
		if (pb->invert) {
			pwm_config(pb->pwm, pb->period, pb->period);
		} else {
			pwm_config(pb->pwm, 0, pb->period);
			pwm_disable(pb->pwm);
		}
	}else{

	}

	if (pb->notify_after)
		pb->notify_after(pb->dev, 0);
	if(!pwm_or_gpio){
		gpio_free(WMT_PIN_GP0_GPIO0);
	}
	return 0;
}

static int pwm_backlight_resume(struct device *dev)
{
	int ret=0;
	printk("%s\n",__func__);
	if(pwm_or_gpio){
		__pwm0_gpio_setup();
	}else{
		ret = gpio_request(WMT_PIN_GP0_GPIO0,"gpio lcd back light");
		if (ret){
			printk("gpio_request gpio lcd back light  failed\n");
			return -1;
		}
		gpio_direction_output(WMT_PIN_GP0_GPIO0,0);
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(pwm_backlight_pm_ops, pwm_backlight_suspend,
			 pwm_backlight_resume);

#endif

static void pwm_backlight_shutdown(struct platform_device *pdev)
{
	pwm_backlight_suspend(&pdev->dev);
}

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name	= "pwm-backlight",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pwm_backlight_pm_ops,
#endif
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
	.shutdown	= pwm_backlight_shutdown,
};

module_platform_driver(pwm_backlight_driver);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

