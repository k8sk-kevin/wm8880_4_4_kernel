/*++
 * linux/drivers/video/wmt/vout-wmt.c
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
#undef DEBUG
/* #define DEBUG */
/* #define DEBUG_DETAIL */

#include "vpp.h"

#ifndef CFG_LOADER
static int vo_plug_flag;
#endif
int vo_plug_vout;
int (*vo_plug_func)(int hotplug);
enum vout_mode_t dvo_vout_mode;
enum vout_mode_t int_vout_mode;
int hdmi_cur_plugin;
struct vout_t *vo_poll_vout;

/* GPIO 10 & 11 */
struct swi2c_reg_s vo_gpio_scl = {
	.bit_mask = BIT10,
	.gpio_en = (__GPIO_BASE + 0x40),
	.out_en = (__GPIO_BASE + 0x80),
	.data_in = (__GPIO_BASE + 0x00),
	.data_out = (__GPIO_BASE + 0xC0),
	.pull_en = (__GPIO_BASE + 0x480),
	.pull_en_bit_mask = BIT10,
};

struct swi2c_reg_s vo_gpio_sda = {
	.bit_mask = BIT11,
	.gpio_en = (__GPIO_BASE + 0x40),
	.out_en = (__GPIO_BASE + 0x80),
	.data_in = (__GPIO_BASE + 0x00),
	.data_out = (__GPIO_BASE + 0xC0),
	.pull_en = (__GPIO_BASE + 0x480),
	.pull_en_bit_mask = BIT11,
};

struct swi2c_handle_s vo_swi2c_dvi = {
	.scl_reg = &vo_gpio_scl,
	.sda_reg = &vo_gpio_sda,
};

#define DVI_POLL_TIME_MS	1000

extern void hdmi_config_audio(struct vout_audio_t *info);

/*---------------------------------- API ------------------------------------*/
#ifdef DEBUG
void vout_print_entry(struct vout_t *vo)
{
	if (vo == 0)
		return;

	MSG(" =============== vout %d ===============\n", vo->num);
	MSG("fix 0x%x", vo->fix_cap);
	MSG("(inf %d,bus %d,govr %d,ext dev %d,fix plug %d,aud %d,edid %d)\n",
		(vo->fix_cap & VOUT_CAP_INTERFACE),
		(vo->fix_cap & VOUT_CAP_BUS) >> 8,
		(vo->fix_cap & VOUT_CAP_GOVR) >> 12,
		(vo->fix_cap & VOUT_CAP_EXT_DEV) ? 1 : 0,
		(vo->fix_cap & VOUT_CAP_FIX_PLUG) ? 1 : 0,
		(vo->fix_cap & VOUT_CAP_AUDIO) ? 1 : 0,
		(vo->fix_cap & VOUT_CAP_EDID) ? 1 : 0);
	MSG("info %d,%s\n", vo->info->num, vpp_mod_str[vo->govr->mod]);
	MSG("inf 0x%x,dev 0x%x\n", (int)vo->inf, (int)vo->dev);
	MSG("resx %d,resy %d,fps %d\n", vo->resx, vo->resy, vo->fps);
	MSG("pixclk %d,option %d,%d,%d,disable %d\n", vo->pixclk,
		vo->option[0], vo->option[1], vo->option[2], vo->disable);
	MSG("sts(reg %d,act %d,plug %d,edid %d,blank %d,pwrdn %d,cp %d)\n",
		(vo->status & VPP_VOUT_STS_REGISTER) ? 1 : 0,
		(vo->status & VPP_VOUT_STS_ACTIVE) ? 1 : 0,
		(vo->status & VPP_VOUT_STS_PLUGIN) ? 1 : 0,
		(vo->status & VPP_VOUT_STS_EDID) ? 1 : 0,
		(vo->status & VPP_VOUT_STS_BLANK) ? 1 : 0,
		(vo->status & VPP_VOUT_STS_POWERDN) ? 1 : 0,
		(vo->status & VPP_VOUT_STS_CONTENT_PROTECT) ? 1 : 0);

	if (vo->inf) {
		MSG(" ===== inf entry =====\n");
		MSG("mode %d, %s\n",
			vo->inf->mode, vout_inf_str[vo->inf->mode]);
	}

	if (vo->dev) {
		MSG(" ===== dev entry =====\n");
		MSG("name %s,inf %d,%s\n", vo->dev->name,
			vo->dev->mode, vout_inf_str[vo->dev->mode]);
		MSG("vout 0x%x,capability 0x%x\n",
			(int)vo->dev->vout, vo->dev->capability);
	}
}
#endif

int vo_i2c_proc(int id, unsigned int addr, unsigned int index,
			char *pdata, int len)
{
	struct swi2c_handle_s *handle = 0;
	int ret = 0;

	switch (id) {
	case 1:	/* dvi */
		if (lcd_get_type())	/* share pin with LVDS */
			return -1;
		handle = &vo_swi2c_dvi;
		break;
	default:
		break;
	}

	if (handle) {
		if (wmt_swi2c_check(handle))
			return -1;
		if (addr & 0x1) { /* read */
			*pdata = 0xff;
#ifdef CONFIG_WMT_EDID
			ret = wmt_swi2c_read(handle, addr & ~0x1,
						index, pdata, len);
#else
			ret = -1;
#endif
		} else { /* write */
			DBG_ERR("not support sw i2c write\n");
		}
	}
	return ret;
}

#ifndef CONFIG_UBOOT
static void vo_do_plug(struct work_struct *ptr)
{
	struct vout_t *vo;
	int plugin;

	if (vo_plug_func == 0)
		return;

	vo = vout_get_entry(vo_plug_vout);
	govrh_set_dvo_enable(vo->govr, 1);
	plugin = vo_plug_func(1);
/*	govrh_set_dvo_enable(vo->govr, plugin); */
	vout_change_status(vo, VPP_VOUT_STS_PLUGIN, plugin);
	vo_plug_flag = 0;
	DBG_DETAIL("vo_do_plug %d\n", plugin);
	/* GPIO irq enable */
	vppif_reg32_write(GPIO_BASE_ADDR + 0x300 + VPP_VOINT_NO, 0x80, 7, 1);
	/* GPIO input mode */
	vppif_reg32_write(GPIO_BASE_ADDR + 0x80, 0x1 << VPP_VOINT_NO,
				VPP_VOINT_NO, 0x0);
#ifdef __KERNEL__
	vpp_netlink_notify_plug(VPP_VOUT_NUM_DVI, plugin);
#endif
	return;
}

DECLARE_DELAYED_WORK(vo_plug_work, vo_do_plug);

static irqreturn_t vo_plug_interrupt_routine
(
	int irq,		/*!<; // irq id */
	void *dev_id		/*!<; // device id */
)
{
	DBG_DETAIL("Enter\n");
	if ((inb(GPIO_BASE_ADDR + 0x360) &
		(0x1 << VPP_VOINT_NO)) == 0)
		return IRQ_NONE;

	/* clear int status */
	outb(0x1 << VPP_VOINT_NO, GPIO_BASE_ADDR + 0x360);
#ifdef __KERNEL__
	/* if (vo_plug_flag == 0) { */
	/* GPIO irq disable */
	vppif_reg32_write(GPIO_BASE_ADDR + 0x300 + VPP_VOINT_NO, 0x80, 7, 0);
	schedule_delayed_work(&vo_plug_work, HZ/5);
	vo_plug_flag = 1;
	/* } */
#else
	if (vo_plug_func)
		vo_do_plug(0);
#endif
	return IRQ_HANDLED;
}

#define CONFIG_VO_POLL_WORKQUEUE
struct timer_list vo_poll_timer;
#ifdef CONFIG_VO_POLL_WORKQUEUE
static void vo_do_poll(struct work_struct *ptr)
{
	struct vout_t *vo;

	vo = vo_poll_vout;
	if (vo) {
		if (vo->dev)
			vo->dev->poll();
		mod_timer(&vo_poll_timer,
			jiffies + msecs_to_jiffies(vo_poll_timer.data));
	}
	return;
}

DECLARE_DELAYED_WORK(vo_poll_work, vo_do_poll);
#else
struct tasklet_struct vo_poll_tasklet;
static void vo_do_poll_tasklet
(
	unsigned long data		/*!<; // tasklet input data */
)
{
	struct vout_t *vo;

	vpp_lock();
	vo = vo_poll_vout;
	if (vo) {
		if (vo->dev)
			vo->dev->poll();
		mod_timer(&vo_poll_timer,
			jiffies + msecs_to_jiffies(vo_poll_timer.data));
	}
	vpp_unlock();
}
#endif

void vo_do_poll_tmr(int ms)
{
#ifdef CONFIG_VO_POLL_WORKQUEUE
	schedule_delayed_work(&vo_poll_work, msecs_to_jiffies(ms));
#else
	tasklet_schedule(&vo_poll_tasklet);
#endif
}

static void vo_set_poll(struct vout_t *vo, int on, int ms)
{
	DMSG("%d\n", on);

	if (on) {
		vo_poll_vout = vo;
		if (vo_poll_timer.function) {
			vo_poll_timer.data = ms / 2;
			mod_timer(&vo_poll_timer,
				jiffies + msecs_to_jiffies(vo_poll_timer.data));
		} else {
			init_timer(&vo_poll_timer);
			vo_poll_timer.data = ms / 2;
			vo_poll_timer.function = (void *) vo_do_poll_tmr;
			vo_poll_timer.expires = jiffies +
				msecs_to_jiffies(vo_poll_timer.data);
			add_timer(&vo_poll_timer);
		}
#ifndef CONFIG_VO_POLL_WORKQUEUE
		tasklet_init(&vo_poll_tasklet, vo_do_poll_tasklet, 0);
#endif
	} else {
		del_timer(&vo_poll_timer);
#ifndef CONFIG_VO_POLL_WORKQUEUE
		tasklet_kill(&vo_poll_tasklet);
#endif
		vo_poll_vout = 0;
	}
}
#endif

void vout_set_int_type(int type)
{
	unsigned char reg;

	reg = inb(GPIO_BASE_ADDR + 0x300 + VPP_VOINT_NO);
	reg &= ~0x7;
	switch (type) {
	case 0:	/* low level */
	case 1:	/* high level */
	case 2:	/* falling edge */
	case 3:	/* rising edge */
	case 4:	/* rising edge or falling */
		reg |= type;
		break;
	default:
		break;
	}
	outb(reg, GPIO_BASE_ADDR + 0x300 + VPP_VOINT_NO);
}
EXPORT_SYMBOL(vout_set_int_type);

void vout_set_int_enable(int enable)
{
	vppif_reg32_write(GPIO_BASE_ADDR + 0x300 +
		VPP_VOINT_NO, 0x80, 7, enable);	/* GPIO irq enable/disable */
}
EXPORT_SYMBOL(vout_set_int_enable);

int vout_get_clr_int(void)
{
	if ((inb(GPIO_BASE_ADDR + 0x360) &
		(0x1 << VPP_VOINT_NO)) == 0)
		return 1;
	/* clear int status */
	outb(0x1 << VPP_VOINT_NO, GPIO_BASE_ADDR + 0x360);
	return 0;
}
EXPORT_SYMBOL(vout_get_clr_int);

static void vo_plug_enable(int enable, void *func, int no)
{
	struct vout_t *vo;

	DBG_DETAIL("%d\n", enable);
	vo_plug_vout = no;
	vo = vout_get_entry(no);
#ifdef CONFIG_WMT_EXT_DEV_PLUG_DISABLE
	vo_plug_func = 0;
	govrh_set_dvo_enable(vo->govr, enable);
#else
	vo_plug_func = func;
	if (vo_plug_func == 0)
		return;

	if (enable) {
		vppif_reg32_write(GPIO_BASE_ADDR + 0x40, 0x1 << VPP_VOINT_NO,
			VPP_VOINT_NO, 0x0); /* GPIO disable */
		vppif_reg32_write(GPIO_BASE_ADDR + 0x80, 0x1 << VPP_VOINT_NO,
			VPP_VOINT_NO, 0x0); /* GPIO input mode */
		vppif_reg32_write(GPIO_BASE_ADDR + 0x480, 0x1 << VPP_VOINT_NO,
			VPP_VOINT_NO, 0x1); /* GPIO pull enable */
		vppif_reg32_write(GPIO_BASE_ADDR + 0x4c0, 0x1 << VPP_VOINT_NO,
			VPP_VOINT_NO, 0x1); /* GPIO pull-up */
#ifndef CONFIG_UBOOT
		vo_do_plug(0);
		if (vpp_request_irq(IRQ_GPIO, vo_plug_interrupt_routine,
			IRQF_SHARED, "vo plug", (void *) &vo_plug_vout))
			DBG_ERR("request GPIO ISR fail\n");

		vppif_reg32_write(GPIO_BASE_ADDR + 0x300 + VPP_VOINT_NO,
			0x80, 7, 1); /* GPIO irq enable */
	} else {
		vpp_free_irq(IRQ_GPIO, (void *) &vo_plug_vout);
#endif
	}
#endif
}

/*--------------------------------- DVI ------------------------------------*/
#ifdef WMT_FTBLK_VOUT_DVI
static int vo_dvi_blank(struct vout_t *vo, enum vout_blank_t arg)
{
	DMSG("(%d, %d)\n", vo->pre_blank, arg);
        if(vo->pre_blank == arg)
                return 0;

        if (vo->pre_blank == VOUT_BLANK_POWERDOWN) {
                if (vo->dev) {
#ifdef __KERNEL__
                        if (!g_vpp.dvi_int_disable && vo->dev->interrupt)
                                vo_plug_enable(VPP_FLAG_ENABLE,
                                                vo->dev->interrupt, vo->num);
                        else if (vo->dev->poll)
                                vo_set_poll(vo, (vo->dev->poll) ? 1 : 0,
                                        DVI_POLL_TIME_MS);
#endif
                }
        }
#ifdef __KERNEL__
        if (arg == VOUT_BLANK_POWERDOWN)
                vo_set_poll(vo, 0, 0);
#endif
        if (!lcd_get_dev()) /* enable DVO not contain LCD */
                govrh_set_dvo_enable(vo->govr,
                                (arg == VOUT_BLANK_UNBLANK) ? 1 : 0);
        vo->pre_blank = arg;
        return 0;
}

static int vo_dvi_config(struct vout_t *vo, int arg)
{
	struct vout_info_t *vo_info;

	DBG_DETAIL("Enter\n");

	vo_info = (struct vout_info_t *) arg;
	govrh_set_dvo_sync_polar(vo->govr,
		(vo_info->option & VPP_DVO_SYNC_POLAR_HI) ? 0 : 1,
		(vo_info->option & VPP_DVO_VSYNC_POLAR_HI) ? 0 : 1);
	return 0;
}

static int vo_dvi_init(struct vout_t *vo, int arg)
{
	unsigned int clk_delay;

	DBG_DETAIL("(%d)\n", arg);

	vo->pre_blank = VOUT_BLANK_POWERDOWN;

	lvds_set_enable(0);
	govrh_set_dvo_color_format(vo->govr, vo->option[0]);
	govrh_set_dvo_outdatw(vo->govr, vo->option[1] & WMT_DISP_FB_DVI_24BIT);
	govrh_IGS_set_mode(vo->govr, 0, WMT_DISP_FB_GET_RGB_MODE(vo->option[1]),
		(vo->option[1] & WMT_DISP_FB_MSB) ? 1 : 0);
	govrh_IGS_set_RGB_swap(vo->govr, WMT_DISP_FB_RGB_SWAP(vo->option[1]));
	clk_delay = (vo->option[1] & WMT_DISP_FB_DVI_24BIT) ?
		VPP_GOVR_DVO_DELAY_24 : VPP_GOVR_DVO_DELAY_12;
	govrh_set_dvo_clock_delay(vo->govr, ((clk_delay & BIT14) != 0x0),
		clk_delay & 0x3FFF);
	if (vo->dev) {
		vo->dev->set_mode(&vo->option[0]);
		vo->dev->set_power_down(VPP_FLAG_DISABLE);
		if (!g_vpp.dvi_int_disable && vo->dev->interrupt)
			vo_plug_enable(VPP_FLAG_ENABLE,
					vo->dev->interrupt, vo->num);
#ifdef __KERNEL__
		else if (vo->dev->poll) {
			vo_set_poll(vo, (vo->dev->poll) ? 1 : 0,
					DVI_POLL_TIME_MS);
		}
#endif
		vout_change_status(vo, VPP_VOUT_STS_PLUGIN,
				vo->dev->check_plugin(0));
	}
	vo->govr->fb_p->set_csc(vo->govr->fb_p->csc_mode);
	if (!lcd_get_dev()) {
		govrh_set_dvo_enable(vo->govr,
			(vo->status & VPP_VOUT_STS_BLANK) ? 0 : 1);
	}
	return 0;
}

static int vo_dvi_uninit(struct vout_t *vo, int arg)
{
	DBG_DETAIL("(%d)\n", arg);

	vo_plug_enable(VPP_FLAG_DISABLE, 0, VPP_VOUT_NUM);
	govrh_set_dvo_enable(vo->govr, VPP_FLAG_DISABLE);
#ifdef __KERNEL__
	vo_set_poll(vo, 0, DVI_POLL_TIME_MS);
#endif
	return 0;
}

static int vo_dvi_chkplug(struct vout_t *vo, int arg)
{
	int plugin = 1;

	DBG_MSG("plugin %d\n", plugin);
	return plugin;
}

static int vo_dvi_get_edid(struct vout_t *vo, int arg)
{
	char *buf;
	int i, cnt;

	DBG_DETAIL("Enter\n");

	buf = (char *) arg;
	memset(&buf[0], 0x0, 128 * EDID_BLOCK_MAX);
	if (vpp_i2c_read(VPP_DVI_EDID_ID, 0xA0, 0, &buf[0], 128)) {
		DBG_ERR("read edid\n");
		return 1;
	}

	if (edid_checksum(buf, 128)) {
		DBG_ERR("checksum\n");
		return 1;
	}

	cnt = buf[0x7E];
	if (cnt >= 3)
		cnt = 3;
	for (i = 1; i <= cnt; i++) {
		vpp_i2c_read(VPP_DVI_EDID_ID, 0xA0, 0x80 * i,
			&buf[128 * i], 128);
	}
	return 0;
}

struct vout_inf_t vo_dvi_inf = {
	.mode = VOUT_INF_DVI,
	.init = vo_dvi_init,
	.uninit = vo_dvi_uninit,
	.blank = vo_dvi_blank,
	.config = vo_dvi_config,
	.chkplug = vo_dvi_chkplug,
	.get_edid = vo_dvi_get_edid,
};

int vo_dvi_initial(void)
{
	vout_inf_register(VOUT_INF_DVI, &vo_dvi_inf);
	return 0;
}
module_init(vo_dvi_initial);

#endif /* WMT_FTBLK_VOUT_DVI */

/*---------------------------------- HDMI -----------------------------------*/
void vo_hdmi_set_clock(int enable)
{
	DBG_DETAIL("(%d)\n", enable);

	enable = (enable) ? CLK_ENABLE : CLK_DISABLE;
	vpp_set_clock_enable(DEV_HDMII2C, enable, 0);
	vpp_set_clock_enable(DEV_HDMI, enable, 0);
	vpp_set_clock_enable(DEV_HDCE, enable, 0);
}

#ifdef WMT_FTBLK_VOUT_HDMI
#ifdef __KERNEL__
struct timer_list hdmi_cp_timer;
static struct timer_list hdmi_plug_timer;
#endif

void vo_hdmi_cp_set_enable_tmr(int sec)
{
#ifdef __KERNEL__
	int ms = sec * 1000;
#endif

	DBG_MSG("[HDMI] set enable tmr %d sec\n", sec);

	if (sec == 0) {
		hdmi_set_cp_enable(VPP_FLAG_ENABLE);
		return ;
	}
#ifdef __KERNEL__
	if (hdmi_cp_timer.function)
		del_timer(&hdmi_cp_timer);
	init_timer(&hdmi_cp_timer);
	hdmi_cp_timer.data = VPP_FLAG_ENABLE;
	hdmi_cp_timer.function = (void *) hdmi_set_cp_enable;
	hdmi_cp_timer.expires = jiffies + msecs_to_jiffies(ms);
	add_timer(&hdmi_cp_timer);
#else
	hdmi_set_cp_enable(VPP_FLAG_ENABLE);
#endif
}
EXPORT_SYMBOL(vo_hdmi_cp_set_enable_tmr);

static int vo_hdmi_blank(struct vout_t *vo, enum vout_blank_t arg)
{
	int enable;

	DBG_DETAIL("(%d)\n", arg);

	enable = (arg == VOUT_BLANK_UNBLANK) ? 1 : 0;
	if (g_vpp.hdmi_cp_enable && enable)
		vo_hdmi_cp_set_enable_tmr(2);
	else
		hdmi_set_cp_enable(VPP_FLAG_DISABLE);
	hdmi_set_enable(enable);
	hdmi_set_power_down((enable) ? 0 : 1);
	return 0;
}

#ifndef CFG_LOADER
static irqreturn_t vo_hdmi_cp_interrupt
(
	int irq,		/*!<; // irq id */
	void *dev_id		/*!<; // device id */
)
{
	struct vout_t *vo;

	DBG_DETAIL("%d\n", irq);
	vo = vout_get_entry(VPP_VOUT_NUM_HDMI);
	switch (hdmi_check_cp_int()) {
	case 1:
		if (hdmi_cp)
			hdmi_cp->enable(VPP_FLAG_DISABLE);
		vo_hdmi_cp_set_enable_tmr(HDMI_CP_TIME);
		vout_change_status(vo, VPP_VOUT_STS_CONTENT_PROTECT, 0);
		vpp_netlink_notify_cp(0);
		break;
	case 0:
		vout_change_status(vo, VPP_VOUT_STS_CONTENT_PROTECT, 1);
		vpp_netlink_notify_cp(1);
		break;
	case 2:
		hdmi_ri_tm_cnt = 3 * 30;
		break;
	default:
		break;
	}
	return IRQ_HANDLED;
}

#ifdef __KERNEL__
static void vo_hdmi_do_plug(struct work_struct *ptr)
#else
static void vo_hdmi_do_plug(void)
#endif
{
	struct vout_t *vo;
	int plugin;
	int option = 0;

	plugin = hdmi_check_plugin(1);
	vo = vout_get_entry(VPP_VOUT_NUM_HDMI);
	vout_change_status(vo, VPP_VOUT_STS_PLUGIN, plugin);
	if (plugin) {
		option = vout_get_edid_option(VPP_VOUT_NUM_HDMI);
#ifdef CONFIG_VPP_DEMO
		option |= (EDID_OPT_HDMI + EDID_OPT_AUDIO);
#endif
		hdmi_set_option(option);
	} else {
		g_vpp.hdmi_bksv[0] = g_vpp.hdmi_bksv[1] = 0;
	}
	vo_hdmi_blank(vo, (vo->status & VPP_VOUT_STS_BLANK) ? 1 : !(plugin));
	if (!g_vpp.hdmi_certify_flag)
		hdmi_hotplug_notify(plugin);
	DBG_MSG("%d\n", plugin);
	return;
}
DECLARE_WORK(vo_hdmi_plug_work, vo_hdmi_do_plug);

static void hdmi_handle_plug(vpp_flag_t enable)
{
	schedule_work(&vo_hdmi_plug_work);
}

static void vo_hdmi_handle_plug_tmr(int ms)
{
	static int timer_init;

	if (timer_init == 0) {
		init_timer(&hdmi_plug_timer);
		hdmi_plug_timer.data = VPP_FLAG_ENABLE;
		hdmi_plug_timer.function = (void *) hdmi_handle_plug;
		timer_init = 1;
	}
	hdmi_plug_timer.expires = jiffies + msecs_to_jiffies(ms);
	mod_timer(&hdmi_plug_timer, hdmi_plug_timer.expires);
}

static irqreturn_t vo_hdmi_plug_interrupt
(
	int irq,		/*!<; // irq id */
	void *dev_id		/*!<; // device id */
)
{
	DBG_MSG("vo_hdmi_plug_interrupt %d\n", irq);
	hdmi_clear_plug_status();
	if (g_vpp.hdmi_certify_flag)
		vo_hdmi_do_plug(0);
	else
		vo_hdmi_handle_plug_tmr(HDMI_PLUG_DELAY);
	return IRQ_HANDLED;
}
#endif

static int vo_hdmi_init(struct vout_t *vo, int arg)
{
	DBG_DETAIL("(%d)\n", arg);

	vo_hdmi_set_clock(1);
	vout_change_status(vout_get_entry(VPP_VOUT_NUM_HDMI),
		VPP_VOUT_STS_PLUGIN, hdmi_check_plugin(0));
	hdmi_enable_plugin(1);

	if (g_vpp.hdmi_disable)
		return 0;
#ifndef CONFIG_UBOOT
	if (vpp_request_irq(VPP_IRQ_HDMI_CP, vo_hdmi_cp_interrupt,
		SA_INTERRUPT, "hdmi cp", (void *) 0)) {
		DBG_ERR("*E* request HDMI ISR fail\n");
	}
	if (vpp_request_irq(VPP_IRQ_HDMI_HPDH, vo_hdmi_plug_interrupt,
		SA_INTERRUPT, "hdmi plug", (void *) 0)) {
		DBG_ERR("*E* request HDMI ISR fail\n");
	}
	if (vpp_request_irq(VPP_IRQ_HDMI_HPDL, vo_hdmi_plug_interrupt,
		SA_INTERRUPT, "hdmi plug", (void *) 0)) {
		DBG_ERR("*E* request HDMI ISR fail\n");
	}
#endif
	hdmi_set_enable((vo->status & VPP_VOUT_STS_BLANK) ?
		VPP_FLAG_DISABLE : VPP_FLAG_ENABLE);
	return 0;
}

static int vo_hdmi_uninit(struct vout_t *vo, int arg)
{
	DBG_DETAIL("(%d)\n", arg);
	hdmi_enable_plugin(0);
	hdmi_set_cp_enable(VPP_FLAG_DISABLE);
	hdmi_set_enable(VPP_FLAG_DISABLE);
#ifndef CONFIG_UBOOT
	vpp_free_irq(VPP_IRQ_HDMI_CP, (void *) 0);
	vpp_free_irq(VPP_IRQ_HDMI_HPDH, (void *) 0);
	vpp_free_irq(VPP_IRQ_HDMI_HPDL, (void *) 0);
#endif
	vo_hdmi_set_clock(0);
	return 0;
}

static int vo_hdmi_config(struct vout_t *vo, int arg)
{
	struct vout_info_t *vo_info;
	vdo_color_fmt colfmt;

	hdmi_set_enable(0);
	vo_info = (struct vout_info_t *) arg;

	DBG_DETAIL("(%dx%d@%d)\n", vo_info->resx, vo_info->resy, vo_info->fps);

	/* 1280x720@60, HDMI pixel clock 74250060 not 74500000 */
	if ((vo_info->resx == 1280)
		&& (vo_info->resy == 720) && (vo_info->pixclk == 74500000))
		vo_info->pixclk = 74250060;
	colfmt = (vo->option[0] == VDO_COL_FMT_YUV422V) ?
		VDO_COL_FMT_YUV422H : vo->option[0];
	hdmi_cur_plugin = hdmi_check_plugin(0);
	hdmi_info.option = (hdmi_cur_plugin) ?
		vout_get_edid_option(VPP_VOUT_NUM_HDMI) : 0;
	hdmi_info.outfmt = colfmt;
	hdmi_info.vic = hdmi_get_vic(vo_info->resx, vo_info->resy,
		vo_info->fps, (vo_info->option & VPP_OPT_INTERLACE) ? 1 : 0);

	govrh_set_csc_mode(vo->govr, vo->govr->fb_p->csc_mode);
	hdmi_set_sync_low_active((vo_info->option & VPP_DVO_SYNC_POLAR_HI) ?
		0 : 1, (vo_info->option & VPP_DVO_VSYNC_POLAR_HI) ? 0 : 1);
	hdmi_config(&hdmi_info);
#ifdef __KERNEL__
	mdelay(200);	/* patch for VIZIO change resolution issue */
#endif
	hdmi_cur_plugin = hdmi_check_plugin(0);
	vo_hdmi_blank(vo, (vo->status & VPP_VOUT_STS_BLANK) ?
		1 : !(hdmi_cur_plugin));
	return 0;
}

static int vo_hdmi_chkplug(struct vout_t *vo, int arg)
{
	int plugin;

	if (g_vpp.hdmi_disable)
		return 0;
	plugin = hdmi_get_plugin();
	DBG_DETAIL("%d\n", plugin);
	return plugin;
}

static int vo_hdmi_get_edid(struct vout_t *vo, int arg)
{
	char *buf;
#ifdef CONFIG_WMT_EDID
	int i, cnt;
#endif
	DBG_DETAIL("Enter\n");
	buf = (char *) arg;
#ifdef CONFIG_WMT_EDID
	memset(&buf[0], 0x0, 128*EDID_BLOCK_MAX);
	if (!hdmi_get_plugin())
		return 1;

	if (hdmi_DDC_read(0xA0, 0x0, &buf[0], 128)) {
		DBG_ERR("read edid\n");
		return 1;
	}

	if (edid_checksum(buf, 128)) {
		DBG_ERR("hdmi checksum\n");
/*		g_vpp.dbg_hdmi_ddc_crc_err++; */
		return 1;
	}

	cnt = buf[0x7E];
	if (cnt >= 3)
		cnt = 3;
	for (i = 1; i <= cnt; i++)
		hdmi_DDC_read(0xA0, 0x80 * i, &buf[128 * i], 128);
#endif
	return 0;
}

struct vout_inf_t vo_hdmi_inf = {
	.mode = VOUT_INF_HDMI,
	.init = vo_hdmi_init,
	.uninit = vo_hdmi_uninit,
	.blank = vo_hdmi_blank,
	.config = vo_hdmi_config,
	.chkplug = vo_hdmi_chkplug,
	.get_edid = vo_hdmi_get_edid,
};

int vo_hdmi_initial(void)
{
	vout_inf_register(VOUT_INF_HDMI, &vo_hdmi_inf);
	return 0;
}
module_init(vo_hdmi_initial);

#endif /* WMT_FTBLK_VOUT_HDMI */

/*--------------------------------- LVDS ------------------------------------*/
#ifdef WMT_FTBLK_VOUT_LVDS
int vo_lvds_init_flag;
static int vo_lvds_blank(struct vout_t *vo, enum vout_blank_t arg)
{
	DBG_DETAIL("(%d)\n", arg);
	if (arg == VOUT_BLANK_POWERDOWN) {
		lvds_regs->test.b.tre_en = 0;
	} else { /* avoid suspend signal not clear */
		lvds_set_enable((arg == VOUT_BLANK_UNBLANK) ? 1 : 0);
	}

	if (vo_lvds_init_flag)
		lvds_set_power_down(arg);
	return 0;
}

static int vo_lvds_config(struct vout_t *vo, int arg)
{
	DBG_DETAIL("(%d)\n", arg);
	lvds_set_power_down(VPP_FLAG_DISABLE);
	vo_lvds_init_flag = 1;
	return 0;
}

static int vo_lvds_init(struct vout_t *vo, int arg)
{
	DBG_DETAIL("(%d)\n", arg);

	vpp_set_clock_enable(DEV_LVDS, 1, 0);
	if (vo->dev)
		vo->dev->set_mode(&vo->option[0]);
	govrh_set_dvo_enable(p_govrh2, 0);
	govrh_set_csc_mode(vo->govr, vo->govr->fb_p->csc_mode);
	lvds_set_enable((vo->status & VPP_VOUT_STS_BLANK) ?
		VPP_FLAG_DISABLE : VPP_FLAG_ENABLE);
	return 0;
}

static int vo_lvds_uninit(struct vout_t *vo, int arg)
{
	DBG_DETAIL("(%d)\n", arg);
	lvds_set_enable(VPP_FLAG_DISABLE);
	if (vo->dev)
		vo->dev->set_mode(0);
	lvds_set_power_down(VPP_FLAG_ENABLE);
	vpp_set_clock_enable(DEV_LVDS, 0, 0);
	vo_lvds_init_flag = 0;
	return 0;
}

static int vo_lvds_chkplug(struct vout_t *vo, int arg)
{
	DBG_DETAIL("\n");
#if 0
	vo = vout_get_info(VOUT_LVDS);
	if (vo->dev)
		return vo->dev->check_plugin(0);
#endif
	return 1;
}

struct vout_inf_t vo_lvds_inf = {
	.mode = VOUT_INF_LVDS,
	.capability = VOUT_INF_CAP_FIX_PLUG,
	.init = vo_lvds_init,
	.uninit = vo_lvds_uninit,
	.blank = vo_lvds_blank,
	.config = vo_lvds_config,
	.chkplug = vo_lvds_chkplug,
#ifdef WMT_FTBLK_VOUT_HDMI
	.get_edid = vo_hdmi_get_edid,
#endif
};

int vo_lvds_initial(void)
{
	vout_inf_register(VOUT_INF_LVDS, &vo_lvds_inf);
	return 0;
}
module_init(vo_lvds_initial);

#endif /* WMT_FTBLK_VOUT_LVDS */
/*---------------------------------- API ------------------------------------*/
#ifndef CFG_LOADER
int vout_set_audio(struct vout_audio_t *arg)
{
	struct vout_t *vout;
	int ret = 0;

#if 0
	vout = vout_get_info(VPP_VOUT_DVO2HDMI);
	if (vout && (vout->status & VPP_VOUT_STS_PLUGIN)) {
		if (vout->dev->set_audio)
			vout->dev->set_audio(arg);
	}
#endif

#ifdef WMT_FTBLK_VOUT_HDMI
	vout = vout_get_entry(VPP_VOUT_NUM_HDMI);
	if (vout) {
		g_vpp.hdmi_ch_change = 1;
		hdmi_config_audio(arg);
		g_vpp.hdmi_ch_change = 0;
		ret = 1;
	}
#endif
	return ret;
}
#endif

/* 3445 port1 : DVI/SDD, port2 : VGA/SDA, port3 : HDMI/LVDS */
/* 3481 port1 : HDMI/LVDS, port2 : DVI */
/* 3498 port1 : HDMI, port2 : DVI/LVDS */
struct vout_t vout_entry_0 = {
	.fix_cap = BIT(VOUT_INF_HDMI),
	.option[0] = VDO_COL_FMT_ARGB,
	.option[1] = VPP_DATAWIDHT_24,
	.option[2] = 0,
};

struct vout_t vout_entry_1 = {
	.fix_cap = BIT(VOUT_INF_DVI) + BIT(VOUT_INF_LVDS) +
		VOUT_CAP_EXT_DEV + 0x100,	/* i2c bus 1,ext dev */
	.option[0] = VDO_COL_FMT_ARGB,
	.option[1] = VPP_DATAWIDHT_24,
	.option[2] = 0,
};

int vout_add_display(int fb_no, unsigned int *parm)
{
	struct vout_info_t *info;
	struct vout_t *vout;
	int ret = 0;

	info = vout_info[fb_no];
	if (!info) {
		info = kmalloc(sizeof(struct vout_info_t), GFP_KERNEL);
		if (!info)
			return 1;
		memset(info, 0, sizeof(struct vout_info_t));
		vout_info[fb_no] = info;
		DBG_MSG("malloc vout_info %d,0x%x\n", fb_no, (int) info);
#ifdef CONFIG_KERNEL
		sema_init(&info->sem, 1);
#endif
	}

	if (parm[0] == VOUT_BOOT) {
		struct vout_t *vo_boot;

		MSG("[VOUT] %s (%d:%d:%d)\n",
			(fb_no == 0) ? "tvbox" : "virtual display",
			parm[0], parm[1], parm[2]);
		if (fb_no == 0) {
			g_vpp.virtual_display = 1;
			g_vpp.fb0_bitblit = 1;
		} else {
			g_vpp.stream_fb = fb_no;
		}
		vo_boot = kmalloc(sizeof(struct vout_t), GFP_KERNEL);
		if (vo_boot == 0)
			return 1;
		memset(vo_boot, 0, sizeof(struct vout_t));
		vo_boot->resx = parm[3];
		vo_boot->resy = parm[4];
		vo_boot->fps = parm[5];
		vo_boot->num = VPP_VOUT_NUM;
		vout_info_add_entry(fb_no, vo_boot);
		kfree(vo_boot);
		return 0;
	}

	vout = vout_get_entry_adapter(parm[0]);
	vout->inf = vout_get_inf_entry_adapter(parm[0]);
	vout->option[0] = parm[1];
	vout->option[1] = parm[2];
	vout->resx = parm[3];
	vout->resy = parm[4];
	vout->fps = parm[5];
	vout->disable = (parm[2] & VOUT_OPT_BLANK) ? 1 : 0;
	switch (parm[0]) {
	case VOUT_LVDS:
		{
		struct fb_videomode *vmode = 0;

		/* lvds auto detect edid */
		if ((parm[1] == 0) && (parm[3] == 0) && (parm[4] == 0)) {
			if (vout_get_edid_option(vout->num)) {
				vmode = &vout->edid_info.detail_timing[0];
				if (vmode->pixclock == 0) {
					vmode = 0;
					DBG_ERR("LVDS timing\n");
				}
			}

			if (vout->inf->get_edid(vout, (int)vout->edid) == 0) {
				if (edid_parse(vout->edid,
					&vout->edid_info) == 0)
					DBG_ERR("LVDS edid parse\n");
			} else {
				DBG_ERR("LVDS edid read\n");
			}
		}

		if (vmode == 0) { /* use internal timing */
			struct lcd_parm_t *p = 0;

			if (parm[1]) {
				p = lcd_get_parm(parm[1], parm[2]);
				if (p)
					lcd_set_lvds_id(parm[1]);
			}

			if (p == 0)
				p = lcd_get_oem_parm(parm[3], parm[4]);
			vmode = &p->vmode;
		}
		vout->option[2] = vmode->vmode;
		info->resx = vmode->xres;
		info->resy = vmode->yres;
		info->fps  = vmode->refresh;
		vout_info_set_fixed_timing(fb_no, vmode);
		lcd_set_type(1);
		}
	case VOUT_LCD:
		{
		struct vout_dev_t *dev;

		lcd_set_parm(parm[1], parm[2] & 0xFF);
		dev = lcd_get_dev();
		vout->dev = dev;
		dev->vout = vout;
		vout->option[0] = VDO_COL_FMT_ARGB;
		vout->option[1] &= ~0xFF;
		vout->option[1] |= VPP_DATAWIDHT_24;
		vout->dev->init(vout);
		vout_info_set_fixed_timing(fb_no, &p_lcd->vmode);
		}
		break;
	case VOUT_DVI:
		{
		struct vout_dev_t *dev = 0;

		g_vpp.dvi_int_disable =
			(parm[2] & WMT_DISP_FB_DISBALE_DVI_INT) ? 1 : 0;
		g_vpp.dvi_int_no = (parm[2] & WMT_DISP_FB_DVI_INT) ?
			((parm[1] & 0xF000) >> 12) : VPP_DVI_INT_DEFAULT;
		g_vpp.dvi_i2c_no = (parm[2] & WMT_DISP_FB_DVI_I2C) ?
			((parm[1] & 0xF00) >> 8) : VPP_DVI_I2C_DEFAULT;
		g_vpp.dvi_i2c_no &= VPP_DVI_I2C_ID_MASK;

		if (parm[2] & WMT_DISP_FB_DISABLE_EXTDEV)
			vout->dev = 0;
		else {
			vpp_i2c_init(VPP_DVI_I2C_ID, 0xA0);
			do {
				dev = vout_get_device(dev);
				if (dev == 0)
					break;
				if (vout->fix_cap & BIT(dev->mode)) {
					vout->inf =
						vout_inf_get_entry(dev->mode);
					if (dev->init(vout) == 0) {
						vout->dev = dev;
						dev->vout = vout;
						break;
					}
				}
			} while (1);
		}

		DBG_MSG("DVI ext dev : %s\n",
			(vout->dev) ? vout->dev->name : "NO");
		}
		info->option = (parm[2] & WMT_DISP_FB_INTERLACE) ?
			VPP_OPT_INTERLACE : 0;
		break;
	case VOUT_HDMI:
		info->option = (parm[2] & WMT_DISP_FB_INTERLACE) ?
			VPP_OPT_INTERLACE : 0;
#if 0 /* use old uboot param and wait next chip */
		g_vpp.hdmi_disable =
			(parm[2] & WMT_DISP_FB_HDMI_DISABLE) ? 1 : 0;

		g_vpp.hdmi_sp_mode =
			(parm[2] & WMT_DISP_FB_HDMI_SP_MODE) ? 1 : 0;
#endif
		break;
	default:
		break;
	}

	if (ret == 0)
		vout_info_add_entry(fb_no, vout);
	return ret;
}

int vout_check_display(void)
{
	#define BUF_LEN 100
	char buf[BUF_LEN];
	int varlen = BUF_LEN;
	unsigned int parm[32];
	int i, idx;

	if (wmt_getsyspara("wmt.display.fb0", buf, &varlen)) {
		/* default for no uboot parameter */
		parm[0] = VOUT_HDMI;
		parm[1] = VDO_COL_FMT_ARGB;
		parm[2] = VPP_DATAWIDHT_24;
		parm[3] = 1280;
		parm[4] = 720;
		parm[5] = 60;
		vout_add_display(0, &parm[0]);
		parm[0] = VOUT_DVI;
		parm[1] = VDO_COL_FMT_ARGB;
		parm[2] = VPP_DATAWIDHT_24;
		parm[3] = 1024;
		parm[4] = 768;
		parm[5] = 60;
		vout_add_display(0, &parm[0]);
		return 1;
	} else {
		int fb_no = 0;
		int num;

		while (fb_no < VPP_VOUT_INFO_NUM) {
			sprintf(buf, "wmt.display.fb%d", fb_no);
			varlen = BUF_LEN;
			if (wmt_getsyspara(buf, buf, &varlen))
				break;

			DBG_DETAIL("fb%d : %s\n", fb_no, buf);
			varlen = vpp_parse_param(buf,
				(unsigned int *)parm, 32, 0x1C1C1C1D);
			DBG_DETAIL("op 0x%x\n", parm[0]);
			num = (varlen - 1) / 7;
			for (i = 0; i < num; i++) {
				idx = 1 + 8 * i; /* [ + 6 + ] = 8 */
				DBG_DETAIL("%d : %x, %x, %x (%dx%d@%d)\n", i,
					parm[idx + 1], parm[idx + 2],
					parm[idx + 3], parm[idx + 4],
					parm[idx + 5], parm[idx + 6]);
				vout_add_display(fb_no, &parm[idx + 1]);
			}

			vout_info[fb_no]->multi =
				(parm[0] & WMT_DISP_FB_MULTI) ? 1 : 0;
			vout_info[fb_no]->alloc_mode = (parm[0] & 0xF);
			vout_info[fb_no]->hwc_mode = (parm[0] & 0xF0) >> 4;
			if (parm[0] & WMT_DISP_FB_COLFMT)
				vout_info[fb_no]->fb.col_fmt = ((parm[0] &
					WMT_DISP_FB_COLFMT_MASK) >> 16);
			fb_no++;
		}
	}

	/* [uboot parameter] oem timing :
		pixclk:option:hsync:hbp:hpixel:hfp:vsync:vbp:vpixel:vfp */
	varlen = BUF_LEN;
	if (wmt_getsyspara("wmt.display.tmr", buf, &varlen) == 0) {
		struct fb_videomode *p;
		int xres, yres;
		struct fb_videomode vo_oem_vmode;

		p = &vo_oem_vmode;
		DBG_MSG("tmr %s\n", buf);
		vpp_parse_param(buf, parm, 12, 0);
		p->pixclock = parm[0];
		p->vmode = parm[1];
		p->hsync_len = parm[2];
		p->left_margin = parm[3];
		p->xres = parm[4];
		p->right_margin = parm[5];
		p->vsync_len = parm[6];
		p->upper_margin = parm[7];
		p->yres = parm[8];
		p->lower_margin = parm[9];
		p->pixclock *= 1000;
		xres = p->hsync_len + p->left_margin + p->xres +
			p->right_margin;
		yres = p->vsync_len + p->upper_margin + p->yres +
			p->lower_margin;
		p->refresh = vpp_calc_refresh(p->pixclock, xres, yres);
		if (p->refresh == 59)
			p->refresh = 60;
		p->vmode = (parm[1] & VPP_OPT_INTERLACE) ? FB_VMODE_INTERLACED : 0;
		p->sync = (parm[1] & VPP_DVO_SYNC_POLAR_HI) ? FB_SYNC_HOR_HIGH_ACT : 0;
		p->sync |= (parm[1] & VPP_DVO_VSYNC_POLAR_HI) ? FB_SYNC_VERT_HIGH_ACT : 0;
		DBG_MSG("tmr pixclk %d,option 0x%x\n",
			p->pixclock, p->vmode);
		DBG_MSG("H sync %d,bp %d,pixel %d,fp %d\n", p->hsync_len,
			p->left_margin,	p->xres, p->right_margin);
		DBG_MSG("V sync %d,bp %d,pixel %d,fp %d\n", p->vsync_len,
			p->upper_margin, p->yres, p->lower_margin);
		p->pixclock = KHZ2PICOS(p->pixclock / 1000);
		vout_info_set_fixed_timing(0, &vo_oem_vmode);
		vout_info[0]->fixed_width = parm[10];
		vout_info[0]->fixed_height = parm[11];
		vout_info[0]->resx = p->xres;
		vout_info[0]->resy = p->yres;
		vout_info[0]->fps = p->refresh;
		vout_info[0]->vout[0]->resx = p->xres;
		vout_info[0]->vout[0]->resy = p->yres;
		vout_info[0]->vout[0]->fps = p->refresh;
	}
	return 0;
}

int vout_init(void)
{
	struct vout_info_t *info;
	struct vout_t *vout;
	int i, j;

	DBG_DETAIL("Enter\n");

	for(i = 0; i < VPP_VOUT_INFO_NUM; i++) {
		if(vout_info[i] != NULL) {
			kfree(vout_info[i]);
			vout_info[i] = NULL;
		}
	}

	/* register vout & set default */
	vout_register(0, &vout_entry_0);
	vout_entry_0.inf = vout_inf_get_entry(VOUT_INF_HDMI);
	vout_entry_0.govr = p_govrh;
	vout_register(1, &vout_entry_1);
	vout_entry_1.inf = vout_inf_get_entry(VOUT_INF_DVI);
	vout_entry_1.govr = p_govrh2;

	/* check vout info */
	DBG_DETAIL("check display\n");
	vout_check_display();

	/* initial vout */
	DBG_DETAIL("init display\n");
	for (i = 0; i < VPP_VOUT_INFO_NUM; i++) {
		info = vout_info[i];
		if (!info)
			break;
		for (j = 0; ; j++) {
			vout = info->vout[j];
			if (vout == 0)
				break;
			if (vout->inf)
				vout->inf->init(vout, 0);
		}
	}

	/* check monitor resolution */
	DBG_DETAIL("check resolution\n");
	for (i = 0; i < VPP_VOUT_INFO_NUM; i++) {
		struct vout_t *vout_first = 0;
		struct vout_t *vout_plug = 0;

		info = vout_info[i];
		if (!info)
			break;
		for (j = 0; ; j++) {
			vout = info->vout[j];
			if (vout == 0)
				break;

			if (vout_first == 0) /* first priority */
				vout_first = vout;

			if (vout_chkplug(vout->num)) {
				struct fb_videomode vmode;

				vmode.xres = vout->resx;
				vmode.yres = vout->resy;
				vmode.refresh = vout->fps;
				vmode.vmode = 0;
				vout_find_match_mode(i, &vmode, 1);
				vout->resx = vmode.xres;
				vout->resy = vmode.yres;
				vout->fps = vmode.refresh;
				if (vout_plug == 0) /* first plugin */
					vout_plug = vout;
			}

			if (info->multi)
				vout_change_status(vout,
					VPP_VOUT_STS_ACTIVE, 1);
		}

		vout = (vout_plug) ? vout_plug : vout_first;
		if (vout) {
			vout_change_status(vout, VPP_VOUT_STS_ACTIVE, 1);
			info->resx = vout->resx;
			info->resy = vout->resy;
			info->fps = vout->fps;
		}
	}

#ifdef DEBUG
	/* show display info */
	for (i = 0; i < VPP_VOUT_INFO_NUM; i++) {
		info = vout_info[i];
		if (!info)
			break;
		MSG("-----------------------------------------------------\n");
		MSG("fb%d, resx %d,resy %d,fps %d\n", i,
			info->resx, info->resy, info->fps);
		MSG("resx_vir %d,resy_vir %d,pixclk %d\n", info->resx_virtual,
			info->resy_virtual, info->pixclk);
		MSG("multi %d,alloc %d,option 0x%x\n", info->multi,
			info->alloc_mode, info->option);

		for (j = 0; ; j++) {
			vout = info->vout[j];
			if (vout == 0)
				break;
			vout_print_entry(vout);
		}
	}
	MSG("-----------------------------------------------------\n");
#endif
	DBG_DETAIL("Leave\n");
	return 0;
}

int vout_exit(void)
{
	return 0;
}
