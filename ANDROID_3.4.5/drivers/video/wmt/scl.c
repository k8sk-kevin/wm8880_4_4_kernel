/*++
 * linux/drivers/video/wmt/scl.c
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

#define SCL_C
#undef DEBUG
/* #define DEBUG */
/* #define DEBUG_DETAIL */

#include "scl.h"

HW_REG struct scl_base1_regs *scl_regs1 = (void *) SCL_BASE_ADDR;
HW_REG struct scl_base2_regs *scl_regs2 = (void *) SCL_BASE2_ADDR;

#ifdef WMT_FTBLK_SCL
void scl_reg_dump(void)
{
	vpp_set_clock_enable(DEV_SCL444U, 1, 0);

	DPRINT("========== SCL register dump ==========\n");
	vpp_reg_dump(REG_SCL_BASE1_BEGIN,
		REG_SCL_BASE1_END - REG_SCL_BASE1_BEGIN);
	vpp_reg_dump(REG_SCL_BASE2_BEGIN,
		REG_SCL_BASE2_END - REG_SCL_BASE2_BEGIN);

	DPRINT("---------- SCL scale ----------\n");
	DPRINT("scale enable %d\n", scl_regs1->en.b.alu_enable);
	DPRINT("mode bilinear(H %d,V %d),recursive(H %d,V %d)\n",
		scl_regs1->true_bilinear.b.h, scl_regs1->true_bilinear.b.v,
		scl_regs2->recursive_mode.b.h, scl_regs2->recursive_mode.b.v);
	DPRINT("src(%d,%d),dst(%d,%d)\n",
		scl_regs1->r_h_size.b.pix_w, scl_regs1->vxwidth.b.vxwidth,
		scl_regs1->hscale1.b.thr, scl_regs1->vscale1.b.thr);
	DPRINT("scale width H %d,V %d\n",
		scl_regs1->hxwidth.b.hxwidth, scl_regs1->vxwidth.b.vxwidth);
	DPRINT("H scale up %d,V scale up %d\n",
		scl_regs1->sclup_en.b.h, scl_regs1->sclup_en.b.v);
	DPRINT("H sub step %d,thr %d,step %d,sub step cnt %d,i step cnt %d\n",
		scl_regs1->hscale1.b.substep, scl_regs1->hscale1.b.thr,
		scl_regs1->hscale2.b.step, scl_regs1->hscale2.b.substepcnt,
		scl_regs1->hscale3.b.stepcnt);
	DPRINT("V sub step %d,thr %d,step %d,sub step cnt %d,i step cnt %d\n",
		scl_regs1->vscale1.b.substep, scl_regs1->vscale1.b.thr,
		scl_regs1->vscale2.b.step, scl_regs1->vscale2.b.substepcnt,
		scl_regs1->vscale3.b.stepcnt);

	DPRINT("---------- SCL filter ----------\n");
	DPRINT("DEBLOCK %d,boundary 1st 0x%x,2nd 0x%x\n,",
		scl_regs2->field_mode.b.deblock,
		scl_regs2->dblk_threshold.b.layer1_boundary,
		scl_regs2->dblk_threshold.b.layer2_boundary);
	DPRINT("FIELD DEFLICKER %d,up %s down,thr Y %d,C %d\n",
		scl_regs2->field_mode.b.field_deflicker,
		(scl_regs2->field_mode.b.field_deflicker) ? "&" : "or",
		scl_regs2->field_flicker.b.y_thd,
		scl_regs2->field_flicker.b.c_thd);
	DPRINT("FRAME DEFLICKER %d,%s,2^%d,scene chg %d\n",
		scl_regs2->field_mode.b.frame_deflicker,
		(scl_regs2->frame_flicker.b.rgb) ? "RGB" : "Y",
		scl_regs2->frame_flicker.b.sampler,
		scl_regs2->frame_flicker.b.scene_chg_thd);
	DPRINT("CSC enable %d,CSC clamp %d\n",
		scl_regs2->csc_ctl.b.enable,
		scl_regs2->csc_ctl.b.clamp_enable);
	DPRINT("---------- SCL TG ----------\n");
	DPRINT("TG source : %s\n",
		(scl_regs1->tg_govw.b.enable) ? "GOVW" : "SCL");
	DPRINT("TG enable %d, wait ready enable %d\n",
		scl_regs1->tg_ctl.b.enable,
		scl_regs1->tg_ctl.b.watchdog_enable);
	DPRINT("clk %d,Read cyc %d,1T %d\n",
		vpp_get_base_clock(VPP_MOD_SCL),
		scl_regs1->tg_ctl.b.rdcyc, scl_regs2->readcyc_1t.b.rdcyc_1t);
	DPRINT("H total %d, beg %d, end %d\n",
		scl_regs1->tg_total.b.h_allpixel,
		scl_regs1->tg_h_active.b.h_actbg,
		scl_regs1->tg_h_active.b.h_actend);
	DPRINT("V total %d, beg %d, end %d\n",
		scl_regs1->tg_total.b.v_allline,
		scl_regs1->tg_v_active.b.v_actbg,
		scl_regs1->tg_v_active.b.v_actend);
	DPRINT("VBIE %d,PVBI %d\n",
		scl_regs1->tg_vbi.b.vbie, scl_regs1->tg_vbi.b.pvbi);
	DPRINT("Watch dog 0x%x\n",
		scl_regs1->tg_watchdog);
	DPRINT("---------- SCLR FB ----------\n");
	DPRINT("SCLR MIF enable %d\n",
		scl_regs1->r_ctl.b.mif_enable);
	DPRINT("color format %s\n", vpp_colfmt_str[sclr_get_color_format()]);
	DPRINT("color bar enable %d,mode %d,inv %d\n",
		scl_regs1->r_ctl.b.colorbar_enable,
		scl_regs1->r_ctl.b.colorbar_mode,
		scl_regs1->r_ctl.b.colorbar_inv);
	DPRINT("sourc mode : %s,H264 %d\n",
		(scl_regs1->r_ctl.b.field) ? "field" : "frame",
		scl_regs1->r_ctl.b.h264);
	DPRINT("Y addr 0x%x, C addr 0x%x\n",
		scl_regs1->r_ysa, scl_regs1->r_csa);
	DPRINT("width %d, fb width %d\n",
		scl_regs1->r_h_size.b.pix_w, scl_regs1->r_h_size.b.fb_w);
	DPRINT("H crop %d, V crop %d\n",
		scl_regs1->r_crop.b.hcrop, scl_regs1->r_crop.b.vcrop);
	DPRINT("---------- SCLW FB ----------\n");
	DPRINT("SCLW MIF enable %d\n", scl_regs1->w_ctl.b.mif_enable);
	DPRINT("color format %s\n", vpp_colfmt_str[sclw_get_color_format()]);
	DPRINT("Y addr 0x%x, C addr 0x%x\n",
		scl_regs1->w_ysa, scl_regs1->w_csa);
	DPRINT("Y width %d, fb width %d\n",
		scl_regs1->w_y_time.b.pxl_w, scl_regs1->w_y_time.b.fb_w);
	DPRINT("C width %d, fb width %d\n",
		scl_regs1->w_c_time.b.pxl_w, scl_regs1->w_c_time.b.fb_w);
	DPRINT("Y err %d, C err %d\n",
		scl_regs1->w_ff_ctl.b.mif_y_err,
		scl_regs1->w_ff_ctl.b.mif_c_err);
	DPRINT("---------- SCLR2 FB ----------\n");
	DPRINT("MIF enable %d\n", scl_regs1->r2_ctl.b.mif_en);
	DPRINT("color format %s\n", vpp_colfmt_str[scl_R2_get_color_format()]);
	DPRINT("color bar enable %d,mode %d,inv %d\n",
		scl_regs1->r2_ctl.b.color_en,
		scl_regs1->r2_ctl.b.color_wide,
		scl_regs1->r2_ctl.b.color_inv);
	DPRINT("sourc mode : %s,H264 %d\n",
		(scl_regs1->r2_ctl.b.iofmt) ? "field" : "frame",
		scl_regs1->r2_ctl.b.h264_fmt);
	DPRINT("Y addr 0x%x, C addr 0x%x\n",
		scl_regs1->r2_ysa, scl_regs1->r2_csa);
	DPRINT("width %d, fb width %d\n",
		scl_regs1->r2_h_size.b.lnsize, scl_regs1->r2_h_size.b.fbw);
	DPRINT("H crop %d, V crop %d\n",
		scl_regs1->r2_crop.b.hcrop, scl_regs1->r2_crop.b.vcrop);
	DPRINT("---------- ALPHA ----------\n");
	DPRINT("src alpha %d,dst alpha %d,swap %d\n",
		scl_regs1->alpha_md.b.src,
		scl_regs1->alpha_md.b.dst,
		scl_regs1->alpha_md.b.swap);
	DPRINT("src fix 0x%x,dst fix 0x%x\n",
		scl_regs1->alpha_fxd.b.src_fixed,
		scl_regs1->alpha_fxd.b.dst_fixed);
	DPRINT("---------- ColorKey ----------\n");
	DPRINT("enable %d\n", scl_regs1->alpha_colorkey.b.enable);
	DPRINT("from %s,comp %d,mode %d\n",
		(scl_regs1->alpha_colorkey.b.from) ? "mif2" : "mif1",
		scl_regs1->alpha_colorkey.b.comp,
		scl_regs1->alpha_colorkey.b.mode);
	DPRINT("R 0x%x,G 0x%x,B 0x%x\n",
		scl_regs1->alpha_colorkey_rgb.b.r,
		scl_regs1->alpha_colorkey_rgb.b.g,
		scl_regs1->alpha_colorkey_rgb.b.b);
	DPRINT("---------- sw status ----------\n");
	DPRINT("complete %d\n", p_scl->scale_complete);

	vpp_set_clock_enable(DEV_SCL444U, 0, 0);
}

void scl_set_enable(vpp_flag_t enable)
{
	scl_regs1->en.b.alu_enable = enable;
}

void scl_set_reg_update(vpp_flag_t enable)
{
	scl_regs1->upd.b.reg_update = enable;
}

void scl_set_reg_level(vpp_reglevel_t level)
{
	scl_regs1->sel.b.reg_level = level;
}

void scl_set_int_enable(vpp_flag_t enable, enum vpp_int_t int_bit)
{
	/* clean status first before enable/disable interrupt */
	scl_clean_int_status(int_bit);

	if (int_bit & VPP_INT_ERR_SCL_TG)
		scl_regs1->w_int_en.b.tg_err = enable;
	if (int_bit & VPP_INT_ERR_SCLR1_MIF)
		scl_regs1->w_int_en.b.r1_mif_enable = enable;
	if (int_bit & VPP_INT_ERR_SCLR2_MIF)
		scl_regs1->w_int_en.b.r2_mif_enable = enable;
	if (int_bit & VPP_INT_ERR_SCLW_MIFRGB)
		scl_regs1->w_int_en.b.mif_rgb_err = enable;
	if (int_bit & VPP_INT_ERR_SCLW_MIFY)
		scl_regs1->w_int_en.b.mif_y_err = enable;
	if (int_bit & VPP_INT_ERR_SCLW_MIFC)
		scl_regs1->w_int_en.b.mif_c_err = enable;
}

enum vpp_int_err_t scl_get_int_status(void)
{
	enum vpp_int_err_t int_sts;

	int_sts = 0;
	if (scl_regs1->tg_sts.b.tgerr)
		int_sts |= VPP_INT_ERR_SCL_TG;
	if (scl_regs1->r_fifo_ctl.b.r1_mif_err)
		int_sts |= VPP_INT_ERR_SCLR1_MIF;
	if (scl_regs1->r_fifo_ctl.b.r2_mif_err)
		int_sts |= VPP_INT_ERR_SCLR2_MIF;
	if (scl_regs1->w_ff_ctl.b.mif_rgb_err)
		int_sts |= VPP_INT_ERR_SCLW_MIFRGB;
	if (scl_regs1->w_ff_ctl.b.mif_y_err)
		int_sts |= VPP_INT_ERR_SCLW_MIFY;
	if (scl_regs1->w_ff_ctl.b.mif_c_err)
		int_sts |= VPP_INT_ERR_SCLW_MIFC;
	return int_sts;
}

void scl_clean_int_status(enum vpp_int_err_t int_sts)
{
	if (int_sts & VPP_INT_ERR_SCL_TG)
		scl_regs1->tg_sts.val = BIT0;
	if (int_sts & VPP_INT_ERR_SCLR1_MIF)
		scl_regs1->r_fifo_ctl.val =
			(scl_regs1->r_fifo_ctl.val & ~0x300) | BIT8;
	if (int_sts & VPP_INT_ERR_SCLR2_MIF)
		scl_regs1->r_fifo_ctl.val =
			(scl_regs1->r_fifo_ctl.val & ~0x300) | BIT9;
	if (int_sts & VPP_INT_ERR_SCLW_MIFRGB)
		scl_regs1->w_ff_ctl.val = BIT16;
	if (int_sts & VPP_INT_ERR_SCLW_MIFY)
		scl_regs1->w_ff_ctl.val = BIT8;
	if (int_sts & VPP_INT_ERR_SCLW_MIFC)
		scl_regs1->w_ff_ctl.val = BIT0;
}

void scl_set_csc_mode(vpp_csc_t mode)
{
	vdo_color_fmt src_fmt, dst_fmt;

	src_fmt = sclr_get_color_format();
	dst_fmt = sclw_get_color_format();
	mode = vpp_check_csc_mode(mode, src_fmt, dst_fmt, 0);
	if (p_scl->abgr_mode) {
		unsigned int parm[5];

		parm[0] = (vpp_csc_parm[mode][1] & 0xFFFF) |
			(vpp_csc_parm[mode][0] & 0xFFFF0000); /* C3,C2 */
		parm[1] = (vpp_csc_parm[mode][0] & 0xFFFF) |
			(vpp_csc_parm[mode][2] & 0xFFFF0000); /* C1,C6 */
		parm[2] = (vpp_csc_parm[mode][2] & 0xFFFF) |
			(vpp_csc_parm[mode][1] & 0xFFFF0000); /* C5,C4 */
		parm[3] = (vpp_csc_parm[mode][4] & 0xFFFF) |
			(vpp_csc_parm[mode][3] & 0xFFFF0000); /* C9,C8 */
		parm[4] = (vpp_csc_parm[mode][3] & 0xFFFF) |
			(vpp_csc_parm[mode][4] & 0xFFFF0000); /* C7,I */

		scl_regs2->csc1 = parm[0];
		scl_regs2->csc2 = parm[1];
		scl_regs2->csc3 = parm[2];
		scl_regs2->csc4 = parm[3];
		scl_regs2->csc5 = parm[4];
	} else {
		scl_regs2->csc1 = vpp_csc_parm[mode][0];
		scl_regs2->csc2 = vpp_csc_parm[mode][1];
		scl_regs2->csc3 = vpp_csc_parm[mode][2];
		scl_regs2->csc4 = vpp_csc_parm[mode][3];
		scl_regs2->csc5 = vpp_csc_parm[mode][4];
	}
	scl_regs2->csc6 = vpp_csc_parm[mode][5];
	scl_regs2->csc_ctl.val = vpp_csc_parm[mode][6];
	scl_regs2->csc_ctl.b.enable = (mode >= VPP_CSC_MAX) ? 0 : 1;
}

void scl_set_scale_enable(vpp_flag_t vscl_enable, vpp_flag_t hscl_enable)
{
	DBGMSG("V %d,H %d\n", vscl_enable, hscl_enable);
	scl_regs1->sclup_en.b.v = vscl_enable;
	scl_regs1->sclup_en.b.h = hscl_enable;
}

void scl_set_V_scale(int A, int B) /* A dst,B src */
{
	unsigned int V_STEP;
	unsigned int V_SUB_STEP;
	unsigned int V_THR_DIV2;

	DBG_DETAIL("scl_set_V_scale(%d,%d)\r\n", A, B);
	if (A > B) {
		V_STEP = (B - 1) * 16 / A;
		V_SUB_STEP = (B - 1) * 16  % A;
	} else {
		V_STEP = (16 * B / A);
		V_SUB_STEP = ((16 * B) % A);
	}
	V_THR_DIV2 = A;

	DBG_DETAIL("V step %d,sub step %d, div2 %d\r\n",
				V_STEP, V_SUB_STEP, V_THR_DIV2);

	scl_regs1->vxwidth.b.dst_vxwidth = (A > B) ? A : B;
	scl_regs1->vxwidth.b.vxwidth = B;
	scl_regs1->vscale2.b.step = V_STEP;
	scl_regs1->vscale1.b.substep = V_SUB_STEP;
	scl_regs1->vscale1.b.thr = V_THR_DIV2;
	scl_regs1->vscale2.b.substepcnt = 0;
}

void scl_set_H_scale(int A, int B) /* A dst,B src */
{
	unsigned int H_STEP;
	unsigned int H_SUB_STEP;
	unsigned int H_THR_DIV2;

	DBG_DETAIL("scl_set_H_scale(%d,%d)\r\n", A, B);
	if (A > B) {
		H_STEP = (B - 1) * 16 / A;
		H_SUB_STEP = (B - 1) * 16  % A;
	} else {
		H_STEP = (16 * B / A);
		H_SUB_STEP = ((16 * B) % A);
	}
	H_THR_DIV2 = A;
	DBG_DETAIL("H step %d,sub step %d, div2 %d\r\n",
				H_STEP, H_SUB_STEP, H_THR_DIV2);
	scl_regs1->hxwidth.b.hxwidth = ((A > B) ? A : B);
	scl_regs1->hscale2.b.step = H_STEP;
	scl_regs1->hscale1.b.substep = H_SUB_STEP;
	scl_regs1->hscale1.b.thr = H_THR_DIV2;
	scl_regs1->hscale2.b.substepcnt = 0;
}

void scl_set_crop(int offset_x, int offset_y)
{
	/* offset_x &= VPU_CROP_ALIGN_MASK; */ /* ~0x7 */
	offset_x &= ~0xf;

	scl_regs1->hscale3.b.stepcnt = offset_x * 16;
	scl_regs1->vscale3.b.stepcnt = offset_y * 16;
	DBGMSG("[VPU] crop - x : 0x%x, y : 0x%x \r\n",
				offset_x * 16, offset_y * 16);
}

void scl_set_tg_enable(vpp_flag_t enable)
{
	scl_regs1->tg_ctl.b.enable = enable;
}

unsigned int scl_set_clock(unsigned int pixel_clock)
{
	unsigned int rd_cyc;
	rd_cyc = vpp_get_base_clock(VPP_MOD_SCL) / pixel_clock;
	return rd_cyc;
}

void scl_set_timing(vpp_clock_t *timing, unsigned int pixel_clock)
{
#if 1
	timing->read_cycle = WMT_SCL_RCYC_MIN;
#else
	timing->read_cycle = scl_set_clock(pixel_clock * 2) - 1;
	timing->read_cycle = (timing->read_cycle < WMT_SCL_RCYC_MIN) ?
		WMT_SCL_RCYC_MIN : timing->read_cycle;
	timing->read_cycle = (timing->read_cycle > 255) ?
		0xFF : timing->read_cycle;
#endif
	scl_regs1->tg_ctl.b.rdcyc = timing->read_cycle;
	scl_regs2->readcyc_1t.b.rdcyc_1t = (timing->read_cycle) ? 0 : 1;
	scl_regs1->tg_total.b.h_allpixel = timing->total_pixel_of_line;
	scl_regs1->tg_h_active.b.h_actbg = timing->begin_pixel_of_active;
	scl_regs1->tg_h_active.b.h_actend = timing->end_pixel_of_active;
	scl_regs1->tg_total.b.v_allline = timing->total_line_of_frame;
	scl_regs1->tg_v_active.b.v_actbg = timing->begin_line_of_active;
	scl_regs1->tg_v_active.b.v_actend = timing->end_line_of_active;
	scl_regs1->tg_vbi.b.vbie = timing->line_number_between_VBIS_VBIE;
	scl_regs1->tg_vbi.b.pvbi = timing->line_number_between_PVBI_VBIS;
#ifdef DEBUG_DETAIL
	vpp_show_timing("scl set timing", 0, timing);
#endif
}

void scl_get_timing(vpp_clock_t *p_timing)
{
	p_timing->read_cycle = scl_regs1->tg_ctl.b.rdcyc;
	p_timing->total_pixel_of_line = scl_regs1->tg_total.b.h_allpixel;
	p_timing->begin_pixel_of_active = scl_regs1->tg_h_active.b.h_actbg;
	p_timing->end_pixel_of_active = scl_regs1->tg_h_active.b.h_actend;
	p_timing->total_line_of_frame = scl_regs1->tg_total.b.v_allline;
	p_timing->begin_line_of_active = scl_regs1->tg_v_active.b.v_actbg;
	p_timing->end_line_of_active = scl_regs1->tg_v_active.b.v_actend;
	p_timing->line_number_between_VBIS_VBIE = scl_regs1->tg_vbi.b.vbie;
	p_timing->line_number_between_PVBI_VBIS = scl_regs1->tg_vbi.b.pvbi;
}

void scl_set_watchdog(U32 count)
{
	if (0 != count) {
		scl_regs1->tg_watchdog = count;
		scl_regs1->tg_ctl.b.watchdog_enable = 1;
	} else
		scl_regs1->tg_ctl.b.watchdog_enable = 0;
}

void scl_set_timing_master(vpp_mod_t mod_bit)
{
	scl_regs1->tg_govw.b.enable = (mod_bit == VPP_MOD_GOVW) ? 1 : 0;
}

vpp_mod_t scl_get_timing_master(void)
{
	return (scl_regs1->tg_govw.b.enable) ? VPP_MOD_GOVW : VPP_MOD_SCL;
}

void scl_set_drop_line(vpp_flag_t enable)
{
	scl_regs1->scldw = enable;
}

/* only one feature can work, other should be disable */
void scl_set_filter_mode(enum vpp_filter_mode_t mode, int enable)
{
	DBG_DETAIL("(%d,%d)\n", mode, enable);
	if (mode != VPP_FILTER_SCALE) {
		if (scl_regs1->sclup_en.b.v || scl_regs1->sclup_en.b.h)
			DPRINT("[SCL] *W* filter can't work w scale\n");
	}
	scl_regs2->field_mode.b.deblock = 0;
	scl_regs2->field_mode.b.field_deflicker = 0;
	scl_regs2->field_mode.b.frame_deflicker = 0;
	switch (mode) {
	default:
	case VPP_FILTER_SCALE: /* scale mode */
		break;
	case VPP_FILTER_DEBLOCK: /* deblock */
		scl_regs2->field_mode.b.deblock = enable;
		break;
	case VPP_FILTER_FIELD_DEFLICKER: /* field deflicker */
		scl_regs2->field_mode.b.field_deflicker = enable;
		break;
	case VPP_FILTER_FRAME_DEFLICKER: /* frame deflicker */
		scl_regs2->field_mode.b.frame_deflicker = enable;
		break;
	}
}

enum vpp_filter_mode_t scl_get_filter_mode(void)
{
	if (scl_regs1->sclup_en.b.v || scl_regs1->sclup_en.b.h)
		return VPP_FILTER_SCALE;

	if (scl_regs2->field_mode.b.deblock)
		return VPP_FILTER_DEBLOCK;

	if (scl_regs2->field_mode.b.field_deflicker)
		return VPP_FILTER_FIELD_DEFLICKER;

	if (scl_regs2->field_mode.b.frame_deflicker)
		return VPP_FILTER_FRAME_DEFLICKER;
	return VPP_FILTER_SCALE;
}

void sclr_set_mif_enable(vpp_flag_t enable)
{
	scl_regs1->r_ctl.b.mif_enable = enable;
}

void sclr_set_mif2_enable(vpp_flag_t enable)
{

}

void sclr_set_colorbar(vpp_flag_t enable, int width, int inverse)
{
	scl_regs1->r_ctl.b.colorbar_mode = width;
	scl_regs1->r_ctl.b.colorbar_inv = inverse;
	scl_regs1->r_ctl.b.colorbar_enable = enable;
}

void sclr_set_field_mode(vpp_display_format_t fmt)
{
	scl_regs1->r_ctl.b.src_disp_fmt = fmt;
}

void sclr_set_display_format(vpp_display_format_t source,
				vpp_display_format_t target)
{
	scl_regs1->r_ctl.b.src_disp_fmt =
		(source == VPP_DISP_FMT_FIELD) ? 1 : 0;
	scl_regs1->r_ctl.b.field = (target == VPP_DISP_FMT_FIELD) ? 1 : 0;
}

void sclr_set_color_format(vdo_color_fmt format)
{
	p_scl->abgr_mode = 0;
	if (format >= VDO_COL_FMT_ARGB) {
		scl_regs1->r_ctl.b.rgb_mode =
			(format == VDO_COL_FMT_RGB_565) ? 0x1 : 0x3;
		if (format == VDO_COL_FMT_ABGR)
			p_scl->abgr_mode = 1;
		return;
	}
	scl_regs1->r_ctl.b.rgb_mode = 0;
	scl_regs1->r_ctl.b.rgb = 0x0;
	switch (format) {
	case VDO_COL_FMT_YUV444:
		scl_regs1->r_ctl.b.yuv = 0x2;
		break;
	case VDO_COL_FMT_YUV422H:
		scl_regs1->r_ctl.b.yuv = 0x0;
		break;
	case VDO_COL_FMT_YUV420:
		scl_regs1->r_ctl.b.yuv = 0x1;
		break;
	default:
		DBG_ERR("color fmt %d\n", format);
		return;
	}
}

vdo_color_fmt sclr_get_color_format(void)
{
	switch (scl_regs1->r_ctl.b.rgb_mode) {
	case 0x1:
		return VDO_COL_FMT_RGB_565;
	case 0x3:
		return VDO_COL_FMT_ARGB;
	default:
		break;
	}
	switch (scl_regs1->r_ctl.b.yuv) {
	case 0:
		return VDO_COL_FMT_YUV422H;
	case 1:
		return VDO_COL_FMT_YUV420;
	case 2:
		return VDO_COL_FMT_YUV444;
	default:
		break;
	}
	return VDO_COL_FMT_YUV444;
}

void sclr_set_media_format(vpp_media_format_t format)
{
	scl_regs1->r_ctl.b.h264 = (format == VPP_MEDIA_FMT_H264) ? 1 : 0;
}

void sclr_set_fb_addr(U32 y_addr, U32 c_addr)
{
	unsigned int line_y, line_c;
	unsigned int offset_y, offset_c;
	unsigned int pre_y, pre_c;

	DBGMSG("y_addr:0x%08x, c_addr:0x%08x\n", y_addr, c_addr);

	offset_y = offset_c = 0;
	line_y = line_c = scl_regs1->r_h_size.b.fb_w;
	switch (sclr_get_color_format()) {
	case VDO_COL_FMT_YUV420:
		offset_c /= 2;
		line_c = 0;
		break;
	case VDO_COL_FMT_YUV422H:
		break;
	case VDO_COL_FMT_ARGB:
		offset_y *= 4;
		line_y *= 4;
		break;
	case VDO_COL_FMT_RGB_565:
		offset_y *= 2;
		line_y *= 2;
		break;
	default:
		offset_c *= 2;
		line_c *= 2;
		break;
	}
	pre_y = scl_regs1->r_ysa;
	pre_c = scl_regs1->r_csa;
	scl_regs1->r_ysa = y_addr + offset_y;
	scl_regs1->r_csa = c_addr + offset_c;
}

void sclr_get_fb_addr(U32 *y_addr, U32 *c_addr)
{
	*y_addr = scl_regs1->r_ysa;
	*c_addr = scl_regs1->r_csa;
/*	DBGMSG("y_addr:0x%08x, c_addr:0x%08x\n", *y_addr, *c_addr); */
}

void sclr_set_width(U32 y_pixel, U32 y_buffer)
{
	scl_regs1->r_h_size.b.pix_w = y_pixel;
	scl_regs1->r_h_size.b.fb_w = y_buffer;
}

void sclr_get_width(U32 *p_y_pixel, U32 *p_y_buffer)
{
	*p_y_pixel = scl_regs1->r_h_size.b.pix_w;
	*p_y_buffer = scl_regs1->r_h_size.b.fb_w;
}

void sclr_set_crop(U32 h_crop, U32 v_crop)
{
	scl_regs1->r_crop.b.hcrop = h_crop;
	scl_regs1->r_crop.b.vcrop = v_crop;
}

void sclr_get_fb_info(U32 *width, U32 *act_width,
				U32 *x_offset, U32 *y_offset)
{
	*width = scl_regs1->r_h_size.b.fb_w;
	*act_width = scl_regs1->r_h_size.b.pix_w;
	*x_offset = scl_regs1->r_crop.b.hcrop;
	*y_offset = scl_regs1->r_crop.b.vcrop;
}

void sclr_set_threshold(U32 value)
{
	scl_regs1->r_fifo_ctl.b.thr = value;
}

void sclw_set_mif_enable(vpp_flag_t enable)
{
	scl_regs1->w_ctl.b.mif_enable = enable;
}

void sclw_set_color_format(vdo_color_fmt format)
{
	/* 0-888(4 byte), 1-5515(2 byte), 2-666(4 byte), 3-565(2 byte) */
	switch (format) {
	case VDO_COL_FMT_RGB_666:
		scl_regs1->w_ctl.b.rgb = 1;
		scl_regs2->igs.b.mode = 2;
		break;
	case VDO_COL_FMT_RGB_565:
		scl_regs1->w_ctl.b.rgb = 1;
		scl_regs2->igs.b.mode = 3;
		break;
	case VDO_COL_FMT_RGB_1555:
		scl_regs1->w_ctl.b.rgb = 1;
		scl_regs2->igs.b.mode = 1;
		break;
	case VDO_COL_FMT_ARGB:
		scl_regs1->w_ctl.b.rgb = 1;
		scl_regs2->igs.b.mode = 0;
		break;
	case VDO_COL_FMT_YUV444:
		scl_regs1->w_ctl.b.rgb = 0;
		scl_regs1->w_ctl.b.yuv = 0;
		scl_regs2->igs.b.mode = 0;
		break;
	case VDO_COL_FMT_YUV422H:
	case VDO_COL_FMT_YUV420:
		scl_regs1->w_ctl.b.rgb = 0;
		scl_regs1->w_ctl.b.yuv = 1;
		scl_regs2->igs.b.mode = 0;
		break;
	default:
		DBGMSG("*E* check the parameter.\n");
		return;
	}
}

vdo_color_fmt sclw_get_color_format(void)
{
	if (scl_regs1->w_ctl.b.rgb) {
		switch (scl_regs2->igs.b.mode) {
		case 0:
			return VDO_COL_FMT_ARGB;
		case 1:
			return VDO_COL_FMT_RGB_1555;
		case 2:
			return VDO_COL_FMT_RGB_666;
		case 3:
			return VDO_COL_FMT_RGB_565;
		}
	}

	if (scl_regs1->w_ctl.b.yuv)
		return VDO_COL_FMT_YUV422H;
	return VDO_COL_FMT_YUV444;
}

void sclw_set_alpha(int enable, char data)
{
	scl_regs2->argb_alpha.b.data = data;
	scl_regs2->argb_alpha.b.enable = enable;
}

void sclw_set_field_mode(vpp_display_format_t fmt)
{
	scl_regs1->r_ctl.b.field = fmt;
}

void sclw_set_fb_addr(U32 y_addr, U32 c_addr)
{
	DBGMSG("y_addr:0x%08x, c_addr:0x%08x\n", y_addr, c_addr);
/*	if( (y_addr & 0x3f) || (c_addr & 0x3f) ){
		DPRINT("[SCL] *E* addr should align 64\n");
	} */
	scl_regs1->w_ysa = y_addr;
	scl_regs1->w_csa = c_addr;
}

void sclw_get_fb_addr(U32 *y_addr, U32 *c_addr)
{
	*y_addr = scl_regs1->w_ysa;
	*c_addr = scl_regs1->w_csa;
	DBGMSG("y_addr:0x%08x, c_addr:0x%08x\n", *y_addr, *c_addr);
}

void sclw_set_fb_width(U32 width, U32 buf_width)
{
	scl_regs1->w_y_time.b.pxl_w = width;
	scl_regs1->w_y_time.b.fb_w = buf_width;
	if (sclw_get_color_format() == VDO_COL_FMT_YUV444) {
		scl_regs1->w_c_time.b.pxl_w = width;
		scl_regs1->w_c_time.b.fb_w = buf_width * 2;
	} else {
		scl_regs1->w_c_time.b.pxl_w = width / 2;
		scl_regs1->w_c_time.b.fb_w = buf_width;
	}
}

void sclw_get_fb_width(U32 *width, U32 *buf_width)
{
	*width = scl_regs1->w_y_time.b.pxl_w;
	*buf_width = scl_regs1->w_y_time.b.fb_w;
}

void scl_R2_set_mif_enable(int enable)
{
	scl_regs1->r2_ctl.b.mif_en = enable;
}

void scl_R2_set_colorbar(int enable, int wide, int inv)
{
	scl_regs1->r2_ctl.b.color_en = enable;
	scl_regs1->r2_ctl.b.color_wide = wide;
	scl_regs1->r2_ctl.b.color_inv = inv;
}

void scl_R2_set_color_format(vdo_color_fmt colfmt)
{
	if (colfmt >= VDO_COL_FMT_ARGB) {
		scl_regs1->r2_ctl.b.rgb_mode =
			(colfmt == VDO_COL_FMT_RGB_565) ? 0x1 : 0x3;
		return;
	}
	scl_regs1->r2_ctl.b.rgb_mode = 0;
	switch (colfmt) {
	case VDO_COL_FMT_YUV444:
		scl_regs1->r2_ctl.b.vfmt = 0x2;
		break;
	case VDO_COL_FMT_YUV422H:
		scl_regs1->r2_ctl.b.vfmt = 0x0;
		break;
	case VDO_COL_FMT_YUV420:
		scl_regs1->r2_ctl.b.vfmt = 0x1;
		break;
	default:
		DBG_ERR("color fmt %d\n", colfmt);
		return;
	}
}

vdo_color_fmt scl_R2_get_color_format(void)
{
	switch (scl_regs1->r2_ctl.b.rgb_mode) {
	case 0:
		switch (scl_regs1->r2_ctl.b.vfmt) {
		case 0:
			return VDO_COL_FMT_YUV422H;
		case 1:
			return VDO_COL_FMT_YUV420;
		case 2:
		default:
			return VDO_COL_FMT_YUV444;
		}
		break;
	case 1:
		return VDO_COL_FMT_RGB_565;
	case 3:
	default:
		break;
	}
	return VDO_COL_FMT_ARGB;
}

void scl_R2_set_csc_mode(vpp_csc_t mode)
{
	vdo_color_fmt src_fmt, dst_fmt;

	src_fmt = scl_R2_get_color_format();
	dst_fmt = sclw_get_color_format();
	mode = vpp_check_csc_mode(mode, src_fmt, dst_fmt, 0);

	scl_regs2->r2_csc1 = vpp_csc_parm[mode][0];
	scl_regs2->r2_csc2 = vpp_csc_parm[mode][1];
	scl_regs2->r2_csc3 = vpp_csc_parm[mode][2];
	scl_regs2->r2_csc4 = vpp_csc_parm[mode][3];
	scl_regs2->r2_csc5 = vpp_csc_parm[mode][4];
	scl_regs2->r2_csc6 = vpp_csc_parm[mode][5];
	scl_regs2->r2_csc.val = vpp_csc_parm[mode][6];
	scl_regs2->r2_csc.b.enable = (mode >= VPP_CSC_MAX) ? 0 : 1;
}

void scl_R2_set_framebuffer(vdo_framebuf_t *fb)
{
	scl_regs1->r2_ctl.b.iofmt = (fb->flag & VDO_FLAG_INTERLACE) ? 1 : 0;
	scl_R2_set_color_format(fb->col_fmt);
	scl_regs1->r2_ysa = fb->y_addr;
	scl_regs1->r2_csa = fb->c_addr;
	scl_regs1->r2_h_size.b.fbw = fb->fb_w;
	scl_regs1->r2_h_size.b.lnsize = fb->img_w;
	scl_regs1->r2_crop.b.hcrop = fb->h_crop;
	scl_regs1->r2_crop.b.vcrop = fb->v_crop;
	scl_R2_set_csc_mode(p_scl->fb_p->csc_mode);
}

void scl_ALPHA_set_enable(int enable)
{
	scl_regs1->alpha_colorkey.b.enable = enable;
}

void scl_ALPHA_set_swap(int enable)
{
	/* 0-(alpha,1-alpha),1:(1-alpha,alpha) */
	scl_regs1->alpha_md.b.swap = enable;
}

void scl_ALPHA_set_src(int mode, int fixed)
{
	/* 0-RMIF1,1-RMIF2,2-Fixed ALPHA */
	scl_regs1->alpha_md.b.src = mode;
	scl_regs1->alpha_fxd.b.src_fixed = fixed;
}

void scl_ALPHA_set_dst(int mode, int fixed)
{
	/* 0-RMIF1,1-RMIF2,2-Fixed ALPHA */
	scl_regs1->alpha_md.b.dst = mode;
	scl_regs1->alpha_fxd.b.dst_fixed = fixed;
}

void scl_ALPHA_set_color_key(int rmif2, int comp, int mode, int colkey)
{
	/* 0-RMIF1,1-RMIF2 */
	scl_regs1->alpha_colorkey.b.from = rmif2;
	/* 0-888,1-777,2-666,3-555 */
	scl_regs1->alpha_colorkey.b.comp = comp;
	/* (Non-Hit,Hit):0/1-(alpha,alpha),
	2-(alpha,pix1),3-(pix1,alpha),4-(alpha,pix2),
	5-(pix2,alpha),6-(pix1,pix2),7-(pix2,pix1) */
	scl_regs1->alpha_colorkey.b.mode = mode;
	scl_regs1->alpha_colorkey_rgb.val = colkey;
}

void scl_set_overlap(vpp_overlap_t *p)
{
#if 0
	DPRINT("alpha src %d,0x%x,dst %d,0x%x\n",
		p->alpha_src_type, p->alpha_src,
		p->alpha_dst_type, p->alpha_dst);
	DPRINT("colkey from %d,comp %d,mode %d,0x%x\n",
		p->color_key_from, p->color_key_comp,
		p->color_key_mode, p->color_key);
#endif
	scl_ALPHA_set_src(p->alpha_src_type, p->alpha_src);
	scl_ALPHA_set_dst(p->alpha_dst_type, p->alpha_dst);
	scl_ALPHA_set_swap(p->alpha_swap);
	scl_ALPHA_set_color_key(p->color_key_from, p->color_key_comp,
		p->color_key_mode, p->color_key);
}

void scl_set_req_num(int ynum, int cnum)
{
	scl_regs1->r_req_num.b.y_req_num = ynum;
	scl_regs1->r_req_num.b.c_req_num = cnum;
}

static void scl_set_scale_PP(unsigned int src, unsigned int dst,
					int horizontal)
{
	int gcd;

/*	DBGMSG("scale PP(s %d,d %d,is H %d)\n",src,dst,horizontal); */

	/* gcd = scl_get_gcd(src,dst); */
	gcd = 1;
	src /= gcd;
	dst /= gcd;

	if (horizontal)
		scl_set_H_scale(dst, src);
	else
		scl_set_V_scale(dst, src);
}

void scl_set_scale(unsigned int SRC_W, unsigned int SRC_H,
			unsigned int DST_W, unsigned int DST_H)
{
	int h_scale_up;
	int v_scale_up;

	DBGMSG("[SCL] src(%dx%d),dst(%dx%d)\n", SRC_W, SRC_H, DST_W, DST_H);

	h_scale_up = (DST_W > SRC_W) ? 1 : 0;
	v_scale_up = (DST_H > SRC_H) ? 1 : 0;

	if (((DST_W / SRC_W) >= 32) || ((DST_W / SRC_W) < 1/32))
		DBGMSG("*W* SCL H scale rate invalid\n");

	if (((DST_H / SRC_H) >= 32) || ((DST_H / SRC_H) < 1/32))
		DBGMSG("*W* SCL V scale rate invalid\n");

/*	DBGMSG("scale H %d,V %d\n",h_scale_up,v_scale_up); */

	sclr_set_mif2_enable(VPP_FLAG_DISABLE);
	scl_set_scale_PP(SRC_W, DST_W, 1);
	scl_set_scale_PP(SRC_H, DST_H, 0);
	scl_set_scale_enable(v_scale_up, h_scale_up);

	{
		int rec_h, rec_v;
		int h, v;

		h = rec_h = 0;
		if (SRC_W > DST_W) { /* scale down */
			switch (p_scl->scale_mode) {
			case VPP_SCALE_MODE_ADAPTIVE:
				if ((DST_W * 2) > SRC_W) /* 1 > mode(3) > 1/2 */
					h = 1; /* bilinear mode */
				else
					rec_h = 1; /* recursive mode */
				break;
			case VPP_SCALE_MODE_BILINEAR:
				h = 1; /* bilinear mode */
				break;
			case VPP_SCALE_MODE_RECURSIVE:
				rec_h = 1;	/* recursive mode */
				break;
			default:
				break;
			}
		}

		v = rec_v = 0;
		if (SRC_H > DST_H) { /* scale down */
			switch (p_scl->scale_mode) {
			case VPP_SCALE_MODE_ADAPTIVE:
				if ((DST_H * 2) > SRC_H) /* 1 > mode(3) > 1/2 */
					v = 1; /* bilinear mode */
				else
					rec_v = 1; /* recursive mode */
				break;
			case VPP_SCALE_MODE_BILINEAR:
				v = 1; /* bilinear mode */
				break;
			case VPP_SCALE_MODE_RECURSIVE:
				rec_v = 1;	/* recursive mode */
				break;
			default:
				break;
			}
		}

		if (SRC_W == DST_W)
			rec_h = 1;
		if (SRC_H == DST_H)
			rec_v = 1;
		scl_regs1->true_bilinear.b.h = h;
		scl_regs1->true_bilinear.b.v = v;
		scl_regs2->recursive_mode.b.h = rec_h;
		scl_regs2->recursive_mode.b.v = rec_v;
		if (v) {
			scl_regs1->vxwidth.b.vxwidth =
				scl_regs1->vxwidth.b.vxwidth - 1;
			scl_regs1->vxwidth.b.dst_vxwidth =
				scl_regs1->vxwidth.b.vxwidth;
		}
		sclr_set_mif2_enable((v) ? VPP_FLAG_ENABLE : VPP_FLAG_DISABLE);
	}
}

void sclr_set_framebuffer(vdo_framebuf_t *inbuf)
{
	sclr_set_color_format(inbuf->col_fmt);
	sclr_set_crop(inbuf->h_crop, inbuf->v_crop);
	sclr_set_width(inbuf->img_w, inbuf->fb_w);
	sclr_set_fb_addr(inbuf->y_addr, inbuf->c_addr);
	sclr_set_field_mode(vpp_get_fb_field(inbuf));
}

void scl_set_scale_timing(vdo_framebuf_t *s, vdo_framebuf_t *d)
{
	vpp_clock_t timing;
	unsigned int pixel_clock;

	/* scl TG */
	timing.total_pixel_of_line = (d->img_w > s->img_w) ?
					d->img_w : s->img_w;
	timing.total_line_of_frame = (d->img_h > s->img_h) ?
					d->img_h : s->img_h;
	timing.begin_pixel_of_active = 60;
	timing.end_pixel_of_active = timing.total_pixel_of_line + 60;
	timing.total_pixel_of_line = timing.total_pixel_of_line + 120;
	timing.begin_line_of_active = 8;
	timing.end_line_of_active = timing.total_line_of_frame + 8;
	timing.total_line_of_frame = timing.total_line_of_frame + 16;
	timing.line_number_between_VBIS_VBIE = 4;
	timing.line_number_between_PVBI_VBIS = 1;
	pixel_clock = timing.total_pixel_of_line *
		timing.total_line_of_frame * p_scl->fb_p->framerate;
	scl_set_timing(&timing, pixel_clock);
}

void sclw_set_framebuffer(vdo_framebuf_t *fb)
{
	unsigned int yaddr, caddr;
	int y_bpp, c_bpp;

	vpp_get_colfmt_bpp(fb->col_fmt, &y_bpp, &c_bpp);
	yaddr = fb->y_addr + ((fb->fb_w * fb->v_crop + fb->h_crop) * y_bpp / 8);
	caddr = (c_bpp) ? (fb->c_addr + (((fb->fb_w * fb->v_crop +
		fb->h_crop) / 2) * 2 * c_bpp / 8)) : 0;
	sclw_set_fb_addr(yaddr, caddr);
	sclw_set_color_format(fb->col_fmt);
	sclw_set_fb_width(fb->img_w, fb->fb_w);
	sclw_set_field_mode(vpp_get_fb_field(fb));
	scl_set_csc_mode(p_scl->fb_p->csc_mode);
}

void scl_init(void *base)
{
	struct scl_mod_t *mod_p;
	struct vpp_fb_base_t *fb_p;

	mod_p = (struct scl_mod_t *) base;
	fb_p = mod_p->fb_p;

	scl_set_reg_level(VPP_REG_LEVEL_1);
	scl_set_tg_enable(VPP_FLAG_DISABLE);
	scl_set_enable(VPP_FLAG_DISABLE);
	scl_set_int_enable(VPP_FLAG_DISABLE, VPP_INT_ALL);
	sclr_set_mif_enable(VPP_FLAG_DISABLE);
	sclr_set_mif2_enable(VPP_FLAG_DISABLE);
	sclr_set_colorbar(VPP_FLAG_DISABLE, 0, 0);

	scl_set_int_enable(VPP_FLAG_ENABLE, mod_p->int_catch);
	scl_set_watchdog(fb_p->wait_ready);
	scl_set_csc_mode(fb_p->csc_mode);
	sclr_set_media_format(fb_p->media_fmt);
	sclr_set_threshold(0xf);

	/* filter default value */
	scl_regs2->dblk_threshold.b.layer1_boundary = 48;
	scl_regs2->dblk_threshold.b.layer2_boundary = 16;

	scl_regs2->field_flicker.b.y_thd = 8;
	scl_regs2->field_flicker.b.c_thd = 8;
	scl_regs2->field_flicker.b.condition = 0;

	scl_regs2->frame_flicker.b.rgb = 0;
	scl_regs2->frame_flicker.b.sampler = 14;
	scl_regs2->frame_flicker.b.scene_chg_thd = 32;
	scl_set_reg_update(VPP_FLAG_ENABLE);
	scl_set_tg_enable(VPP_FLAG_DISABLE);
}

void sclw_init(void *base)
{
	sclw_set_mif_enable(VPP_FLAG_DISABLE);
	sclw_set_fb_width(VPP_HD_DISP_RESX, VPP_HD_MAX_RESX);
/*	vppif_reg32_write(SCL_SCLDW_METHOD,0x1); */	/* drop line enable */
}

#ifdef __KERNEL__
/* static struct work_struct scl_proc_scale_wq; */
DECLARE_WAIT_QUEUE_HEAD(scl_proc_scale_event);
static void scl_proc_scale_complete_work(struct work_struct *work)
#else
static void scl_proc_scale_complete_work(int arg)
#endif
{
/*	DPRINT("[SCL] scl_proc_scale_complete_work\n"); */
	p_scl->scale_complete = 1;
#ifdef __KERNEL__
	wake_up_interruptible(&scl_proc_scale_event);
#endif
#if 0 /* avoid mutex in irq */
	vpp_mod_set_clock(VPP_MOD_SCL, VPP_FLAG_DISABLE, 0);
#endif
}

#ifdef __KERNEL__
struct timer_list scl_scale_timer;
#endif
int scl_proc_scale_complete(void *arg)
{
#ifdef __KERNEL__
	del_timer(&scl_scale_timer);
#endif
/*	DPRINT("[SCL] scl_proc_scale_complete\n"); */
	if (scl_regs1->tg_sts.b.tgerr) {
		DPRINT("[SCL] scale TG err 0x%x,0x%x\n",
			scl_regs1->tg_sts.val, scl_regs1->w_ff_ctl.val);
		scl_regs1->tg_sts.val = BIT0;
		scl_regs1->w_ff_ctl.val =  0x10101;
	}
	scl_set_tg_enable(VPP_FLAG_DISABLE);
#ifndef CONFIG_UBOOT
	vppm_set_int_enable(VPP_FLAG_DISABLE, SCL_COMPLETE_INT);
#endif
	sclw_set_mif_enable(VPP_FLAG_DISABLE);
	sclr_set_mif_enable(VPP_FLAG_DISABLE);
	sclr_set_mif2_enable(VPP_FLAG_DISABLE);
	scl_set_enable(VPP_FLAG_DISABLE);

/* #ifdef __KERNEL__ */
#if 0
	INIT_WORK(&scl_proc_scale_wq, scl_proc_scale_complete_work);
	schedule_work(&scl_proc_scale_wq);
#else
	scl_proc_scale_complete_work(0);
#endif
	return 0;
}

void scl_scale_timeout(int arg)
{
	DBG_ERR("scale timeout\n");
#if 0
	scl_reg_dump();
	g_vpp.dbg_msg_level = VPP_DBGLVL_STREAM;
	g_vpp.dbg_cnt = 1000;
#endif
	scl_proc_scale_complete(0);
}

void scl_set_scale_timer(int ms)
{
#ifdef __KERNEL__
	if (scl_scale_timer.function)
		del_timer(&scl_scale_timer);
	init_timer(&scl_scale_timer);
	scl_scale_timer.function = (void *) scl_scale_timeout;
	scl_scale_timer.expires = jiffies + msecs_to_jiffies(ms);
	add_timer(&scl_scale_timer);
#endif
}

int scl_proc_scale_finish(void)
{
	int ret = 0;

#ifdef __KERNEL__

/*	DPRINT("[SCL] scl_proc_scale_finish\n"); */
	ret = wait_event_interruptible_timeout(scl_proc_scale_event,
				(p_scl->scale_complete != 0), 3 * HZ);
	if (ret == 0) { /* timeout */
		DPRINT("[SCL] *E* wait scale timeout\n");
		ret = -1;
	} else {
		ret = 0;
	}
#endif
#if 1 /* avoid mutex in irq */
	vpp_mod_set_clock(VPP_MOD_SCL, VPP_FLAG_DISABLE, 0);
#endif
	return ret;
}

void scl_check_framebuf(vdo_framebuf_t *s,
				vdo_framebuf_t *in, vdo_framebuf_t *out)
{
	if (s) {
		if (s->img_w > WMT_SCL_H_DIV_MAX)
			DBG_ERR("src w %d over %d\n",
					s->img_w, WMT_SCL_H_DIV_MAX);
		if (s->img_h > WMT_SCL_V_DIV_MAX)
			DBG_ERR("src h %d over %d\n",
					s->img_h, WMT_SCL_V_DIV_MAX);
		if (s->col_fmt >= VDO_COL_FMT_ARGB) {
			if (s->y_addr % 4)
				DBG_ERR("src addr 0x%x not align 4\n",
						s->y_addr);
		}
		if (s->fb_w > WMT_SCL_H_DIV_MAX)
			DBG_ERR("src fb w %d over %d\n",
				s->fb_w, WMT_SCL_H_DIV_MAX);
		if (s->y_addr % 64)
			DBG_ERR("src fb addr 0x%x no align 64\n", s->y_addr);
	} else {
		DBG_ERR("src null\n");
	}

	if (in) {
		if (in->img_w > WMT_SCL_H_DIV_MAX)
			DBG_ERR("in w %d over %d\n",
					in->img_w, WMT_SCL_H_DIV_MAX);
		if (in->img_h > WMT_SCL_V_DIV_MAX)
			DBG_ERR("in h %d over %d\n",
					in->img_h, WMT_SCL_V_DIV_MAX);
		if (in->col_fmt >= VDO_COL_FMT_ARGB) {
			if (in->y_addr % 4)
				DBG_ERR("in addr 0x%x not align 4\n",
						in->y_addr);
		}
		if (in->fb_w > WMT_SCL_H_DIV_MAX)
			DBG_ERR("in fb w %d over %d\n",
				in->fb_w, WMT_SCL_H_DIV_MAX);
		if (in->y_addr % 64)
			DBG_ERR("in fb addr 0x%x no align 64\n", in->y_addr);
	}

	if (out) {
		if (s && (s->img_w == out->img_w)) {
			if (out->img_w > WMT_SCL_H_DIV_MAX)
				DBG_ERR("out w %d over %d\n", out->img_w,
					WMT_SCL_H_DIV_MAX);
		} else {
			if (out->img_w > WMT_SCL_SCALE_DST_H_MAX)
				DBG_ERR("out w %d over %d\n", out->img_w,
					WMT_SCL_SCALE_DST_H_MAX);

		}
		if (out->col_fmt >= VDO_COL_FMT_ARGB) {
			if (out->y_addr % 4)
				DBG_ERR("out addr 0x%x not align 4\n",
						out->y_addr);
		}
		if (out->fb_w > WMT_SCL_H_DIV_MAX)
			DBG_ERR("out fb w %d over %d\n",
				out->fb_w, WMT_SCL_H_DIV_MAX);
		if (out->y_addr % 64)
			DBG_ERR("out fb addr 0x%x no align 64\n", out->y_addr);
	} else {
		DBG_ERR("out null\n");
	}
}

#define CONFIG_VPP_CHECK_SCL_STATUS
int scl_set_scale_overlap(vdo_framebuf_t *s,
				vdo_framebuf_t *in, vdo_framebuf_t *out)
{
	int ret = 0;

	if (vpp_check_dbg_level(VPP_DBGLVL_SCALE)) {
		scl_check_framebuf(s, in, out);

		if (s)
			vpp_show_framebuf("src1", s);
		if (in)
			vpp_show_framebuf("src2", in);
		if (out)
			vpp_show_framebuf("dst", out);
	}

	if (p_scl->scale_sync)
		vpp_mod_set_clock(VPP_MOD_SCL, VPP_FLAG_ENABLE, 0);

	scl_set_timing_master(VPP_MOD_SCL);

	if (s) {
		p_scl->fb_p->fb = *s;
		sclr_set_framebuffer(s);
		sclr_set_mif_enable(VPP_FLAG_ENABLE);
	} else {
		DPRINT("[SCL] *E* no source\n");
		return -1;
	}

	if (in && (in->y_addr == 0))
		in = 0;

	scl_ALPHA_set_enable((in) ? VPP_FLAG_ENABLE : VPP_FLAG_DISABLE);
	scl_R2_set_mif_enable((in) ? VPP_FLAG_ENABLE : VPP_FLAG_DISABLE);
	if (in)
		scl_R2_set_framebuffer(in);
	if (out) {
		p_sclw->fb_p->fb = *out;
		sclw_set_framebuffer(out);
	} else {
		DPRINT("[SCL] *E* no dest\n");
		return -1;
	}

	scl_set_scale(s->img_w, s->img_h, out->img_w, out->img_h);
	scl_set_scale_timing(s, out);

	/* scale process */
	scl_set_enable(VPP_FLAG_ENABLE);
	scl_regs1->tg_ctl.b.oneshot = 1;
	sclw_set_mif_enable(VPP_FLAG_ENABLE);
	scl_set_tg_enable(VPP_FLAG_ENABLE);
#ifdef CONFIG_VPP_CHECK_SCL_STATUS
	scl_regs1->tg_sts.val = BIT0;
	scl_regs1->w_ff_ctl.val = 0x10101;
#endif
	p_scl->scale_complete = 0;
#ifndef CONFIG_UBOOT
	vppm_set_int_enable(VPP_FLAG_ENABLE, SCL_COMPLETE_INT);
#endif
#if 0   /* for debug scale */
	scl_reg_dump();
#endif
#ifdef CONFIG_UBOOT
	while (scl_regs1->tg_ctl.b.enable);
	scl_proc_scale_complete(0);
	scl_proc_scale_finish();
#else
	if (p_scl->scale_sync) {
		ret = vpp_irqproc_work(SCL_COMPLETE_INT,
			(void *)scl_proc_scale_complete, 0, 100, 1);
		scl_proc_scale_finish();
	} else {
		vpp_irqproc_work(SCL_COMPLETE_INT,
			(void *)scl_proc_scale_complete, 0, 0, 1);
		scl_set_scale_timer(100);
	}
#endif
	return ret;
}

int scl_proc_scale(vdo_framebuf_t *src_fb, vdo_framebuf_t *dst_fb)
{
	int ret = 0;

	if (dst_fb->col_fmt == VDO_COL_FMT_YUV420) {
		int size;
		unsigned int buf;
		vdo_framebuf_t dfb;

		dfb = *dst_fb;	/* backup dst fb */

		/* alloc memory */
		size = dst_fb->img_w * dst_fb->img_h + 64;
		buf = (unsigned int) kmalloc(size, GFP_KERNEL);
		if (!buf) {
			DPRINT("[SCL] *E* malloc fail %d\n", size);
			return -1;
		}

		if (buf % 64)
			buf = buf + (64 - (buf % 64));

		/* scale for Y */
		dst_fb->c_addr = buf;
		ret = scl_set_scale_overlap(src_fb, 0, dst_fb);
		if (ret == 0) {
			/* V 1/2 scale for C */
			dst_fb->y_addr = buf;
			dst_fb->c_addr = dfb.c_addr;
			dst_fb->img_h = dfb.img_h / 2;
			ret = scl_set_scale_overlap(src_fb, 0, dst_fb);
		}
		kfree((void *)buf);
		*dst_fb = dfb;	/* restore dst fb */
	} else {
		ret = scl_set_scale_overlap(src_fb, 0, dst_fb);
	}
	return ret;
}

#ifdef CONFIG_PM
static unsigned int *scl_pm_bk2;
static unsigned int scl_pm_enable, scl_pm_tg;
static unsigned int scl_pm_r_mif1, scl_pm_r_mif2, scl_pm_w_mif;
void scl_suspend(int sts)
{
	switch (sts) {
	case 0:	/* disable module */
		vpp_mod_set_clock(VPP_MOD_SCL, VPP_FLAG_ENABLE, 1);
		scl_pm_enable = scl_regs1->en.b.alu_enable;
		scl_regs1->en.b.alu_enable = 0;
		scl_pm_r_mif1 = scl_regs1->r_ctl.b.mif_enable;
		scl_pm_r_mif2 = scl_regs1->r2_ctl.b.mif_en;
		scl_regs1->r2_ctl.b.mif_en = 0;
		scl_regs1->r_ctl.b.mif_enable = 0;
		scl_pm_w_mif = scl_regs1->w_ctl.b.mif_enable;
		scl_regs1->w_ctl.b.mif_enable = 0;
		break;
	case 1: /* disable tg */
		scl_pm_tg = scl_regs1->tg_ctl.b.enable;
		scl_regs1->tg_ctl.b.enable = 0;
		break;
	case 2:	/* backup register */
		p_scl->reg_bk = vpp_backup_reg(REG_SCL_BASE1_BEGIN,
			(REG_SCL_BASE1_END - REG_SCL_BASE1_BEGIN));
		scl_pm_bk2 = vpp_backup_reg(REG_SCL_BASE2_BEGIN,
			(REG_SCL_BASE2_END - REG_SCL_BASE2_BEGIN));
		break;
	default:
		break;
	}
}

void scl_resume(int sts)
{
	switch (sts) {
	case 0:	/* restore register */
		vpp_restore_reg(REG_SCL_BASE1_BEGIN,
			(REG_SCL_BASE1_END - REG_SCL_BASE1_BEGIN),
			p_scl->reg_bk);
		vpp_restore_reg(REG_SCL_BASE2_BEGIN,
			(REG_SCL_BASE2_END - REG_SCL_BASE2_BEGIN), scl_pm_bk2);
		p_scl->reg_bk = 0;
		scl_pm_bk2 = 0;
		break;
	case 1:	/* enable module */
		scl_regs1->w_ctl.b.mif_enable = scl_pm_w_mif;
		scl_regs1->r_ctl.b.mif_enable = scl_pm_r_mif1;
		scl_regs1->r2_ctl.b.mif_en = scl_pm_r_mif2;
		scl_regs1->en.b.alu_enable = scl_pm_enable;
		break;
	case 2: /* enable tg */
		scl_regs1->tg_ctl.b.enable = scl_pm_tg;
		vpp_mod_set_clock(VPP_MOD_SCL, VPP_FLAG_DISABLE, 1);
		break;
	default:
		break;
	}
}
#else
#define scl_suspend NULL
#define scl_resume NULL
#endif

int scl_mod_init(void)
{
	struct vpp_fb_base_t *mod_fb_p;
	vdo_framebuf_t *fb_p;

	/* -------------------- SCL module -------------------- */
	{
	struct scl_mod_t *scl_mod_p;

	scl_mod_p = (struct scl_mod_t *) vpp_mod_register(VPP_MOD_SCL,
				sizeof(struct scl_mod_t),
				VPP_MOD_FLAG_FRAMEBUF);
	if (!scl_mod_p) {
		DPRINT("*E* SCL module register fail\n");
		return -1;
	}

	/* module member variable */
	scl_mod_p->int_catch = VPP_INT_NULL;
	scl_mod_p->scale_mode = VPP_SCALE_MODE_ADAPTIVE;
	scl_mod_p->pm = DEV_SCL444U;
	scl_mod_p->filter_mode = VPP_FILTER_SCALE;

	/* module member function */
	scl_mod_p->init = scl_init;
	scl_mod_p->set_enable = scl_set_enable;
	scl_mod_p->set_colorbar = sclr_set_colorbar;
	scl_mod_p->dump_reg = scl_reg_dump;
	scl_mod_p->get_sts = scl_get_int_status;
	scl_mod_p->clr_sts = scl_clean_int_status;
	scl_mod_p->scale = scl_proc_scale;
	scl_mod_p->scale_finish = scl_proc_scale_finish;
	scl_mod_p->suspend = scl_suspend;
	scl_mod_p->resume = scl_resume;

	/* module frame buffer variable */
	mod_fb_p = scl_mod_p->fb_p;
	fb_p = &mod_fb_p->fb;

	fb_p->y_addr = 0;
	fb_p->c_addr = 0;
	fb_p->col_fmt = VDO_COL_FMT_YUV422H;
	fb_p->img_w = VPP_HD_DISP_RESX;
	fb_p->img_h = VPP_HD_DISP_RESY;
	fb_p->fb_w = VPP_HD_MAX_RESX;
	fb_p->fb_h = VPP_HD_MAX_RESY;
	fb_p->h_crop = 0;
	fb_p->v_crop = 0;

	/* module frame buffer member function */
	mod_fb_p->csc_mode = VPP_CSC_RGB2YUV_SDTV_0_255;
	mod_fb_p->set_framebuf = sclr_set_framebuffer;
	mod_fb_p->set_addr = sclr_set_fb_addr;
	mod_fb_p->get_addr = sclr_get_fb_addr;
	mod_fb_p->set_csc = scl_set_csc_mode;
	mod_fb_p->framerate = 0x7fffffff;
	mod_fb_p->wait_ready = 0xffffffff;
	mod_fb_p->capability = BIT(VDO_COL_FMT_YUV420)
		| BIT(VDO_COL_FMT_YUV422H) | BIT(VDO_COL_FMT_YUV444)
		| BIT(VDO_COL_FMT_ARGB) | BIT(VDO_COL_FMT_RGB_565)
		| VPP_FB_FLAG_CSC | VPP_FB_FLAG_FIELD;
	p_scl = scl_mod_p;
	p_scl->scale_complete = 1;
	p_scl->scale_sync = 1;
	}

	/* -------------------- SCLW module -------------------- */
	{
	struct sclw_mod_t *sclw_mod_p;

	sclw_mod_p = (struct sclw_mod_t *) vpp_mod_register(VPP_MOD_SCLW,
				sizeof(struct sclw_mod_t),
				VPP_MOD_FLAG_FRAMEBUF);
	if (!sclw_mod_p) {
		DPRINT("*E* SCLW module register fail\n");
		return -1;
	}

	/* module member variable */
	sclw_mod_p->int_catch = VPP_INT_NULL;

	/* module member function */
	sclw_mod_p->init = sclw_init;
	sclw_mod_p->set_enable = sclw_set_mif_enable;

	/* module frame buffer */
	mod_fb_p = sclw_mod_p->fb_p;
	fb_p = &mod_fb_p->fb;

	fb_p->y_addr = 0;
	fb_p->c_addr = 0;
	fb_p->col_fmt = VDO_COL_FMT_YUV422H;
	fb_p->img_w = VPP_HD_DISP_RESX;
	fb_p->img_h = VPP_HD_DISP_RESY;
	fb_p->fb_w = VPP_HD_MAX_RESX;
	fb_p->fb_h = VPP_HD_MAX_RESY;
	fb_p->h_crop = 0;
	fb_p->v_crop = 0;

	/* module frame buffer member function */
	mod_fb_p->csc_mode = VPP_CSC_RGB2YUV_SDTV_0_255;
	mod_fb_p->set_framebuf = sclw_set_framebuffer;
	mod_fb_p->set_addr = sclw_set_fb_addr;
	mod_fb_p->get_addr = sclw_get_fb_addr;
	mod_fb_p->set_csc = scl_set_csc_mode;
	mod_fb_p->wait_ready = 0xffffffff;
	mod_fb_p->capability = BIT(VDO_COL_FMT_YUV422H)
		| BIT(VDO_COL_FMT_YUV444) | BIT(VDO_COL_FMT_ARGB)
		| BIT(VDO_COL_FMT_RGB_565) | VPP_FB_FLAG_CSC
		| BIT(VDO_COL_FMT_YUV420);
	p_sclw = sclw_mod_p;
	}
	return 0;
}
module_init(scl_mod_init);
#endif /* WMT_FTBLK_SCL */
