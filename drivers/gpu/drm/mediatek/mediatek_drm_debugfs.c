/*
 * Copyright (c) 2014 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/fb.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <linux/wait.h>

#include <linux/of_device.h>
#include <drm/drm_mipi_dsi.h>
#include <video/videomode.h>

#include <drm/drmP.h>
#include "mediatek_drm_ddp_comp.h"
#include "mediatek_drm_plane.h"
#include "mediatek_drm_crtc.h"
#include "mediatek_drm_drv.h"

struct mtk_ddp {
	struct device			*dev;
	struct drm_device		*drm_dev;
	struct mediatek_drm_crtc	*crtc;
	int				pipe;

	struct clk			*mutex_disp_clk;

	void __iomem			*config_regs;
	void __iomem			*mutex_regs;
};

struct crtc_ddp_context {
	struct device			*dev;
	struct drm_device		*drm_dev;
	struct mtk_drm_crtc		*crtc;
	struct mtk_drm_plane		planes[OVL_LAYER_NR];
	int				pipe;

	struct device			*ddp_dev;
	u32				ddp_comp_nr;
	struct clk			**ddp_comp_clk;
	void __iomem			**ddp_comp_regs;
	const struct mtk_ddp_comp_funcs	**ddp_comp;

	bool				pending_config;
	unsigned int			pending_width;
	unsigned int			pending_height;

	bool				pending_ovl_config[OVL_LAYER_NR];
	bool				pending_ovl_enable[OVL_LAYER_NR];
	unsigned int			pending_ovl_addr[OVL_LAYER_NR];
	unsigned int			pending_ovl_pitch[OVL_LAYER_NR];
	unsigned int			pending_ovl_format[OVL_LAYER_NR];
	int				pending_ovl_x[OVL_LAYER_NR];
	int				pending_ovl_y[OVL_LAYER_NR];
	unsigned int			pending_ovl_size[OVL_LAYER_NR];
	bool				pending_ovl_dirty[OVL_LAYER_NR];
};

struct mediatek_drm_debugfs_table {
	char name[8];
	unsigned int reg_base;
	unsigned int offset[2];
	unsigned int length[2];
};

/* ------------------------------------------------------------------------- */
/* External variable declarations */
/* ------------------------------------------------------------------------- */
static char debug_buffer[2048];
void __iomem *gdrm_disp_base[8];
void __iomem *gdrm_hdmi_base[6];
struct mediatek_drm_debugfs_table gdrm_disp_table[8] = {
	{"OVL0 ", 0x1400c000, {0, 0xf40}, {0x260, 0x80}},
	{"COLOR0 ", 0x14013000, {0x400, 0xc00}, {0x100, 0x100}},
	{"AAL ", 0x14015000, {0, 0}, {0x100, 0}},
	{"OD ", 0x14023000, {0, 0}, {0x100, 0}},
	{"RDMA0", 0x1400e000, {0, 0}, {0x100, 0}},
	{"UFOE ", 0x1401a000, {0, 0}, {0x100, 0}},
	{"CONFIG ", 0x14000000, {0, 0}, {0x120, 0}},
	{"MUTEX ", 0x14020000, {0, 0}, {0x100, 0}}
};

struct mediatek_drm_debugfs_table gdrm_hdmi_table[6] = {
	{"OVL1 ", 0x1400d000, {0, 0xf40}, {0x260, 0x80}},
	{"COLOR1 ", 0x14014000, {0x400, 0xc00}, {0x100, 0x100}},
	{"GAMMA ", 0x14016000, {0, 0}, {0x100, 0}},
	{"RDMA1 ", 0x1400f000, {0, 0}, {0x100, 0}},
	{"CONFIG ", 0x14000000, {0, 0}, {0x120, 0}},
	{"MUTEX ", 0x14020000, {0, 0}, {0x100, 0}}
};

struct drm_device* gdev;

/* ------------------------------------------------------------------------- */
/* Debug Options */
/* ------------------------------------------------------------------------- */
static char STR_HELP[] =
	"\n"
	"USAGE\n"
	"        echo [ACTION]... > mtkdrm\n"
	"\n"
	"ACTION\n"
	"\n"
	"        dump:\n"
	"             dump all hw registers\n"
	"\n"
	"        regw:addr=val\n"
	"             write hw register\n"
	"\n"
	"        regr:addr\n"
	"             read hw register\n";

/* ------------------------------------------------------------------------- */
/* Command Processor */
/* ------------------------------------------------------------------------- */
static void process_dbg_opt(const char *opt)
{
	if (0 == strncmp(opt, "regw:", 5)) {
		char *p = (char *)opt + 5;
		char *np;
		unsigned long addr, val;
		int i;

		np = strsep(&p, "=");
		if (kstrtoul(np, 16, &addr))
			goto error;

		if (p == NULL)
			goto error;

		np = strsep(&p, "=");
		if (kstrtoul(np, 16, &val))
			goto error;

		for (i = 0; i < sizeof(gdrm_disp_table)/sizeof(struct mediatek_drm_debugfs_table); i++) {
			if (addr > gdrm_disp_table[i].reg_base &&
			addr < gdrm_disp_table[i].reg_base + 0x1000) {
				writel(val, gdrm_disp_base[i] + addr - gdrm_disp_table[i].reg_base);
				break;
			}
		}

		for (i = 0; i < sizeof(gdrm_hdmi_table)/sizeof(struct mediatek_drm_debugfs_table); i++) {
			if (addr > gdrm_hdmi_table[i].reg_base &&
			addr < gdrm_hdmi_table[i].reg_base + 0x1000) {
				writel(val, gdrm_hdmi_base[i] + addr - gdrm_hdmi_table[i].reg_base);
				break;
			}
		}
	} else if (0 == strncmp(opt, "regr:", 5)) {
		char *p = (char *)opt + 5;
		unsigned long addr;
		int i;

		if (kstrtoul(p, 16, &addr))
			goto error;

		for (i = 0; i < sizeof(gdrm_disp_table)/sizeof(struct mediatek_drm_debugfs_table); i++) {
			if (addr >= gdrm_disp_table[i].reg_base &&
			addr < gdrm_disp_table[i].reg_base + 0x1000) {
				DRM_INFO("%8s Read register 0x%08lX: 0x%08X\n",
					gdrm_disp_table[i].name, addr,
					readl(gdrm_disp_base[i] + addr - gdrm_disp_table[i].reg_base));
				break;
			}
		}

		for (i = 0; i < sizeof(gdrm_hdmi_table)/sizeof(struct mediatek_drm_debugfs_table); i++) {
			if (addr >= gdrm_hdmi_table[i].reg_base &&
			addr < gdrm_hdmi_table[i].reg_base + 0x1000) {
				DRM_INFO("%8s Read register 0x%08lX: 0x%08X\n",
					gdrm_hdmi_table[i].name, addr,
					readl(gdrm_hdmi_base[i] + addr - gdrm_hdmi_table[i].reg_base));
				break;
			}
		}
	} else if (0 == strncmp(opt, "dump:", 5)) {
		int i, j;

		for (i = 0; i < sizeof(gdrm_disp_table)/sizeof(struct mediatek_drm_debugfs_table); i++) {
			for (j = gdrm_disp_table[i].offset[0]; j < gdrm_disp_table[i].offset[0] + gdrm_disp_table[i].length[0]; j += 16)
			DRM_INFO("%8s 0x%08X: %08X %08X %08X %08X\n",
					gdrm_disp_table[i].name,
					gdrm_disp_table[i].reg_base + j,
					readl(gdrm_disp_base[i] + j),
					readl(gdrm_disp_base[i] + j + 0x4),
					readl(gdrm_disp_base[i] + j + 0x8),
					readl(gdrm_disp_base[i] + j + 0xc));

			for (j = gdrm_disp_table[i].offset[1]; j < gdrm_disp_table[i].offset[1] + gdrm_disp_table[i].length[1]; j += 16)
			DRM_INFO("%8s 0x%08X: %08X %08X %08X %08X\n",
					gdrm_disp_table[i].name,
					gdrm_disp_table[i].reg_base + j,
					readl(gdrm_disp_base[i] + j),
					readl(gdrm_disp_base[i] + j + 0x4),
					readl(gdrm_disp_base[i] + j + 0x8),
					readl(gdrm_disp_base[i] + j + 0xc));
		}
	} else if (0 == strncmp(opt, "hdmi:", 5)) {
		int i, j;

		for (i = 0; i < sizeof(gdrm_hdmi_table)/sizeof(struct mediatek_drm_debugfs_table); i++) {
			for (j = gdrm_hdmi_table[i].offset[0]; j < gdrm_hdmi_table[i].offset[0] + gdrm_hdmi_table[i].length[0]; j += 16)
			DRM_INFO("%8s 0x%08X: %08X %08X %08X %08X\n",
					gdrm_hdmi_table[i].name,
					gdrm_hdmi_table[i].reg_base + j,
					readl(gdrm_hdmi_base[i] + j),
					readl(gdrm_hdmi_base[i] + j + 0x4),
					readl(gdrm_hdmi_base[i] + j + 0x8),
					readl(gdrm_hdmi_base[i] + j + 0xc));

			for (j = gdrm_hdmi_table[i].offset[1]; j < gdrm_hdmi_table[i].offset[1] + gdrm_hdmi_table[i].length[1]; j += 16)
			DRM_INFO("%8s 0x%08X: %08X %08X %08X %08X\n",
					gdrm_hdmi_table[i].name,
					gdrm_hdmi_table[i].reg_base + j,
					readl(gdrm_hdmi_base[i] + j),
					readl(gdrm_hdmi_base[i] + j + 0x4),
					readl(gdrm_hdmi_base[i] + j + 0x8),
					readl(gdrm_hdmi_base[i] + j + 0xc));
		}
	} else {
	    goto error;
	}

	return;
 error:
	DRM_ERROR("Parse command error!\n\n%s", STR_HELP);
}

static void process_dbg_cmd(char *cmd)
{
	char *tok;

	DRM_INFO("[mtkdrm_dbg] %s\n", cmd);
	memset(debug_buffer, 0, sizeof(debug_buffer));
	while ((tok = strsep(&cmd, " ")) != NULL)
		process_dbg_opt(tok);
}

/* ------------------------------------------------------------------------- */
/* Debug FileSystem Routines */
/* ------------------------------------------------------------------------- */
static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t debug_read(struct file *file, char __user *ubuf, size_t count,
	loff_t *ppos)
{
	if (strlen(debug_buffer))
		return simple_read_from_buffer(ubuf, count, ppos, debug_buffer,
			strlen(debug_buffer));
	else
		return simple_read_from_buffer(ubuf, count, ppos, STR_HELP,
			strlen(STR_HELP));
}

static char dis_cmd_buf[512];
static ssize_t debug_write(struct file *file, const char __user *ubuf,
	size_t count, loff_t *ppos)
{
	const int debug_bufmax = sizeof(dis_cmd_buf) - 1;
	size_t ret;

	ret = count;

	if (count > debug_bufmax)
		count = debug_bufmax;

	if (copy_from_user(&dis_cmd_buf, ubuf, count))
		return -EFAULT;

	dis_cmd_buf[count] = 0;

	process_dbg_cmd(dis_cmd_buf);

	return ret;
}

struct dentry *mtkdrm_dbgfs;
static const struct file_operations debug_fops = {
	.read = debug_read,
	.write = debug_write,
	.open = debug_open,
};

void mediatek_drm_debugfs_init(struct drm_device *dev)
{
	struct mtk_drm_private *priv = dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc;
	struct crtc_ddp_context *crtc_ddp;
	struct mtk_ddp *ddp;

	int i;

	DRM_INFO("mediatek_drm_debugfs_init\n");
	mtkdrm_dbgfs = debugfs_create_file("mtkdrm", S_IFREG | S_IRUGO |
			S_IWUSR | S_IWGRP, NULL, (void *)0, &debug_fops);

	mtk_crtc = to_mtk_crtc(priv->crtc[0]);
	crtc_ddp = (struct crtc_ddp_context *)mtk_crtc->ctx;

	gdev = crtc_ddp->drm_dev;
	ddp = dev_get_drvdata(crtc_ddp->ddp_dev);
	//ddp = crtc_ddp->ddp_dev;

	for (i = 0; crtc_ddp->ddp_comp_regs[i]; i++)
		gdrm_disp_base[i] = crtc_ddp->ddp_comp_regs[i];
	gdrm_disp_base[i++] = ddp->config_regs;
	gdrm_disp_base[i++] = ddp->mutex_regs;

	mtk_crtc = to_mtk_crtc(priv->crtc[1]);
	crtc_ddp = (struct crtc_ddp_context *)mtk_crtc->ctx;
	for (i = 0; crtc_ddp->ddp_comp_regs[i]; i++)
		gdrm_hdmi_base[i] = crtc_ddp->ddp_comp_regs[i];
	gdrm_hdmi_base[i++] = ddp->config_regs;
	gdrm_hdmi_base[i++] = ddp->mutex_regs;

	DRM_INFO("mediatek_drm_debugfs_init..done\n");
}

void mediatek_drm_debugfs_deinit(void)
{
	debugfs_remove(mtkdrm_dbgfs);
}
