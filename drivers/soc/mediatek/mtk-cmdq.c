/*
 * Copyright (c) 2015 MediaTek Inc.
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

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/ftrace.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/memory.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <soc/mediatek/cmdq.h>

#include "mtk-cmdq-debug.h"
#include "mtk-cmdq-private.h"

static const struct of_device_id cmdq_of_ids[] = {
	{.compatible = "mediatek,mt8173-gce",},
	{}
};

struct cmdq_event_name {
	enum cmdq_event	event;
	char		*name;
};

struct cmdq_subsys {
	u32	base_addr;
	int	id;
	char	*grp_name;
};

struct reg_def {
	int offset;
	const char *name;
};

static const struct cmdq_event_name g_event_name[] = {
	/* Display start of frame(SOF) events */
	{CMDQ_EVENT_DISP_OVL0_SOF, "CMDQ_EVENT_DISP_OVL0_SOF",},
	{CMDQ_EVENT_DISP_OVL1_SOF, "CMDQ_EVENT_DISP_OVL1_SOF",},
	{CMDQ_EVENT_DISP_RDMA0_SOF, "CMDQ_EVENT_DISP_RDMA0_SOF",},
	{CMDQ_EVENT_DISP_RDMA1_SOF, "CMDQ_EVENT_DISP_RDMA1_SOF",},
	{CMDQ_EVENT_DISP_RDMA2_SOF, "CMDQ_EVENT_DISP_RDMA2_SOF",},
	{CMDQ_EVENT_DISP_WDMA0_SOF, "CMDQ_EVENT_DISP_WDMA0_SOF",},
	{CMDQ_EVENT_DISP_WDMA1_SOF, "CMDQ_EVENT_DISP_WDMA1_SOF",},
	/* Display end of frame(EOF) events */
	{CMDQ_EVENT_DISP_OVL0_EOF, "CMDQ_EVENT_DISP_OVL0_EOF",},
	{CMDQ_EVENT_DISP_OVL1_EOF, "CMDQ_EVENT_DISP_OVL1_EOF",},
	{CMDQ_EVENT_DISP_RDMA0_EOF, "CMDQ_EVENT_DISP_RDMA0_EOF",},
	{CMDQ_EVENT_DISP_RDMA1_EOF, "CMDQ_EVENT_DISP_RDMA1_EOF",},
	{CMDQ_EVENT_DISP_RDMA2_EOF, "CMDQ_EVENT_DISP_RDMA2_EOF",},
	{CMDQ_EVENT_DISP_WDMA0_EOF, "CMDQ_EVENT_DISP_WDMA0_EOF",},
	{CMDQ_EVENT_DISP_WDMA1_EOF, "CMDQ_EVENT_DISP_WDMA1_EOF",},
	/* Mutex end of frame(EOF) events */
	{CMDQ_EVENT_MUTEX0_STREAM_EOF, "CMDQ_EVENT_MUTEX0_STREAM_EOF",},
	{CMDQ_EVENT_MUTEX1_STREAM_EOF, "CMDQ_EVENT_MUTEX1_STREAM_EOF",},
	{CMDQ_EVENT_MUTEX2_STREAM_EOF, "CMDQ_EVENT_MUTEX2_STREAM_EOF",},
	{CMDQ_EVENT_MUTEX3_STREAM_EOF, "CMDQ_EVENT_MUTEX3_STREAM_EOF",},
	{CMDQ_EVENT_MUTEX4_STREAM_EOF, "CMDQ_EVENT_MUTEX4_STREAM_EOF",},
	/* Display underrun events */
	{CMDQ_EVENT_DISP_RDMA0_UNDERRUN, "CMDQ_EVENT_DISP_RDMA0_UNDERRUN",},
	{CMDQ_EVENT_DISP_RDMA1_UNDERRUN, "CMDQ_EVENT_DISP_RDMA1_UNDERRUN",},
	{CMDQ_EVENT_DISP_RDMA2_UNDERRUN, "CMDQ_EVENT_DISP_RDMA2_UNDERRUN",},
	/* Keep this at the end of HW events */
	{CMDQ_MAX_HW_EVENT_COUNT, "CMDQ_MAX_HW_EVENT_COUNT",},
	/* GPR events */
	{CMDQ_SYNC_TOKEN_GPR_SET_0, "CMDQ_SYNC_TOKEN_GPR_SET_0",},
	{CMDQ_SYNC_TOKEN_GPR_SET_1, "CMDQ_SYNC_TOKEN_GPR_SET_1",},
	{CMDQ_SYNC_TOKEN_GPR_SET_2, "CMDQ_SYNC_TOKEN_GPR_SET_2",},
	{CMDQ_SYNC_TOKEN_GPR_SET_3, "CMDQ_SYNC_TOKEN_GPR_SET_3",},
	{CMDQ_SYNC_TOKEN_GPR_SET_4, "CMDQ_SYNC_TOKEN_GPR_SET_4",},
	/* This is max event and also can be used as mask. */
	{CMDQ_SYNC_TOKEN_MAX, "CMDQ_SYNC_TOKEN_MAX",},
	/* Invalid event */
	{CMDQ_SYNC_TOKEN_INVALID, "CMDQ_SYNC_TOKEN_INVALID",},
};

static const struct cmdq_subsys g_subsys[] = {
	{0x1400, 1, "MMSYS"},
	{0x1401, 2, "DISP"},
	{0x1402, 3, "DISP"},
};

static void cmdq_core_handle_error(struct cmdq_context *cqctx,
				   int thread,
				   int value,
				   unsigned long long *got_irq);

static void cmdq_core_handle_done(struct cmdq_context *cqctx,
				  int thread,
				  int value,
				  unsigned long long *got_irq);

static void cmdq_core_consume_waiting_list(struct work_struct *work);

static bool cmdq_core_verfiy_task_command_end(const struct cmdq_task *task);

static void cmdq_core_invalidate_hw_fetched_buffer(void __iomem *gce_base_va,
						   int thread);

const char *cmdq_event_get_name(enum cmdq_event event)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_event_name); i++)
		if (g_event_name[i].event == event)
			return g_event_name[i].name;

	return "CMDQ_EVENT_UNKNOWN";
}

static void cmdq_event_set(void __iomem *gce_base_va, enum cmdq_event event)
{
	writel((1 << 16) | event, gce_base_va + CMDQ_SYNC_TOKEN_UPD_OFFSET);
}

static void cmdq_event_reset(struct cmdq_context *cqctx)
{
	int i;
	u32 val;
	void __iomem *gce_base_va = cqctx->cqdev.base_va;

	/* set all defined events to 0 */
	for (i = 0; i < ARRAY_SIZE(g_event_name); i++) {
		if (g_event_name[i].event >= CMDQ_MAX_HW_EVENT_COUNT)
			break;

		val = CMDQ_SYNC_TOKEN_MAX & g_event_name[i].event;
		writel(val, gce_base_va + CMDQ_SYNC_TOKEN_UPD_OFFSET);
	}

	/* set GPR(resource flags) to 1 */
	cmdq_event_set(gce_base_va, CMDQ_SYNC_TOKEN_GPR_SET_0);
	cmdq_event_set(gce_base_va, CMDQ_SYNC_TOKEN_GPR_SET_1);
	cmdq_event_set(gce_base_va, CMDQ_SYNC_TOKEN_GPR_SET_2);
	cmdq_event_set(gce_base_va, CMDQ_SYNC_TOKEN_GPR_SET_3);
	cmdq_event_set(gce_base_va, CMDQ_SYNC_TOKEN_GPR_SET_4);
}

static int cmdq_subsys_base_addr_to_id(u32 base_addr)
{
	int i;
	int id = -EFAULT;

	for (i = 0; i < ARRAY_SIZE(g_subsys); i++) {
		if (g_subsys[i].base_addr == base_addr) {
			id = g_subsys[i].id;
			break;
		}
	}

	return id;
}

static u32 cmdq_subsys_id_to_base_addr(int id)
{
	int i;
	u32 base_addr = 0;

	for (i = 0; i < ARRAY_SIZE(g_subsys); i++) {
		if (g_subsys[i].id == id) {
			base_addr = g_subsys[i].base_addr;
			break;
		}
	}

	return base_addr;
}

static const char *cmdq_subsys_base_addr_to_grp_name(u32 base_addr)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_subsys); i++)
		if (g_subsys[i].base_addr == base_addr)
			return g_subsys[i].grp_name;

	return NULL;
}

bool cmdq_core_support_sync_non_suspendable(void)
{
	return true;
}

static bool cmdq_core_should_enable_prefetch(enum cmdq_scenario scenario)
{
	switch (scenario) {
	case CMDQ_SCENARIO_PRIMARY_DISP:
		/*
		 * Primary DISP HW should enable prefetch.
		 * Since thread 0/1 shares one prefetch buffer,
		 * we allow only PRIMARY path to use prefetch.
		 */
		return true;
	default:
		return false;
	}

	return false;
}

const char *cmdq_core_parse_module_from_reg_addr(u32 reg_addr)
{
	const u32 addr_base_and_page = (reg_addr & 0xfffff000);
	const u32 addr_base_shifted = (reg_addr & 0xffff0000) >> 16;
	const char *module = "CMDQ";
	const char *grp_name;

	/*
	 * for well-known base, we check them with 12-bit mask
	 * defined in mt_reg_base.h
	 */
	switch (addr_base_and_page) {
	case 0x14000000:
		return "MMSYS_CONFIG";
	case 0x1400c000:
		return "DISP_OVL0";
	case 0x1400d000:
		return "DISP_OVL1";
	case 0x1400e000:
		return "DISP_RDMA0";
	case 0x1400f000:
		return "DISP_RDMA1";
	case 0x14010000:
		return "DISP_RDMA2";
	case 0x14011000:
		return "DISP_WDMA0";
	case 0x14012000:
		return "DISP_WDMA1";
	case 0x14013000:
		return "DISP_COLOR0";
	case 0x14014000:
		return "DISP_COLOR1";
	case 0x14015000:
		return "DISP_AAL";
	case 0x14016000:
		return "DISP_GAMMA";
	case 0x14017000:
		return "DISP_MERGE";
	case 0x14018000:
		return "DISP_SPLIT0";
	case 0x14019000:
		return "DISP_SPLIT1";
	case 0x1401a000:
		return "DISP_UFOE";
	case 0x1401b000:
		return "DSI0";
	case 0x1401c000:
		return "DSI1";
	case 0x1401d000:
		return "DPI0";
	case 0x1401e000:
		return "DISP_PWM0";
	case 0x1401f000:
		return "DISP_PWM1";
	case 0x14020000:
		return "MM_MUTEX";
	case 0x14021000:
		return "SMI_LARB0";
	case 0x14022000:
		return "SMI_COMMON";
	case 0x14024000:
		return "DPI1";
	case 0x14025000:
		return "HDMI";
	case 0x14026000:
		return "LVDS";
	case 0x14027000:
		return "SMI_LARB4";
	}

	/*
	 * for other register address we rely on GCE subsys to group them with
	 * 16-bit mask.
	 */
	grp_name = cmdq_subsys_base_addr_to_grp_name(addr_base_shifted);

	if (grp_name != NULL)
		return grp_name;
	else
		return module;
}

static int cmdq_core_get_thread_index_from_scenario(enum cmdq_scenario scenario)
{
	if (cmdq_core_should_enable_prefetch(scenario))
		return 0;

	switch (scenario) {
	case CMDQ_SCENARIO_PRIMARY_DISP:
		/* primary display: thread 0 */
		return 0;
	case CMDQ_SCENARIO_SUB_DISP:
		/*
		 * When HW thread 0 enables pre-fetch,
		 * sub display need to use thread 5 instead of 1
		 */
		return 5;
	default:
		/* freely dispatch */
		return CMDQ_INVALID_THREAD;
	}

	/* freely dispatch */
	return CMDQ_INVALID_THREAD;
}

static const char *cmdq_core_module_from_event_id(enum cmdq_event event)
{
	const char *module = "CMDQ";

	switch (event) {
	case CMDQ_EVENT_DISP_RDMA0_SOF:
	case CMDQ_EVENT_DISP_RDMA1_SOF:
	case CMDQ_EVENT_DISP_RDMA2_SOF:
	case CMDQ_EVENT_DISP_RDMA0_EOF:
	case CMDQ_EVENT_DISP_RDMA1_EOF:
	case CMDQ_EVENT_DISP_RDMA2_EOF:
	case CMDQ_EVENT_DISP_RDMA0_UNDERRUN:
	case CMDQ_EVENT_DISP_RDMA1_UNDERRUN:
	case CMDQ_EVENT_DISP_RDMA2_UNDERRUN:
		module = "DISP_RDMA";
		break;
	case CMDQ_EVENT_DISP_WDMA0_SOF:
	case CMDQ_EVENT_DISP_WDMA1_SOF:
	case CMDQ_EVENT_DISP_WDMA0_EOF:
	case CMDQ_EVENT_DISP_WDMA1_EOF:
		module = "DISP_WDMA";
		break;
	case CMDQ_EVENT_DISP_OVL0_SOF:
	case CMDQ_EVENT_DISP_OVL1_SOF:
	case CMDQ_EVENT_DISP_OVL0_EOF:
	case CMDQ_EVENT_DISP_OVL1_EOF:
		module = "DISP_OVL";
		break;
	case CMDQ_EVENT_MUTEX0_STREAM_EOF ... CMDQ_EVENT_MUTEX4_STREAM_EOF:
		module = "DISP";
		break;
	default:
		module = "CMDQ";
		break;
	}

	return module;
}

static u64 cmdq_rec_flag_from_scenario(struct cmdq_context *cqctx,
				       enum cmdq_scenario scn)
{
	struct device *dev = cqctx->dev;
	u64 flag;

	switch (scn) {
	case CMDQ_SCENARIO_PRIMARY_DISP:
		flag = ((1LL << CMDQ_ENG_DISP_OVL0) |
			(1LL << CMDQ_ENG_DISP_COLOR0) |
			(1LL << CMDQ_ENG_DISP_AAL) |
			(1LL << CMDQ_ENG_DISP_RDMA0) |
			(1LL << CMDQ_ENG_DISP_UFOE) |
			(1LL << CMDQ_ENG_DISP_DSI0_CMD));
		break;
	case CMDQ_SCENARIO_SUB_DISP:
		flag = ((1LL << CMDQ_ENG_DISP_OVL1) |
			(1LL << CMDQ_ENG_DISP_COLOR1) |
			(1LL << CMDQ_ENG_DISP_GAMMA) |
			(1LL << CMDQ_ENG_DISP_RDMA1) |
			(1LL << CMDQ_ENG_DISP_DSI1_CMD));
		break;
	default:
		dev_err(dev, "unknown scenario type %d\n", scn);
		flag = 0LL;
		break;
	}

	return flag;
}


static bool cmdq_core_should_profile(enum cmdq_scenario scenario)
{
	/* need to modify */
	/*
	 * e.g.
	 * switch (scenario) {
	 * case CMDQ_SCENARIO_PRIMARY_DISP:
	 *   return true;
	 * default:
	 *   return false;
	 * }
	 */
	return false;
}

void cmdq_core_print_event_seq(struct seq_file *m, struct cmdq_context *cqctx)
{
	const u32 events[] = {
		CMDQ_EVENT_MUTEX0_STREAM_EOF,
		CMDQ_EVENT_MUTEX1_STREAM_EOF
	};
	int i;

	seq_puts(m, "====== Events =======\n");

	for (i = 0; i < (sizeof(events) / sizeof(u32)); ++i) {
		const u32 e = 0x3ff & events[i];
		u32 event_value;
		void __iomem *gce_base_va = cqctx->cqdev.base_va;

		writel(e, gce_base_va + CMDQ_SYNC_TOKEN_ID_OFFSET);
		event_value = readl(gce_base_va + CMDQ_SYNC_TOKEN_VAL_OFFSET);
		seq_printf(m, "[EVENT]%s = %d\n",
			   cmdq_event_get_name(e), event_value);
	}
}

u32 cmdq_get_cookie_cnt(void __iomem *gce_base_va, int tid)
{
	u32 cnt;

	cnt = readl(gce_base_va + CMDQ_THR_EXEC_CNT_OFFSET +
		    CMDQ_THR_SHIFT * tid);
	cnt &= CMDQ_COOKIE_MASK;
	return cnt;
}

void cmdq_core_long_string(bool force_log, char *buf, u32 *offset,
			   int *max_size, const char *string, ...)
{
	int msg_len;
	va_list argptr;
	char *buf_ptr = buf + (*offset);

	if (!force_log)
		return;

	va_start(argptr, string);
	msg_len = vsnprintf(buf_ptr, *max_size, string, argptr);
	*max_size -= msg_len;
	*offset += msg_len;
	va_end(argptr);
}

static void *cmdq_core_alloc_hw_buffer(struct device *dev, size_t size,
				       dma_addr_t *pa, const gfp_t flag)
{
	void *va;

	va = dma_alloc_coherent(dev, size, pa, flag);
	CMDQ_MSG("%s: va:0x%p, pa:0x%llx\n", __func__, va, *pa);
	return va;
}

static void cmdq_core_free_hw_buffer(struct device *dev, size_t size,
				     void *va, dma_addr_t pa)
{
	CMDQ_MSG("%s: va:0x%p, pa:0x%llx\n", __func__, va, pa);
	dma_free_coherent(dev, size, va, pa);
}

static int cmdq_emergency_buf_init(struct cmdq_context *cqctx)
{
	int i;
	int ret = 0;
	struct device *dev = cqctx->dev;
	struct cmdq_emergency_buf *buf;

	memset(cqctx->emergency_buf, 0, sizeof(cqctx->emergency_buf));

	for (i = 0; i < ARRAY_SIZE(cqctx->emergency_buf); i++) {
		buf = &cqctx->emergency_buf[i];
		buf->va = cmdq_core_alloc_hw_buffer(
			dev, CMDQ_EMERGENCY_BLOCK_SIZE, &buf->pa, GFP_KERNEL);
		if (!buf->va) {
			dev_err(dev, "failed to alloc emergency buffer\n");
			ret = -ENOMEM;
			goto fail_alloc;
		}
		atomic_set(&buf->used, 0);
	}

	return 0;

fail_alloc:
	for (i -= 1; i >= 0 ; i--) {
		buf = &cqctx->emergency_buf[i];
		cmdq_core_free_hw_buffer(
			dev, CMDQ_EMERGENCY_BLOCK_SIZE, buf->va, buf->pa);
		cqctx->emergency_buf[i].va = NULL;
		cqctx->emergency_buf[i].pa = 0;
	}

	return ret;
}

static void cmdq_emergency_buf_uninit(struct cmdq_context *cqctx)
{
	int i;
	struct device *dev = cqctx->dev;
	struct cmdq_emergency_buf *buf;

	for (i = 0; i < ARRAY_SIZE(cqctx->emergency_buf); i++) {
		buf = &cqctx->emergency_buf[i];
		cmdq_core_free_hw_buffer(
			dev, CMDQ_EMERGENCY_BLOCK_SIZE, buf->va, buf->pa);
		if (atomic_read(&buf->used))
			dev_err(dev,
				"emergency buffer %d, 0x%p, 0x%p still in use\n",
				i, buf->va, &buf->pa);
	}
	memset(cqctx->emergency_buf, 0, sizeof(cqctx->emergency_buf));
}

/* get buffer from emergency buffer */
static struct cmdq_emergency_buf *
cmdq_emergency_buf_get(struct cmdq_context *cqctx)
{
	int i;
	struct cmdq_emergency_buf *buf;

	for (i = 0; i < ARRAY_SIZE(cqctx->emergency_buf); i++) {
		buf = &cqctx->emergency_buf[i];
		if (!atomic_cmpxchg(&buf->used, 0, 1)) {
			CMDQ_MSG("emergency buf[%d]: va:0x%p, pa:0x%p\n",
				 i,
				 cqctx->emergency_buf[i].va,
				 &cqctx->emergency_buf[i].pa);
			break;
		}
	}

	if (i >= ARRAY_SIZE(cqctx->emergency_buf))
		buf = NULL;

	return buf;
}

/* put buffer back into emergency buffer */
static void cmdq_emergency_buf_put(struct cmdq_emergency_buf *buf)
{
	atomic_set(&buf->used, 0);
}

static int cmdq_subsys_from_phys_addr(struct cmdq_context *cqctx,
				      u32 cmdq_phys_addr)
{
	u32 base_addr = cmdq_phys_addr >> 16;
	int subsys = cmdq_subsys_base_addr_to_id(base_addr);

	if (subsys < 0)
		dev_err(cqctx->dev,
			"unknown subsys: error=%d, phys=0x%08x\n",
			subsys, cmdq_phys_addr);

	return subsys;
}

/* for kmemcache, initialize variables of struct cmdq_task (without buffers) */
static void cmdq_core_task_ctor(void *param)
{
	struct cmdq_task *task = param;

	CMDQ_MSG("%s: 0x%p\n", __func__, param);
	memset(task, 0, sizeof(*task));
	INIT_LIST_HEAD(&task->list_entry);
	task->task_state = TASK_STATE_IDLE;
	task->thread = CMDQ_INVALID_THREAD;
}

static void cmdq_task_free_task_command_buffer(struct cmdq_task *task)
{
	struct cmdq_context *cqctx = task->cqctx;
	struct device *dev = cqctx->dev;

	if (!task->va_base)
		return;

	if (task->emergency_buf)
		cmdq_emergency_buf_put(task->emergency_buf);
	else
		cmdq_core_free_hw_buffer(
			dev, task->buf_size, task->va_base, task->mva_base);

	task->va_base = NULL;
	task->mva_base = 0;
	task->buf_size = 0;
	task->command_size = 0;
	task->num_cmd = 0;
	task->emergency_buf = NULL;
}

/*
 * Ensures size of command buffer of the given task
 * Existing buffer will be copied to new buffer.
 * This buffer is guaranteed to be physically continuous.
 * returns -ENOMEM if cannot allocate new buffer
 */
static int cmdq_core_task_realloc_buffer_size(struct cmdq_task *task, u32 size)
{
	void *new_buf;
	dma_addr_t new_mva_base;
	u32 cmd_size;
	u32 num_cmd;
	struct cmdq_context *cqctx = task->cqctx;
	struct device *dev = cqctx->dev;
	struct cmdq_emergency_buf *ebuf = NULL;

	if (task->va_base && task->buf_size >= size)
		return 0;

	/* allocate new buffer, try if we can alloc without reclaim */
	new_buf = cmdq_core_alloc_hw_buffer(
		dev, size, &new_mva_base, GFP_KERNEL); // | __GFP_NO_KSWAPD);

	/* if failed, try emergency buffer */
	if (!new_buf && size <= CMDQ_EMERGENCY_BLOCK_SIZE)
		ebuf = cmdq_emergency_buf_get(cqctx);
	if (ebuf) {
		new_buf = ebuf->va;
		new_mva_base = ebuf->pa;
		memset(new_buf, 0, size);
		CMDQ_LOG("emergency buffer 0x%p allocated\n", new_buf);
	}

	/* if failed, finally try reclaim */
	if (!new_buf) {
		new_buf = cmdq_core_alloc_hw_buffer(
			dev, size, &new_mva_base, GFP_KERNEL);
		if (new_buf)
			CMDQ_LOG("buffer 0x%p allocated after reclaim\n", new_buf);
	}

	if (!new_buf) {
		dev_err(dev, "realloc cmd buffer of size %d failed\n", size);
		return -ENOMEM;
	}

	/* copy and release old buffer */
	if (task->va_base)
		memcpy(new_buf, task->va_base, task->buf_size);

	/*
	 * we should keep track of num_cmd and cmd_size
	 * since they are cleared in free command buffer
	 */
	num_cmd = task->num_cmd;
	cmd_size = task->command_size;
	cmdq_task_free_task_command_buffer(task);

	/* attach the new buffer */
	task->va_base = new_buf;
	task->mva_base = new_mva_base;
	task->buf_size = size;
	task->num_cmd = num_cmd;
	task->command_size = cmd_size;
	task->emergency_buf = ebuf;

	CMDQ_MSG("task buffer:0x%p, va:0x%p pa:0x%p\n",
		 task, task->va_base, &task->mva_base);
	return 0;
}

/* allocate and initialize struct cmdq_task and its command buffer */
static struct cmdq_task *cmdq_core_task_create(struct cmdq_context *cqctx)
{
	struct cmdq_task *task;
	int status;
	struct device *dev = cqctx->dev;

	task = kmem_cache_alloc(cqctx->task_cache, GFP_KERNEL);
	task->cqctx = cqctx;
	status = cmdq_core_task_realloc_buffer_size(
		task, CMDQ_INITIAL_CMD_BLOCK_SIZE);
	if (status < 0) {
		dev_err(dev, "allocate command buffer failed\n");
		kmem_cache_free(cqctx->task_cache, task);
		task = NULL;
	}
	return task;
}

static void cmdq_core_reset_cmdq_engine(struct cmdq_context *cqctx)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cqctx->engine); i++)
		cqctx->engine[i].curr_owner = CMDQ_INVALID_THREAD;
}

static long of_get_pa(struct device_node *node, int index)
{
	struct resource res;

	if (of_address_to_resource(node, index, &res))
		return 0;

	return res.start;
}

static int cmdq_dev_init(struct platform_device *pdev,
			 struct cmdq_device *cqdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cqdev->base_va = devm_ioremap_resource(dev, res);
	if (IS_ERR(cqdev->base_va)) {
		dev_err(dev, "failed to ioremap gce\n");
		return PTR_ERR(cqdev->base_va);
	}

	cqdev->base_pa = of_get_pa(node, 0);
	if (!cqdev->base_pa) {
		dev_err(dev, "failed to get gce pa\n");
		return -EINVAL;
	}

	cqdev->irq = irq_of_parse_and_map(node, 0);
	if (!cqdev->irq) {
		dev_err(dev, "failed to get irq\n");
		return -EINVAL;
	}

	dev_dbg(dev, "cmdq device: addr:0x%p, pa:0x%lx, va:0x%p, irq:%d\n",
		dev, cqdev->base_pa, cqdev->base_va, cqdev->irq);
	return 0;
}

static int cmdq_core_initialize(struct platform_device *pdev,
				struct cmdq_context **cqctx)
{
	int i;
	struct cmdq_context *lcqctx; /* local cmdq context */
	int ret = 0;

	lcqctx = devm_kzalloc(&pdev->dev, sizeof(*lcqctx), GFP_KERNEL);

	/* save dev */
	lcqctx->dev = &pdev->dev;

	/* initial cmdq device related data */
	ret = cmdq_dev_init(pdev, &lcqctx->cqdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to init cmdq device\n");
		goto fail_dev;
	}

	/* initial mutex, spinlock */
	mutex_init(&lcqctx->task_mutex);
	mutex_init(&lcqctx->clock_mutex);
	spin_lock_init(&lcqctx->thread_lock);
	spin_lock_init(&lcqctx->exec_lock);
	spin_lock_init(&lcqctx->record_lock);

	/* initial wait queue for notification */
	for (i = 0; i < ARRAY_SIZE(lcqctx->wait_queue); i++)
		init_waitqueue_head(&lcqctx->wait_queue[i]);
	init_waitqueue_head(&lcqctx->thread_dispatch_queue);

	/* some fields has non-zero initial value */
	cmdq_core_set_log_level(CMDQ_CORE_DEFAULT_LOG_LEVEL);
	cmdq_core_set_show_interface(CMDQ_CORE_DEFAULT_SHOW_API);
	cmdq_core_reset_cmdq_engine(lcqctx);

	/* create task pool */
	lcqctx->task_cache = kmem_cache_create(
		CMDQ_DRIVER_DEVICE_NAME "_task", sizeof(struct cmdq_task),
		__alignof__(struct cmdq_task),
		SLAB_POISON | SLAB_HWCACHE_ALIGN | SLAB_RED_ZONE,
		&cmdq_core_task_ctor);

	/* initialize task lists */
	INIT_LIST_HEAD(&lcqctx->task_free_list);
	INIT_LIST_HEAD(&lcqctx->task_active_list);
	INIT_LIST_HEAD(&lcqctx->task_wait_list);
	INIT_WORK(&lcqctx->task_consume_wait_queue_item,
		  cmdq_core_consume_waiting_list);

	/* initialize emergency buffer */
	ret = cmdq_emergency_buf_init(lcqctx);
	if (ret) {
		dev_err(&pdev->dev, "failed to init emergency buf\n");
		goto fail_emergency_buf;
	}

	lcqctx->task_auto_release_wq = alloc_ordered_workqueue(
		"%s", WQ_MEM_RECLAIM | WQ_HIGHPRI, "cmdq_auto_release");
	lcqctx->task_consume_wq = alloc_ordered_workqueue(
		"%s", WQ_MEM_RECLAIM | WQ_HIGHPRI, "cmdq_task");

	/* initialize cmdq_print */
	lcqctx->print_log_struct.cqctx = lcqctx;
	INIT_WORK(&lcqctx->print_log_struct.cmdq_print_work,
		  cmdq_core_run_print_log);
	lcqctx->print_log_struct.cmdq_print_wq =
		create_singlethread_workqueue("cmdq_print");

	*cqctx = lcqctx;
	return ret;

fail_emergency_buf:
	destroy_workqueue(lcqctx->task_auto_release_wq);
	destroy_workqueue(lcqctx->task_consume_wq);
	kmem_cache_destroy(lcqctx->task_cache);

fail_dev:
	return ret;
}

/* This func should be inside cqctx->task_mutex mutex */
static void cmdq_core_release_task_unlocked(struct cmdq_task *task)
{
	struct cmdq_context *cqctx = task->cqctx;
	struct device *dev = cqctx->dev;

	lockdep_assert_held(&cqctx->task_mutex);

	task->task_state = TASK_STATE_IDLE;
	task->thread = CMDQ_INVALID_THREAD;

	if (task->profile_data) {
		cmdq_core_free_hw_buffer(dev,
					 2 * sizeof(u32),
					 task->profile_data,
					 task->profile_data_pa);
		task->profile_data = NULL;
		task->profile_data_pa = 0;
	}

	cmdq_task_free_task_command_buffer(task);

	/* remove from active/waiting list */
	list_del_init(&task->list_entry);
	/* insert into free list. Currently we don't shrink free list. */
	list_add_tail(&task->list_entry, &cqctx->task_free_list);
}

static void cmdq_core_release_task_internal(struct cmdq_task *task)
{
	struct cmdq_context *cqctx = task->cqctx;

	CMDQ_MSG("-->TASK: Release task structure 0x%p begin\n", task);
	mutex_lock(&cqctx->task_mutex);
	cmdq_core_release_task_unlocked(task);
	mutex_unlock(&cqctx->task_mutex);
	CMDQ_MSG("<--TASK: Release task structure end\n");
}

static struct cmdq_task *cmdq_core_find_free_task(struct cmdq_context *cqctx)
{
	struct cmdq_task *task;

	mutex_lock(&cqctx->task_mutex);

	/*
	 * Pick from free list first;
	 * create one if there is no free entry.
	 */
	if (list_empty(&cqctx->task_free_list)) {
		task = cmdq_core_task_create(cqctx);
	} else {
		task = list_first_entry(&cqctx->task_free_list,
					struct cmdq_task, list_entry);
		/* remove from free list */
		list_del_init(&task->list_entry);
	}

	mutex_unlock(&cqctx->task_mutex);

	return task;
}

static void cmdq_core_reorder_task_array(struct cmdq_thread *cq_td,
					 int prev_id)
{
	int loop, next_id, search_loop, search_id;
	int reorder_count = 0;
	struct cmdq_task *cqtsk;

	next_id = prev_id + 1;
	for (loop = 1; loop < (CMDQ_MAX_TASK_IN_THREAD - 1);
	     loop++, next_id++) {
		if (next_id >= CMDQ_MAX_TASK_IN_THREAD)
			next_id = 0;

		if (cq_td->cur_task[next_id])
			break;

		search_id = next_id + 1;
		for (search_loop = (loop + 1);
		     search_loop < CMDQ_MAX_TASK_IN_THREAD;
		     search_loop++, search_id++) {
			if (search_id >= CMDQ_MAX_TASK_IN_THREAD)
				search_id = 0;

			if (cq_td->cur_task[search_id]) {
				cq_td->cur_task[next_id] =
					cq_td->cur_task[search_id];
				cq_td->cur_task[search_id] = NULL;
				CMDQ_LOG("WAIT: reorder slot %d to slot 0%d.\n",
					 search_id, next_id);
				if ((search_loop - loop) > reorder_count)
					reorder_count = search_loop - loop;

				break;
			}
		}

		cqtsk = cq_td->cur_task[next_id];
		if ((cqtsk->va_base[cqtsk->num_cmd-1] == 0x10000000) &&
		    (cqtsk->va_base[cqtsk->num_cmd-2] == 0x00000008)) {
			/* We reached the last task */
			break;
		}
	}

	cq_td->next_cookie -= reorder_count;
	CMDQ_MSG("WAIT: nextcookie minus %d.\n", reorder_count);
}

static void cmdq_core_append_command_into_task(struct cmdq_task *task,
					       u32 arg_a,
					       u32 arg_b)
{
	task->va_base[task->num_cmd] = arg_b;
	task->va_base[task->num_cmd+1] = arg_a;
	task->command_size += CMDQ_INST_SIZE;
	task->num_cmd += 2;
}

static void
cmdq_core_insert_backup_instruction(struct cmdq_task *task,
				    const u32 reg_addr,
				    const dma_addr_t write_addr,
				    const enum cmdq_data_register value_reg_id,
				    const enum cmdq_data_register dest_reg_id)
{
	u32 arg_a;
	int subsys_code;

	/*
	 * register to read from
	 * note that we force convert to physical reg address.
	 * if it is already physical address,
	 * it won't be affected (at least on this platform)
	 */
	arg_a = reg_addr;
	/* arg_a = IO_VIRT_TO_PHYS(reg_addr); */
	subsys_code = cmdq_subsys_from_phys_addr(task->cqctx, arg_a);

	/* load into 32-bit GPR (R0-R15) */
	cmdq_core_append_command_into_task(task,
					   (CMDQ_CODE_READ << 24) |
					   (arg_a & 0xffff) |
					   ((subsys_code & 0x1f) << 16) |
					   (2 << 21),	/* 0 1 0 */
					   value_reg_id);

	/*
	 * note that <MOVE> arg_b is 48-bit
	 * so write_addr is split into 2 parts
	 * and we store address in 64-bit GPR (P0-P7)
	 */
	cmdq_core_append_command_into_task(task,
					   (CMDQ_CODE_MOVE << 24) |
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
					   ((write_addr >> 32) & 0xffff) |
#endif
					   ((dest_reg_id & 0x1f) << 16) |
					   (4 << 21),
					   (u32)write_addr);

	/* write to memory */
	cmdq_core_append_command_into_task(task,
					   (CMDQ_CODE_WRITE << 24) |
					   (0 & 0xffff) |
					   ((dest_reg_id & 0x1f) << 16) |
					   (6 << 21),	/* 1 0 */
					   value_reg_id);

	CMDQ_MSG("COMMAND: copy reg:0x%08x to phys:0x%p, GPR(%d, %d)\n",
		 arg_a, &write_addr, value_reg_id, dest_reg_id);
}

static int cmdq_core_sync_command(struct cmdq_task *task,
				  struct cmdq_command *cmd_desc)
{
#define CMDQ_PROFILE_COMMAND

	int status = 0;
	u32 extra_buf_size = 0;
	u32 prepend_buf_size = 0;
	struct cmdq_context *cqctx = task->cqctx;
	struct device *dev = cqctx->dev;
	u32 size;
	void *src, *dst;
#ifdef CMDQ_PROFILE_COMMAND
	bool profile_cmd = false;
#endif
	u32 copied_before_profile = 0;

	size = task->command_size + CMDQ_INST_SIZE;
	CMDQ_MSG("%s:%d original command size = %d\n",
		 __func__,  __LINE__, task->command_size);

#ifdef CMDQ_PROFILE_COMMAND
	profile_cmd = task && cmdq_core_should_profile(task->scenario) &&
		      (task->profile_data != NULL);

	if (profile_cmd) {
		/* backup GPT at begin and end */
		extra_buf_size += (CMDQ_INST_SIZE * 3);

		/*
		 * insert after the first MARKER instruction
		 * and first SYNC instruction (if any)
		 */
		prepend_buf_size += 2 * (CMDQ_INST_SIZE * 3);
		if (task->command_size < prepend_buf_size)
			prepend_buf_size = 0;
	}
#endif

	size = task->command_size + prepend_buf_size +
	       extra_buf_size + CMDQ_INST_SIZE;
	status = cmdq_core_task_realloc_buffer_size(task, size);
	if (status < 0) {
		dev_err(dev, "failed to realloc command buffer\n");
		dev_err(dev, "task=0x%p, request size=%d\n", task, size);
		return status;
	}

	task->num_cmd = 0;
#ifdef CMDQ_PROFILE_COMMAND
	if (profile_cmd) {
		if (cmdq_core_should_enable_prefetch(task->scenario)) {
			/* MARKER + WAIT_FOR_EOF */
			copied_before_profile = 2 * CMDQ_INST_SIZE;
		} else {
			/*
			 * we copy the 1st "WAIT FOR EOF" instruction,
			 * because this is the point
			 * where we start writing registers!
			 */
			copied_before_profile = 1 * CMDQ_INST_SIZE;
		}

		memcpy(task->va_base + task->num_cmd,
		       cmd_desc->va_base,
		       copied_before_profile);
		task->num_cmd =
			copied_before_profile / sizeof(task->va_base[0]);

		/* now we start insert backup instructions */
		CMDQ_MSG("%s:%d\n", __func__, __LINE__);
		CMDQ_MSG("[BACKUP]va=0x%p, pa=0x%p, task=0x%p\n",
			 task->profile_data, &task->profile_data_pa, task);

		cmdq_core_insert_backup_instruction
			(task, CMDQ_APXGPT2_COUNT, task->profile_data_pa,
			 CMDQ_DATA_REG_R0, CMDQ_DATA_REG_P1);
		/* this increases task->command_size */
	}
#endif

	/* copy the commands to our DMA buffer */
	size = cmd_desc->block_size - copied_before_profile;
	src = cmd_desc->va_base +
	      (copied_before_profile / sizeof(cmd_desc->va_base[0]));
	dst = task->va_base + task->num_cmd;
	CMDQ_MSG("COMMAND: Copy kernel to 0x%p\n", task->va_base);
	memcpy(dst, src, size);

	/* re-adjust num_cmd according to command_size */
	task->num_cmd = task->command_size / sizeof(task->va_base[0]);
	CMDQ_MSG("%s:%d cmd num=%d, base=0x%p, size=%d\n",
		 __func__,  __LINE__,
		 task->num_cmd, task->va_base, task->command_size);

	return status;
}

static struct cmdq_task *cmdq_core_acquire_task(struct cmdq_command *cmd_desc,
						struct cmdq_task_cb *cb)
{
	int status;
	struct cmdq_task *task;
	struct cmdq_context *cqctx = cmd_desc->cqctx;
	struct device *dev = cqctx->dev;

	CMDQ_MSG("-->TASK: acquire task begin CMD: 0x%p, size: %d, Eng: 0x%016llx\n",
		 cmd_desc->va_base, cmd_desc->block_size,
		 cmd_desc->engine_flag);

	task = cmdq_core_find_free_task(cqctx);
	if (!task) {
		dev_err(dev, "can't acquire task info\n");
		return NULL;
	}

	/* initialize field values */
	task->scenario = cmd_desc->scenario;
	task->priority = cmd_desc->priority;
	task->engine_flag = cmd_desc->engine_flag;
	task->task_state = TASK_STATE_WAITING;
	task->reorder = 0;
	task->thread = CMDQ_INVALID_THREAD;
	task->irq_flag = 0x0;
	memcpy(&task->cb, cb, sizeof(*cb));

	/* profile data */
	if (cmdq_core_should_profile(task->scenario)) {
		task->profile_data =
			 cmdq_core_alloc_hw_buffer
				(dev, 2 * sizeof(u32),
				 &task->profile_data_pa, GFP_KERNEL);
	} else {
		task->profile_data = NULL;
	}

	/* profile timers */
	memset(&task->trigger, 0, sizeof(task->trigger));
	memset(&task->begin_wait, 0, sizeof(task->begin_wait));
	memset(&task->got_irq, 0, sizeof(task->got_irq));
#ifdef CMDQ_CORE_PROFILE_WAKE_UP
	memset(&task->send_wake_up, 0, sizeof(task->send_wake_up));
	memset(&task->receive_wake_up, 0, sizeof(task->receive_wake_up));
#endif
	memset(&task->waked_up, 0, sizeof(task->waked_up));

	task->command_size = cmd_desc->block_size;

	/* store caller info for debug */
	if (current) {
		task->caller_pid = current->pid;
		memcpy(task->caller_name, current->comm, sizeof(current->comm));
	}

	status = cmdq_core_sync_command(task, cmd_desc);
	if (status < 0) {
		dev_err(dev, "fail to sync command\n");
		cmdq_core_release_task_internal(task);
		return NULL;
	}

	/* insert into waiting list to process */
	mutex_lock(&cqctx->task_mutex);
	if (task) {
		struct list_head *in_item = &cqctx->task_wait_list;
		struct cmdq_task *cq_tsk = NULL;
		struct list_head *p = NULL;

		task->submit = sched_clock();

		/*
		 * add to waiting list, keep it sorted by priority
		 * so that we add high-priority tasks first.
		 */
		list_for_each(p, &cqctx->task_wait_list) {
			cq_tsk = list_entry(p, struct cmdq_task, list_entry);
			/*
			 * keep the list sorted.
			 * higher priority tasks are inserted
			 * in front of the queue
			 */
			if (cq_tsk->priority < task->priority)
				break;

			in_item = p;
		}

		list_add(&task->list_entry, in_item);
	}
	mutex_unlock(&cqctx->task_mutex);

	CMDQ_MSG("<--TASK: acquire task 0x%p end\n", task);
	return task;
}

static int cmdq_core_get_clk(struct cmdq_context *cqctx)
{
	struct device *dev = cqctx->dev;

	cqctx->clock = devm_clk_get(dev, CMDQ_CLK_NAME);
	if (IS_ERR(cqctx->clock)) {
		dev_err(dev, "get clk:%s fail\n", CMDQ_CLK_NAME);
		return PTR_ERR(cqctx->clock);
	}

	return 0;
}

static int cmdq_core_enable_clock(struct cmdq_context *cqctx)
{
	int ret = 0;
	struct device *dev = cqctx->dev;

	CMDQ_MSG("enable CMDQ clk, usage=%d\n", cqctx->thread_usage);

	if (!cqctx->thread_usage) {
		ret = clk_prepare_enable(cqctx->clock);
		if (ret) {
			dev_err(dev, "prepare and enable clk:%s fail\n",
				CMDQ_CLK_NAME);
			return ret;
		}
		cmdq_event_reset(cqctx);
	}
	cqctx->thread_usage++;

	return ret;
}

static void cmdq_core_disable_clock(struct cmdq_context *cqctx)
{
	CMDQ_MSG("disable CMDQ clk, usage=%d\n", cqctx->thread_usage);
	cqctx->thread_usage--;
	if (cqctx->thread_usage <= 0)
		clk_disable_unprepare(cqctx->clock);
}

static int cmdq_core_find_a_free_hw_thread(struct cmdq_context *cqctx,
					   u32 scenario)
{
	struct cmdq_thread *cqtd;
	int tid;
	u32 next_cookie;
	struct device *dev = cqctx->dev;

	cqtd = cqctx->thread;
	tid = cmdq_core_get_thread_index_from_scenario(scenario);
	CMDQ_MSG("THREAD: find a free thread, scenario=%d, thread=%d\n",
		 scenario, tid);

	/*
	 * Currently, we only support disp,
	 * so it's error if tid is CMDQ_INVALID_THREAD.
	 */
	if (tid == CMDQ_INVALID_THREAD) {
		dev_err(dev, "got CMDQ_INVALID_THREAD!!!\n");
		return tid;
	}

	/*
	 * make sure the found thread has enough space for the task;
	 * cmdq_thread->cur_task has size limitation.
	 */
	if (cqtd[tid].task_count >= CMDQ_MAX_TASK_IN_THREAD) {
		CMDQ_MSG("THREAD: thread %d task count=%d full\n",
			 tid, cqtd[tid].task_count);
		tid = CMDQ_INVALID_THREAD;
	}

	next_cookie = cqtd[tid].next_cookie % CMDQ_MAX_TASK_IN_THREAD;
	if (cqtd[tid].cur_task[next_cookie]) {
		CMDQ_MSG("THREAD: thread %d next_cookie=%d already has task\n",
			 tid, cqtd[tid].next_cookie);
		tid = CMDQ_INVALID_THREAD;
	}

	return tid;
}

static int cmdq_core_acquire_thread(struct cmdq_context *cqctx, u32 scenario)
{
	int tid;

	mutex_lock(&cqctx->clock_mutex);
	tid = cmdq_core_find_a_free_hw_thread(cqctx, scenario);
	if (tid != CMDQ_INVALID_THREAD) {
		CMDQ_MSG("-->CLOCK: enable, thread=%d, begin\n", tid);
		cmdq_core_enable_clock(cqctx);
		CMDQ_MSG("<--CLOCK: enable, thread=%d, end\n", tid);
	}
	mutex_unlock(&cqctx->clock_mutex);

	return tid;
}

static void cmdq_core_add_consume_task(struct cmdq_context *cqctx)
{
	if (!work_pending(&cqctx->task_consume_wait_queue_item))
		queue_work(cqctx->task_consume_wq,
			   &cqctx->task_consume_wait_queue_item);
}

static void cmdq_core_release_thread(struct cmdq_task *task)
{
	int thread = task->thread;
	struct cmdq_context *cqctx = task->cqctx;

	if (unlikely(thread == CMDQ_INVALID_THREAD)) {
		WARN_ON(1);
		return;
	}

	mutex_lock(&cqctx->clock_mutex);
	CMDQ_MSG("-->CLOCK: disable, thread=%d, begin\n", task->thread);
	cmdq_core_disable_clock(cqctx);
	CMDQ_MSG("<--CLOCK: disable, thread=%d, end\n", task->thread);
	task->thread = CMDQ_INVALID_THREAD;
	mutex_unlock(&cqctx->clock_mutex);
}

static int cmdq_core_suspend_hw_thread(struct cmdq_context *cqctx, int thread)
{
	int i;
	u32 enabled;
	void __iomem *gce_base_va = cqctx->cqdev.base_va;
	struct device *dev = cqctx->dev;
	u32 t_status;

	if (thread == CMDQ_INVALID_THREAD) {
		dev_err(dev, "suspend invalid thread\n");
		return -EFAULT;
	}

	/* write suspend bit */
	writel(0x01,
	       gce_base_va + CMDQ_THR_SUSPEND_TASK_OFFSET +
	       CMDQ_THR_SHIFT * thread);

	/* If already disabled, treat as suspend successful. */
	enabled = readl(gce_base_va + CMDQ_THR_ENABLE_TASK_OFFSET +
			CMDQ_THR_SHIFT * thread);
	if (!(enabled & 0x01)) {
		CMDQ_LOG("[WARNING] thread %d suspend not effective, enable=%d\n",
			 thread, enabled);
		return 0;
	}

	t_status = 0;
	for (i = 0; i < CMDQ_MAX_LOOP_COUNT; i++) {
		t_status = readl(gce_base_va + CMDQ_THR_CURR_STATUS_OFFSET +
				 CMDQ_THR_SHIFT * thread);
		t_status &= 0x2;
		if (t_status)
			break;
	}
	if (!t_status) {
		dev_err(dev, "Suspend HW thread %d failed\n", thread);
		return -EFAULT;
	}

	return 0;
}

static void cmdq_core_resume_hw_thread(void __iomem *gce_base_va,
				       int thread)
{
	writel(0x00,
	       gce_base_va + CMDQ_THR_SUSPEND_TASK_OFFSET +
	       CMDQ_THR_SHIFT * thread);
}

static int cmdq_core_reset_hw_thread(struct cmdq_context *cqctx, int thread)
{
	int i;
	void __iomem *gce_base_va = cqctx->cqdev.base_va;
	struct device *dev = cqctx->dev;
	u32 wr; /* warm reset */

	CMDQ_MSG("reset HW thread(%d)\n", thread);

	writel(0x01,
	       gce_base_va + CMDQ_THR_WARM_RESET_OFFSET +
	       CMDQ_THR_SHIFT * thread);
	for (i = 0; i <= CMDQ_MAX_LOOP_COUNT; i++) {
		wr = readl(gce_base_va + CMDQ_THR_WARM_RESET_OFFSET +
			   CMDQ_THR_SHIFT * thread);
		if (wr != 0x1)
			break;
	}

	if (i > CMDQ_MAX_LOOP_COUNT) {
		dev_err(dev, "Reset HW thread %d failed\n", thread);
		return -EFAULT;
	}

	writel(0x3200, gce_base_va + CMDQ_THR_SLOT_CYCLES_OFFSET);
	return 0;
}

static int cmdq_core_disable_hw_thread(struct cmdq_context *cqctx,
				       int thread)
{
	void __iomem *gce_base_va = cqctx->cqdev.base_va;

	CMDQ_MSG("disable HW thread(%d)\n", thread);
	cmdq_core_reset_hw_thread(cqctx, thread);
	writel(0x00,
	       gce_base_va + CMDQ_THR_ENABLE_TASK_OFFSET +
	       CMDQ_THR_SHIFT * thread);
	return 0;
}

u32 *cmdq_core_get_pc(const struct cmdq_task *task, u32 thread, u32 insts[2])
{
	unsigned long pc_pa;
	u8 *pc_va;
	struct cmdq_context *cqctx;
	void __iomem *gce_base_va;
	u8 *cmd_end;

	memset(insts, 0, sizeof(u32) * 2);

	if ((!task) ||
	    (!task->va_base) ||
	    (thread == CMDQ_INVALID_THREAD)) {
		pr_err("cmdq get pc failed since invalid param, task 0x%p, task->va_base:0x%p, thread:%d\n",
		       task, task->va_base, thread);
		return NULL;
	}

	cqctx = task->cqctx;
	gce_base_va = cqctx->cqdev.base_va;

	pc_pa = (unsigned long)readl(gce_base_va + CMDQ_THR_CURR_ADDR_OFFSET +
				     CMDQ_THR_SHIFT * thread);
	pc_va = (u8 *)task->va_base + (pc_pa - task->mva_base);
	cmd_end = (u8 *)(task->va_base + task->num_cmd - 1);

	if (((u8 *)task->va_base <= pc_va) && (pc_va <= cmd_end)) {
		if (pc_va < cmd_end) {
			/*
			 * If PC points to start of CMD,
			 * - 8 causes access violation
			 */
			insts[0] = readl(pc_va);
			insts[1] = readl(pc_va + 4);
		} else {
			insts[0] = readl(pc_va - 8);
			insts[1] = readl(pc_va - 4);
		}
	} else {
		/* invalid PC address */
		return NULL;
	}

	return (u32 *)pc_va;
}

u32 cmdq_core_subsys_to_reg_addr(u32 arg_a)
{
	u32 bit = CMDQ_SUBSYS_LSB;
	int id = (arg_a & CMDQ_ARG_A_SUBSYS_MASK) >> bit;
	u32 offset = (arg_a & 0xffff);
	u32 ba = cmdq_subsys_id_to_base_addr(id);

	return (ba << bit) | offset;
}

const char *cmdq_core_parse_op(u32 op_code)
{
	switch (op_code) {
	case CMDQ_CODE_WRITE:
		return "WRIT";
	case CMDQ_CODE_WFE:
		return "SYNC";
	case CMDQ_CODE_READ:
		return "READ";
	case CMDQ_CODE_MOVE:
		return "MASK";
	case CMDQ_CODE_JUMP:
		return "JUMP";
	case CMDQ_CODE_EOC:
		return "MARK";
	}
	return NULL;
}

static bool cmdq_core_verfiy_command_end(struct device *dev,
					 u32 *va_base,
					 u32 cmd_size,
					 u32 num_cmd,
					 const char *tag)
{
	/* make sure we have sufficient command to parse */
	if (!va_base || cmd_size < (2 * CMDQ_INST_SIZE)) {
		dev_err(dev, "[%s][CMD] doesn't have sufficient commands\n",
			tag);
		return false;
	}

	/* make sure EOC enable IRQ */
	if ((va_base[num_cmd-4] & 0x1) != 1) {
		dev_err(dev, "[%s][CMD] doesn't enable IRQ (%08x:%08x)\n",
			tag, va_base[num_cmd-4], va_base[num_cmd-3]);
		return false;
	}

	/* make sure the command is ended by EOC + JUMP */
	if ((va_base[num_cmd-3] >> 24) != CMDQ_CODE_EOC ||
	    (va_base[num_cmd-1] >> 24) != CMDQ_CODE_JUMP) {
		dev_err(dev,
			"[%s][CMD] doesn't end in EOC+JUMP (%08x:%08x, %08x:%08x)\n",
			tag, va_base[num_cmd-4], va_base[num_cmd-3],
			va_base[num_cmd-2], va_base[num_cmd-1]);
		return false;
	}

	return true;
}

static bool cmdq_core_verfiy_desc_command_end(struct cmdq_command *cmd_desc)
{
	struct device *dev;
	u32 num_cmd;
	bool valid;

	dev = cmd_desc->cqctx->dev;
	num_cmd = cmd_desc->block_size / sizeof(u32);
	valid = cmdq_core_verfiy_command_end(
		dev, cmd_desc->va_base, cmd_desc->block_size, num_cmd, "DESC");

	if (!valid)
		dev_err(dev, "invalid command desc 0x%p\n", cmd_desc);

	return valid;
}

static bool cmdq_core_verfiy_task_command_end(const struct cmdq_task *task)
{
	struct device *dev;
	bool valid;

	dev = task->cqctx->dev;
	valid = cmdq_core_verfiy_command_end(
		dev, task->va_base, task->command_size, task->num_cmd, "TASK");

	if (!valid)
		dev_err(dev, "invalid task 0x%p\n", task);

	return valid;
}

static void cmdq_core_fill_task_record(struct cmdq_record *cqr,
				       const struct cmdq_task *task,
				       u32 tid)
{
	u32 begin, end;

	if (cqr && task) {
		/* record scenario */
		cqr->user = task->caller_pid;
		cqr->scenario = task->scenario;
		cqr->priority = task->priority;
		cqr->thread = tid;
		cqr->reorder = task->reorder;
		cqr->engine_flag = task->engine_flag;

		if (!task->profile_data) {
			cqr->write_time_ns = 0;
		} else {
			/* command exec time, each count is 76ns */
			u32 *addr = (u32 *)task->profile_data;

			begin = readl(addr);
			end = readl(addr + 1);
			cqr->write_time_ns = (end - begin) * 76;
			cqr->write_time_ns_begin = (begin) * 76;
			cqr->write_time_ns_end = (end) * 76;
		}

		/* record time */
		cqr->submit = task->submit;
		cqr->trigger = task->trigger;
		cqr->got_irq = task->got_irq;
		cqr->begin_wait = task->begin_wait;
		cqr->waked_up = task->waked_up;
#ifdef CMDQ_CORE_PROFILE_WAKE_UP
		cqr->send_wake_up = task->send_wake_up;
		cqr->receive_wake_up = task->receive_wake_up;
#endif
	}
}

static void cmdq_core_track_task_record(struct cmdq_task *task, u32 thread)
{
	struct cmdq_record *cqr;
	unsigned long flags;
	unsigned long long done;
	struct cmdq_context *cqctx = task->cqctx;

	done = sched_clock();

	spin_lock_irqsave(&cqctx->record_lock, flags);

	cqr = &cqctx->record[cqctx->last_id];
	cmdq_core_fill_task_record(cqr, task, thread);
	cqr->done = done;

	/* only record unacceptable running time */
	if (cqr->done - cqr->submit < CMDQ_CORE_RECORD_MIN_TIME) {
		spin_unlock_irqrestore(&cqctx->record_lock, flags);
		return;
	}

	cqctx->last_id++;
	if (cqctx->last_id >= CMDQ_MAX_RECORD_COUNT)
		cqctx->last_id = 0;

	cqctx->rec_num++;
	if (cqctx->rec_num >= CMDQ_MAX_RECORD_COUNT)
		cqctx->rec_num = CMDQ_MAX_RECORD_COUNT;

	spin_unlock_irqrestore(&cqctx->record_lock, flags);
}

static void
cmdq_core_copy_error_task_instruction(struct cmdq_error_task_item *err_tsk,
				      const struct cmdq_task *task)
{
	int i;

	err_tsk->task = task;
	err_tsk->command_size = task->command_size;
	err_tsk->trigger = task->trigger;
	if (err_tsk->va_base)
		return;

	err_tsk->va_base = kmalloc(task->command_size, GFP_KERNEL);
	if (!err_tsk->va_base)
		return;

	memset(err_tsk->va_base, 0, task->command_size);

	for (i = 0; i < (task->command_size) / 4; i++)
		err_tsk->va_base[i] = task->va_base[i];
}

static void cmdq_core_attach_error_task(const struct cmdq_task *task, int tid)
{
	struct cmdq_engine *cq_e;
	int i;
	struct cmdq_context *cqctx = task->cqctx;
	struct device *dev = cqctx->dev;
	u64 eng_flag;

	cq_e = cqctx->engine;

	/* update engine fail count */
	eng_flag = task->engine_flag;
	for (i = 0; i < CMDQ_MAX_ENGINE_COUNT; i++) {
		if (eng_flag & (1LL << i))
			cq_e[i].fail_count++;
	}

	/* register error record */
	if (cqctx->err_num < CMDQ_MAX_ERROR_COUNT) {
		struct cmdq_error *err =
			 &cqctx->error[cqctx->err_num];

		cmdq_core_fill_task_record(&err->error_rec, task, tid);
		err->ts_nsec = local_clock();
	}

	dev_err(dev, "task error - error:%d task:0x%p\n", cqctx->err_num, task);
	cmdq_core_dump_error_task(task, tid);

	cqctx->err_num++;
}

static void cmdq_core_parse_error(struct cmdq_task *task, u32 thread,
				  const char **module_name, int *flag,
				  u32 *inst_a, u32 *inst_b)
{
	u32 insts[2] = { 0 };
	const char *module;
	int irq_flag = task->irq_flag;

	/*
	 * other cases, use instruction to judge
	 * because scenario / HW flag are not sufficient
	 */
	if (cmdq_core_get_pc(task, thread, insts)) {
		u32 op, arg_a, arg_b;
		u32 addr;

		op = (insts[1] & 0xff000000) >> 24;
		arg_a = insts[1] & (~0xff000000);
		arg_b = insts[0];

		switch (op) {
		case CMDQ_CODE_WRITE:
			addr = cmdq_core_subsys_to_reg_addr(arg_a);
			module = cmdq_core_parse_module_from_reg_addr(addr);
			break;
		case CMDQ_CODE_WFE:
			/* arg_a is the event id */
			module = cmdq_core_module_from_event_id(
				(enum cmdq_event)arg_a);
			break;
		case CMDQ_CODE_READ:
		case CMDQ_CODE_MOVE:
		case CMDQ_CODE_JUMP:
		case CMDQ_CODE_EOC:
		default:
			module = "CMDQ";
			break;
		}
	} else {
		module = "CMDQ";
	}

	/* fill output parameter */
	*module_name = module;
	*flag = irq_flag;
	*inst_a = insts[1];
	*inst_b = insts[0];
}

static int
cmdq_core_insert_task_from_thread_array_by_cookie(struct cmdq_task *task,
						  struct cmdq_thread *cqtd,
						  const int cookie,
						  const bool reset_hw_thread)
{
	if (!task || !cqtd) {
		pr_err("invalid param, task[0x%p], thread[0x%p], cookie[%d], reset[%d]\n",
		       task, cqtd, cookie, reset_hw_thread);
		return -EFAULT;
	}

	if (reset_hw_thread) {
		cqtd->wait_cookie = cookie;
		cqtd->next_cookie = cookie + 1;
		if (cqtd->next_cookie > CMDQ_MAX_COOKIE_VALUE)
			cqtd->next_cookie = 0;

		/*
		 * task_count must start from 0.
		 * if we are the first task, set to 1.
		 */
		cqtd->task_count = 1;
	} else {
		cqtd->next_cookie += 1;
		if (cqtd->next_cookie > CMDQ_MAX_COOKIE_VALUE)
			cqtd->next_cookie = 0;

		cqtd->task_count++;
	}

	/* genernal part */
	cqtd->cur_task[cookie % CMDQ_MAX_TASK_IN_THREAD] = task;

	return 0;
}

static int cmdq_core_remove_task_from_thread_array_by_cookie(
	struct cmdq_thread *cqtd, int index, enum cmdq_task_state new_state)
{
	struct cmdq_task *task;
	int tid;
	struct device *dev;

	if ((!cqtd) || (index >= CMDQ_MAX_TASK_IN_THREAD) ||
	    (index < 0)) {
		pr_err("remove task from thread array, invalid param. THR[0x%p], task_slot[%d], new_state[%d]\n",
		       cqtd, index, new_state);
		return -EINVAL;
	}

	task = cqtd->cur_task[index];
	if (!task) {
		pr_err("remove fail, task_slot[%d] on thread[0x%p] is NULL\n",
		       index, cqtd);
		return -EINVAL;
	}
	dev = task->cqctx->dev;

	/*
	 * note timing to switch a task to done_status(_ERROR, _KILLED, _DONE)
	 * is aligned with thread's taskcount change
	 * check task status to prevent double clean-up thread's taskcount
	 */
	if (task->task_state != TASK_STATE_BUSY) {
		dev_err(dev,
			"remove task, task_status err[%d]. THR[0x%p], task_slot[%d], target task state[%d]\n",
			task->task_state, cqtd, index, new_state);
		return -EINVAL;
	}

	CMDQ_MSG("remove task, slot[%d], target state: %d\n",
		 index, new_state);
	tid = task->thread;
	task->task_state = new_state;
	cqtd->cur_task[index] = NULL;
	cqtd->task_count--;

	if ((int)cqtd->task_count < 0) {
		dev_err(dev,
			"task count < 0 after cmdq_core_remove_task_from_thread_array_by_cookie\n");
		dev_err(dev, "thread[%d], index=%d\n", tid, index);
		/* for error handle */
		cqtd->task_count = 0;
	}

	return 0;
}

static int cmdq_core_force_remove_task_from_thread(struct cmdq_task *task,
						   int tid)
{
	int status;
	int cookie;
	struct cmdq_task *exec_tsk;
	struct cmdq_context *cqctx = task->cqctx;
	struct cmdq_thread *cqtd = &cqctx->thread[tid];
	void __iomem *gce_base_va = cqctx->cqdev.base_va;

	status = cmdq_core_suspend_hw_thread(cqctx, tid);

	writel(0,
	       gce_base_va + CMDQ_THR_INST_CYCLES_OFFSET +
	       CMDQ_THR_SHIFT * tid);

	/* The cookie of the task currently being processed */
	cookie = cmdq_get_cookie_cnt(gce_base_va, tid) + 1;

	exec_tsk = cqtd->cur_task[cookie % CMDQ_MAX_TASK_IN_THREAD];
	if (exec_tsk != NULL && exec_tsk == task) {
		dma_addr_t eoc_pa = task->mva_base + task->command_size - 16;

		/* The task is executed now, set the PC to EOC for bypass */
		writel(eoc_pa,
		       gce_base_va + CMDQ_THR_CURR_ADDR_OFFSET +
		       CMDQ_THR_SHIFT * tid);

		cqtd->cur_task[cookie % CMDQ_MAX_TASK_IN_THREAD] = NULL;
		task->task_state = TASK_STATE_KILLED;
	} else {
		int i, j;

		j = cqtd->task_count;
		for (i = (cookie % CMDQ_MAX_TASK_IN_THREAD); j > 0; j--, i++) {
			i %= CMDQ_MAX_TASK_IN_THREAD;

			exec_tsk = cqtd->cur_task[i];
			if (!exec_tsk)
				continue;

			if ((exec_tsk->va_base[exec_tsk->num_cmd-1] ==
			     0x10000000) &&
			    (exec_tsk->va_base[exec_tsk->num_cmd-2] ==
			     0x00000008)) {
				/* reached the last task */
				break;
			} else if (exec_tsk->va_base[exec_tsk->num_cmd-2] ==
				   task->mva_base) {
				/* fake EOC command */
				exec_tsk->va_base[exec_tsk->num_cmd-2] =
					0x00000001;
				exec_tsk->va_base[exec_tsk->num_cmd-1] =
					0x40000000;

				/* bypass the task */
				exec_tsk->va_base[exec_tsk->num_cmd] =
					task->va_base[task->num_cmd-2];
				exec_tsk->va_base[exec_tsk->num_cmd+1] =
					task->va_base[task->num_cmd-1];

				i++;
				i %= CMDQ_MAX_TASK_IN_THREAD;

				cqtd->cur_task[i] = NULL;
				task->task_state = TASK_STATE_KILLED;
				status = 0;
				break;
			}
		}
	}

	return status;
}

static struct cmdq_task *cmdq_core_search_task_by_pc(u32 thread_pc,
						     const struct cmdq_thread *cqtd)
{
	struct cmdq_task *task;
	int i;

	for (i = 0; i < CMDQ_MAX_TASK_IN_THREAD; i++) {
		task = cqtd->cur_task[i];
		if (task &&
		    thread_pc >= task->mva_base &&
		    thread_pc <= (task->mva_base + task->command_size)) {
			break;
		}
	}

	return task;
}

static int cmdq_core_wait_task_done_with_timeout_impl(struct cmdq_task *task,
						      int tid)
{
	int wait_q;
	unsigned long flags;
	int retry;
	struct cmdq_error_task_item *err_tsk_item;
	const unsigned long out = msecs_to_jiffies(CMDQ_PREDUMP_TIMEOUT_MS);
	struct cmdq_context *cqctx = task->cqctx;
	void __iomem *gce_base_va = cqctx->cqdev.base_va;

	/*
	 * timeout wait & make sure this task is finished.
	 * task->task_state flag is updated in IRQ handlers
	 * (cmdq_core_handle_done)
	 */
	retry = 0;
	wait_q = wait_event_timeout(cqctx->wait_queue[tid],
				    (task->task_state != TASK_STATE_BUSY &&
				     task->task_state != TASK_STATE_WAITING),
				    out);

	/* if SW-timeout, pre-dump hang instructions */
	while (!wait_q && retry < CMDQ_PREDUMP_RETRY_COUNT) {
		CMDQ_LOG("==========[CMDQ] timeout predump(%d) task[0x%p], trigger[%lld]==========\n",
			 retry, task, task->trigger);

		if (!retry) {
			struct cmdq_error_task *err_tsk = &cqctx->error_task;

			/* dump task instructions */
			CMDQ_LOG("------------task[0x%p] trigger[%lld] dump begin------------\n",
				 task, task->trigger);
			cmdq_core_dump_instructions((u64 *)(task->va_base),
						    task->command_size,
						    LOG_LEVEL_LOG);
			CMDQ_LOG("------------task[0x%p] trigger[%lld] dump finish------------\n",
				 task, task->trigger);
			if (err_tsk->err_task_count < CMDQ_MAX_ERROR_COUNT) {
				/* copy error task to error task buffer */
				err_tsk_item = &err_tsk->error_task_buffer
						[cqctx->err_num];
				cmdq_core_copy_error_task_instruction
					(err_tsk_item, task);
				err_tsk->err_task_count++;
			}
		}

		++retry;

		spin_lock_irqsave(&cqctx->exec_lock, flags);
		cmdq_core_dump_status(gce_base_va, "INFO");
		cmdq_core_dump_pc(task, tid, "INFO");
		spin_unlock_irqrestore(&cqctx->exec_lock, flags);

		/* wait again */
		wait_q = wait_event_timeout
				(cqctx->wait_queue[tid],
				 (task->task_state != TASK_STATE_BUSY &&
				  task->task_state != TASK_STATE_WAITING),
				 out);
	}

#ifdef CMDQ_CORE_PROFILE_WAKE_UP
	task->receive_wake_up = sched_clock();
#endif

	return wait_q;
}

static int cmdq_core_handle_wait_task_result_impl(struct cmdq_task *task,
						  int tid, int wait_q)
{
	int status = 0;
	int i;
	unsigned long flags;
	bool is_err = false;
	bool throw_err = false;
	const char *module = NULL;
	u32 inst_a = 0, inst_b = 0;
	u32 irq_flag;
	struct cmdq_context *cqctx = task->cqctx;
	void __iomem *gce_base_va = cqctx->cqdev.base_va;
	struct device *dev = cqctx->dev;
	struct cmdq_thread *cqtd = &cqctx->thread[tid];

	/*
	 * Note that although we disable IRQ, HW continues to execute
	 * so it's possible to have pending IRQ
	 */
	spin_lock_irqsave(&cqctx->exec_lock, flags);

	do {
		struct cmdq_task *next_tsk;
		struct cmdq_task *prev_tsk;
		int cookie;
		unsigned long thread_pc;

		if (task->task_state == TASK_STATE_DONE)
			break;

		dev_err(dev,
			"Task state of 0x%p is not TASK_STATE_DONE, %d\n",
			task, task->task_state);

		/*
		 * Oops, tha tasks is not done.
		 * We have several possible error scenario:
		 * 1. task still running (hang / timeout)
		 * 2. IRQ pending (done or error/timeout IRQ)
		 * 3. task's SW thread has been signaled (e.g. SIGKILL)
		 */

		/*
		 * suspend HW thread first,
		 * so that we work in a consistent state
		 */
		status = cmdq_core_suspend_hw_thread(cqctx, tid);
		if (status < 0)
			throw_err = true;

		/* The cookie of the task currently being processed */
		cookie = cmdq_get_cookie_cnt(gce_base_va, tid) + 1;
		thread_pc = (unsigned long)readl(gce_base_va + CMDQ_THR_CURR_ADDR_OFFSET +
						 CMDQ_THR_SHIFT * tid);

		/* process any pending IRQ */
		irq_flag = readl(gce_base_va + CMDQ_THR_IRQ_STATUS_OFFSET +
				 CMDQ_THR_SHIFT * tid);
		if (irq_flag & 0x12)
			cmdq_core_handle_error(
				cqctx, tid, irq_flag, &task->waked_up);
		else if (irq_flag & 0x01)
			cmdq_core_handle_done(
				cqctx, tid, irq_flag, &task->waked_up);

		writel(~irq_flag,
		       gce_base_va + CMDQ_THR_IRQ_STATUS_OFFSET +
		       CMDQ_THR_SHIFT * tid);

		/* check if this task has finished after handling pending IRQ */
		if (task->task_state == TASK_STATE_DONE)
			break;

		/* Then decide we are SW timeout or SIGNALed (not an error) */
		if (!wait_q) {
			/* SW timeout and no IRQ received */
			is_err = true;

			/*
			 * if we reach here, we're in errornous state.
			 * print error log immediately.
			 */
			cmdq_core_attach_error_task(task, tid);

			dev_err(dev, "SW timeout of task 0x%p on tid %d\n",
				task, tid);
			throw_err = true;
			cmdq_core_parse_error(task, tid, &module, &irq_flag,
					      &inst_a, &inst_b);
			status = -ETIMEDOUT;
		} else if (wait_q < 0) {
			/*
			 * Task is killed.
			 * Not an error, but still need to remove.
			 */
			is_err = false;

			if (wait_q == -ERESTARTSYS)
				dev_err(dev,
					"Task 0x%p KILLED by wait_q = -ERESTARTSYS\n",
					task);
			else if (wait_q == -EINTR)
				dev_err(dev,
					"Task 0x%p KILLED by wait_q = -EINTR\n",
					task);
			else
				dev_err(dev,
					"Task 0x%p KILLED by wait_q = %d\n",
					task, wait_q);

			status = wait_q;
		}

		if (task->task_state == TASK_STATE_ERROR) {
			/* do nothing */
		} else if (task->task_state == TASK_STATE_BUSY) {
			/*
			 * if task_state is BUSY,
			 * this means we did not reach EOC,
			 * did not have error IRQ.
			 * - remove the task from thread.cur_task[]
			 * - and decrease thread.task_count
			 * NOTE: after this,
			 * the cur_task will not contain link to task anymore.
			 * and task should become TASK_STATE_ERROR
			 */

			/* we find our place in cqtd->cur_task[]. */
			for (i = 0; i < CMDQ_MAX_TASK_IN_THREAD; i++) {
				if (cqtd->cur_task[i] == task) {
					/* update task_count and cur_task[] */
					cmdq_core_remove_task_from_thread_array_by_cookie
						(cqtd, i, is_err ?
						 TASK_STATE_ERROR :
						 TASK_STATE_KILLED);
					break;
				}
			}
		}

		next_tsk = NULL;

		/* find task's jump destination or no next task*/
		if (task->va_base[task->num_cmd-1] == 0x10000001)
			next_tsk = cmdq_core_search_task_by_pc(
				task->va_base[task->num_cmd-2], cqtd);
		else
			CMDQ_MSG("No next task: LAST instruction : (0x%08x, 0x%08x)\n",
				 task->va_base[task->num_cmd-1],
				 task->va_base[task->num_cmd-2]);

		/*
		 * Then, we try remove task from the chain of cqtd->cur_task.
		 * . if HW PC falls in task range
		 * . HW EXEC_CNT += 1
		 * . thread.wait_cookie += 1
		 * . set HW PC to next task head
		 * . if not, find previous task
		 *                (whose jump address is task->mva_base)
		 * . check if HW PC points is not at the EOC/JUMP end
		 * . change jump to fake EOC(no IRQ)
		 * . insert jump to next task head and increase cmd buffer size
		 * . if there is no next task, set HW End Address
		 */
		if (task->num_cmd && thread_pc >= task->mva_base &&
		    thread_pc <= (task->mva_base + task->command_size)) {
			if (next_tsk) {
				/* cookie already +1 */
				writel(cookie,
				       gce_base_va + CMDQ_THR_EXEC_CNT_OFFSET +
				       CMDQ_THR_SHIFT * tid);
				cqtd->wait_cookie = cookie + 1;
				writel(next_tsk->mva_base,
				       gce_base_va + CMDQ_THR_CURR_ADDR_OFFSET +
				       CMDQ_THR_SHIFT * tid);
				CMDQ_MSG("WAIT: resume task 0x%p from err\n",
					 next_tsk);
			}
		} else {
			prev_tsk = NULL;
			for (i = 0; i < CMDQ_MAX_TASK_IN_THREAD; i++) {
				u32 *prev_va, *curr_va;
				u32 prev_num, curr_num;

				prev_tsk = cqtd->cur_task[i];
				if (!prev_tsk)
					continue;

				prev_va = prev_tsk->va_base;
				prev_num = prev_tsk->num_cmd;
				if (!prev_num)
					continue;

				curr_va = task->va_base;
				curr_num = task->num_cmd;

				/* find which task JUMP into task */
				if (prev_va[prev_num-2] == task->mva_base &&
				    prev_va[prev_num-1] == 0x10000001) {
					/* Copy Jump instruction */
					prev_va[prev_num-2] =
						curr_va[curr_num-2];
					prev_va[prev_num-1] =
						curr_va[curr_num-1];

					if (next_tsk)
						cmdq_core_reorder_task_array(
							cqtd, i);

					CMDQ_MSG("WAIT: modify jump to 0x%08x (prev:0x%p, task:0x%p)\n",
						 task->va_base[task->num_cmd-2],
						 prev_tsk, task);

					/*
					 * Give up fetched command,
					 * invoke CMDQ HW to re-fetch command.
					 */
					cmdq_core_invalidate_hw_fetched_buffer(
						gce_base_va, tid);

					break;
				}
			}
		}
	} while (0);

	if (cqtd->task_count <= 0)
		cmdq_core_disable_hw_thread(cqctx, tid);
	else
		cmdq_core_resume_hw_thread(gce_base_va, tid);

	spin_unlock_irqrestore(&cqctx->exec_lock, flags);

	if (throw_err) {
		u32 op = (inst_a & 0xff000000) >> 24;

		switch (op) {
		case CMDQ_CODE_WFE:
			dev_err(dev,
				"%s in CMDQ IRQ:0x%02x, INST:(0x%08x, 0x%08x), OP:WAIT EVENT:%s\n",
				module, irq_flag, inst_a, inst_b,
				cmdq_event_get_name(inst_a & (~0xff000000)));
			break;
		default:
			dev_err(dev,
				"%s in CMDQ IRQ:0x%02x, INST:(0x%08x, 0x%08x), OP:%s\n",
				module, irq_flag, inst_a, inst_b,
				cmdq_core_parse_op(op));
			break;
		}
	}

	return status;
}

/*
 * Re-fetch thread's command buffer
 * Usage:
 *     If SW motifies command buffer content after SW configed command to GCE,
 *     SW should notify GCE to re-fetch command in order to
 *     ensure inconsistent command buffer content between DRAM and GCE's SRAM.
 */
static void cmdq_core_invalidate_hw_fetched_buffer(void __iomem *gce_base_va,
						   int tid)
{
	/*
	 * Setting HW thread PC will invoke that
	 * GCE (CMDQ HW) gives up fetched command buffer,
	 * and fetch command from DRAM to GCE's SRAM again.
	 */
	u32 pc = readl(gce_base_va + CMDQ_THR_CURR_ADDR_OFFSET +
		       CMDQ_THR_SHIFT * tid);

	writel(pc,
	       gce_base_va + CMDQ_THR_CURR_ADDR_OFFSET + CMDQ_THR_SHIFT * tid);
}

static int cmdq_core_wait_task_done(struct cmdq_task *task)
{
	int wait_q;
	int status;
	int tid;
	unsigned long timeout = msecs_to_jiffies(
		CMDQ_ACQUIRE_THREAD_TIMEOUT_MS);
	struct cmdq_context *cqctx = task->cqctx;
	struct device *dev = cqctx->dev;

	status = 0;
	tid = task->thread;
	if (tid == CMDQ_INVALID_THREAD) {
		CMDQ_LOG("pid:%d task:0x%p wait for valid thread first\n",
			 current->pid, task);

		/*
		 * wait for acquire thread
		 * (this is done by cmdq_core_consume_waiting_list);
		 */
		wait_q = wait_event_timeout(
			cqctx->thread_dispatch_queue,
			(task->thread != CMDQ_INVALID_THREAD), timeout);

		if (!wait_q || task->thread == CMDQ_INVALID_THREAD) {
			mutex_lock(&cqctx->task_mutex);

			/*
			 * it's possible that the task was just consumed now.
			 * so check again.
			 */
			if (task->thread == CMDQ_INVALID_THREAD) {
				/*
				 * Task may have released,
				 * or starved to death.
				 */
				dev_err(dev,
					"task(0x%p) timeout with invalid thread\n",
					task);

				cmdq_core_dump_task(task);
				cmdq_core_dump_task_with_engine_flag
					(cqctx, task->engine_flag);

				/*
				 * remove from waiting list,
				 * so that it won't be consumed in the future
				 */
				list_del_init(&task->list_entry);

				mutex_unlock(&cqctx->task_mutex);
				return -EINVAL;
			}

			/* valid thread, so we keep going */
			mutex_unlock(&cqctx->task_mutex);
		}
	}

	tid = task->thread;
	if ((tid < 0) || (tid >= CMDQ_MAX_THREAD_COUNT)) {
		dev_err(dev, "invalid thread %d in %s\n", tid, __func__);
		return -EINVAL;
	}

	/* start to wait */
	task->begin_wait = sched_clock();
	CMDQ_MSG("-->WAIT: task 0x%p on thread %d timeout: %d(ms) begin\n",
		 task, tid, jiffies_to_msecs(CMDQ_DEFAULT_TIMEOUT_MS));
	wait_q = cmdq_core_wait_task_done_with_timeout_impl(task, tid);

	/*
	 * wake up!
	 * so the maximum total waiting time would be
	 * CMDQ_PREDUMP_TIMEOUT_MS * CMDQ_PREDUMP_RETRY_COUNT
	 */
	task->waked_up = sched_clock();
	CMDQ_MSG("WAIT: task 0x%p waitq=%d state=%d\n",
		 task, wait_q, task->task_state);

	status = cmdq_core_handle_wait_task_result_impl(task, tid, wait_q);

	CMDQ_MSG("<--WAIT: task 0x%p on thread %d end\n", task, tid);

	return status;
}

static int cmdq_core_exec_find_task_slot(struct cmdq_task *last,
					 struct cmdq_task *task,
					 int tid, int loop)
{
	int status = 0;
	struct cmdq_thread *cqtd;
	struct cmdq_task *prev_tsk;
	int index;
	int prev;
	int cookie;
	struct cmdq_context *cqctx = task->cqctx;
	void __iomem *gce_base_va = cqctx->cqdev.base_va;
	struct device *dev = cqctx->dev;

	cqtd = &cqctx->thread[tid];
	cookie = cqtd->next_cookie;

	/*
	 * Traverse forward to adjust tasks' order
	 * according to their priorities.
	 */
	for (prev = (cookie % CMDQ_MAX_TASK_IN_THREAD); loop > 0; loop--) {
		index = prev;
		if (index < 0)
			index = CMDQ_MAX_TASK_IN_THREAD - 1;

		prev = index - 1;
		if (prev < 0)
			prev = CMDQ_MAX_TASK_IN_THREAD - 1;

		prev_tsk = cqtd->cur_task[prev];

		/* maybe the job is killed, search a new one */
		while ((!prev_tsk) && (loop > 1)) {
			dev_err(dev,
				"prev_tsk is NULL, prev:%d, loop:%d, index:%d\n",
				prev, loop, index);

			prev = prev - 1;
			if (prev < 0)
				prev = CMDQ_MAX_TASK_IN_THREAD - 1;

			prev_tsk = cqtd->cur_task[prev];
			loop--;
		}

		if (!prev_tsk) {
			cmdq_core_attach_error_task(task, tid);
			dev_err(dev,
				"invalid task state for reorder %d %d\n",
				index, loop);
			status = -EFAULT;
			break;
		}

		/* insert this task */
		if (loop <= 1) {
			CMDQ_MSG("Set current(%d) order for the new task, line:%d\n",
				 index, __LINE__);

			CMDQ_MSG("Original PC 0x%p, end 0x%p\n",
				 &prev_tsk->mva_base,
				 &prev_tsk->mva_base + prev_tsk->command_size);
			CMDQ_MSG("Original instruction 0x%08x, 0x%08x\n",
				 prev_tsk->va_base[prev_tsk->num_cmd-1],
				 prev_tsk->va_base[prev_tsk->num_cmd-2]);

			cqtd->cur_task[index] = task;
			cqtd->cur_task[index] = task;
			/* Jump: Absolute */
			prev_tsk->va_base[prev_tsk->num_cmd - 1] = 0x10000001;
			/* Jump to here */
			prev_tsk->va_base[prev_tsk->num_cmd - 2] = task->mva_base;
			CMDQ_MSG("EXEC: modify jump to 0x%p, line:%d\n",
				 &task->mva_base, __LINE__);

			/* re-fetch command buffer again. */
			cmdq_core_invalidate_hw_fetched_buffer(
				gce_base_va, tid);

			break;
		}

		if (prev_tsk->priority < task->priority) {
			/* new task has higher priority */

			CMDQ_LOG("Switch prev(%d, 0x%p) and curr(%d, 0x%p) order\n",
				 prev, prev_tsk, index, task);

			cqtd->cur_task[index] = prev_tsk;
			prev_tsk->va_base[prev_tsk->num_cmd - 1] =
				task->va_base[task->num_cmd - 1];
			prev_tsk->va_base[prev_tsk->num_cmd - 2] =
				task->va_base[task->num_cmd - 2];

			/* Boot priority for the task */
			prev_tsk->priority += CMDQ_MIN_AGE_VALUE;
			prev_tsk->reorder++;

			cqtd->cur_task[prev] = task;
			/* Jump: Absolute */
			task->va_base[task->num_cmd - 1] = 0x10000001;
			/* Jump to here */
			task->va_base[task->num_cmd - 2] = prev_tsk->mva_base;

			CMDQ_MSG("EXEC: modify jump to 0x%p, line:%d\n",
				 &prev_tsk->mva_base, __LINE__);

			/* re-fetch command buffer again. */
			cmdq_core_invalidate_hw_fetched_buffer(
				gce_base_va, tid);

			if (last == task) {
				CMDQ_LOG("update last from 0x%p to 0x%p\n",
					 task, prev_tsk);
				last = prev_tsk;
			}
		} else {
			/* previous task has higher priority */

			CMDQ_MSG("Set current(%d) order for the new task, line:%d\n",
				 index, __LINE__);

			CMDQ_MSG("Original PC 0x%p, end 0x%p\n",
				 &prev_tsk->mva_base,
				 &prev_tsk->mva_base + prev_tsk->command_size);
			CMDQ_MSG("Original instruction 0x%08x, 0x%08x\n",
				 prev_tsk->va_base[prev_tsk->num_cmd-1],
				 prev_tsk->va_base[prev_tsk->num_cmd-2]);

			cqtd->cur_task[index] = task;
			/* Jump: Absolute */
			prev_tsk->va_base[prev_tsk->num_cmd - 1] = 0x10000001;
			/* Jump to here */
			prev_tsk->va_base[prev_tsk->num_cmd - 2] = task->mva_base;

			CMDQ_MSG("EXEC: modify jump to 0x%p, line:%d\n",
				 &task->mva_base, __LINE__);

			/* re-fetch command buffer again. */
			cmdq_core_invalidate_hw_fetched_buffer(
				gce_base_va, tid);

			break;
		}
	}

	CMDQ_MSG("Reorder %d tasks for performance end, last:0x%p\n",
		 loop, last);

	return status;
}

static int cmdq_core_exec_task_async_impl(struct cmdq_task *task, int tid)
{
	int status;
	struct cmdq_thread *cqtd;
	struct cmdq_task *last_tsk;
	unsigned long flags;
	int loop;
	int minimum;
	int cookie;
	int thread_prio;
	char long_msg[CMDQ_LONGSTRING_MAX];
	u32 msg_offset;
	int msg_max_size;
	struct cmdq_context *cqctx = task->cqctx;
	void __iomem *gce_base_va = cqctx->cqdev.base_va;
	struct device *dev = cqctx->dev;

	long_msg[0] = '\0';
	msg_offset = 0;
	msg_max_size = CMDQ_LONGSTRING_MAX;
	cmdq_core_long_string(false, long_msg, &msg_offset, &msg_max_size,
			      "-->EXEC: task 0x%p on thread %d begin, VABase: 0x%p, mva_base: 0x%p,",
			      task, tid, task->va_base, &task->mva_base);
	cmdq_core_long_string(false, long_msg, &msg_offset, &msg_max_size,
			      " Size: %d, buf_size: %d, scenario:%d, flag:0x%llx\n",
			      task->command_size, task->buf_size,
			      task->scenario, task->engine_flag);
	if (msg_offset > 0)
		CMDQ_MSG("%s", long_msg);

	status = 0;
	cqtd = &cqctx->thread[tid];
	task->trigger = sched_clock();

	spin_lock_irqsave(&cqctx->exec_lock, flags);

	/* update task's thread info */
	task->thread = tid;
	task->irq_flag = 0;
	task->task_state = TASK_STATE_BUSY;

	if (cqtd->task_count <= 0) {
		bool is_prefetch;

		CMDQ_MSG("EXEC: new HW thread(%d)\n", tid);

		if (cmdq_core_reset_hw_thread(cqctx, tid) < 0) {
			spin_unlock_irqrestore(&cqctx->exec_lock, flags);
			return -EFAULT;
		}

		writel(0,
		       gce_base_va + CMDQ_THR_INST_CYCLES_OFFSET +
		       CMDQ_THR_SHIFT * tid);

		is_prefetch = cmdq_core_should_enable_prefetch(task->scenario);
		if (is_prefetch) {
			CMDQ_MSG("EXEC: set HW thread(%d) enable prefetch!\n",
				 tid);
			writel(0x1,
			       gce_base_va + CMDQ_THR_PREFETCH_OFFSET +
			       CMDQ_THR_SHIFT * tid);
		}

		thread_prio = CMDQ_THR_PRIO_DISPLAY_CONFIG;
		CMDQ_MSG("EXEC: set HW thread(%d) pc:0x%p, prio:%d\n",
			 tid, &task->mva_base, thread_prio);
		writel(task->mva_base,
		       gce_base_va + CMDQ_THR_CURR_ADDR_OFFSET +
		       CMDQ_THR_SHIFT * tid);
		writel(task->mva_base + task->command_size,
		       gce_base_va + CMDQ_THR_END_ADDR_OFFSET +
		       CMDQ_THR_SHIFT * tid);
		writel(thread_prio & 0x7,
		       gce_base_va + CMDQ_THR_CFG_OFFSET +
		       CMDQ_THR_SHIFT * tid);

		/* enable timeout */
		writel(0x013,
		       gce_base_va + CMDQ_THR_IRQ_ENABLE_OFFSET +
		       CMDQ_THR_SHIFT * tid);

		minimum = cmdq_get_cookie_cnt(gce_base_va, tid);
		cmdq_core_insert_task_from_thread_array_by_cookie
			(task, cqtd, (minimum + 1), true);

		/* verify that we don't corrupt EOC + JUMP pattern */
		cmdq_core_verfiy_task_command_end(task);

		/* enable HW thread */
		CMDQ_MSG("enable HW thread(%d)\n", tid);
		writel(0x01,
		       gce_base_va + CMDQ_THR_ENABLE_TASK_OFFSET +
		       CMDQ_THR_SHIFT * tid);
	} else {
		unsigned long curr_pa, end_pa;

		CMDQ_MSG("EXEC: reuse HW thread(%d), task_count:%d\n",
			 tid, cqtd->task_count);

		status = cmdq_core_suspend_hw_thread(cqctx, tid);
		if (status < 0) {
			spin_unlock_irqrestore(&cqctx->exec_lock, flags);
			return status;
		}

		writel(0,
		       gce_base_va + CMDQ_THR_INST_CYCLES_OFFSET +
		       CMDQ_THR_SHIFT * tid);

		cookie = cqtd->next_cookie;

		/*
		 * Boundary case tested: EOC have been executed,
		 *                       but JUMP is not executed
		 * Thread PC: 0x9edc0dd8, End: 0x9edc0de0,
		 * Curr Cookie: 1, Next Cookie: 2
		 * PC = END - 8, EOC is executed
		 * PC = END - 0, All CMDs are executed
		 */

		curr_pa = (unsigned long)readl(gce_base_va + CMDQ_THR_CURR_ADDR_OFFSET +
					       CMDQ_THR_SHIFT * tid);
		end_pa = (unsigned long)readl(gce_base_va + CMDQ_THR_END_ADDR_OFFSET +
					      CMDQ_THR_SHIFT * tid);
		if ((curr_pa == end_pa - 8) || (curr_pa == end_pa - 0)) {
			long_msg[0] = '\0';
			msg_offset = 0;
			msg_max_size = CMDQ_LONGSTRING_MAX;
			cmdq_core_long_string(true, long_msg,
					      &msg_offset, &msg_max_size,
					      "EXEC: Set HW thread(%d) pc from 0x%08x(end:0x%08x) to 0x%p,",
					      tid, curr_pa, end_pa,
					      &task->mva_base);
			cmdq_core_long_string(true, long_msg,
					      &msg_offset, &msg_max_size,
					      " oriNextCookie:%d, oriTaskCount:%d\n",
					      cookie, cqtd->task_count);
			if (msg_offset > 0)
				CMDQ_LOG("%s", long_msg);

			/* set to task directly */
			writel(task->mva_base,
			       gce_base_va + CMDQ_THR_CURR_ADDR_OFFSET +
			       CMDQ_THR_SHIFT * tid);
			writel(task->mva_base + task->command_size,
			       gce_base_va + CMDQ_THR_END_ADDR_OFFSET +
			       CMDQ_THR_SHIFT * tid);
			cqtd->cur_task[cookie % CMDQ_MAX_TASK_IN_THREAD] = task;
			cqtd->task_count++;
		} else {
			CMDQ_MSG("Connect new task's MVA to previous one\n");

			/* Current task that shuld be processed */
			minimum = cmdq_get_cookie_cnt(gce_base_va, tid) + 1;
			if (minimum > CMDQ_MAX_COOKIE_VALUE)
				minimum = 0;

			/* Calculate loop count to adjust the tasks' order */
			if (minimum <= cookie)
				loop = cookie - minimum;
			else
				/* Counter wrapped */
				loop = (CMDQ_MAX_COOKIE_VALUE - minimum + 1) +
				       cookie;

			CMDQ_MSG("Reorder task in range [%d, %d] with count %d\n",
				 minimum, cookie, loop);

			/*
			 * .note cqtd->task_count--
			 *       when remove task from cqtd in ISR
			 * .In mutlple SW clients or async case,
			 *  clients may continue submit tasks
			 *  with overlap engines
			 *  it's okey 0 = abs(cqtd->next_cookie, THR_CNT+1) when
			 *      .submit task_1, trigger GCE
			 *      .submit task_2:
			 *          .GCE exec task1 done
			 *          .task_2 lock exec lock
			 *                  when insert task to thread
			 *          .task 1's IRQ
			 */

			if (loop < 0) {
				cmdq_core_dump_task_in_thread(cqctx,
							      tid,
							      true,
							      true,
							      false);

				long_msg[0] = '\0';
				msg_offset = 0;
				msg_max_size = CMDQ_LONGSTRING_MAX;
				cmdq_core_long_string
					(true, long_msg,
					 &msg_offset, &msg_max_size,
					 "Invalid task count(%d) in thread %d for reorder,",
					 loop, tid);
				cmdq_core_long_string
					(true, long_msg,
					 &msg_offset, &msg_max_size,
					 " next_cookie:%d, next_cookieHW:%d, task:0x%p\n",
					 cqtd->next_cookie, minimum, task);
				if (msg_offset > 0)
					dev_err(dev, "%s", long_msg);

				spin_unlock_irqrestore(&cqctx->exec_lock,
						       flags);
				return -EFAULT;
			}
			if (loop > CMDQ_MAX_TASK_IN_THREAD) {
				CMDQ_LOG("loop = %d, execeed max task in thread",
					 loop);
				loop = loop % CMDQ_MAX_TASK_IN_THREAD;
			}
			CMDQ_MSG("Reorder %d tasks for performance begin\n",
				 loop);

			/*
			 * By default, task is the last task,
			 * and insert [cookie % CMDQ_MAX_TASK_IN_THREAD]
			 */
			last_tsk = task;	/* Default last task */

			status = cmdq_core_exec_find_task_slot(
				last_tsk, task, tid, loop);
			if (status < 0) {
				spin_unlock_irqrestore(
					&cqctx->exec_lock, flags);
				dev_err(dev,
					"invalid task state for reorder.\n");
				return status;
			}

			smp_mb(); /* modify jump before enable thread */

			writel(task->mva_base + task->command_size,
			       gce_base_va + CMDQ_THR_END_ADDR_OFFSET +
			       CMDQ_THR_SHIFT * tid);
			cqtd->task_count++;
		}

		cqtd->next_cookie += 1;
		if (cqtd->next_cookie > CMDQ_MAX_COOKIE_VALUE)
			cqtd->next_cookie = 0;

		/* verify that we don't corrupt EOC + JUMP pattern */
		cmdq_core_verfiy_task_command_end(task);

		/* resume HW thread */
		cmdq_core_resume_hw_thread(gce_base_va, tid);
	}

	spin_unlock_irqrestore(&cqctx->exec_lock, flags);

	CMDQ_MSG("<--EXEC: status: %d\n", status);

	return status;
}

static void cmdq_core_handle_done_with_cookie_impl(struct cmdq_context *cqctx,
						   int tid,
						   int value,
						   unsigned long long *got_irq,
						   u32 cookie)
{
	struct cmdq_thread *cqtd = &cqctx->thread[tid];
	int count;
	int inner;
	struct device *dev = cqctx->dev;

	if (cqtd->wait_cookie <= cookie) {
		count = cookie - cqtd->wait_cookie + 1;
	} else if ((cookie+1) % CMDQ_MAX_COOKIE_VALUE == cqtd->wait_cookie) {
		count = 0;
		CMDQ_MSG("IRQ: duplicated cookie: wait cookie:%d, hw cookie:%d",
			 cqtd->wait_cookie, cookie);
	} else {
		/* counter wrapped */
		count = (CMDQ_MAX_COOKIE_VALUE - cqtd->wait_cookie + 1) +
			(cookie + 1);
		dev_err(dev,
			"IRQ: counter wrapped: wait cookie:%d, hw cookie:%d, count=%d",
			cqtd->wait_cookie, cookie, count);
	}

	for (inner = (cqtd->wait_cookie % CMDQ_MAX_TASK_IN_THREAD); count > 0;
	     count--, inner++) {
		if (inner >= CMDQ_MAX_TASK_IN_THREAD)
			inner = 0;

		if (cqtd->cur_task[inner]) {
			struct cmdq_task *task = cqtd->cur_task[inner];

			task->got_irq = *got_irq;
			task->irq_flag = value;
#ifdef CMDQ_CORE_PROFILE_WAKE_UP
			task->send_wake_up = sched_clock();
#endif
			if (task->cb.isr_cb)
				task->cb.isr_cb(task->cb.isr_data);
			cmdq_core_remove_task_from_thread_array_by_cookie(
				cqtd, inner, TASK_STATE_DONE);
		}
	}

	cqtd->wait_cookie = cookie + 1;
	if (cqtd->wait_cookie > CMDQ_MAX_COOKIE_VALUE)
		cqtd->wait_cookie -= (CMDQ_MAX_COOKIE_VALUE + 1);
			/* min cookie value is 0 */

	wake_up(&cqctx->wait_queue[tid]);
}

static void cmdq_core_handle_error(struct cmdq_context *cqctx,
				   int tid,
				   int value,
				   unsigned long long *got_irq)
{
	struct cmdq_thread *cqtd;
	struct cmdq_task *task;
	int cookie;
	int count;
	int inner;
	int status;
	u32 curr_pa, end_pa;
	void __iomem *gce_base_va = cqctx->cqdev.base_va;
	struct device *dev = cqctx->dev;

	curr_pa = readl(gce_base_va + CMDQ_THR_CURR_ADDR_OFFSET +
			CMDQ_THR_SHIFT * tid);
	end_pa = readl(gce_base_va + CMDQ_THR_END_ADDR_OFFSET +
		       CMDQ_THR_SHIFT * tid);

	dev_err(dev, "IRQ: error thread=%d, irq_flag=0x%x\n", tid, value);
	dev_err(dev, "IRQ: Thread PC: 0x%08x, End PC:0x%08x\n",
		curr_pa, end_pa);

	cqtd = &cqctx->thread[tid];

	cookie = cmdq_get_cookie_cnt(gce_base_va, tid);

	/*
	 * we assume error happens BEFORE EOC
	 * because it wouldn't be error if this interrupt is issue by EOC.
	 * so we should inc by 1 to locate "current" task
	 */
	cookie += 1;

	/* set the issued task to error state */
	cmdq_core_dump_task_in_thread(cqctx, tid, true, true, true);
	if (cqtd->cur_task[cookie % CMDQ_MAX_TASK_IN_THREAD]) {
		task = cqtd->cur_task[cookie % CMDQ_MAX_TASK_IN_THREAD];
		task->got_irq = *got_irq;
		task->irq_flag = value;
		cmdq_core_attach_error_task(task, tid);
		cmdq_core_remove_task_from_thread_array_by_cookie(
			cqtd, cookie % CMDQ_MAX_TASK_IN_THREAD,
			TASK_STATE_ERROR);
	} else {
		dev_err(dev,
			"IRQ: can not find task in %s, pc:0x%08x, end_pc:0x%08x\n",
			__func__, curr_pa, end_pa);
		if (cqtd->task_count <= 0) {
			/*
			 * suspend HW thread first,
			 * so that we work in a consistent state
			 * outer function should acquire spinlock:
			 *   cqctx->exec_lock
			 */
			status = cmdq_core_suspend_hw_thread(cqctx, tid);
			if (status < 0)
				dev_err(dev, "IRQ: suspend HW thread failed!");

			cmdq_core_disable_hw_thread(cqctx, tid);
			dev_err(dev,
				"IRQ: there is no task for thread (%d) %s\n",
				tid, __func__);
		}
	}

	/* set the remain tasks to done state */
	if (cqtd->wait_cookie <= cookie) {
		count = cookie - cqtd->wait_cookie + 1;
	} else if ((cookie+1) % CMDQ_MAX_COOKIE_VALUE == cqtd->wait_cookie) {
		count = 0;
		CMDQ_MSG("IRQ: duplicated cookie: wait cookie:%d, hw cookie:%d",
			 cqtd->wait_cookie, cookie);
	} else {
		/* counter wrapped */
		count = (CMDQ_MAX_COOKIE_VALUE - cqtd->wait_cookie + 1) +
			(cookie + 1);
		dev_err(dev,
			"IRQ: counter wrapped: wait cookie:%d, hw cookie:%d, count=%d",
			cqtd->wait_cookie, cookie, count);
	}

	for (inner = (cqtd->wait_cookie % CMDQ_MAX_TASK_IN_THREAD); count > 0;
	     count--, inner++) {
		if (inner >= CMDQ_MAX_TASK_IN_THREAD)
			inner = 0;

		if (cqtd->cur_task[inner]) {
			task = cqtd->cur_task[inner];
			task->got_irq = *got_irq;
			task->irq_flag = 0;	/* don't know irq flag */
			/* still call isr_cb to prevent lock */
			if (task->cb.isr_cb)
				task->cb.isr_cb(task->cb.isr_data);
			cmdq_core_remove_task_from_thread_array_by_cookie(
				cqtd, inner, TASK_STATE_DONE);
		}
	}

	cqtd->wait_cookie = cookie + 1;
	if (cqtd->wait_cookie > CMDQ_MAX_COOKIE_VALUE)
		cqtd->wait_cookie -= (CMDQ_MAX_COOKIE_VALUE + 1);
			/* min cookie value is 0 */

	wake_up(&cqctx->wait_queue[tid]);
}

static void cmdq_core_handle_done(struct cmdq_context *cqctx,
				  int tid,
				  int value,
				  unsigned long long *got_irq)
{
	struct cmdq_thread *cqtd;
	int cookie;
	int loop_result = 0;
	void __iomem *gce_base_va = cqctx->cqdev.base_va;

	cqtd = &cqctx->thread[tid];

	if (loop_result < 0) {
		/* The loop CB failed, so stop HW thread now. */
		cmdq_core_disable_hw_thread(cqctx, tid);

		/*
		 * loop CB failed.
		 * the EXECUTION count should not be used as cookie,
		 * since it will increase by each loop iteration.
		 */
		cookie = cqtd->wait_cookie;
	} else {
		/* normal task cookie */
		cookie = cmdq_get_cookie_cnt(gce_base_va, tid);
		CMDQ_MSG("Done: thread %d got cookie: %d\n", tid, cookie);
	}

	cmdq_core_handle_done_with_cookie_impl(cqctx, tid, value,
					       got_irq, cookie);
}

static void cmdq_core_handle_irq(struct cmdq_context *cqctx, int tid)
{
	unsigned long flags = 0L;
	unsigned long long got_irq;
	int value;
	int enabled;
	int cookie;
	void __iomem *gce_base_va = cqctx->cqdev.base_va;
	struct device *dev = cqctx->dev;

	got_irq = sched_clock();

	/*
	 * normal execution, marks tasks done and remove from thread
	 * also, handle "loop CB fail" case
	 */
	spin_lock_irqsave(&cqctx->exec_lock, flags);

	/*
	 * it is possible for another CPU core
	 * to run "release task" right before we acquire the spin lock
	 * and thus reset / disable this HW thread
	 * so we check both the IRQ flag and the enable bit of this thread
	 */
	value = readl(gce_base_va + CMDQ_THR_IRQ_STATUS_OFFSET +
		      CMDQ_THR_SHIFT * tid);
	if (!(value & 0x13)) {
		dev_err(dev,
			"IRQ: thread %d got interrupt but IRQ flag is 0x%08x in NWd\n",
			tid, value);
		spin_unlock_irqrestore(&cqctx->exec_lock, flags);
		return;
	}

	enabled = readl(gce_base_va + CMDQ_THR_ENABLE_TASK_OFFSET +
			CMDQ_THR_SHIFT * tid);
	if (!(enabled & 0x01)) {
		dev_err(dev,
			"IRQ: thread %d got interrupt already disabled 0x%08x\n",
			tid, enabled);
		spin_unlock_irqrestore(&cqctx->exec_lock, flags);
		return;
	}

	/* read HW cookie here for printing message */
	cookie = cmdq_get_cookie_cnt(gce_base_va, tid);

	/*
	 * Move the reset IRQ before read HW cookie
	 * to prevent race condition and save the cost of suspend
	 */
	writel(~value,
	       gce_base_va + CMDQ_THR_IRQ_STATUS_OFFSET +
	       CMDQ_THR_SHIFT * tid);

	CMDQ_MSG("IRQ: thread %d got interrupt, after reset, and IRQ flag is 0x%08x, cookie: %d\n",
		 tid, value, cookie);

	if (value & 0x12)
		cmdq_core_handle_error(cqctx, tid, value, &got_irq);
	else if (value & 0x01)
		cmdq_core_handle_done(cqctx, tid, value, &got_irq);

	spin_unlock_irqrestore(&cqctx->exec_lock, flags);
}

static int cmdq_suspend(struct device *dev)
{
	unsigned long flags = 0L;
	struct cmdq_engine *cqe;
	u32 exec_threads;
	int ref_count;
	bool kill_tasks = false;
	struct cmdq_task *task;
	struct list_head *p;
	int i;
	struct cmdq_context *cqctx;
	void __iomem *gce_base_va;

	cqctx = dev_get_drvdata(dev);
	cqe = cqctx->engine;
	gce_base_va = cqctx->cqdev.base_va;
	exec_threads = readl(gce_base_va + CMDQ_CURR_LOADED_THR_OFFSET);
	ref_count = cqctx->thread_usage;

	if ((ref_count > 0) || (exec_threads & 0x80000000)) {
		dev_err(dev,
			"[SUSPEND] other running, kill tasks. threads:0x%08x, ref:%d\n",
			exec_threads, ref_count);
		kill_tasks = true;
	}

	/*
	 * We need to ensure the system is ready to suspend,
	 * so kill all running CMDQ tasks
	 * and release HW engines.
	 */
	if (kill_tasks) {
		/* print active tasks */
		dev_err(dev, "[SUSPEND] active tasks during suspend:\n");
		list_for_each(p, &cqctx->task_active_list) {
			task = list_entry(p, struct cmdq_task, list_entry);
			cmdq_core_dump_task(task);
		}

		/* remove all active task from thread */
		dev_err(dev, "[SUSPEND] remove all active tasks\n");
		list_for_each(p, &cqctx->task_active_list) {
			task = list_entry(p, struct cmdq_task, list_entry);
			if (task->thread != CMDQ_INVALID_THREAD) {
				spin_lock_irqsave(&cqctx->exec_lock, flags);
				cmdq_core_force_remove_task_from_thread(
					task, task->thread);
				task->task_state = TASK_STATE_KILLED;
				spin_unlock_irqrestore(
					&cqctx->exec_lock, flags);

				/*
				 * release all thread and
				 * mark all active tasks as "KILLED"
				 * (so that thread won't release again)
				 */
				dev_err(dev,
					"[SUSPEND] release all threads and HW clocks\n");
				cmdq_core_release_thread(task);
			}
		}

		/* disable all HW thread */
		dev_err(dev, "[SUSPEND] disable all HW threads\n");
		for (i = 0; i < CMDQ_MAX_THREAD_COUNT; i++)
			cmdq_core_disable_hw_thread(cqctx, i);

		/* reset all cmdq_thread */
		memset(&cqctx->thread[0], 0, sizeof(cqctx->thread));

		/* reset all cmd_engine */
		memset(&cqctx->engine[0], 0, sizeof(cqctx->engine));
		cmdq_core_reset_cmdq_engine(cqctx);
	}

	spin_lock_irqsave(&cqctx->thread_lock, flags);
	cqctx->suspended = true;
	spin_unlock_irqrestore(&cqctx->thread_lock, flags);

	/* ALWAYS allow suspend */
	return 0;
}

static int cmdq_resume(struct device *dev)
{
	return 0;
}

static int cmdq_core_resumed_notifier(struct cmdq_context *cqctx)
{
	/*
	 * Note:
	 * delay resume timing until process-unfreeze done in order to
	 * ensure M4U driver had restore M4U port setting
	 */

	unsigned long flags = 0L;

	CMDQ_MSG("[RESUME] %s\n", __func__);

	spin_lock_irqsave(&cqctx->thread_lock, flags);

	CMDQ_MSG("[RESUME] thread_usage=%d\n", cqctx->thread_usage);

	cqctx->suspended = false;

	/*
	 * during suspending, there may be queued tasks.
	 * we should process them if any.
	 */
	if (!work_pending(&cqctx->task_consume_wait_queue_item)) {
		CMDQ_MSG("there are undone task, process them\n");
		/* we use system global work queue (kernel thread kworker/n) */
		queue_work(cqctx->task_consume_wq,
			   &cqctx->task_consume_wait_queue_item);
	}

	spin_unlock_irqrestore(&cqctx->thread_lock, flags);

	return 0;
}

static int cmdq_core_exec_task_async_with_retry(struct cmdq_task *task, int tid)
{
	int retry;
	int status;
	struct device *dev = task->cqctx->dev;

	retry = 0;
	status = -EFAULT;
	do {
		cmdq_core_verfiy_task_command_end(task);

		status = cmdq_core_exec_task_async_impl(task, tid);

		if (status >= 0)
			break;

		if ((task->task_state == TASK_STATE_KILLED) ||
		    (task->task_state == TASK_STATE_ERROR)) {
			dev_err(dev, "cmdq_core_exec_task_async_impl fail\n");
			status = -EFAULT;
			break;
		}

		retry++;
	} while (retry < CMDQ_MAX_RETRY_COUNT);

	return status;
}

int cmdq_get_time_in_ms(unsigned long long start, unsigned long long end)
{
	unsigned long long _duration = end - start;

	do_div(_duration, 1000000);
	return (int)_duration;
}

static void cmdq_core_consume_waiting_list(struct work_struct *work)
{
	struct list_head *p, *n = NULL;
	bool thread_acquired;
	unsigned long long consume_time;
	int waiting_time_ms;
	bool need_log;
	char long_msg[CMDQ_LONGSTRING_MAX];
	u32 msg_offset;
	int msg_max_size;
	bool need_print_log;
	struct cmdq_context *cqctx;
	struct device *dev;

	cqctx = container_of(work, struct cmdq_context,
			     task_consume_wait_queue_item);
	dev = cqctx->dev;

	/*
	 * when we're suspending,
	 * do not execute any tasks. delay & hold them.
	 */
	if (cqctx->suspended)
		return;

	consume_time = sched_clock();

	mutex_lock(&cqctx->task_mutex);

	thread_acquired = false;

	/* scan and remove (if executed) waiting tasks */
	list_for_each_safe(p, n, &cqctx->task_wait_list) {
		struct cmdq_task *task;
		struct cmdq_thread *cqtd = NULL;
		int tid;
		int status;
		enum cmdq_hw_thread_priority thread_prio;

		task = list_entry(p, struct cmdq_task, list_entry);

		thread_prio = CMDQ_THR_PRIO_DISPLAY_CONFIG;

		long_msg[0] = '\0';
		msg_offset = 0;
		msg_max_size = CMDQ_LONGSTRING_MAX;
		need_print_log = cmdq_core_should_print_msg(LOG_LEVEL_MSG);
		cmdq_core_long_string(need_print_log, long_msg, &msg_offset,
				      &msg_max_size,
				      "-->THREAD: try acquire thread for task: 0x%p, thread_prio: %d,",
				      task, thread_prio);
		cmdq_core_long_string(need_print_log, long_msg, &msg_offset,
				      &msg_max_size,
				      " task_prio: %d, flag: 0x%llx, scenario:%d begin\n",
				      task->priority, task->engine_flag,
				      task->scenario);
		if (msg_offset > 0)
			CMDQ_MSG("%s", long_msg);

		waiting_time_ms = cmdq_get_time_in_ms(
			task->submit, consume_time);
		need_log = waiting_time_ms >= CMDQ_PREDUMP_TIMEOUT_MS;
		/* allocate hw thread */
		tid = cmdq_core_acquire_thread(cqctx, task->scenario);
		if (tid != CMDQ_INVALID_THREAD)
			cqtd = &cqctx->thread[tid];

		if (tid == CMDQ_INVALID_THREAD || !cqtd) {
			/* have to wait, remain in wait list */
			CMDQ_MSG("<--THREAD: acquire thread fail, need to wait\n");
			if (need_log) {
				/* task wait too long */
				long_msg[0] = '\0';
				msg_offset = 0;
				msg_max_size = CMDQ_LONGSTRING_MAX;
				need_print_log = cmdq_core_should_print_msg
							(LOG_LEVEL_ERR);
				cmdq_core_long_string(need_print_log,
						      long_msg,
						      &msg_offset,
						      &msg_max_size,
						      "acquire thread pre-dump for task: 0x%p,",
						      task);
				cmdq_core_long_string(need_print_log,
						      long_msg,
						      &msg_offset,
						      &msg_max_size,
						      " thread_prio: %d, task_prio: %d, flag: 0x%llx,",
						      thread_prio,
						      task->priority,
						      task->engine_flag);
				cmdq_core_long_string(need_print_log,
						      long_msg,
						      &msg_offset,
						      &msg_max_size,
						      " scenario:%d, submit:%#llx, state=%d\n",
						      task->scenario,
						      task->submit,
						      task->task_state);
				if (msg_offset > 0)
					CMDQ_MSG("%s", long_msg);
			}
			continue;
		}

		/* some task is ready to run */
		thread_acquired = true;

		/*
		 * start execution
		 * remove from wait list and put into active list
		 */
		list_del_init(&task->list_entry);
		list_add_tail(&task->list_entry,
			      &cqctx->task_active_list);

		CMDQ_MSG("<--THREAD: acquire thread w/flag: 0x%llx on thread(%d): 0x%p end\n",
			 task->engine_flag, tid, cqtd);

		/* run task on thread */
		status = cmdq_core_exec_task_async_with_retry(task, tid);
		if (status < 0) {
			dev_err(dev, "%s fail, release task 0x%p\n",
				__func__, task);
			cmdq_core_track_task_record(task, tid);
			cmdq_core_release_thread(task);
			cmdq_core_release_task_unlocked(task);
			task = NULL;
		}
	}

	if (thread_acquired) {
		/*
		 * notify some task's sw thread to change their waiting state.
		 * (if they have already called
		 *  cmdq_core_wait_result_and_release_task())
		 */
		wake_up_all(&cqctx->thread_dispatch_queue);
	}

	mutex_unlock(&cqctx->task_mutex);
}

static int cmdq_core_submit_task_async(struct cmdq_command *cmd_desc,
				       struct cmdq_task **task_out,
				       struct cmdq_task_cb *cb)
{
	struct cmdq_task *task;
	struct cmdq_context *cqctx = cmd_desc->cqctx;

	cmdq_core_verfiy_desc_command_end(cmd_desc);

	if (cqctx->regular_count) {
		int i = cmdq_core_get_thread_index_from_scenario(
			cmd_desc->scenario);

		cqctx->task_count[i]++;
	}

	CMDQ_MSG("-->SUBMIT_ASYNC: cmd 0x%p begin\n", cmd_desc->va_base);

	/* creates a new task and put into tail of waiting list */
	task = cmdq_core_acquire_task(cmd_desc, cb);

	if (!task)
		return -EFAULT;

	if (task_out)
		*task_out = task;

	/*
	 * Consume the waiting list.
	 * This may or may not execute the task, depending on available threads.
	 */
	cmdq_core_consume_waiting_list(&cqctx->task_consume_wait_queue_item);

	CMDQ_MSG("<--SUBMIT_ASYNC: task: 0x%p end\n", cmd_desc->va_base);
	return 0;
}

static int cmdq_core_release_task(struct cmdq_task *task)
{
	unsigned long flags;
	int status;
	struct cmdq_context *cqctx = task->cqctx;
	int tid = task->thread;
	struct cmdq_thread *cqtd = &cqctx->thread[tid];
	void __iomem *gce_base_va = cqctx->cqdev.base_va;

	CMDQ_MSG("<--TASK: %s 0x%p begin\n", __func__, task);

	if (tid != CMDQ_INVALID_THREAD && cqtd) {
		/* this task is being executed (or queueed) on a hw thread */

		/* get sw lock first to ensure atomic access hw */
		spin_lock_irqsave(&cqctx->exec_lock, flags);
		smp_mb();	/* make sure atomic access hw */

		status = cmdq_core_force_remove_task_from_thread(task, tid);
		if (cqtd->task_count > 0)
			cmdq_core_resume_hw_thread(gce_base_va, tid);

		spin_unlock_irqrestore(&cqctx->exec_lock, flags);
		wake_up(&cqctx->wait_queue[tid]);
	}

	cmdq_core_track_task_record(task, tid);
	cmdq_core_release_thread(task);
	cmdq_core_release_task_internal(task);

	CMDQ_MSG("-->TASK: %s 0x%p end\n", __func__, task);
	return 0;
}

static int cmdq_core_wait_result_and_release_task(struct cmdq_task *task)
{
	int status;
	int tid;
	struct cmdq_context *cqctx;

	if (!task) {
		pr_err("%s err ptr=0x%p\n", __func__, task);
		return -EFAULT;
	}

	if (task->task_state == TASK_STATE_IDLE) {
		pr_err("%s task=0x%p is IDLE\n", __func__, task);
		return -EFAULT;
	}

	cqctx = task->cqctx;

	/* wait for task finish */
	tid = task->thread;
	status = cmdq_core_wait_task_done(task);

	cmdq_core_track_task_record(task, tid);

	/* release */
	cmdq_core_release_thread(task);
	cmdq_core_release_task_internal(task);

	return status;
}

static void cmdq_core_auto_release_work(struct work_struct *work_item)
{
	struct cmdq_task *task;
	int status;
	struct cmdq_task_cb cb;

	task = container_of(work_item, struct cmdq_task, auto_release_work);
	memcpy(&cb, &task->cb, sizeof(cb));
	status = cmdq_core_wait_result_and_release_task(task);
	CMDQ_MSG("[Auto Release] released task=0x%p, statu=%d\n",
		 task, status);
	task = NULL;

	/* isr fail, so call isr_cb here to prevent lock */
	if (status && cb.isr_cb)
		cb.isr_cb(cb.isr_data);

	if (cb.done_cb) {
		CMDQ_MSG("[Auto Release] call user callback 0x%p with data 0x%p\n",
			 cb.done_cb, cb.done_data);
		cb.done_cb(cb.done_data);
	}
}

static int cmdq_core_auto_release_task(struct cmdq_task *task)
{
	struct cmdq_context *cqctx = task->cqctx;

	/*
	 * the work item is embeded in task already
	 * but we need to initialized it
	 */
	INIT_WORK(&task->auto_release_work, cmdq_core_auto_release_work);
	queue_work(cqctx->task_auto_release_wq, &task->auto_release_work);
	return 0;
}

static int cmdq_core_submit_task(struct cmdq_command *cmd_desc)
{
	int status;
	struct cmdq_task *task;
	struct device *dev = cmd_desc->cqctx->dev;

	CMDQ_MSG("-->SUBMIT: SYNC cmd 0x%p begin\n", cmd_desc->va_base);
	status = cmdq_core_submit_task_async(cmd_desc, &task, NULL);

	if (status >= 0) {
		status = cmdq_core_wait_result_and_release_task(task);
		if (status < 0)
			dev_err(dev, "task(0x%p) wait fail\n", task);
	} else {
		dev_err(dev, "cmdq_core_submit_task_async failed=%d", status);
	}

	CMDQ_MSG("<--SUBMIT: SYNC cmd 0x%p end\n", cmd_desc->va_base);
	return status;
}

static void cmdq_core_deinitialize(struct platform_device *pdev)
{
	struct list_head *p;
	int i;
	struct cmdq_context *cqctx = platform_get_drvdata(pdev);
	struct list_head *lists[] = {
		&cqctx->task_free_list,
		&cqctx->task_active_list,
		&cqctx->task_wait_list
	};
	struct cmdq_error_task *err_tsk;

	/*
	 * Directly destroy the auto release WQ
	 * since we're going to release tasks anyway.
	 */
	destroy_workqueue(cqctx->task_auto_release_wq);
	cqctx->task_auto_release_wq = NULL;

	destroy_workqueue(cqctx->task_consume_wq);
	cqctx->task_consume_wq = NULL;

	err_tsk = &cqctx->error_task;
	for (i = 0; i < CMDQ_MAX_ERROR_COUNT; i++)
		kfree(err_tsk->error_task_buffer[i].va_base);
	err_tsk->err_task_count = 0;

	/* release all tasks in both list */
	for (i = 0; i < ARRAY_SIZE(lists); i++) {
		list_for_each(p, lists[i]) {
			struct cmdq_task *task;

			mutex_lock(&cqctx->task_mutex);

			task = list_entry(p, struct cmdq_task, list_entry);

			/* free allocated DMA buffer */
			cmdq_task_free_task_command_buffer(task);
			kmem_cache_free(cqctx->task_cache, task);
			list_del(p);

			mutex_unlock(&cqctx->task_mutex);
		}
	}

	kmem_cache_destroy(cqctx->task_cache);
	cqctx->task_cache = NULL;

	/* release emergency buffer */
	cmdq_emergency_buf_uninit(cqctx);

	destroy_workqueue(cqctx->print_log_struct.cmdq_print_wq);
	cqctx->print_log_struct.cmdq_print_wq = NULL;

	devm_kfree(&pdev->dev, cqctx);
}

bool cmdq_core_clock_is_on(struct cmdq_context *cqctx)
{
	if (__clk_get_enable_count(cqctx->clock) > 0)
		return 1;
	else
		return 0;
}

static int cmdq_pm_notifier_cb(struct notifier_block *nb, unsigned long event,
			       void *ptr)
{
	struct cmdq_context *cqctx =
		container_of(nb, struct cmdq_context, pm_notifier);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		/*
		 * Going to suspend the system
		 * The next stage is freeze process.
		 * We will queue all request in suspend callback,
		 * so don't care this stage
		 */
		return NOTIFY_DONE;
	case PM_POST_SUSPEND:
		/*
		 * processes had resumed in previous stage
		 * (system resume callback)
		 * resume CMDQ driver to execute.
		 */
		cmdq_core_resumed_notifier(cqctx);
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_DONE;
}

static irqreturn_t cmdq_irq_handler(int irq, void *dev)
{
	int index;
	u32 irq_status;
	bool handled = false;
	struct cmdq_context *cqctx = dev;

	/*
	 * we share IRQ bit with CQ-DMA,
	 * so it is possible that this handler
	 * is called but GCE does not have IRQ flag.
	 */
	if (cqctx->cqdev.irq == irq) {
		irq_status = readl(cqctx->cqdev.base_va +
				   CMDQ_CURR_IRQ_STATUS_OFFSET);
		irq_status &= 0xffff;
		for (index = 0;
		     (irq_status != 0xffff) && index < CMDQ_MAX_THREAD_COUNT;
		     index++) {
			/* STATUS bit set to 0 means IRQ asserted */
			if (irq_status & (1 << index))
				continue;

			/*
			 * We mark irq_status to 1 to denote finished
			 * processing, and we can early-exit if no more
			 * threads being asserted.
			 */
			irq_status |= (1 << index);

			cmdq_core_handle_irq(cqctx, index);
			handled = true;
		}
	}

	if (handled) {
		cmdq_core_add_consume_task(cqctx);
		return IRQ_HANDLED;
	}
	/* allow CQ-DMA to process this IRQ bit */
	return IRQ_NONE;
}

static int cmdq_probe(struct platform_device *pdev)
{
	struct cmdq_context *cqctx;
	int ret = 0;

	/* init cmdq context, and save it */
	ret = cmdq_core_initialize(pdev, &cqctx);
	if (ret) {
		dev_err(&pdev->dev, "failed to init cmdq context\n");
		return ret;
	}
	platform_set_drvdata(pdev, cqctx);

	ret = devm_request_irq(&pdev->dev, cqctx->cqdev.irq, cmdq_irq_handler,
			       IRQF_TRIGGER_LOW | IRQF_SHARED,
			       CMDQ_DRIVER_DEVICE_NAME, cqctx);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ISR (%d)\n", ret);
		goto fail;
	}

	ret = cmdq_core_get_clk(cqctx);
	if (ret) {
		dev_err(&pdev->dev, "failed to get clk\n");
		goto fail;
	}

	/* hibernation and suspend events */
	cqctx->pm_notifier.notifier_call = cmdq_pm_notifier_cb;
	cqctx->pm_notifier.priority = 5;
	ret = register_pm_notifier(&cqctx->pm_notifier);
	if (ret) {
		dev_err(&pdev->dev, "failed to register cmdq pm notifier\n");
		goto fail;
	}

	cmdq_create_debugfs_entries();

	return ret;

fail:
	cmdq_core_deinitialize(pdev);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int cmdq_remove(struct platform_device *pdev)
{
	int status;
	struct cmdq_context *cqctx = platform_get_drvdata(pdev);

	status = unregister_pm_notifier(&cqctx->pm_notifier);
	if (status)
		dev_err(&pdev->dev, "unregister pm notifier failed\n");

	disable_irq(cqctx->cqdev.irq);
	cmdq_core_deinitialize(pdev);
	return 0;
}

static const struct dev_pm_ops cmdq_pm_ops = {
	.suspend = cmdq_suspend,
	.resume = cmdq_resume,
};

static struct platform_driver cmdq_driver = {
	.probe = cmdq_probe,
	.remove = cmdq_remove,
	.driver = {
		.name = CMDQ_DRIVER_DEVICE_NAME,
		.owner = THIS_MODULE,
		.pm = &cmdq_pm_ops,
		.of_match_table = cmdq_of_ids,
	}
};

static int __init cmdq_init(void)
{
	int status;

	status = platform_driver_register(&cmdq_driver);
	if (status) {
		pr_err("register cmdq driver failed(%d)\n", status);
		return -ENODEV;
	}

	return 0;
}

static void __exit cmdq_exit(void)
{
	platform_driver_unregister(&cmdq_driver);
}

static int cmdq_rec_realloc_cmd_buffer(struct cmdq_rec *handle, u32 size)
{
	void *new_buf;

	new_buf = krealloc(handle->buf_ptr, size, GFP_KERNEL | __GFP_ZERO);
	if (!new_buf)
		return -ENOMEM;
	handle->buf_ptr = new_buf;
	handle->buf_size = size;
	return 0;
}

static struct cmdq_context *cmdq_rec_get_valid_ctx(struct cmdq_rec *handle)
{
	if (!handle) {
		pr_err("cmdq handle is NULL\n");
		return NULL;
	}

	if (!handle->cqctx)
		pr_err("cmdq context is NULL\n");

	return handle->cqctx;
}

static int cmdq_rec_stop_running_task(struct cmdq_rec *handle)
{
	int status;

	status = cmdq_core_release_task(handle->running_task_ptr);
	handle->running_task_ptr = NULL;
	return status;
}

int cmdq_rec_create(struct platform_device *pdev,
		    enum cmdq_scenario scenario,
		    struct cmdq_rec **handle_ptr)
{
	struct cmdq_rec *handle;
	struct cmdq_context *cqctx;
	struct device *dev = &pdev->dev;
	int ret;

	cqctx = platform_get_drvdata(pdev);
	if (!cqctx) {
		dev_err(dev, "cmdq context is NULL\n");
		return -EINVAL;
	}

	CMDQ_API("scenario=%d pid=%d\n", scenario, current->pid);

	if (scenario < 0 || scenario >= CMDQ_MAX_SCENARIO_COUNT) {
		dev_err(dev, "unknown scenario type %d\n", scenario);
		return -EINVAL;
	}

	handle = kzalloc(sizeof(u8 *) * sizeof(struct cmdq_rec),
			 GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->cqctx = platform_get_drvdata(pdev);
	handle->scenario = scenario;
	handle->engine_flag = cmdq_rec_flag_from_scenario(cqctx, scenario);
	handle->priority = CMDQ_THR_PRIO_NORMAL;

	ret = cmdq_rec_realloc_cmd_buffer(handle, CMDQ_INITIAL_CMD_BLOCK_SIZE);
	if (ret) {
		kfree(handle);
		return ret;
	}

	*handle_ptr = handle;

	CMDQ_API("success! handle=0x%p\n", handle);

	return 0;
}

static int cmdq_rec_mark(struct cmdq_rec *handle);

static int cmdq_append_command(struct cmdq_rec *handle, enum cmdq_code code,
			       u32 arg_a, u32 arg_b)
{
	int subsys;
	u32 *cmd_ptr;
	struct cmdq_context *cqctx;
	struct device *dev;
	int ret;

	/* subsys encoding position is different among platforms */
	u32 subsys_bit = CMDQ_SUBSYS_LSB;

	if (!cmdq_rec_get_valid_ctx(handle))
		return -EFAULT;

	cqctx = handle->cqctx;
	dev = cqctx->dev;
	cmd_ptr = (u32 *)((u8 *)handle->buf_ptr + handle->block_size);

	if (handle->finalized) {
		dev_err(dev,
			"already finalized record(cannot add more command)");
		dev_err(dev, "handle=0x%p, pid=%d\n", handle, current->pid);
		return -EBUSY;
	}

	/* check if we have sufficient buffer size */
	if (unlikely(handle->block_size + CMDQ_INST_SIZE > handle->buf_size)) {
		ret = cmdq_rec_realloc_cmd_buffer(handle, handle->buf_size * 2);
		if (ret)
			return ret;
	}

	/*
	 * force insert MARKER if prefetch memory is full
	 * GCE deadlocks if we don't do so
	 */
	if (code != CMDQ_CODE_EOC &&
	    cmdq_core_should_enable_prefetch(handle->scenario)) {
		if (handle->prefetch_count >= CMDQ_MAX_PREFETCH_INSTUCTION) {
			CMDQ_MSG("%d > MAX_PREFETCH_INSTUCTION, ",
				 handle->prefetch_count);
			CMDQ_MSG("force insert disable prefetch marker\n");

			/* Mark END of prefetch section */
			cmdq_rec_disable_prefetch(handle);
			/* BEGINNING of next prefetch section */
			cmdq_rec_mark(handle);
		} else {
			/* prefetch enabled marker exist */
			if (handle->prefetch_count >= 1) {
				++handle->prefetch_count;
				CMDQ_MSG("handle->prefetch_count: %d, %s, %d\n",
					 handle->prefetch_count,
					 __func__, __LINE__);
			}
		}
	}

	/*
	 * we must re-calculate current PC
	 * because we may already insert MARKER inst.
	 */
	cmd_ptr = (u32 *)((u8 *)handle->buf_ptr + handle->block_size);

	CMDQ_MSG("0x%p CMD: 0x%p, op: 0x%02x, arg_a: 0x%08x, arg_b: 0x%08x\n",
		 handle, cmd_ptr, code, arg_a, arg_b);

	switch (code) {
	case CMDQ_CODE_READ:
		/*
		 * arg_a is the HW register address to read from
		 * arg_b is the register id to read into
		 * bit 54: arg_b type, 1 for GPR
		 */
		subsys = cmdq_subsys_from_phys_addr(cqctx, arg_a);
		*cmd_ptr++ = arg_b;
		*cmd_ptr++ = (CMDQ_CODE_READ << 24) | (arg_a & 0xffff) |
			     ((subsys & 0x1f) << subsys_bit) | (2 << 21);
		break;
	case CMDQ_CODE_MOVE:
		*cmd_ptr++ = arg_b;
		*cmd_ptr++ = CMDQ_CODE_MOVE << 24 | (arg_a & 0xffffff);
		break;
	case CMDQ_CODE_WRITE:
		subsys = cmdq_subsys_from_phys_addr(cqctx, arg_a);
		if (subsys < 0) {
			dev_err(dev,
				"unsupported memory base address 0x%08x\n",
				arg_a);
			return -EFAULT;
		}

		*cmd_ptr++ = arg_b;
		*cmd_ptr++ = (CMDQ_CODE_WRITE << 24) | (arg_a & 0x0ffff) |
			     ((subsys & 0x01f) << subsys_bit);
		break;
	case CMDQ_CODE_JUMP:
		*cmd_ptr++ = arg_b;
		*cmd_ptr++ = (CMDQ_CODE_JUMP << 24) | (arg_a & 0x0ffffff);
		break;
	case CMDQ_CODE_WFE:
		/*
		 * bit 0-11: wait_value, 1
		 * bit 15: to_wait, true
		 * bit 31: to_update, true
		 * bit 16-27: update_value, 0
		 */
		*cmd_ptr++ = ((1 << 31) | (1 << 15) | 1);
		*cmd_ptr++ = (CMDQ_CODE_WFE << 24) | arg_a;
		break;
	case CMDQ_CODE_EOC:
		*cmd_ptr++ = arg_b;
		*cmd_ptr++ = (CMDQ_CODE_EOC << 24) | (arg_a & 0x0ffffff);
		break;
	default:
		return -EFAULT;
	}

	handle->block_size += CMDQ_INST_SIZE;

	return 0;
}

int cmdq_rec_reset(struct cmdq_rec *handle)
{
	struct cmdq_context *cqctx = cmdq_rec_get_valid_ctx(handle);

	if (!cqctx)
		return -EFAULT;

	CMDQ_API("handle=0x%p pid=%d\n", handle, current->pid);

	if (handle->running_task_ptr)
		cmdq_rec_stop_running_task(handle);

	handle->block_size = 0;
	handle->prefetch_count = 0;
	handle->finalized = false;

	return 0;
}

static int cmdq_rec_mark(struct cmdq_rec *handle)
{
	int status;

	/*
	 * bit 53: non-suspendable. set to 1 because we don't want
	 * CPU suspend this thread during pre-fetching.
	 * If CPU change PC, then there will be a mess,
	 * because prefetch buffer is not properly cleared.
	 * bit 48: do not increase CMD_COUNTER
	 *         (because this is not the end of the task)
	 * bit 20: prefetch_marker
	 * bit 17: prefetch_marker_en
	 * bit 16: prefetch_en
	 * bit 0:  irq_en (set to 0 since we don't want EOC interrupt)
	 */
	status = cmdq_append_command(handle, CMDQ_CODE_EOC,
				     (0x1 << (53 - 32)) | (0x1 << (48 - 32)),
				     0x00130000);
	if (status)
		return status;

	/*
	 * if we're in a prefetch region,
	 * this ends the region so set count to 0.
	 * otherwise we start the region by setting count to 1.
	 */
	handle->prefetch_count = 1;

	return 0;
}

int cmdq_rec_write(struct cmdq_rec *handle, u32 value, u32 addr)
{
	struct cmdq_context *cqctx = cmdq_rec_get_valid_ctx(handle);

	if (!cqctx)
		return -EFAULT;

	CMDQ_API("handle=0x%p pid=%d\n", handle, current->pid);

	return cmdq_append_command(handle, CMDQ_CODE_WRITE, addr, value);
}

int cmdq_rec_write_mask(struct cmdq_rec *handle, u32 value,
			u32 addr, u32 mask)
{
	int status;
	struct cmdq_context *cqctx = cmdq_rec_get_valid_ctx(handle);

	if (!cqctx)
		return -EFAULT;

	CMDQ_API("handle=0x%p pid=%d\n", handle, current->pid);

	if (mask != 0xffffffff) {
		status = cmdq_append_command(handle, CMDQ_CODE_MOVE, 0, ~mask);
		if (status)
			return status;

		addr = addr | 0x1;
	}

	status = cmdq_append_command(handle, CMDQ_CODE_WRITE, addr, value);
	if (status)
		return status;

	return 0;
}

int cmdq_rec_wait(struct cmdq_rec *handle, enum cmdq_event event)
{
	struct cmdq_context *cqctx = cmdq_rec_get_valid_ctx(handle);

	if (!cqctx)
		return -EFAULT;

	CMDQ_API("handle=0x%p pid=%d\n", handle, current->pid);

	if (event == CMDQ_SYNC_TOKEN_INVALID || event >= CMDQ_SYNC_TOKEN_MAX ||
	    event < 0)
		return -EINVAL;

	return cmdq_append_command(handle, CMDQ_CODE_WFE, event, 0);
}

int cmdq_rec_disable_prefetch(struct cmdq_rec *handle)
{
	int status = 0;
	struct cmdq_context *cqctx = cmdq_rec_get_valid_ctx(handle);

	if (!cqctx)
		return -EFAULT;

	CMDQ_API("handle=0x%p pid=%d\n", handle, current->pid);

	if (!handle->finalized) {
		if (handle->prefetch_count > 0) {
			u32 arg_a, arg_b;

			/*
			 * with prefetch threads we should end with
			 * bit 48: no_inc_exec_cmds_cnt = 1
			 * bit 20: prefetch_mark = 1
			 * bit 17: prefetch_mark_en = 0
			 * bit 16: prefetch_en = 0
			 */
			arg_b = 0x00100000;
			arg_a = (0x1 << 16); /* not increse execute counter */
			/* since we're finalized, no more prefetch */
			handle->prefetch_count = 0;
			status = cmdq_append_command(handle, CMDQ_CODE_EOC,
						     arg_a, arg_b);
		}

		if (status)
			return status;
	}

	CMDQ_MSG("cmdq_rec_disable_prefetch, status:%d\n", status);
	return status;
}

static int cmdq_rec_finalize_command(struct cmdq_rec *handle)
{
	int status = 0;
	struct device *dev;

	if (!cmdq_rec_get_valid_ctx(handle))
		return -EFAULT;

	dev = handle->cqctx->dev;

	if (!handle->finalized) {
		u32 arg_b;

		if ((handle->prefetch_count > 0) &&
		    cmdq_core_should_enable_prefetch(handle->scenario)) {
			dev_err(dev, "not insert prefetch disble marker ");
			dev_err(dev, "if prefetch enabled, prefetch_count:%d\n",
				handle->prefetch_count);
			cmdq_rec_dump_command(handle);

			status = -EFAULT;
			return status;
		}

		/* insert EOF instruction */
		arg_b = 0x1; /* generate IRQ for each command iteration */
		status = cmdq_append_command(handle, CMDQ_CODE_EOC, 0, arg_b);
		if (status)
			return status;

		/* JUMP to begin */
		status = cmdq_append_command(handle, CMDQ_CODE_JUMP, 0, 8);
		if (status)
			return status;

		handle->finalized = true;
	}

	return status;
}

static int cmdq_rec_fill_cmd_desc(struct cmdq_rec *handle,
				  struct cmdq_command *desc)
{
	int ret;

	ret = cmdq_rec_finalize_command(handle);
	if (ret)
		return ret;

	desc->cqctx = handle->cqctx;
	desc->scenario = handle->scenario;
	desc->priority = handle->priority;
	desc->engine_flag = handle->engine_flag;
	desc->va_base = handle->buf_ptr;
	desc->block_size = handle->block_size;

	return ret;
}

int cmdq_rec_flush(struct cmdq_rec *handle)
{
	int ret;
	struct cmdq_command desc = { 0 };
	struct cmdq_context *cqctx = cmdq_rec_get_valid_ctx(handle);

	if (!cqctx)
		return -EFAULT;

	CMDQ_API("handle=0x%p pid=%d\n", handle, current->pid);

	ret = cmdq_rec_fill_cmd_desc(handle, &desc);
	if (ret)
		return ret;

	return cmdq_core_submit_task(&desc);
}

static int cmdq_rec_flush_async_cb(struct cmdq_rec *handle,
				   cmdq_async_flush_cb isr_cb,
				   void *isr_data,
				   cmdq_async_flush_cb done_cb,
				   void *done_data)
{
	int ret;
	struct cmdq_command desc = { 0 };
	struct cmdq_task *task;
	struct cmdq_task_cb cb;

	ret = cmdq_rec_fill_cmd_desc(handle, &desc);
	if (ret)
		return ret;

	cb.isr_cb = isr_cb;
	cb.isr_data = isr_data;
	cb.done_cb = done_cb;
	cb.done_data = done_data;

	ret = cmdq_core_submit_task_async(&desc, &task, &cb);
	if (ret)
		return ret;

	CMDQ_MSG("%s scenario: %d, priority: %d, engine: 0x%llx, ",
		 __func__, handle->scenario, handle->priority,
		 handle->engine_flag);
	CMDQ_MSG("buffer: 0x%p, size: %d\n",
		 handle->buf_ptr, handle->block_size);

	if (task)
		ret = cmdq_core_auto_release_task(task);
	else
		ret = -ENOMEM;

	return ret;
}

int cmdq_rec_flush_async(struct cmdq_rec *handle)
{
	struct cmdq_context *cqctx = cmdq_rec_get_valid_ctx(handle);

	if (!cqctx)
		return -EFAULT;

	CMDQ_API("handle=0x%p pid=%d\n", handle, current->pid);

	return cmdq_rec_flush_async_cb(handle, NULL, NULL, NULL, NULL);
}

int cmdq_rec_flush_async_callback(struct cmdq_rec *handle,
				  cmdq_async_flush_cb isr_cb,
				  void *isr_data,
				  cmdq_async_flush_cb done_cb,
				  void *done_data)
{
	struct cmdq_context *cqctx = cmdq_rec_get_valid_ctx(handle);

	if (!cqctx)
		return -EFAULT;

	CMDQ_API("handle=0x%p pid=%d\n", handle, current->pid);

	return cmdq_rec_flush_async_cb(handle, isr_cb, isr_data,
				       done_cb, done_data);
}

/* debug */
int cmdq_rec_dump_command(struct cmdq_rec *handle)
{
	int status = 0;
	struct cmdq_task *task;
	struct cmdq_context *cqctx = cmdq_rec_get_valid_ctx(handle);

	if (!cqctx)
		return -EFAULT;

	CMDQ_API("handle=0x%p pid=%d\n", handle, current->pid);

	task = handle->running_task_ptr;
	if (task) {
		/* running, so dump from core task direct */
		status = cmdq_core_debug_dump_command(task);
	} else {
		/* not running, dump from rec->buf_ptr */
		const u32 *cmd_ptr = NULL;
		static char text_buf[128] = { 0 };
		int i = 0;

		CMDQ_LOG("======REC 0x%p command buffer:\n", handle->buf_ptr);
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 16, 4,
			       handle->buf_ptr, handle->block_size, false);

		CMDQ_LOG("======REC 0x%p command buffer END\n",
			 handle->buf_ptr);
		CMDQ_LOG("REC 0x%p command buffer TRANSLATED:\n",
			 handle->buf_ptr);
		for (i = 0, cmd_ptr = handle->buf_ptr; i < handle->block_size;
		     i += 8, cmd_ptr += 2) {
			cmdq_core_parse_instruction(cmd_ptr, text_buf, 128);
			CMDQ_LOG("%s", text_buf);
		}
		CMDQ_LOG("======REC 0x%p command END\n", handle->buf_ptr);

		return 0;
	}

	return status;
}

void cmdq_rec_destroy(struct cmdq_rec *handle)
{
	struct cmdq_context *cqctx = cmdq_rec_get_valid_ctx(handle);

	if (!cqctx)
		return;

	CMDQ_API("handle=0x%p pid=%d\n", handle, current->pid);

	if (handle->running_task_ptr)
		cmdq_rec_stop_running_task(handle);

	/* free command buffer */
	kfree(handle->buf_ptr);
	handle->buf_ptr = NULL;

	/* free command handle */
	kfree(handle);
}

subsys_initcall(cmdq_init);
module_exit(cmdq_exit);

MODULE_DESCRIPTION("Mediatek CMDQ driver");
MODULE_AUTHOR("HS Liao <hs.liao@mediatek.com>");
MODULE_LICENSE("GPL");
