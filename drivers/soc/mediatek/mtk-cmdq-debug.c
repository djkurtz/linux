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

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/memory.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <soc/mediatek/cmdq.h>

#include "mtk-cmdq-debug.h"
#include "mtk-cmdq-private.h"

/* cmdq_print - begin */

#define MT_ALLOW_RT_PRIO_BIT		0x0
#define RTPM_PRIO_MM_GROUP_BASE		10

static int rtpm_prio_scrn_update(void)
{
	u32 rtpm_prio_mm_group_i = RTPM_PRIO_MM_GROUP_BASE + 80;
	u32 reg_rt_prio = (rtpm_prio_mm_group_i + 4) | MT_ALLOW_RT_PRIO_BIT;

	return reg_rt_prio;
}

static int cmdq_core_print_log_kthread(void *data)
{
	int i;
	char long_msg[CMDQ_LONGSTRING_MAX];
	u32 msg_offset;
	int msg_max_size;
	bool need_print_log;
	struct sched_param param = {.sched_priority = rtpm_prio_scrn_update() };
	struct cmdq_context *cqctx = data;

	sched_setscheduler(current, SCHED_RR, &param);

	while (1) {
		long_msg[0] = '\0';
		msg_offset = 0;
		msg_max_size = CMDQ_LONGSTRING_MAX;
		need_print_log = cmdq_core_should_print_msg(LOG_LEVEL_LOG);

		cmdq_core_long_string(need_print_log, long_msg, &msg_offset,
				      &msg_max_size, "task_count print,");

		for (i = 0; i < CMDQ_MAX_THREAD_COUNT; i++) {
			if (cqctx->task_count[i])
				cmdq_core_long_string(need_print_log,
						      long_msg,
						      &msg_offset,
						      &msg_max_size,
						      " Thread[%d]=%d,",
						      i,
						      cqctx->task_count[i]);
		}

		for (i = 0; i < CMDQ_MAX_THREAD_COUNT; i++) {
			if (cqctx->task_count_buf[i])
				cmdq_core_long_string(need_print_log,
						      long_msg,
						      &msg_offset,
						      &msg_max_size,
						      " thread %d task count = %d full,",
						      i,
						      cqctx->task_count_buf[i]);
		}

		for (i = 0; i < CMDQ_MAX_THREAD_COUNT; i++) {
			if (cqctx->occupied_thread[i])
				cmdq_core_long_string
					(need_print_log,
					 long_msg,
					 &msg_offset,
					 &msg_max_size,
					 " thread %d nextCookie = %d already has task,",
					 i,
					 cqctx->occupied_next_cookie[i]);
		}

		cmdq_core_long_string(need_print_log, long_msg, &msg_offset,
				      &msg_max_size, "end\n");

		if (msg_offset > 0)
			CMDQ_LOG("%s", long_msg);

		memset(cqctx->task_count, 0, sizeof(cqctx->task_count));
		memset(cqctx->task_count_buf, 0, sizeof(cqctx->task_count_buf));
		memset(cqctx->occupied_thread, 0,
		       sizeof(cqctx->occupied_thread));

		msleep(1000);

		if (kthread_should_stop())
			break;
	}

	return 0;
}

static int cmdq_core_enable_print_log(struct cmdq_print *cqp, bool enable)
{
	CMDQ_LOG("%s: enable=%d\n", __func__, enable);
	cqp->enable_print = enable;
	return queue_work(cqp->cmdq_print_wq, &cqp->cmdq_print_work);
}

void cmdq_core_run_print_log(struct work_struct *work_item)
{
	struct cmdq_print *cqp;
	struct cmdq_context *cqctx;

	cqp = container_of(work_item, struct cmdq_print, cmdq_print_work);
	cqctx = cqp->cqctx;
	CMDQ_LOG("%s:enable_print=%d\n", __func__, cqp->enable_print);

	cqctx->regular_count = cqp->enable_print;
	if (cqp->enable_print) {
		if (!cqctx->print_log_task) {
			cqctx->print_log_task =
			    kthread_create(cmdq_core_print_log_kthread, cqctx,
					   "cmdq_core_print_log_kthread");
			if (IS_ERR(cqctx->print_log_task))
				CMDQ_ERR("cannot create cqctx->print_log_task kthread\n");

			wake_up_process(cqctx->print_log_task);
		}
	} else {
		if (cqctx->print_log_task) {
			kthread_stop(cqctx->print_log_task);
			cqctx->print_log_task = NULL;
		}
	}
}

/* cmdq_print - end */

/* log level/interface - begin */

struct log_policy {
	enum log_level	log_level;
	bool		show_interface;
};

struct log_policy g_log_policy;

void cmdq_core_set_log_level(const enum log_level value)
{
	g_log_policy.log_level = value;
	CMDQ_DBG("set log_level to %d\n", value);
}

enum log_level cmdq_core_get_log_level(void)
{
	return g_log_policy.log_level;
}

bool cmdq_core_should_print_msg(enum log_level log_level)
{
	return g_log_policy.log_level <= log_level;
}

void cmdq_core_set_show_interface(const bool show)
{
	g_log_policy.show_interface = show;
}

bool cmdq_core_get_show_interface(void)
{
	return g_log_policy.show_interface;
}

static ssize_t cmdq_read_debugfs_level_proc(struct file *file,
					    char __user *page,
					    size_t count,
					    loff_t *ppos)
{
	char buf[50];

	memset(buf, 0, 50);
	snprintf(buf, 50, "debug level is: %d\n", cmdq_core_get_log_level());

	return simple_read_from_buffer(page, count, ppos, buf, strlen(buf));
}

static ssize_t cmdq_write_debugfs_level_proc(struct file *file,
					     const char __user *buf,
					     size_t count,
					     loff_t *ppos)
{
	int len;
	int value;
	char text_buf[15] = { 0 };
	long tmp;
	struct device_node *node =
		of_find_compatible_node(NULL, NULL, "mediatek,mt8173-gce");
	struct platform_device *pdev = of_find_device_by_node(node);
	struct cmdq_context *cqctx = platform_get_drvdata(pdev);

	if (count >= 15)
		return -EFAULT;

	len = count;
	if (copy_from_user(text_buf, buf, len))
		return -EFAULT;

	text_buf[len] = '\0';

	if (!strncmp(text_buf, "en_count", 8)) {
		CMDQ_LOG("en_count\n");
		cmdq_core_enable_print_log(&cqctx->print_log_struct, true);
		return len;
	} else if (!strncmp(text_buf, "dis_count", 9)) {
		CMDQ_LOG("dis_count\n");
		cmdq_core_enable_print_log(&cqctx->print_log_struct, false);
		return len;
	}

	if (kstrtol(text_buf, 0, &tmp) != 0) {
		CMDQ_ERR("set log_level error: %s\n", text_buf);
		return len;
	}

	value = tmp;
	if (value < LOG_LEVEL_MSG || value > LOG_LEVEL_MAX) {
		CMDQ_ERR("set log level error: %d\n", value);
		value = LOG_LEVEL_LOG;
	}
	cmdq_core_set_log_level((enum log_level)(value));
	return len;
}

static ssize_t cmdq_read_debugfs_api_proc(struct file *file,
					  char __user *page,
					  size_t count,
					  loff_t *ppos)
{
	char buffer[50];

	memset(buffer, 0, 50);
	snprintf(buffer, 50, "show interface is: %s\n",
		 cmdq_core_get_show_interface() ? "true" : "false");

	return simple_read_from_buffer(page, count, ppos, buffer,
				       strlen(buffer));
}

static ssize_t cmdq_write_debugfs_api_proc(struct file *file,
					   const char __user *buf,
					   size_t count,
					   loff_t *ppos)
{
	int value;
	char buffer[20];
	int length;

	memset(buffer, 0, 20);
	length = simple_write_to_buffer(buffer, 20, ppos, buf, count);
	buffer[19] = '\0';

	if (kstrtoint(buffer, 20, &value) < 0) {
		CMDQ_ERR("set show_interface error: %s\n", buffer);
		return 0;
	}

	cmdq_core_set_show_interface(value != 0);
	return length;
}

/* log level/interface - end */

/* record - begin */

#ifdef CMDQ_CORE_PROFILE_WAKE_UP
	#define TIME_PROFILE_NUM 6
#else
	#define TIME_PROFILE_NUM 5
#endif

static int cmdq_get_time_in_us_part(unsigned long long start,
				    unsigned long long end)
{
	unsigned long long _duration = end - start;

	do_div(_duration, 1000);
	return (int)_duration;
}

static int cmdq_core_print_record_internal(const struct cmdq_record *cqr,
					   int index, char *buf, int buf_len)
{
	int length;
	char *unit[TIME_PROFILE_NUM];
	int irq_time;
	int exec_time;
	int begin_wait_time;
	int total_time;
	int acquire_thread_time;
#ifdef CMDQ_CORE_PROFILE_WAKE_UP
	int wake_up_time;
#endif
	unsigned long rem_nsec;
	unsigned long long submit_time = cqr->submit;
	int i;

	rem_nsec = do_div(submit_time, 1000000000);

	for (i = 0; i < TIME_PROFILE_NUM; ++i)
		unit[i] = "ms";

	total_time = cmdq_get_time_in_ms(cqr->submit, cqr->done);
	acquire_thread_time = cmdq_get_time_in_ms(cqr->submit, cqr->trigger);
	begin_wait_time = cmdq_get_time_in_ms(cqr->submit, cqr->begin_wait);
#ifdef CMDQ_CORE_PROFILE_WAKE_UP
	wake_up_time = cmdq_get_time_in_ms(cqr->send_wake_up,
					   cqr->receive_wake_up);
#endif
	irq_time = cmdq_get_time_in_ms(cqr->trigger, cqr->got_irq);
	exec_time = cmdq_get_time_in_ms(cqr->trigger, cqr->waked_up);

	/* detect us interval */
	if (!acquire_thread_time) {
		acquire_thread_time = cmdq_get_time_in_us_part(cqr->submit,
							       cqr->trigger);
		unit[0] = "us";
	}
	if (!irq_time) {
		irq_time = cmdq_get_time_in_us_part(cqr->trigger, cqr->got_irq);
		unit[1] = "us";
	}
	if (!begin_wait_time) {
		begin_wait_time = cmdq_get_time_in_us_part(cqr->submit,
							   cqr->begin_wait);
		unit[2] = "us";
	}
	if (!exec_time) {
		exec_time = cmdq_get_time_in_us_part(cqr->trigger,
						     cqr->waked_up);
		unit[3] = "us";
	}
#ifdef CMDQ_CORE_PROFILE_WAKE_UP
	if (!wake_up_time) {
		wake_up_time = cmdq_get_time_in_us_part(cqr->send_wake_up,
							cqr->receive_wake_up);
		unit[4] = "us";
	}
#endif
	if (!total_time) {
		total_time = cmdq_get_time_in_us_part(cqr->submit, cqr->done);
		unit[TIME_PROFILE_NUM - 1] = "us";
	}
	length = snprintf(buf,
			  buf_len,
#ifdef CMDQ_CORE_PROFILE_WAKE_UP
			  "%4d:(%5d, %2d, 0x%012llx, %2d)(%02d, %02d)(%5dns : %lld, %lld)(%5llu.%06lu, %4d%s, %4d%s, %4d%s, %4d%s, %4d%s)%4d%s\n",
#else
			  "%4d:(%5d, %2d, 0x%012llx, %2d)(%02d, %02d)(%5dns : %lld, %lld)(%5llu.%06lu, %4d%s, %4d%s, %4d%s, %4d%s)%4d%s\n",
#endif
			  index, cqr->user, cqr->scenario,
			  cqr->engine_flag, cqr->priority,
			  cqr->thread,
			  /* currently, only support display */
			  CMDQ_THR_PRIO_DISPLAY_CONFIG,
			  cqr->write_time_ns,
			  cqr->write_time_ns_begin, cqr->write_time_ns_end,
			  submit_time, rem_nsec / 1000,
			  acquire_thread_time, unit[0],
			  irq_time, unit[1],
			  begin_wait_time, unit[2],
			  exec_time, unit[3],
#ifdef CMDQ_CORE_PROFILE_WAKE_UP
			  wake_up_time, unit[4],
#endif
			  total_time, unit[TIME_PROFILE_NUM - 1]);

	return length;
}

static int cmdq_core_print_record_seq(struct seq_file *m, void *v)
{
	unsigned long flags;
	int index;
	int num_rec;
	struct cmdq_record record;
	char msg[512] = { 0 };
	struct device_node *node =
		of_find_compatible_node(NULL, NULL, "mediatek,mt8173-gce");
	struct platform_device *pdev = of_find_device_by_node(node);
	struct cmdq_context *cqctx = platform_get_drvdata(pdev);

	/*
	 * we try to minimize time spent in spin lock
	 * since record is an array so it is okay to
	 * allow displaying an out-of-date entry.
	 */
	spin_lock_irqsave(&cqctx->record_lock, flags);
	num_rec = cqctx->rec_num;
	index = cqctx->last_id - 1;
	spin_unlock_irqrestore(&cqctx->record_lock, flags);

	/* we print record in reverse order. */
	for (; num_rec > 0; --num_rec, --index) {
		if (index >= CMDQ_MAX_RECORD_COUNT)
			index = 0;
		else if (index < 0)
			index = CMDQ_MAX_RECORD_COUNT - 1;

		/*
		 * Make sure we don't print a record that is during updating.
		 * However, this record may already be different
		 * from the time of entering cmdq_core_print_record_seq().
		 */
		spin_lock_irqsave(&cqctx->record_lock, flags);
		record = cqctx->record[index];
		spin_unlock_irqrestore(&cqctx->record_lock, flags);

		cmdq_core_print_record_internal(&record, index,
						msg, sizeof(msg));

		seq_printf(m, "%s", msg);
	}

	return 0;
}

/* record - end */

/* status - begin */

static void cmdq_core_print_thread_seq(struct seq_file *m,
				       struct cmdq_context *cqctx,
				       int tid)
{
	u32 value[10] = { 0 };
	void __iomem *gce_base_va = cqctx->cqdev.base_va;

	seq_puts(m, "trigger thread:\n");
	value[0] = readl(gce_base_va + CMDQ_THR_CURR_ADDR_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[1] = readl(gce_base_va + CMDQ_THR_END_ADDR_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[2] = readl(gce_base_va + CMDQ_THR_WAIT_TOKEN_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[3] = cmdq_get_cookie_cnt(gce_base_va, tid);
	value[4] = readl(gce_base_va + CMDQ_THR_IRQ_STATUS_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[5] = readl(gce_base_va + CMDQ_THR_INST_CYCLES_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[6] = readl(gce_base_va + CMDQ_THR_CURR_STATUS_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[7] = readl(gce_base_va + CMDQ_THR_IRQ_ENABLE_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[8] = readl(gce_base_va + CMDQ_THR_ENABLE_TASK_OFFSET +
			 CMDQ_THR_SHIFT * tid);

	seq_printf(m, "index: %d, enabled: %d, irq: 0x%08x, ",
		   tid, value[8], value[4]);
	seq_printf(m, "thread pc: 0x%08x, end: 0x%08x, wait token: 0x%08x, ",
		   value[0], value[1], value[2]);
	seq_printf(m, "timeout cycle:%d, status:0x%08x, irq_en: 0x%08x\n",
		   value[5], value[6], value[7]);
}

static void cmdq_core_print_status_seq_clk(struct seq_file *m,
					   struct cmdq_context *cqctx)
{
	seq_puts(m, "====== Clock =======\n");
	seq_printf(m, "MT_CG_INFRA_GCE: %d\n", cmdq_core_clock_is_on(cqctx));
}

static void cmdq_core_print_status_seq_status(struct seq_file *m,
					      struct cmdq_context *cqctx)
{
	int core_exec_thread;
	u32 value[6] = { 0 };
	void __iomem *gce_base_va = cqctx->cqdev.base_va;

	seq_puts(m, "====== Status =======\n");

	value[0] = readl(gce_base_va + CMDQ_CURR_LOADED_THR_OFFSET);
	value[1] = readl(gce_base_va + CMDQ_THR_EXEC_CYCLES_OFFSET);
	value[2] = readl(gce_base_va + CMDQ_THR_TIMEOUT_TIMER_OFFSET);
	value[3] = readl(gce_base_va + CMDQ_BUS_CONTROL_TYPE_OFFSET);
	/* this returns (1 + index of least bit set) or 0 if input is 0. */
	core_exec_thread = __builtin_ffs(value[0]) - 1;
	seq_printf(m, "IRQ flag:0x%08x, Execing:%d, Exec Thread:%d, CMDQ_CURR_LOADED_THR: 0x%08x\n",
		   readl(gce_base_va + CMDQ_CURR_IRQ_STATUS_OFFSET),
		   (0x80000000 & value[0]) ? 1 : 0,
		   core_exec_thread, value[0]);
	if (core_exec_thread != CMDQ_INVALID_THREAD) {
		seq_printf(m, "exec thread %d dump:\n", core_exec_thread);
		cmdq_core_print_thread_seq(m, cqctx, core_exec_thread);
	}
	seq_printf(m, "CMDQ_THR_EXEC_CYCLES:0x%08x, CMDQ_THR_TIMER:0x%08x, CMDQ_BUS_CTRL:0x%08x\n",
		   value[1], value[2], value[3]);
	seq_printf(m, "CMDQ_DEBUG_1: 0x%08x\n", readl(gce_base_va + 0xf0));
	seq_printf(m, "CMDQ_DEBUG_2: 0x%08x\n", readl(gce_base_va + 0xf4));
	seq_printf(m, "CMDQ_DEBUG_3: 0x%08x\n", readl(gce_base_va + 0xf8));
	seq_printf(m, "CMDQ_DEBUG_4: 0x%08x\n", readl(gce_base_va + 0xfc));
}

static void cmdq_core_print_status_seq_task(struct seq_file *m,
					    struct cmdq_context *cqctx)
{
	int list_idx;
	const struct list_head *lists[] = {
		&cqctx->task_free_list,
		&cqctx->task_active_list,
		&cqctx->task_wait_list
	};

	/* print all tasks in both list */
	for (list_idx = 0; list_idx < ARRAY_SIZE(lists); list_idx++) {
		int index;
		struct list_head *p;
		const char *const list_names[] = {
			"Free",
			"Active",
			"Wait"
		};

		/* skip free tasks by default, only LOG_LEVEL_MSG show it */
		if (cmdq_core_get_log_level() >= LOG_LEVEL_LOG &&
		    list_idx == 0)
			continue;

		index = 0;
		list_for_each(p, lists[list_idx]) {
			struct cmdq_task *task;

			task = list_entry(p, struct cmdq_task, list_entry);
			seq_printf(m, "====== %s Task(%d) 0x%p Usage =======\n",
				   list_names[list_idx], index, task);

			seq_printf(m, "State %d, VABase: 0x%p, mva_base: 0x%p, Size: %d\n",
				   task->task_state, task->va_base,
				   &task->mva_base, task->command_size);
			seq_printf(m, "Scenario %d, Priority: %d, Flag: 0x%08llx, Num: %d\n",
				   task->scenario, task->priority,
				   task->engine_flag, task->num_cmd);
			seq_printf(m, "Reorder:%d, Trigger %lld, IRQ: %lld, Wait: %lld, Wake Up: %lld\n",
				   task->reorder, task->trigger, task->got_irq,
				   task->begin_wait, task->waked_up);
			++index;
		}
		seq_printf(m, "====== Total %d %s Task =======\n",
			   index, list_names[list_idx]);
	}
}

static void cmdq_core_print_status_seq_thread_task(struct seq_file *m,
						   struct cmdq_context *cqctx,
						   struct cmdq_thread *thread,
						   int index)
{
	unsigned long flags = 0;
	int inner;

	spin_lock_irqsave(&cqctx->thread_lock, flags);

	for (inner = 0;
	     inner < CMDQ_MAX_TASK_IN_THREAD;
	     inner++) {
		struct cmdq_task *task;

		task = thread->cur_task[inner];
		if (!task)
			continue;
		/* dump task basic info */
		seq_printf(m, "Slot: %d, Task: 0x%p, Pid: %d, Name: %s, Scn: %d, VABase: 0x%p, mva_base: 0x%p, Size: %d, Last Command: 0x%08x:0x%08x\n",
			   index, task, task->caller_pid,
			   task->caller_name, task->scenario,
			   task->va_base, &task->mva_base,
			   task->command_size,
			   task->va_base[task->num_cmd-2],
			   task->va_base[task->num_cmd-1]);

		/* dump PC info */
		do {
			u32 *pc_va = NULL;
			u32 insts[2] = { 0 };
			char parsed_inst[128] = { 0 };

			pc_va = cmdq_core_get_pc(task, index, insts);
			if (pc_va) {
				cmdq_core_parse_instruction
					(pc_va, parsed_inst,
					 sizeof(parsed_inst));
				seq_printf(m, "PC(VA): 0x%p, 0x%08x:0x%08x => %s",
					   pc_va, insts[0], insts[1],
					   parsed_inst);
			} else {
				seq_puts(m, "PC(VA): Not available\n");
			}
		} while (0);
	}
	spin_unlock_irqrestore(&cqctx->thread_lock, flags);
}

static void cmdq_core_print_status_seq_thread(struct seq_file *m,
					      struct cmdq_context *cqctx)
{
	int index;

	for (index = 0; index < CMDQ_MAX_THREAD_COUNT; index++) {
		struct cmdq_thread *thread;

		thread = &cqctx->thread[index];

		if (thread->task_count > 0) {
			seq_printf(m, "====== Thread %d Usage =======\n",
				   index);
			seq_printf(m, "Wait Cookie %d, Next Cookie %d\n",
				   thread->wait_cookie, thread->next_cookie);
			cmdq_core_print_status_seq_thread_task(m, cqctx,
							       thread, index);
		}
	}
}

static int cmdq_core_print_status_seq(struct seq_file *m, void *v)
{
	struct device_node *node =
		of_find_compatible_node(NULL, NULL, "mediatek,mt8173-gce");
	struct platform_device *pdev = of_find_device_by_node(node);
	struct cmdq_context *cqctx = platform_get_drvdata(pdev);

	cmdq_core_print_status_seq_clk(m, cqctx);
	cmdq_core_print_status_seq_status(m, cqctx);
	cmdq_core_print_event_seq(m, cqctx);

	mutex_lock(&cqctx->task_mutex);
	cmdq_core_print_status_seq_task(m, cqctx);
	cmdq_core_print_status_seq_thread(m, cqctx);
	mutex_unlock(&cqctx->task_mutex);

	return 0;
}

/* status - end */

/* dump - begin */

int cmdq_core_parse_instruction(const u32 *cmd, char *text_buf, int buf_len)
{
	int req_len;
	const u32 op = (cmd[1] & 0xff000000) >> 24;
	const u32 arg_a = cmd[1] & (~0xff000000);
	const u32 arg_b = cmd[0];

	switch (op) {
	case CMDQ_CODE_MOVE:
		if (1 & (arg_a >> 23)) {
			req_len = snprintf(text_buf, buf_len,
					   "MOVE: 0x%08x to R%d\n",
					   arg_b, (arg_a >> 16) & 0x1f);
		} else {
			req_len = snprintf(text_buf, buf_len,
					   "MASK: 0x%08x\n", arg_b);
		}
		break;
	case CMDQ_CODE_READ:
	case CMDQ_CODE_WRITE:
		req_len = snprintf(text_buf, buf_len,
				   "%s: ", cmdq_core_parse_op(op));
		buf_len -= req_len;
		text_buf += req_len;

		/* data (value) */
		if (arg_a & (1 << 22)) {
			req_len = snprintf(text_buf, buf_len, "R%d, ", arg_b);
			buf_len -= req_len;
			text_buf += req_len;
		} else {
			req_len = snprintf(text_buf, buf_len,
					   "0x%08x, ", arg_b);
			buf_len -= req_len;
			text_buf += req_len;
		}

		/* address */
		if (arg_a & (1 << 23)) {
			req_len = snprintf(text_buf, buf_len,
					   "R%d\n", (arg_a >> 16) & 0x1f);
			buf_len -= req_len;
			text_buf += req_len;
		} else {
			u32 addr;
			const char *module;

			addr = cmdq_core_subsys_to_reg_addr(arg_a);
			module = cmdq_core_parse_module_from_reg_addr(addr);
			req_len = snprintf(text_buf, buf_len,
					   "addr=0x%08x [%s],use_mask=%d\n",
					   (addr & 0xfffffffe),
					   module, (addr & 0x1));
			buf_len -= req_len;
			text_buf += req_len;
		}
		break;
	case CMDQ_CODE_JUMP:
		if (arg_a) {
			if (arg_a & (1 << 22)) {
				/* jump by register */
				req_len = snprintf(text_buf, buf_len,
						   "JUMP(REG): R%d\n",
						   arg_b);
			} else {
				/* absolute */
				req_len = snprintf(text_buf, buf_len,
						   "JUMP(ABS): 0x%08x\n",
						   arg_b);
			}
		} else {
			/* relative */
			if ((int)arg_b >= 0) {
				req_len = snprintf(text_buf, buf_len,
						   "JUMP(REL): +%d\n",
						   (int)arg_b);
			} else {
				req_len = snprintf(text_buf, buf_len,
						   "JUMP(REL): %d\n",
						   (int)arg_b);
			}
		}
		break;
	case CMDQ_CODE_WFE:
		if (arg_b == 0x80008001) {
			req_len = snprintf(text_buf, buf_len, "WFE: %s\n",
					   cmdq_event_get_name(arg_a));
		} else {
			req_len = snprintf(text_buf, buf_len,
					   "SYNC: %s, upd=%d, op=%d, val=%d, wait=%d, wop=%d, val=%d\n",
					   cmdq_event_get_name(arg_a),
					   (arg_b >> 31) & 0x1,
					   (arg_b >> 28) & 0x7,
					   (arg_b >> 16) & 0xfff,
					   (arg_b >> 15) & 0x1,
					   (arg_b >> 12) & 0x7,
					   (arg_b >> 0) & 0xfff);
		}
		break;
	case CMDQ_CODE_EOC:
		if (arg_a == 0 && arg_b == 0x00000001) {
			req_len = snprintf(text_buf, buf_len, "EOC\n");
		} else {
			if (cmdq_core_support_sync_non_suspendable()) {
				req_len = snprintf(text_buf, buf_len,
						   "MARKER: sync_no_suspnd=%d",
						   (arg_a & (1 << 20)) > 0);
			} else {
				req_len = snprintf(text_buf, buf_len,
						   "MARKER:");
			}

			req_len = snprintf(text_buf, buf_len,
					   "no_suspnd=%d, no_inc=%d, m=%d, m_en=%d, prefetch=%d, irq=%d\n",
					   (arg_a & (1 << 21)) > 0,
					   (arg_a & (1 << 16)) > 0,
					   (arg_b & (1 << 20)) > 0,
					   (arg_b & (1 << 17)) > 0,
					   (arg_b & (1 << 16)) > 0,
					   (arg_b & (1 << 0)) > 0);
		}
		break;
	default:
		req_len = snprintf(text_buf, buf_len, "UNDEFINED\n");
		break;
	}

	return req_len;
}

static void cmdq_core_print_event_info(void __iomem *gce_base_va,
				       u32 inst, const char *tag)
{
	u32 reg_value;
	const u32 event_id = 0x3ff & inst;

	writel(event_id, gce_base_va + CMDQ_SYNC_TOKEN_ID_OFFSET);
	reg_value = readl(gce_base_va + CMDQ_SYNC_TOKEN_VAL_OFFSET);
	CMDQ_LOG("[%s]CMDQ_SYNC_TOKEN_VAL of %s is %d\n", tag,
		 cmdq_event_get_name(event_id), reg_value);
}

u32 *cmdq_core_dump_pc(const struct cmdq_task *task, int thread,
		       const char *tag)
{
	u32 *pc_va;
	u32 insts[2] = { 0 };
	char parsed_inst[128] = { 0 };

	pc_va = cmdq_core_get_pc(task, thread, insts);
	if (pc_va) {
		const u32 op = (insts[1] & 0xff000000) >> 24;
		struct cmdq_context *cqctx = task->cqctx;
		void __iomem *gce_base_va = cqctx->cqdev.base_va;

		cmdq_core_parse_instruction(pc_va, parsed_inst,
					    sizeof(parsed_inst));
		CMDQ_LOG("[%s]Thread %d PC(VA): 0x%p, 0x%08x:0x%08x => %s",
			 tag, thread, pc_va, insts[0], insts[1], parsed_inst);

		/* for WFE, we specifically dump the event value */
		if (op == CMDQ_CODE_WFE)
			cmdq_core_print_event_info(gce_base_va, insts[1], tag);
	} else {
		CMDQ_LOG("[%s]Thread %d PC(VA): Not available\n", tag, thread);
	}

	return pc_va;
}

void cmdq_core_dump_status(void __iomem *gce_base_va, const char *tag)
{
	int core_exec_thread;
	u32 value[6] = { 0 };

	value[0] = readl(gce_base_va + CMDQ_CURR_LOADED_THR_OFFSET);
	value[1] = readl(gce_base_va + CMDQ_THR_EXEC_CYCLES_OFFSET);
	value[2] = readl(gce_base_va + CMDQ_THR_TIMEOUT_TIMER_OFFSET);
	value[3] = readl(gce_base_va + CMDQ_BUS_CONTROL_TYPE_OFFSET);
	/* this returns (1 + index of least bit set) or 0 if input is 0. */
	core_exec_thread = __builtin_ffs(value[0]) - 1;
	CMDQ_LOG("[%s]IRQ flag:0x%08x, Execing:%d, Exec Thread:%d, CMDQ_CURR_LOADED_THR: 0x%08x\n",
		 tag,
		 readl(gce_base_va + CMDQ_CURR_IRQ_STATUS_OFFSET),
		 (0x80000000 & value[0]) ? 1 : 0,
		 core_exec_thread,
		 value[0]);
	CMDQ_LOG("[%s]CMDQ_THR_EXEC_CYCLES:0x%08x, CMDQ_THR_TIMER:0x%08x, CMDQ_BUS_CTRL:0x%08x\n",
		 tag, value[1], value[2], value[3]);
	CMDQ_LOG("[%s]CMDQ_DEBUG_1: 0x%08x\n", tag, readl(gce_base_va + 0xf0));
	CMDQ_LOG("[%s]CMDQ_DEBUG_2: 0x%08x\n", tag, readl(gce_base_va + 0xf4));
	CMDQ_LOG("[%s]CMDQ_DEBUG_3: 0x%08x\n", tag, readl(gce_base_va + 0xf8));
	CMDQ_LOG("[%s]CMDQ_DEBUG_4: 0x%08x\n", tag, readl(gce_base_va + 0xfc));
}

void cmdq_core_dump_thread_pc(struct cmdq_context *cqctx, const int tid)
{
	int i;
	struct cmdq_thread *cq_td;
	struct cmdq_task *cq_tsk;
	u32 *pc_va;
	u32 insts[2] = { 0 };
	char parsed_inst[128] = { 0 };
	void __iomem *gce_base_va = cqctx->cqdev.base_va;

	if (tid == CMDQ_INVALID_THREAD)
		return;

	cq_td = &cqctx->thread[tid];
	pc_va = NULL;

	for (i = 0; i < CMDQ_MAX_TASK_IN_THREAD; i++) {
		cq_tsk = cq_td->cur_task[i];

		if (cq_tsk) {
			pc_va = cmdq_core_get_pc(cq_tsk, tid, insts);
			if (pc_va) {
				const u32 op =
					(insts[1] & 0xff000000) >> 24;

				cmdq_core_parse_instruction(pc_va, parsed_inst,
							    sizeof(parsed_inst)
							    );
				CMDQ_LOG("[INFO]task:0x%p(index:%d), Thread %d PC(VA): 0x%p, 0x%08x:0x%08x => %s",
					 cq_tsk, i, tid, pc_va,
					 insts[0], insts[1], parsed_inst);

				/*
				 * for WFE,
				 * we specifically dump the event value
				 */
				if (op == CMDQ_CODE_WFE)
					cmdq_core_print_event_info
						(gce_base_va, insts[1], "INFO");
				break;
			}
		}
	}
}

void cmdq_core_dump_task(const struct cmdq_task *task)
{
	CMDQ_ERR("Task: 0x%p, Scenario: %d, State: %d, Priority: %d, Flag: 0x%016llx, VABase: 0x%p\n",
		 task, task->scenario, task->task_state, task->priority,
		 task->engine_flag, task->va_base);

	/*
	 * dump last Inst only when VALID command buffer
	 * otherwise data abort is happened
	 */
	if (task->va_base) {
		CMDQ_ERR("CMDEnd: 0x%p, mva_base: 0x%p, Size: %d, Last Inst: 0x%08x:0x%08x, 0x%08x:0x%08x\n",
			 task->va_base + task->num_cmd - 1,
			 &task->mva_base, task->command_size,
			 task->va_base[task->num_cmd-4],
			 task->va_base[task->num_cmd-3],
			 task->va_base[task->num_cmd-2],
			 task->va_base[task->num_cmd-1]);
	} else {
		CMDQ_ERR("CMDEnd: 0x%p, mva_base: 0x%p, Size: %d\n",
			 task->va_base + task->num_cmd - 1,
			 &task->mva_base, task->command_size);
	}

	CMDQ_ERR("Reorder:%d, Trigger: %lld, Got IRQ: %lld, Wait: %lld, Finish: %lld\n",
		 task->reorder, task->trigger, task->got_irq,
		 task->begin_wait, task->waked_up);
	CMDQ_ERR("Caller pid:%d name:%s\n",
		 task->caller_pid, task->caller_name);
}

void cmdq_core_dump_task_with_engine_flag(struct cmdq_context *cqctx,
					  u64 engine_flag)
{
	struct cmdq_task *cq_tsk;
	struct list_head *p;

	CMDQ_ERR("=============== [CMDQ] All active tasks sharing same engine flag 0x%08llx===============\n",
		 engine_flag);

	list_for_each(p, &cqctx->task_active_list) {
		cq_tsk = list_entry(p, struct cmdq_task, list_entry);
		if (cq_tsk && (engine_flag & cq_tsk->engine_flag)) {
			CMDQ_ERR("Thr %d, Task: 0x%p, VABase: 0x%p, mva_base 0x%pa, Size: %d, Flag: 0x%08llx,",
				 cq_tsk->thread, cq_tsk, cq_tsk->va_base,
				 &cq_tsk->mva_base, cq_tsk->command_size,
				 cq_tsk->engine_flag);
			CMDQ_ERR(" Last Inst 0x%08x:0x%08x, 0x%08x:0x%08x\n",
				 cq_tsk->va_base[cq_tsk->num_cmd-4],
				 cq_tsk->va_base[cq_tsk->num_cmd-3],
				 cq_tsk->va_base[cq_tsk->num_cmd-2],
				 cq_tsk->va_base[cq_tsk->num_cmd-1]);
		}
	}
}

void cmdq_core_dump_task_in_thread(struct cmdq_context *cqctx,
				   const int tid,
				   const bool dump_full_task,
				   const bool dump_cookie,
				   const bool dump_cmd)
{
	struct cmdq_thread *cq_td;
	struct cmdq_task *cq_tsk;
	int index;
	u32 value[4] = { 0 };
	u32 cookie;
	char long_msg[CMDQ_LONGSTRING_MAX];
	u32 msg_offset;
	int msg_max_size;
	void __iomem *gce_base_va = cqctx->cqdev.base_va;

	if (tid == CMDQ_INVALID_THREAD)
		return;

	cq_td = &cqctx->thread[tid];
	cq_tsk = NULL;

	CMDQ_ERR("=============== [CMDQ] All Task in Error Thread %d ===============\n",
		 tid);
	cookie = cmdq_get_cookie_cnt(gce_base_va, tid);
	if (dump_cookie)
		CMDQ_ERR("Curr Cookie: %d, Wait Cookie: %d, Next Cookie: %d, Task Count %d\n",
			 cookie, cq_td->wait_cookie, cq_td->next_cookie,
			 cq_td->task_count);

	for (index = 0; index < CMDQ_MAX_TASK_IN_THREAD; index++) {
		cq_tsk = cq_td->cur_task[index];
		if (!cq_tsk)
			continue;

		if (dump_full_task) {
			CMDQ_ERR("Slot %d, Task: 0x%p\n", index, cq_tsk);
			cmdq_core_dump_task(cq_tsk);

			if (dump_cmd)
				print_hex_dump(KERN_ERR, "",
					       DUMP_PREFIX_ADDRESS, 16, 4,
					       cq_tsk->va_base,
					       (cq_tsk->command_size), true);

			continue;
		}

		/* simple dump task info */
		if (!cq_tsk->va_base) {
			value[0] = 0xbcbcbcbc;
			value[1] = 0xbcbcbcbc;
			value[2] = 0xbcbcbcbc;
			value[3] = 0xbcbcbcbc;
		} else {
			value[0] = cq_tsk->va_base[cq_tsk->num_cmd-4];
			value[1] = cq_tsk->va_base[cq_tsk->num_cmd-3];
			value[2] = cq_tsk->va_base[cq_tsk->num_cmd-2];
			value[3] = cq_tsk->va_base[cq_tsk->num_cmd-1];
		}

		long_msg[0] = '\0';
		msg_offset = 0;
		msg_max_size = CMDQ_LONGSTRING_MAX;
		cmdq_core_long_string(true, long_msg,
				      &msg_offset, &msg_max_size,
				      "Slot %d, Task: 0x%p, VABase: 0x%p, mva_base 0x%p, Size: %d,",
				      index, cq_tsk, cq_tsk->va_base,
				      &cq_tsk->mva_base, cq_tsk->command_size);
		cmdq_core_long_string(true, long_msg,
				      &msg_offset, &msg_max_size,
				      " Last Inst 0x%08x:0x%08x, 0x%08x:0x%08x, priority:%d\n",
				      value[0], value[1], value[2], value[3],
				      cq_tsk->priority);

		if (msg_offset > 0) /* print message */
			CMDQ_ERR("%s", long_msg);

		if (dump_cmd)
			print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 16, 4,
				       cq_tsk->va_base, (cq_tsk->command_size),
				       true);
	}
}

void cmdq_core_dump_instructions(u64 *instr_buffer, u32 buf_size,
				 enum log_level level)
{
	u64 *buf_ptr = instr_buffer;
	char text_buffer[100];
	int i = 0;
	char *level_str;

	while (buf_ptr <= (u64 *)((u8 *)(instr_buffer) + buf_size)) {
		switch (level) {
		case LOG_LEVEL_ERR:
			level_str = "ERR";
			break;
		case LOG_LEVEL_LOG:
			level_str = "LOG";
			break;
		case LOG_LEVEL_MSG:
		default:
			level_str = "MSG";
			break;
		}
		memset(text_buffer, 0, 100);
		cmdq_core_parse_instruction((u32 *)buf_ptr, text_buffer, 99);
		if (cmdq_core_should_print_msg(level))
			pr_err("[CMDQ][%s]index:%05d,INST:0x%016llx    ----->      %s",
			       level_str, i, *buf_ptr, text_buffer);
		buf_ptr++;
		i++;
	}
}

int cmdq_core_debug_dump_command(struct cmdq_task *task)
{
	const u32 *cmd;
	static char text_buf[128] = { 0 };
	int i;
	struct cmdq_context *cqctx;

	if (!task)
		return -EFAULT;

	cqctx = task->cqctx;

	mutex_lock(&cqctx->task_mutex);

	CMDQ_LOG("======TASK 0x%p command buffer:\n", task);
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 16, 4,
		       task->va_base, task->command_size, false);
	CMDQ_LOG("======TASK 0x%p command buffer END\n", task);
	CMDQ_LOG("TASK 0x%p command buffer TRANSLATED:\n", task);
	for (i = 0, cmd = task->va_base; i < task->command_size;
	     i += 8, cmd += 2) {
		cmdq_core_parse_instruction(cmd, text_buf, 128);
		CMDQ_LOG("%s", text_buf);
	}
	CMDQ_LOG("======TASK 0x%p command END\n", task);

	mutex_unlock(&cqctx->task_mutex);

	return 0;
}

void cmdq_core_dump_error_task(const struct cmdq_task *task, int tid)
{
	struct cmdq_thread *cq_td;
	u32 value[10] = { 0 };
	struct cmdq_context *cqctx = task->cqctx;
	void __iomem *gce_base_va = cqctx->cqdev.base_va;

	cq_td = &cqctx->thread[tid];

	CMDQ_ERR("====== [CMDQ] Begin of Error %d task[0x%p] trigger[%lld] ======\n",
		 cqctx->err_num, task, task->trigger);

	CMDQ_ERR("=============== [CMDQ] Error Thread Status ===============\n");
	value[0] = readl(gce_base_va + CMDQ_THR_CURR_ADDR_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[1] = readl(gce_base_va + CMDQ_THR_END_ADDR_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[2] = readl(gce_base_va + CMDQ_THR_WAIT_TOKEN_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[3] = cmdq_get_cookie_cnt(gce_base_va, tid);
	value[4] = readl(gce_base_va + CMDQ_THR_IRQ_STATUS_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[5] = readl(gce_base_va + CMDQ_THR_INST_CYCLES_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[6] = readl(gce_base_va + CMDQ_THR_CURR_STATUS_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[7] = readl(gce_base_va + CMDQ_THR_IRQ_ENABLE_OFFSET +
			 CMDQ_THR_SHIFT * tid);
	value[8] = readl(gce_base_va + CMDQ_THR_ENABLE_TASK_OFFSET +
			 CMDQ_THR_SHIFT * tid);

	CMDQ_ERR("Index: %d, Enabled: %d, IRQ: 0x%08x, Thread PC: 0x%08x, End: 0x%08x, Wait Token: 0x%08x\n",
		 tid, value[8], value[4], value[0], value[1], value[2]);
	CMDQ_ERR("Curr Cookie: %d, Wait Cookie: %d, Next Cookie: %d, Task Count %d,\n",
		 value[3], cq_td->wait_cookie, cq_td->next_cookie,
		 cq_td->task_count);
	CMDQ_ERR("Timeout Cycle:%d, Status:0x%08x, IRQ_EN: 0x%08x\n",
		 value[5], value[6], value[7]);

	CMDQ_ERR("=============== [CMDQ] CMDQ Status ===============\n");
	cmdq_core_dump_status(gce_base_va, "ERR");

	if (task) {
		u32 *hw_pc;

		CMDQ_ERR("=============== [CMDQ] Error Thread PC Task[0x%p] trigger[%lld]===============\n",
			 task, task->trigger);
		hw_pc = cmdq_core_dump_pc(task, tid, "ERR");

		CMDQ_ERR("=============== [CMDQ] Begin of Error Task[0x%p] trigger[%lld] Status ===============\n",
			 task, task->trigger);
		cmdq_core_dump_task(task);
		cmdq_core_dump_instructions((u64 *)task->va_base,
					    task->command_size,
					    LOG_LEVEL_ERR);

		CMDQ_ERR("=============== [CMDQ] End of Error Task[0x%p] trigger[%lld] Status ===============\n",
			 task, task->trigger);
	}

	/* dump tasks in error thread */
	cmdq_core_dump_task_in_thread(cqctx, tid, false, false, false);
	cmdq_core_dump_task_with_engine_flag(cqctx, task->engine_flag);

	CMDQ_ERR("====== [CMDQ] End of Error %d task[0x%p] trigger[%lld] ======\n",
		 cqctx->err_num, task, task->trigger);
}

/* dump - end */

/* debugfs - begin */

static int cmdq_debugfs_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdq_core_print_status_seq, inode->i_private);
}

static int cmdq_debugfs_record_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdq_core_print_record_seq, inode->i_private);
}

static const struct file_operations cmdq_debug_status_op = {
	.owner = THIS_MODULE,
	.open = cmdq_debugfs_status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations cmdq_debug_record_op = {
	.owner = THIS_MODULE,
	.open = cmdq_debugfs_record_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations cmdq_debug_level_op = {
	.owner = THIS_MODULE,
	.read = cmdq_read_debugfs_level_proc,
	.write = cmdq_write_debugfs_level_proc
};

static const struct file_operations cmdq_debug_api_op = {
	.owner = THIS_MODULE,
	.read = cmdq_read_debugfs_api_proc,
	.write = cmdq_write_debugfs_api_proc
};

int cmdq_create_debugfs_entries(void)
{
	struct dentry *cmdq_debugfs_dir_entry = NULL;

	cmdq_debugfs_dir_entry =
		debugfs_create_dir(CMDQ_DRIVER_DEVICE_NAME, NULL);
	if (!cmdq_debugfs_dir_entry) {
		CMDQ_ERR("create CMDQ debugfs dir failed\n");
		return 0;
	}

	if (!debugfs_create_file("status", 0440, cmdq_debugfs_dir_entry,
				 NULL, &cmdq_debug_status_op))
		CMDQ_ERR("CMDQ debugfs create 'status' failed\n");

	if (!debugfs_create_file("record", 0440, cmdq_debugfs_dir_entry,
				 NULL, &cmdq_debug_record_op))
		CMDQ_ERR("CMDQ debugfs create 'record' failed\n");

	if (!debugfs_create_file("log_level", 0660,
				 cmdq_debugfs_dir_entry, NULL,
				 &cmdq_debug_level_op))
		CMDQ_ERR("CMDQ debugfs create 'log_level' failed\n");

	if (!debugfs_create_file("log_interface", 0660,
				 cmdq_debugfs_dir_entry, NULL,
				 &cmdq_debug_api_op))
		CMDQ_ERR("CMDQ debugfs create 'log_interface' failed\n");

	return 0;
}

/* debugfs - end */
