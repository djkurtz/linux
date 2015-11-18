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

#ifndef __MTK_CMDQ_DEBUG_H__
#define __MTK_CMDQ_DEBUG_H__

#include <linux/types.h>
#include <linux/workqueue.h>
#include <soc/mediatek/cmdq.h>

#define CMDQ_CORE_DEFAULT_LOG_LEVEL LOG_LEVEL_ERR
#define CMDQ_CORE_DEFAULT_SHOW_API false
/* #define CMDQ_CORE_PROFILE_WAKE_UP */
#define CMDQ_CORE_RECORD_MIN_TIME 0 /* e.g. 17000000 -> 17ms */

#define CMDQ_LONGSTRING_MAX 512

#define CMDQ_LOG(string, args...)					\
	do {								\
		if (cmdq_core_should_print_msg(LOG_LEVEL_LOG)) {	\
			pr_err("[CMDQ][LOG]"string, ##args);		\
		}							\
	} while (0)

#define CMDQ_MSG(string, args...)					\
	do {								\
		if (cmdq_core_should_print_msg(LOG_LEVEL_MSG)) {	\
			pr_warn("[CMDQ][MSG]"string, ##args);		\
		}							\
	} while (0)

#define CMDQ_ERR(string, args...)					\
	do {								\
		if (cmdq_core_should_print_msg(LOG_LEVEL_ERR)) {	\
			pr_err("[CMDQ][ERR]"string, ##args);		\
		}							\
	} while (0)

#define CMDQ_DBG(string, args...)					\
	do {								\
		if (1) {						\
			pr_err("[CMDQ][DBG]"string, ##args);		\
		}							\
	} while (0)

#define CMDQ_API(string, args...)					\
	do {								\
		if (cmdq_core_get_show_interface()) {			\
			pr_err("[CMDQ][API]%s "string,			\
			__func__, ##args);				\
		}							\
	} while (0)

/*
 * for CMDQ_LOG_LEVEL
 * set log level to 0 to enable all logs
 * set log level to 3 to close all logs
 */
enum log_level {
	LOG_LEVEL_MSG = 0,
	LOG_LEVEL_LOG = 1,
	LOG_LEVEL_ERR = 2,
	LOG_LEVEL_MAX,
};

struct cmdq_context;
struct cmdq_rec;
struct cmdq_task;

void cmdq_core_set_log_level(const enum log_level value);
enum log_level cmdq_core_get_log_level(void);
bool cmdq_core_should_print_msg(const enum log_level log_level);
void cmdq_core_set_show_interface(const bool show);
bool cmdq_core_get_show_interface(void);

int cmdq_create_debugfs_entries(void);
void cmdq_core_run_print_log(struct work_struct *work_item);
int cmdq_core_debug_dump_command(struct cmdq_task *task);
u32 *cmdq_core_dump_pc(const struct cmdq_task *task, int thread,
		       const char *tag);
void cmdq_core_dump_status(void __iomem *gce_base_va, const char *tag);
void cmdq_core_dump_thread_pc(struct cmdq_context *cqctx, const int tid);
void cmdq_core_dump_task(const struct cmdq_task *task);
void cmdq_core_dump_task_with_engine_flag(struct cmdq_context *cqctx,
					  u64 engine_flag);
void cmdq_core_dump_task_in_thread(struct cmdq_context *cqctx,
				   const int tid,
				   const bool dump_full_task,
				   const bool dump_cookie,
				   const bool dump_cmd);
int cmdq_core_parse_instruction(const u32 *cmd, char *text_buf, int buf_len);
void cmdq_core_dump_instructions(u64 *instr_buffer, u32 buf_size,
				 enum log_level level);
void cmdq_core_dump_error_task(const struct cmdq_task *task, int tid);

const char *cmdq_event_get_name(enum cmdq_event event);
bool cmdq_core_support_sync_non_suspendable(void);
const char *cmdq_core_parse_module_from_reg_addr(u32 reg_addr);
void cmdq_core_print_event_seq(struct seq_file *m, struct cmdq_context *cqctx);
u32 cmdq_get_cookie_cnt(void __iomem *gce_base_va, int tid);
void cmdq_core_long_string(bool force_log, char *buf, u32 *offset,
			   int *max_size, const char *string, ...);
int cmdq_get_time_in_ms(unsigned long long start, unsigned long long end);
bool cmdq_core_clock_is_on(struct cmdq_context *cqctx);
u32 *cmdq_core_get_pc(const struct cmdq_task *task, u32 thread, u32 insts[2]);
const char *cmdq_core_parse_op(u32 op_code);
u32 cmdq_core_subsys_to_reg_addr(u32 arg_a);

#endif	/* __MTK_CMDQ_DEBUG_H__ */
