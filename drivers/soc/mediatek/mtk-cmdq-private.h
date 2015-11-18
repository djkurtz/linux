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

#ifndef __MTK_CMDQ_PRIVATE_H__
#define __MTK_CMDQ_PRIVATE_H__

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <soc/mediatek/cmdq.h>

#include "mtk-cmdq-debug.h"

#define CMDQ_MAX_THREAD_COUNT		6

#define CMDQ_MAX_RECORD_COUNT		1024
#define CMDQ_MAX_ERROR_COUNT		2
#define CMDQ_MAX_RETRY_COUNT		1
#define CMDQ_MAX_TASK_IN_THREAD		16

/*
 * Maximum prefetch buffer size.
 * Unit is instructions.
 */
#define CMDQ_MAX_PREFETCH_INSTUCTION	240

#define CMDQ_INITIAL_CMD_BLOCK_SIZE	PAGE_SIZE
#define CMDQ_EMERGENCY_BLOCK_SIZE	(256 * 1024)
#define CMDQ_EMERGENCY_BLOCK_NUM	4
#define CMDQ_INST_SIZE			8 /* instruction is 64-bit */

#define CMDQ_MAX_LOOP_COUNT		1000000
#define CMDQ_MIN_AGE_VALUE		5

/*
 * cmdq_thread cookie value is from 0 to CMDQ_MAX_COOKIE_VALUE.
 * And, this value also be used as MASK.
 */
#define CMDQ_MAX_COOKIE_VALUE		0xffff
#define CMDQ_COOKIE_MASK		CMDQ_MAX_COOKIE_VALUE

#define CMDQ_ARG_A_SUBSYS_MASK		0x001f0000

/* CMDQ_DEFAULT_TIMEOUT_MS = CMDQ_PREDUMP_TIMEOUT_MS * CMDQ_PREDUMP_RETRY_COUNT */
#define CMDQ_DEFAULT_TIMEOUT_MS		1000
#define CMDQ_ACQUIRE_THREAD_TIMEOUT_MS	2000
#define CMDQ_PREDUMP_TIMEOUT_MS		200
#define CMDQ_PREDUMP_RETRY_COUNT	5

#define CMDQ_INVALID_THREAD		-1

#define CMDQ_DRIVER_DEVICE_NAME		"mtk_cmdq"

#define CMDQ_CLK_NAME			"gce"

#define CMDQ_CURR_IRQ_STATUS_OFFSET	0x010
#define CMDQ_CURR_LOADED_THR_OFFSET	0x018
#define CMDQ_THR_SLOT_CYCLES_OFFSET	0x030
#define CMDQ_THR_EXEC_CYCLES_OFFSET	0x034
#define CMDQ_THR_TIMEOUT_TIMER_OFFSET	0x038
#define CMDQ_BUS_CONTROL_TYPE_OFFSET	0x040

#define CMDQ_SYNC_TOKEN_ID_OFFSET	0x060
#define CMDQ_SYNC_TOKEN_VAL_OFFSET	0x064
#define CMDQ_SYNC_TOKEN_UPD_OFFSET	0x068

#define CMDQ_GPR_SHIFT			0x004
#define CMDQ_GPR_OFFSET			0x080

#define CMDQ_THR_SHIFT			0x080
#define CMDQ_THR_WARM_RESET_OFFSET	0x100
#define CMDQ_THR_ENABLE_TASK_OFFSET	0x104
#define CMDQ_THR_SUSPEND_TASK_OFFSET	0x108
#define CMDQ_THR_CURR_STATUS_OFFSET	0x10c
#define CMDQ_THR_IRQ_STATUS_OFFSET	0x110
#define CMDQ_THR_IRQ_ENABLE_OFFSET	0x114
#define CMDQ_THR_CURR_ADDR_OFFSET	0x120
#define CMDQ_THR_END_ADDR_OFFSET	0x124
#define CMDQ_THR_EXEC_CNT_OFFSET	0x128
#define CMDQ_THR_WAIT_TOKEN_OFFSET	0x130
#define CMDQ_THR_CFG_OFFSET		0x140
#define CMDQ_THR_PREFETCH_OFFSET	0x144
#define CMDQ_THR_INST_CYCLES_OFFSET	0x150
#define CMDQ_THR_INST_THRESX_OFFSET	0x154
#define CMDQ_THR_STATUS_OFFSET		0x18c

/* get lsb for subsys encoding in arg_a (range: 0 - 31) */
#define CMDQ_SUBSYS_LSB	16

#define CMDQ_APXGPT2_COUNT		0x10008028	/* each count is 76ns */

/* debug - begin */

struct cmdq_record {
	pid_t			user;		/* SW thread tid */
	int			scenario;	/* task scenario */
	int			priority;	/* task priority */
	int			thread;		/* HW thread tid */
	int			reorder;
	u64			engine_flag;	/* task engine flag */

	/*
	 * submit: submit time
	 * trigger: trigger time
	 * begin_wait: begin wait time
	 * got_irq: got IRQ time
	 * waked_up: wake up time
	 * done: task done time
	 * send_wake_up: send "wake up" time
	 * receive_wake_up: receive "wake up" time
	 */
	unsigned long long	submit;
	unsigned long long	trigger;
	unsigned long long	begin_wait;
	unsigned long long	got_irq;
	unsigned long long	waked_up;
	unsigned long long	done;
#ifdef CMDQ_CORE_PROFILE_WAKE_UP
	unsigned long long	send_wake_up;
	unsigned long long	receive_wake_up;
#endif

	/* if profile enabled, the time of command execution */
	u32			write_time_ns;
	unsigned long long	write_time_ns_begin;
	unsigned long long	write_time_ns_end;
};

struct cmdq_error {
	/* the record of the error task */
	struct cmdq_record	error_rec;
	/* kernel time of attach_error_task */
	u64			ts_nsec;
};

struct cmdq_error_task_item {
	const struct cmdq_task	*task;
	int			command_size;
	unsigned long long	trigger;
	u32			*va_base;
};

struct cmdq_error_task {
	struct cmdq_error_task_item	error_task_buffer[CMDQ_MAX_ERROR_COUNT];
	int				err_task_count;
};

struct cmdq_print {
	struct cmdq_context	*cqctx;
	struct work_struct	cmdq_print_work;
	struct workqueue_struct	*cmdq_print_wq;
	bool			enable_print;
};

/* debug - end */

enum cmdq_data_register {
	/*
	 * Value Reg, we use 32-bit, Rx
	 * Address Reg, we use 64-bit, Px
	 * (R1 cannot be used. Reserved.)
	 */

	CMDQ_DATA_REG_R0 = 0x00,
	CMDQ_DATA_REG_P1 = 0x11,

	/* invalid register ID */
	CMDQ_DATA_REG_INVALID = -1,
};

enum cmdq_eng {
	CMDQ_ENG_DISP_UFOE = 0,
	CMDQ_ENG_DISP_AAL,
	CMDQ_ENG_DISP_COLOR0,
	CMDQ_ENG_DISP_COLOR1,
	CMDQ_ENG_DISP_RDMA0,
	CMDQ_ENG_DISP_RDMA1,
	CMDQ_ENG_DISP_RDMA2,
	CMDQ_ENG_DISP_WDMA0,
	CMDQ_ENG_DISP_WDMA1,
	CMDQ_ENG_DISP_OVL0,
	CMDQ_ENG_DISP_OVL1,
	CMDQ_ENG_DISP_GAMMA,
	CMDQ_ENG_DISP_DSI0_CMD,
	CMDQ_ENG_DISP_DSI1_CMD,
	CMDQ_MAX_ENGINE_COUNT	/* ALWAYS keep at the end */
};

struct cmdq_command {
	struct cmdq_context	*cqctx;
	u32			scenario;
	/* task priority (NOT HW thread priority) */
	u32			priority;
	/* bit flag of used engines */
	u64			engine_flag;
	/*
	 * pointer of instruction buffer
	 * This must point to an 64-bit aligned u32 array
	 */
	u32			*va_base;
	/* size of instruction buffer, in bytes. */
	u32			block_size;
};

enum cmdq_code {
	/* These are actual HW op code. */
	CMDQ_CODE_READ = 0x01,
	CMDQ_CODE_MOVE = 0x02,
	CMDQ_CODE_WRITE = 0x04,
	CMDQ_CODE_JUMP = 0x10,
	CMDQ_CODE_WFE = 0x20,	/* wait for event (and clear) */
	CMDQ_CODE_EOC = 0x40,	/* end of command */
};

enum cmdq_task_state {
	TASK_STATE_IDLE,	/* free task */
	TASK_STATE_BUSY,	/* task running on a thread */
	TASK_STATE_KILLED,	/* task process being killed */
	TASK_STATE_ERROR,	/* task execution error */
	TASK_STATE_DONE,	/* task finished */
	TASK_STATE_WAITING,	/* allocated but waiting for available thread */
};

struct cmdq_emergency_buf {
	atomic_t		used;
	void			*va;
	dma_addr_t		pa;
};

struct cmdq_task_cb {
	/* called by isr */
	cmdq_async_flush_cb	isr_cb;
	void			*isr_data;
	/* called by releasing task */
	cmdq_async_flush_cb	done_cb;
	void			*done_data;
};

struct cmdq_task {
	struct cmdq_context	*cqctx;
	struct list_head	list_entry;

	/* state for task life cycle */
	enum cmdq_task_state	task_state;
	/* virtual address of command buffer */
	u32			*va_base;
	/* physical address of command buffer */
	dma_addr_t		mva_base;
	/* size of allocated command buffer */
	u32			buf_size;
	/* It points to an emergency buf if this task use it. */
	struct cmdq_emergency_buf *emergency_buf;

	int			scenario;
	int			priority;
	u64			engine_flag;
	u32			command_size;
	u32			num_cmd;
	int			reorder;
	/* HW thread ID; CMDQ_INVALID_THREAD if not running */
	int			thread;
	/* flag of IRQ received */
	int			irq_flag;
	/* callback functions */
	struct cmdq_task_cb	cb;
	/* work item when auto release is used */
	struct work_struct	auto_release_work;

	/* output section for "read from reg to mem" */
	u32			reg_count;
	u32			*reg_results;
	dma_addr_t		reg_results_mva;

	/*
	 * profile or debug
	 * submit: submit time
	 * trigger: trigger time
	 * begin_wait: begin wait time
	 * got_irq: got IRQ time
	 * send_wake_up: send "wake up" time
	 * receive_wake_up: receive "wake up" time
	 * profile_data: profile command
	 * profile_data_pa: profile command
	 */
	unsigned long long	submit;
	unsigned long long	trigger;
	unsigned long long	begin_wait;
	unsigned long long	got_irq;
#ifdef CMDQ_CORE_PROFILE_WAKE_UP
	unsigned long long	send_wake_up;
	unsigned long long	receive_wake_up;
#endif
	unsigned long long	waked_up;
	u32			*profile_data;
	dma_addr_t		profile_data_pa;

	pid_t			caller_pid;
	char			caller_name[TASK_COMM_LEN];
};

struct cmdq_engine {
	int	curr_owner;
	int	fail_count;
};

struct cmdq_thread {
	u32			task_count;
	u32			wait_cookie;
	u32			next_cookie;
	struct cmdq_task	*cur_task[CMDQ_MAX_TASK_IN_THREAD];
};

struct cmdq_write_addr {
	struct list_head	list_node;
	u32			count;
	void			*va;
	dma_addr_t		pa;
	pid_t			user;
};

struct cmdq_device {
	void __iomem	*base_va;
	unsigned long	base_pa;
	u32		irq;
};

struct cmdq_context {
	struct device		*dev;
	struct notifier_block	pm_notifier;

	/* device info */
	struct cmdq_device	cqdev;

	/*
	 * task information
	 * task_cache: struct cmdq_task object cache
	 * task_free_list: unused free tasks
	 * task_active_list: active tasks
	 * task_consume_wait_queue_item: task consumption work item
	 * task_auto_release_wq: auto-release workqueue
	 * task_consume_wq: task consumption workqueue (for queued tasks)
	 */
	struct kmem_cache	*task_cache;
	struct list_head	task_free_list;
	struct list_head	task_active_list;
	struct list_head	task_wait_list;
	struct work_struct	task_consume_wait_queue_item;
	struct workqueue_struct	*task_auto_release_wq;
	struct workqueue_struct	*task_consume_wq;
	u16			task_count[CMDQ_MAX_THREAD_COUNT];

	/* basic information */
	struct cmdq_engine	engine[CMDQ_MAX_ENGINE_COUNT];
	struct cmdq_thread	thread[CMDQ_MAX_THREAD_COUNT];

	/* profile and debug */
	int			last_id;
	int			rec_num;
	struct cmdq_record	record[CMDQ_MAX_RECORD_COUNT];
	struct cmdq_error_task	error_task;

	/* debug or error information */
	int			err_num;
	struct cmdq_error	error[CMDQ_MAX_ERROR_COUNT];

	/* mutex, spinlock, flag */
	struct mutex		task_mutex;	/* for task list */
	struct mutex		clock_mutex;	/* for clock operation */
	spinlock_t		thread_lock;	/* for cmdq hardware thread */
	int			thread_usage;
	spinlock_t		exec_lock;	/* for exec task */
	spinlock_t		record_lock;	/* for cmdq record statistics */

	/* suspend */
	bool			suspended;

	/* emergency buffer */
	struct cmdq_emergency_buf	emergency_buf[CMDQ_EMERGENCY_BLOCK_NUM];

	/*
	 * notification
	 * wait_queue: for task done
	 * thread_dispatch_queue: for thread acquiring
	 */
	wait_queue_head_t	wait_queue[CMDQ_MAX_THREAD_COUNT];
	wait_queue_head_t	thread_dispatch_queue;

	/* ccf */
	struct clk		*clock;

	/* regular log printing task */
	bool			regular_count;
	u32			task_count_buf[CMDQ_MAX_THREAD_COUNT];
	bool			occupied_thread[CMDQ_MAX_THREAD_COUNT];
	u32			occupied_next_cookie[CMDQ_MAX_THREAD_COUNT];
	struct task_struct	*print_log_task;
	struct cmdq_print	print_log_struct;
};

#endif	/* __MTK_CMDQ_PRIVATE_H__ */
