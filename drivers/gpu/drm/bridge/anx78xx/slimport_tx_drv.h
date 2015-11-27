/*
 * Copyright(c) 2015, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SLIMPORT_TX_DRV_H
#define __SLIMPORT_TX_DRV_H

#include <linux/hdmi.h>

#include "anx78xx.h"
#include "slimport_tx_reg.h"

#define FW_VERSION	0x23

#define HDMI_INFOFRAME_TYPE_MPEG	0x85
#define HDMI_MPEG_INFOFRAME_SIZE	10
#define HDMI_VSI_INFOFRAME_SIZE		10

enum sp_tx_state {
	STATE_WAITING_CABLE_PLUG,
	STATE_SP_INITIALIZED,
	STATE_SINK_CONNECTION,
	STATE_PARSE_EDID,
	STATE_LINK_TRAINING,
	STATE_VIDEO_OUTPUT,
	STATE_HDCP_AUTH,
	STATE_AUDIO_OUTPUT,
	STATE_PLAY_BACK
};

enum sp_tx_power_block {
	SP_TX_PWR_REG = SP_REGISTER_PD,
	SP_TX_PWR_HDCP = SP_HDCP_PD,
	SP_TX_PWR_AUDIO = SP_AUDIO_PD,
	SP_TX_PWR_VIDEO = SP_VIDEO_PD,
	SP_TX_PWR_LINK = SP_LINK_PD,
	SP_TX_PWR_TOTAL = SP_TOTAL_PD,
	SP_TX_PWR_NUMS
};

enum hdmi_color_depth {
	HDMI_LEGACY = 0x00,
	HDMI_24BIT = 0x04,
	HDMI_30BIT = 0x05,
	HDMI_36BIT = 0x06,
	HDMI_48BIT = 0x07,
};

enum sp_tx_lt_status {
	LT_INIT,
	LT_WAIT_PLL_LOCK,
	LT_CHECK_LINK_BW,
	LT_START,
	LT_WAITING_FINISH,
	LT_ERROR,
	LT_FINISH,
};

enum hdcp_status {
	HDCP_CAPABLE_CHECK,
	HDCP_WAITING_VID_STB,
	HDCP_HW_ENABLE,
	HDCP_WAITING_FINISH,
	HDCP_FINISH,
	HDCP_FAILED,
	HDCP_NOT_SUPPORTED,
};

enum repeater_status {
	HDCP_DONE,
	HDCP_DOING,
	HDCP_ERROR,
};

enum video_output_status {
	VO_WAIT_VIDEO_STABLE,
	VO_WAIT_TX_VIDEO_STABLE,
	VO_CHECK_VIDEO_INFO,
	VO_FINISH,
};

enum audio_output_status {
	AO_INIT,
	AO_CTS_RCV_INT,
	AO_AUDIO_RCV_INT,
	AO_RCV_INT_FINISH,
	AO_OUTPUT,
};

struct packet_audio {
	struct hdmi_any_infoframe infoframe;
	u8 data[HDMI_AUDIO_INFOFRAME_SIZE];
};

struct packet_avi {
	struct hdmi_any_infoframe infoframe;
	u8 data[HDMI_AVI_INFOFRAME_SIZE];
};

struct packet_mpeg {
	struct hdmi_any_infoframe infoframe;
	u8 data[HDMI_MPEG_INFOFRAME_SIZE];
};

struct packet_vsi {
	struct hdmi_any_infoframe infoframe;
	u8 data[HDMI_VSI_INFOFRAME_SIZE];
};

enum packets_type {
	AVI_PACKETS,
	MPEG_PACKETS,
	VSI_PACKETS,
	AUDIF_PACKETS
};

enum sp_ssc_dep {
	SSC_DEP_DISABLE = 0x0,
	SSC_DEP_500PPM,
	SSC_DEP_1000PPM,
	SSC_DEP_1500PPM,
	SSC_DEP_2000PPM,
	SSC_DEP_2500PPM,
	SSC_DEP_3000PPM,
	SSC_DEP_3500PPM,
	SSC_DEP_4000PPM,
	SSC_DEP_4500PPM,
	SSC_DEP_5000PPM,
	SSC_DEP_5500PPM,
	SSC_DEP_6000PPM
};

bool sp_main_process(struct anx78xx *anx78xx);
int sp_system_init(struct anx78xx *anx78xx);
void sp_variable_init(struct anx78xx *anx78xx);
u8 sp_get_link_bandwidth(struct anx78xx *anx78xx);

#endif
