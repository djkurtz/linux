/*
 * Copyright (c) 2014 MediaTek Inc.
 * Author: Jie Qiu <jie.qiu@mediatek.com>
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
#include <drm/drm_edid.h>
#include <linux/clk.h>
#include "mediatek_hdmi.h"
#include "mediatek_hdmi_hw.h"

static u8 mtk_hdmi_aud_get_chnl_count(
			enum hdmi_aud_channel_type channel_type)
{
	u8 output_chan_number = 0;

	switch (channel_type) {
	case HDMI_AUD_CHAN_TYPE_1_0:
	case HDMI_AUD_CHAN_TYPE_1_1:
	case HDMI_AUD_CHAN_TYPE_2_0:
		output_chan_number = 2;
		break;
	case HDMI_AUD_CHAN_TYPE_2_1:
	case HDMI_AUD_CHAN_TYPE_3_0:
		output_chan_number = 3;
		break;
	case HDMI_AUD_CHAN_TYPE_3_1:
	case HDMI_AUD_CHAN_TYPE_4_0:
	case HDMI_AUD_CHAN_TYPE_3_0_LRS:
		output_chan_number = 4;
		break;
	case HDMI_AUD_CHAN_TYPE_4_1:
	case HDMI_AUD_CHAN_TYPE_5_0:
	case HDMI_AUD_CHAN_TYPE_3_1_LRS:
	case HDMI_AUD_CHAN_TYPE_4_0_CLRS:
		output_chan_number = 5;
		break;
	case HDMI_AUD_CHAN_TYPE_5_1:
	case HDMI_AUD_CHAN_TYPE_6_0:
	case HDMI_AUD_CHAN_TYPE_4_1_CLRS:
	case HDMI_AUD_CHAN_TYPE_6_0_CS:
	case HDMI_AUD_CHAN_TYPE_6_0_CH:
	case HDMI_AUD_CHAN_TYPE_6_0_OH:
	case HDMI_AUD_CHAN_TYPE_6_0_CHR:
		output_chan_number = 6;
		break;
	case HDMI_AUD_CHAN_TYPE_6_1:
	case HDMI_AUD_CHAN_TYPE_6_1_CS:
	case HDMI_AUD_CHAN_TYPE_6_1_CH:
	case HDMI_AUD_CHAN_TYPE_6_1_OH:
	case HDMI_AUD_CHAN_TYPE_6_1_CHR:
	case HDMI_AUD_CHAN_TYPE_7_0:
	case HDMI_AUD_CHAN_TYPE_7_0_LH_RH:
	case HDMI_AUD_CHAN_TYPE_7_0_LSR_RSR:
	case HDMI_AUD_CHAN_TYPE_7_0_LC_RC:
	case HDMI_AUD_CHAN_TYPE_7_0_LW_RW:
	case HDMI_AUD_CHAN_TYPE_7_0_LSD_RSD:
	case HDMI_AUD_CHAN_TYPE_7_0_LSS_RSS:
	case HDMI_AUD_CHAN_TYPE_7_0_LHS_RHS:
	case HDMI_AUD_CHAN_TYPE_7_0_CS_CH:
	case HDMI_AUD_CHAN_TYPE_7_0_CS_OH:
	case HDMI_AUD_CHAN_TYPE_7_0_CS_CHR:
	case HDMI_AUD_CHAN_TYPE_7_0_CH_OH:
	case HDMI_AUD_CHAN_TYPE_7_0_CH_CHR:
	case HDMI_AUD_CHAN_TYPE_7_0_OH_CHR:
	case HDMI_AUD_CHAN_TYPE_7_0_LSS_RSS_LSR_RSR:
	case HDMI_AUD_CHAN_TYPE_8_0_LH_RH_CS:
		output_chan_number = 7;
		break;

	case HDMI_AUD_CHAN_TYPE_7_1:
	case HDMI_AUD_CHAN_TYPE_7_1_LH_RH:
	case HDMI_AUD_CHAN_TYPE_7_1_LSR_RSR:
	case HDMI_AUD_CHAN_TYPE_7_1_LC_RC:
	case HDMI_AUD_CHAN_TYPE_7_1_LW_RW:
	case HDMI_AUD_CHAN_TYPE_7_1_LSD_RSD:
	case HDMI_AUD_CHAN_TYPE_7_1_LSS_RSS:
	case HDMI_AUD_CHAN_TYPE_7_1_LHS_RHS:
	case HDMI_AUD_CHAN_TYPE_7_1_CS_CH:
	case HDMI_AUD_CHAN_TYPE_7_1_CS_OH:
	case HDMI_AUD_CHAN_TYPE_7_1_CS_CHR:
	case HDMI_AUD_CHAN_TYPE_7_1_CH_OH:
	case HDMI_AUD_CHAN_TYPE_7_1_CH_CHR:
	case HDMI_AUD_CHAN_TYPE_7_1_OH_CHR:
	case HDMI_AUD_CHAN_TYPE_7_1_LSS_RSS_LSR_RSR:
		output_chan_number = 8;
		break;

	default:
		output_chan_number = 2;
		break;
	}

	return output_chan_number;
}

int mtk_hdmi_signal_output(struct mediatek_hdmi *hdmi, bool enable)
{
	if (hdmi->output == enable)
		return -EINVAL;

	mtk_hdmi_hw_on_off_tmds(hdmi, enable);

	hdmi->output = enable;
	return 0;
}

static int mtk_hdmi_video_change_vpll(
					struct mediatek_hdmi *hdmi,
					u32 clock,
					enum HDMI_DISPLAY_COLOR_DEPTH depth)
{
	int ret;

	ret = mtk_hdmi_hw_set_clock(hdmi, clock);
	if (ret) {
		mtk_hdmi_err("change vpll failed!\n");
		return ret;
	}
	mtk_hdmi_hw_set_pll(hdmi, clock, depth);
	mtk_hdmi_hw_config_sys(hdmi);
	mtk_hdmi_hw_set_deep_color_mode(hdmi, depth);
	return 0;
}

static void mtk_hdmi_video_set_display_mode(struct mediatek_hdmi
					    *hdmi,
					    struct drm_display_mode *mode)
{
	mtk_hdmi_hw_reset(hdmi, true);
	mtk_hdmi_hw_reset(hdmi, false);
	mtk_hdmi_hw_enable_notice(hdmi, true);
	mtk_hdmi_hw_write_int_mask(hdmi, 0xff);
	mtk_hdmi_hw_enable_dvi_mode(hdmi, hdmi->dvi_mode);
	mtk_hdmi_hw_ncts_auto_write_enable(hdmi, true);

	mtk_hdmi_hw_msic_setting(hdmi, mode);
}

static int mtk_hdmi_aud_enable_packet(struct mediatek_hdmi
				       *hdmi, bool enable)
{
	mtk_hdmi_hw_send_aud_packet(hdmi, enable);
	return 0;
}

static int mtk_hdmi_aud_on_off_hw_ncts(struct mediatek_hdmi
					*hdmi, bool on)
{
	mtk_hdmi_hw_ncts_enable(hdmi, on);
	return 0;
}

static int mtk_hdmi_aud_set_input(struct mediatek_hdmi *hdmi)
{
	u8 chan_count;

	mtk_hdmi_hw_aud_set_channel_swap(hdmi, HDMI_AUD_SWAP_LFE_CC);
	mtk_hdmi_hw_aud_raw_data_enable(hdmi, true);

	if (hdmi->aud_param.aud_input_type == HDMI_AUD_INPUT_SPDIF &&
	    hdmi->aud_param.aud_codec == HDMI_AUDIO_CODING_TYPE_DST) {
		mtk_hdmi_hw_aud_set_bit_num(hdmi,
					    HDMI_AUDIO_SAMPLE_SIZE_24);
	} else if (hdmi->aud_param.aud_i2s_fmt ==
			HDMI_I2S_MODE_LJT_24BIT) {
		hdmi->aud_param.aud_i2s_fmt = HDMI_I2S_MODE_LJT_16BIT;
	}

	mtk_hdmi_hw_aud_set_i2s_fmt(hdmi,
				    hdmi->aud_param.aud_i2s_fmt);
	mtk_hdmi_hw_aud_set_bit_num(hdmi, HDMI_AUDIO_SAMPLE_SIZE_24);

	mtk_hdmi_hw_aud_set_high_bitrate(hdmi, false);
	mtk_hdmi_phy_aud_dst_normal_double_enable(hdmi, false);
	mtk_hdmi_hw_aud_dst_enable(hdmi, false);

	if (hdmi->aud_param.aud_input_type == HDMI_AUD_INPUT_SPDIF) {
		mtk_hdmi_hw_aud_dsd_enable(hdmi, false);
		if (hdmi->aud_param.aud_codec ==
			HDMI_AUDIO_CODING_TYPE_DST) {
			mtk_hdmi_phy_aud_dst_normal_double_enable(hdmi,
								  true);
			mtk_hdmi_hw_aud_dst_enable(hdmi, true);
		}

		chan_count = mtk_hdmi_aud_get_chnl_count
						 (HDMI_AUD_CHAN_TYPE_2_0);
		mtk_hdmi_hw_aud_set_i2s_chan_num(hdmi,
						 HDMI_AUD_CHAN_TYPE_2_0,
						 chan_count);
		mtk_hdmi_hw_aud_set_input_type(hdmi,
					       HDMI_AUD_INPUT_SPDIF);
	} else {
		mtk_hdmi_hw_aud_dsd_enable(hdmi, false);
		chan_count =
			mtk_hdmi_aud_get_chnl_count(
			hdmi->aud_param.aud_input_chan_type);
		mtk_hdmi_hw_aud_set_i2s_chan_num(
			hdmi,
			hdmi->aud_param.aud_input_chan_type,
			chan_count);
		mtk_hdmi_hw_aud_set_input_type(hdmi,
					       HDMI_AUD_INPUT_I2S);
	}
	return 0;
}

static int mtk_hdmi_aud_set_src(struct mediatek_hdmi *hdmi,
				struct drm_display_mode *mode)
{
	mtk_hdmi_aud_on_off_hw_ncts(hdmi, false);

	if (hdmi->aud_param.aud_input_type == HDMI_AUD_INPUT_I2S) {
		switch (hdmi->aud_param.aud_hdmi_fs) {
		case HDMI_AUDIO_SAMPLE_FREQUENCY_32000:
		case HDMI_AUDIO_SAMPLE_FREQUENCY_44100:
		case HDMI_AUDIO_SAMPLE_FREQUENCY_48000:
		case HDMI_AUDIO_SAMPLE_FREQUENCY_88200:
		case HDMI_AUDIO_SAMPLE_FREQUENCY_96000:
			mtk_hdmi_hw_aud_src_off(hdmi);
			/* mtk_hdmi_hw_aud_src_enable(hdmi, false); */
			mtk_hdmi_hw_aud_set_mclk(
			hdmi,
			hdmi->aud_param.aud_mclk);
			mtk_hdmi_hw_aud_aclk_inv_enable(hdmi, false);
			break;
		default:
			break;
		}
	} else {
		switch (hdmi->aud_param.iec_frame_fs) {
		case HDMI_IEC_32K:
			hdmi->aud_param.aud_hdmi_fs =
			    HDMI_AUDIO_SAMPLE_FREQUENCY_32000;
			mtk_hdmi_hw_aud_src_off(hdmi);
			mtk_hdmi_hw_aud_set_mclk(hdmi,
						 HDMI_AUD_MCLK_128FS);
			mtk_hdmi_hw_aud_aclk_inv_enable(hdmi, false);
			break;
		case HDMI_IEC_48K:
			hdmi->aud_param.aud_hdmi_fs =
			    HDMI_AUDIO_SAMPLE_FREQUENCY_48000;
			mtk_hdmi_hw_aud_src_off(hdmi);
			mtk_hdmi_hw_aud_set_mclk(hdmi,
						 HDMI_AUD_MCLK_128FS);
			mtk_hdmi_hw_aud_aclk_inv_enable(hdmi, false);
			break;
		case HDMI_IEC_44K:
			hdmi->aud_param.aud_hdmi_fs =
			    HDMI_AUDIO_SAMPLE_FREQUENCY_44100;
			mtk_hdmi_hw_aud_src_off(hdmi);
			mtk_hdmi_hw_aud_set_mclk(hdmi,
						 HDMI_AUD_MCLK_128FS);
			mtk_hdmi_hw_aud_aclk_inv_enable(hdmi, false);
			break;
		default:
			break;
		}
	}
	mtk_hdmi_hw_aud_set_ncts(
		hdmi, hdmi->depth,
	hdmi->aud_param.aud_hdmi_fs, mode->clock);

	mtk_hdmi_hw_aud_src_reenable(hdmi);
	return 0;
}

static int mtk_hdmi_aud_set_chnl_status(struct mediatek_hdmi
					 *hdmi)
{
	mtk_hdmi_hw_aud_set_channel_status(
		hdmi,
	   hdmi->aud_param.hdmi_l_channel_state,
	   hdmi->aud_param.hdmi_r_channel_state,
	   hdmi->aud_param.aud_hdmi_fs);
	return 0;
}

static int mtk_hdmi_aud_output_config(struct mediatek_hdmi
				       *hdmi,
				       struct drm_display_mode *mode)
{
	mtk_hdmi_hw_aud_mute(hdmi, true);
	mtk_hdmi_aud_enable_packet(hdmi, false);

	mtk_hdmi_aud_set_input(hdmi);
	mtk_hdmi_aud_set_src(hdmi, mode);
	mtk_hdmi_aud_set_chnl_status(hdmi);

	usleep_range(50, 100);

	mtk_hdmi_aud_on_off_hw_ncts(hdmi, true);
	mtk_hdmi_aud_enable_packet(hdmi, true);
	mtk_hdmi_hw_aud_mute(hdmi, false);
	return 0;
}

static int mtk_hdmi_setup_av_mute_packet(struct mediatek_hdmi
					  *hdmi)
{
	mtk_hdmi_hw_send_av_mute(hdmi);
	return 0;
}

static int mtk_hdmi_setup_av_unmute_packet(struct mediatek_hdmi
					    *hdmi)
{
	mtk_hdmi_hw_send_av_unmute(hdmi);
	return 0;
}

static int mtk_hdmi_setup_avi_infoframe(struct mediatek_hdmi
					 *hdmi,
					 struct drm_display_mode *mode)
{
	struct hdmi_avi_infoframe frame;
	u8 buffer[17];
	ssize_t err;

	err = drm_hdmi_avi_infoframe_from_display_mode(&frame, mode);
	if (err < 0) {
		mtk_hdmi_err
		    (" failed, err = %ld\n", err);
		return err;
	}

	err = hdmi_avi_infoframe_pack(&frame, buffer, sizeof(buffer));
	if (err < 0) {
		mtk_hdmi_err("hdmi_avi_infoframe_pack failed, err = %ld\n",
			     err);
		return err;
	}

	mtk_hdmi_hw_send_info_frame(hdmi, buffer, sizeof(buffer));
	return 0;
}

static int mtk_hdmi_setup_spd_infoframe(struct mediatek_hdmi
					 *hdmi, const char *vendor,
					 const char *product)
{
	struct hdmi_spd_infoframe frame;
	u8 buffer[29];
	ssize_t err;

	err = hdmi_spd_infoframe_init(&frame, vendor, product);
	if (err < 0) {
		mtk_hdmi_err("hdmi_spd_infoframe_init failed! ,err = %ld\n",
			     err);
		return err;
	}

	err = hdmi_spd_infoframe_pack(&frame, buffer, sizeof(buffer));
	if (err < 0) {
		mtk_hdmi_err("hdmi_spd_infoframe_pack failed! ,err = %ld\n",
			     err);
		return err;
	}

	mtk_hdmi_hw_send_info_frame(hdmi, buffer, sizeof(buffer));
	return 0;
}

static int mtk_hdmi_setup_audio_infoframe(struct mediatek_hdmi
					   *hdmi)
{
	struct hdmi_audio_infoframe frame;
	u8 buffer[14];
	ssize_t err;

	err = hdmi_audio_infoframe_init(&frame);
	if (err < 0) {
		mtk_hdmi_err("hdmi_audio_infoframe_init failed! ,err = %ld\n",
			     err);
		return err;
	}

	frame.coding_type = HDMI_AUDIO_CODING_TYPE_STREAM;
	frame.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_STREAM;
	frame.sample_size = HDMI_AUDIO_SAMPLE_SIZE_STREAM;
	frame.channels =
	    mtk_hdmi_aud_get_chnl_count(
	    hdmi->aud_param.aud_input_chan_type);

	err = hdmi_audio_infoframe_pack(&frame, buffer, sizeof(buffer));
	if (err < 0) {
		mtk_hdmi_err("hdmi_audio_infoframe_pack failed! ,err = %ld\n",
			     err);
		return err;
	}

	mtk_hdmi_hw_send_info_frame(hdmi, buffer, sizeof(buffer));
	return 0;
}

static int mtk_hdmi_setup_vendor_specific_infoframe(struct
						     mediatek_hdmi
						     *hdmi,
						     struct drm_display_mode
						     *mode)
{
	struct hdmi_vendor_infoframe frame;
	u8 buffer[10];
	ssize_t err;

	err = drm_hdmi_vendor_infoframe_from_display_mode(&frame, mode);
	if (err) {
		mtk_hdmi_err
		    ("failed! err = %ld\n",
		     err);
		return err;
	}

	err = hdmi_vendor_infoframe_pack(&frame, buffer, sizeof(buffer));
	if (err) {
		mtk_hdmi_err("hdmi_vendor_infoframe_pack failed! ,err = %ld\n",
			     err);
		return err;
	}

	mtk_hdmi_hw_send_info_frame(hdmi, buffer, sizeof(buffer));
	return 0;
}

int mtk_hdmi_hpd_high(struct mediatek_hdmi *hdmi)
{
	return mtk_hdmi_hw_is_hpd_high(hdmi);
}

void mtk_hdmi_htplg_irq_clr(struct mediatek_hdmi *hdmi)
{
	mtk_hdmi_hw_clear_htplg_irq(hdmi);
}

int mtk_hdmi_output_init(void *private_data)
{
	struct mediatek_hdmi *hdmi =
		(struct mediatek_hdmi *)private_data;
	struct hdmi_audio_param *aud_param;

	if (hdmi->init)
		return -EINVAL;
	aud_param = &hdmi->aud_param;
	hdmi->csp = HDMI_COLORSPACE_RGB;
	hdmi->depth = HDMI_DEEP_COLOR_24BITS;
	hdmi->output = true;
	aud_param->aud_codec = HDMI_AUDIO_CODING_TYPE_PCM;
	aud_param->aud_hdmi_fs = HDMI_AUDIO_SAMPLE_FREQUENCY_48000;
	aud_param->aud_sampe_size = HDMI_AUDIO_SAMPLE_SIZE_16;
	aud_param->aud_input_type = HDMI_AUD_INPUT_I2S;
	aud_param->aud_i2s_fmt = HDMI_I2S_MODE_I2S_24BIT;
	aud_param->aud_mclk = HDMI_AUD_MCLK_128FS;
	aud_param->iec_frame_fs = HDMI_IEC_48K;
	aud_param->aud_input_chan_type = HDMI_AUD_CHAN_TYPE_2_0;
	aud_param->hdmi_l_channel_state[2] = 2;
	aud_param->hdmi_r_channel_state[2] = 2;
	hdmi->init = true;

	return 0;
}

void mtk_hdmi_power_on(struct mediatek_hdmi *hdmi)
{
	mtk_hdmi_hw_make_reg_writable(hdmi, true);
	mtk_hdmi_hw_1p4_version_enable(hdmi, true);
	mtk_hdmi_hw_htplg_irq_enable(hdmi);
}

void mtk_hdmi_power_off(struct mediatek_hdmi *hdmi)
{
	mtk_hdmi_hw_make_reg_writable(hdmi, false);
	mtk_hdmi_hw_1p4_version_enable(hdmi, true);
	mtk_hdmi_hw_htplg_irq_disable(hdmi);
}

void mtk_hdmi_audio_enable(struct mediatek_hdmi *hdmi)
{
	mtk_hdmi_aud_enable_packet(hdmi, true);
	hdmi->audio_enable = true;
}

void mtk_hdmi_audio_disable(struct mediatek_hdmi *hdmi)
{
	mtk_hdmi_aud_enable_packet(hdmi, false);
	hdmi->audio_enable = false;
}

int mtk_hdmi_audio_set_param(struct mediatek_hdmi *hdmi,
			     struct hdmi_audio_param *param)
{
	if (!hdmi->audio_enable) {
		mtk_hdmi_err("hdmi audio is in disable state!\n");
		return -EINVAL;
	}
	mtk_hdmi_info("codec:%d, input:%d, channel:%d, fs:%d\n",
		      param->aud_codec, param->aud_input_type,
		      param->aud_input_chan_type, param->aud_hdmi_fs);
	memcpy(&hdmi->aud_param, param, sizeof(*param));
	return mtk_hdmi_aud_output_config(hdmi, &hdmi->display_node->mode);
}

int mtk_hdmi_detect_dvi_monitor(struct mediatek_hdmi *hdmi)
{
	return hdmi->dvi_mode;
}

int mtk_hdmi_output_set_display_mode(struct drm_display_mode *display_mode,
				     void *private_data)
{
	struct mediatek_hdmi *hdmi = private_data;
	int ret;

	if (!hdmi->init) {
		mtk_hdmi_err("doesn't init hdmi control context!\n");
		return -EINVAL;
	}

	mtk_hdmi_hw_vid_black(hdmi, true);
	mtk_hdmi_hw_aud_mute(hdmi, true);
	mtk_hdmi_setup_av_mute_packet(hdmi);
	mtk_hdmi_signal_output(hdmi, false);

	ret = mtk_hdmi_video_change_vpll(hdmi,
					 display_mode->clock * 1000,
					 hdmi->depth);
	if (ret) {
		mtk_hdmi_err("set vpll failed!\n");
		return ret;
	}
	mtk_hdmi_video_set_display_mode(hdmi, display_mode);

	mtk_hdmi_signal_output(hdmi, true);
	mtk_hdmi_aud_output_config(hdmi, display_mode);

	mtk_hdmi_setup_audio_infoframe(hdmi);
	mtk_hdmi_setup_avi_infoframe(hdmi, display_mode);
	mtk_hdmi_setup_spd_infoframe(hdmi, "mediatek", "chromebook");
	if (display_mode->flags & DRM_MODE_FLAG_3D_MASK) {
		mtk_hdmi_setup_vendor_specific_infoframe(hdmi,
							 display_mode);
	}

	mtk_hdmi_hw_vid_black(hdmi, false);
	mtk_hdmi_hw_aud_mute(hdmi, false);
	mtk_hdmi_setup_av_unmute_packet(hdmi);

	return 0;
}
