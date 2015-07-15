/*
 * Copyright (c) 2014-2015 MediaTek Inc.
 * Author: Yong Wu <yong.wu@mediatek.com>
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
#ifndef __DTS_IOMMU_PORT_MT8173_H
#define __DTS_IOMMU_PORT_MT8173_H

#define M4U_LARB0_ID			0
#define M4U_LARB1_ID			1
#define M4U_LARB2_ID			2
#define M4U_LARB3_ID			3
#define M4U_LARB4_ID			4
#define M4U_LARB5_ID			5

/* larb0 */
#define	M4U_PORT_DISP_OVL0		0
#define	M4U_PORT_DISP_RDMA0		1
#define	M4U_PORT_DISP_WDMA0		2
#define	M4U_PORT_DISP_OD_R		3
#define	M4U_PORT_DISP_OD_W		4
#define	M4U_PORT_MDP_RDMA0		5
#define	M4U_PORT_MDP_WDMA		6
#define	M4U_PORT_MDP_WROT0		7

/* larb1 */
#define	M4U_PORT_HW_VDEC_MC_EXT		0
#define	M4U_PORT_HW_VDEC_PP_EXT		1
#define	M4U_PORT_HW_VDEC_UFO_EXT	2
#define	M4U_PORT_HW_VDEC_VLD_EXT	3
#define	M4U_PORT_HW_VDEC_VLD2_EXT	4
#define	M4U_PORT_HW_VDEC_AVC_MV_EXT	5
#define	M4U_PORT_HW_VDEC_PRED_RD_EXT	6
#define	M4U_PORT_HW_VDEC_PRED_WR_EXT	7
#define	M4U_PORT_HW_VDEC_PPWRAP_EXT	8
#define	M4U_PORT_HW_VDEC_TILE		9

/* larb2 */
#define	M4U_PORT_IMGO			0
#define	M4U_PORT_RRZO			1
#define	M4U_PORT_AAO			2
#define	M4U_PORT_LCSO			3
#define	M4U_PORT_ESFKO			4
#define	M4U_PORT_IMGO_D			5
#define	M4U_PORT_LSCI			6
#define	M4U_PORT_LSCI_D			7
#define	M4U_PORT_BPCI			8
#define	M4U_PORT_BPCI_D			9
#define	M4U_PORT_UFDI			10
#define	M4U_PORT_IMGI			11
#define	M4U_PORT_IMG2O			12
#define	M4U_PORT_IMG3O			13
#define	M4U_PORT_VIPI			14
#define	M4U_PORT_VIP2I			15
#define	M4U_PORT_VIP3I			16
#define	M4U_PORT_LCEI			17
#define	M4U_PORT_RB			18
#define	M4U_PORT_RP			19
#define	M4U_PORT_WR			20

/* larb3 */
#define	M4U_PORT_VENC_RCPU		0
#define	M4U_PORT_VENC_REC		1
#define	M4U_PORT_VENC_BSDMA		2
#define	M4U_PORT_VENC_SV_COMV		3
#define	M4U_PORT_VENC_RD_COMV		4
#define	M4U_PORT_JPGENC_RDMA		5
#define	M4U_PORT_JPGENC_BSDMA		6
#define	M4U_PORT_JPGDEC_WDMA		7
#define	M4U_PORT_JPGDEC_BSDMA		8
#define	M4U_PORT_VENC_CUR_LUMA		9
#define	M4U_PORT_VENC_CUR_CHROMA	10
#define	M4U_PORT_VENC_REF_LUMA		11
#define	M4U_PORT_VENC_REF_CHROMA	12
#define	M4U_PORT_VENC_NBM_RDMA		13
#define	M4U_PORT_VENC_NBM_WDMA		14

/* larb4 */
#define	M4U_PORT_DISP_OVL1		0
#define	M4U_PORT_DISP_RDMA1		1
#define	M4U_PORT_DISP_RDMA2		2
#define	M4U_PORT_DISP_WDMA1		3
#define	M4U_PORT_MDP_RDMA1		4
#define	M4U_PORT_MDP_WROT1		5

/* larb5 */
#define	M4U_PORT_VENC_RCPU_SET2		0
#define	M4U_PORT_VENC_REC_FRM_SET2	1
#define	M4U_PORT_VENC_REF_LUMA_SET2	2
#define	M4U_PORT_VENC_REC_CHROMA_SET2	3
#define	M4U_PORT_VENC_BSDMA_SET2	4
#define	M4U_PORT_VENC_CUR_LUMA_SET2	5
#define	M4U_PORT_VENC_CUR_CHROMA_SET2	6
#define	M4U_PORT_VENC_RD_COMA_SET2	7
#define	M4U_PORT_VENC_SV_COMA_SET2	8

#endif
