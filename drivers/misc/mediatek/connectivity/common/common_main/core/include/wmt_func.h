/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
/*! \file
*    \brief  Declaration of library functions
*
*    Any definitions in this file will be shared among GLUE Layer and internal Driver Stack.
*/


#ifndef _WMT_FUNC_H_
#define _WMT_FUNC_H_

#include "wmt_core.h"
#include "wmt_plat.h"
#include "osal.h"
/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

#if 1				/* defined(CONFIG_MTK_COMBO_HCI_DRIVER) || defined(CONFIG_MTK_COMBO_BT) */
#define CFG_FUNC_BT_SUPPORT 1
#else
#define CFG_FUNC_BT_SUPPORT 0
#endif


#if IS_ENABLED(CONFIG_MTK_FMRADIO)
#define CFG_FUNC_FM_SUPPORT 1
#else
#define CFG_FUNC_FM_SUPPORT 0
#endif

/*
 * GPS is a function of this same CONNSYS block on MT6785, so it must be
 * compiled in: without these ops gpWmtFuncOps[WMTDRV_TYPE_GPS] stays NULL and
 * every FUNC_CTRL(GPS on) fails. That is exactly what lead/comb7 logged when
 * the GPS stage opened /dev/stpgps:
 *
 *   [WMT-CORE][W]opfunc_func_on:WMT-CORE: ops for type(2) not found
 *   [WMT-CORE][E]opfunc_func_on(1333):WMT-CORE:type(0x2) function on failed, ret(-5)
 *
 * and why the bridge read 0 bytes: stpgps_open() returns -EIO when the
 * function does not power on. The vendor gates this on CONFIG_MTK_COMBO_GPS,
 * a BSP Kconfig symbol that does not exist in mainline, so the test was
 * always false here.
 *
 * GPSL5 stays off: it is the second GNSS band of the newer combo chips
 * (MT6877 and up); MT6631 has no L5 task, and WMTDRV_TYPE_GPSL5 is not a
 * function this stack may switch on.
 */
#define CFG_FUNC_GPS_SUPPORT 1
#define CFG_FUNC_GPSL5_SUPPORT 0

#if 1				/* IS_ENABLED(CONFIG_MTK_COMBO_WIFI) */
#define CFG_FUNC_WIFI_SUPPORT 1
#else
#define CFG_FUNC_WIFI_SUPPORT 0
#endif

#if 1
#define CFG_FUNC_ANT_SUPPORT 1
#else
#define CFG_FUNC_ANT_SUPPORT 0
#endif


/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/


/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/


/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/

typedef INT32 (*SUBSYS_FUNC_ON)(P_WMT_IC_OPS pOps, P_WMT_GEN_CONF pConf);
typedef INT32 (*SUBSYS_FUNC_OFF)(P_WMT_IC_OPS pOps, P_WMT_GEN_CONF pConf);

typedef struct _WMT_FUNC_OPS_ {
	SUBSYS_FUNC_ON func_on;
	SUBSYS_FUNC_OFF func_off;
} WMT_FUNC_OPS, *P_WMT_FUNC_OPS;

typedef struct _CMB_PIN_CTRL_REG_ {
	UINT32 regAddr;
	UINT32 regValue;
	UINT32 regMask;

} CMB_PIN_CTRL_REG, *P_CMB_PIN_CTRL_REG;

typedef struct _CMB_PIN_CTRL_ {
	UINT32 pinId;
	UINT32 regNum;
	P_CMB_PIN_CTRL_REG pFuncOnArray;
	P_CMB_PIN_CTRL_REG pFuncOffArray;

} CMB_PIN_CTRL, *P_CMB_PIN_CTRL;

typedef enum _ENUM_CMP_PIN_ID_ {
	CMB_PIN_EEDI_ID = 0,
	CMB_PIN_EEDO_ID = 1,
	CMB_PIN_GSYNC_ID = 2,
} ENUM_CMP_PIN_ID, *P_ENUM_CMP_PIN_ID;



/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/


/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/

/*******************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

#if CFG_FUNC_BT_SUPPORT
extern WMT_FUNC_OPS wmt_func_bt_ops;
#endif

#if CFG_FUNC_FM_SUPPORT
extern WMT_FUNC_OPS wmt_func_fm_ops;
#endif

#if CFG_FUNC_GPS_SUPPORT
extern WMT_FUNC_OPS wmt_func_gps_ops;
#endif

#if CFG_FUNC_GPSL5_SUPPORT
extern WMT_FUNC_OPS wmt_func_gpsl5_ops;
#endif

#if CFG_FUNC_WIFI_SUPPORT
extern WMT_FUNC_OPS wmt_func_wifi_ops;
#endif

#if CFG_FUNC_ANT_SUPPORT
extern WMT_FUNC_OPS wmt_func_ant_ops;
#endif

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/







#endif				/* _WMT_FUNC_H_ */
