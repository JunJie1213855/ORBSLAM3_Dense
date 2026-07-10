/*****************************************************************************
 *  Orbbec Calibration
 *  Copyright (C) 2017 by ORBBEC Technology., Inc.
 *
 *  This file is part of Orbbec Calibration
 *
 *  This file belongs to ORBBEC Technology., Inc.
 *  It is considered a trade secret, and is not to be divulged or used by
 * parties who have NOT received written authorization from the owner.
 *
 *  Description
 ****************************************************************************/
#ifndef __DEPTH_TO_COLOR_D2C_VERSION_H__
#define __DEPTH_TO_COLOR_D2C_VERSION_H__

#define D2C_VERSION_EPOCH 0
#define D2C_VERSION_MAJOR 3
#define D2C_VERSION_MINOR 8

#define D2C_STR_EXP(__A)    #__A
#define D2C_STR(__A)        D2C_STR_EXP(__A)

#define D2C_STRW_EXP(__A)   L#__A
#define D2C_STRW(__A)       D2C_STRW_EXP(__A)

#define D2C_VERSION     "Biometrics_" "201216" "_v" D2C_STR(D2C_VERSION_EPOCH) "." D2C_STR(D2C_VERSION_MAJOR) "." D2C_STR(D2C_VERSION_MINOR)

#endif //__DEPTH_TO_COLOR_D2C_VERSION_H__

/*

V0.3.1: 1:fix Memory Overflow & access to cross-border

V0.3.2: 1:fix memset to zero

V0.3.3: add D2C_IR, for aligning ir to color;

V0.3.4: 20191017: add SetMirrorMode() for image input;

V0.3.5: 20201207. ir2color兼容RGB888格式ir

V0.3.6: 20201215. 
1、针对低性能平台，新增D2C_IR_FAST接口，通过平移实现快速IR2COLOR，没考虑IR和RGB的：图像旋转、缩放（焦距差异）
2、D2C_IR性能优化，需要增加初始化配置接口d2c.PrepareIrResolution(color_width, color_height)

V0.3.7: 20201216
1. 去掉镜像标志和其他参数设置的顺序依赖，防止参数配置顺序不当而导致镜像标志传递不正常

V0.3.8: 20210115
1. bugFix：D2C_IR修复参数错误导致对齐偏差

*/
