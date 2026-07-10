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
#ifndef __DEPTH_TO_COLOR_H__
#define __DEPTH_TO_COLOR_H__

#include <stdint.h>

#if (defined(WIN32) || defined(_WIN32) || defined(WINCE))
#  ifdef D2C_EXPORTS
#    define D2C_API __declspec(dllexport)
#  else
#    define D2C_API __declspec(dllimport)
#  endif
#else
#  define D2C_API
#endif

struct SW_D2C{
	float d_intr_p[4];//[fx,fy,cx,cy]
	float c_intr_p[4];//[fx,fy,cx,cy]
	float d2c_r[9];//[r00,r01,r02;r10,r11,r12;r20,r21,r22]
	float d2c_t[3];//[t1,t2,t3]
	float d_k[5];//[k1,k2,p1,p2,k3]
	float c_k[5];
};

enum IR_TYPE {IR_8U = 1, IR_16U, IR_888};

class DepthToColorImpl;

#ifdef __cplusplus
extern "C"
{
#endif
class D2C_API DepthToColor
{
public:
	DepthToColor();

	~DepthToColor();

    /**
     * 获取版本号
     * @return
     */
	const char * GetVersion(void);

    /**
     * 加载内外参配置ini文件
     * @param param_ini     相机内外参配置文件名
     * @retval  true    success
     * @retval  false   failed
     */

	 /* 参数格式
	 [Left Camera Intrinsic]
	 575.191 0 315.172
	 0 575.191 228.181
	 0 0 1
	 [Right Camera Intrinsic]
	 507.743 0 319.123
	 0 507.743 243.314
	 0 0 1
	 [Right to Left Camera Rotate Matrix]
	 0.999992 -0.000163 -0.003892
	 0.000174 0.999995 0.003037
	 0.003892 -0.003038 0.999987
	 [Right to Left Camera Translate]
	 -23.919303 -0.236935 2.397271
	 [Left Camera Distorted Params] 【k1 k2 k3 p1 p2】
	 -0.117383 0.736364 -1.915923 -0.000904 -0.001727
	 [Right Camera Distorted Params]【k1 k2 k3 p1 p2】
	 0.020919 -0.004472 -0.458156 -0.000225 -0.003270
	 */

	bool LoadParameters(const char *param_ini);

	/**
	* 加载内外参结构体数据
	* @param d2c     相机内外参
	* @retval  true    success
	* @retval  false   failed
	*/
	bool LoadParameters(const SW_D2C *d2c);

    /**
     * 为转换某个分辨率的深度图做准备
     * @param width
     * @param height
     * @return
     */
	bool PrepareDepthResolution(int width, int height);

	bool PrepareIrResolution(int color_width, int color_height);

    /**
     * 停止某个（深度图的）分辨率
     * @param width
     * @param height
     * @return
     */
    bool DiscardDepthResolution(int width, int height);

    /**
     * 设置畸变使能状态
     * @param flag  true --> distortion on,  false --> distortion off
     *
     * @note 当畸变很小时，disable distortion会有更快的速度
     */
    bool EnableDistortion(bool flag = true);

    /**
     * 填补深度注册形成的条纹缝隙
     * @param flag  true --> enable depth gap fill ,  false --> disable depth gap fill
     *
     */
    bool EnableDepthGapFill(bool flag = false);

    /**
     * 设置深度值的单位（以毫米计）
     * @param unit_mm
     * @return
     */
    bool SetDepthUnit(float unit_mm);

    /**
     * 获取深度值单位（以毫米计）
     * @return  以毫米计的单位深度(0.0为无效值)
     */
    bool GetDepthUnit(float &unit);

	/**
	* 设置深度镜像模式
	* @param isMirror: false-非镜像，true-镜像
	* @return
	*/
	bool SetMirrorMode(bool isMirror);

    /**
     * 设置转换过程中感兴趣的深度范围
     * @param min_value 最小值(mm)
     * @param max_value 最大值(mm)
     */
    bool SetDepthRange(float min_value, float max_value);

    /**
     * 获取转换过程中感兴趣的深度范围
     * @param min_value
     * @param max_value
     */
    bool GetDepthRange(float &min_value, float &max_value);

    /**
     * 深度图转换到彩色相机坐标系
     * @param[in]   depth_buffer  深度图buffer，行优先存储
     * @param[in]   depth_width   深度图宽度
     * @param[in]   depth_height  深度图高度
     * @param[out]  out_depth     输出深度图（须在外部分配好内存，大小为 color_width * color_height * sizeof(uint16_t)，行优先存储）
     * @param[in]   color_width   彩色图宽度
     * @param[in]   color_height  彩色图高度
     * @retval -1   failed
     * @retval  0   success
     */
    int D2C(const uint16_t *depth_buffer, int depth_width, int depth_height,
            uint16_t *out_depth, int color_width, int color_height) ;
	/**
	* 深度图转换到彩色相机坐标系
	* @param[in]   ir_buffer	 IR图buffer，与深度图同size
	* @param[in]   depth_buffer  深度图buffer，行优先存储
	* @param[in]   depth_width   深度图宽度
	* @param[in]   depth_height  深度图高度
	* @param[out]  out_depth     输出IR图（须在外部分配好内存，大小为 color_width * color_height * sizeof(uint16_t)，行优先存储）
	* @param[in]   color_width   彩色图宽度
	* @param[in]   color_height  彩色图高度
	* @retval -1   failed
	* @retval  0   success
	*/
	int D2C_IR(const uint8_t *ir_buffer, const uint16_t *depth_buffer, int ir_width, int ir_height,
		uint8_t* out_ir, int color_width, int color_height, enum IR_TYPE irType);

	int D2C_IR(const uint8_t *ir_buffer, int depth_value, int ir_width, int ir_height,
		uint8_t* out_ir, int color_width, int color_height, enum IR_TYPE irType);

	// 要求IR和RGB焦距相差5%以内
	int D2C_IR_FAST(const uint8_t *ir_buffer, int depth_value, int ir_width, int ir_height,
		uint8_t* out_ir, int color_width, int color_height, enum IR_TYPE irType);

#ifdef HISI_OPT
	int D2C_IR_HI_TDE(const uint8_t *ir_buffer, int depth_value, int ir_width, int ir_height,
		uint8_t* out_ir, int color_width, int color_height, enum IR_TYPE irType);
#endif


private:
    DepthToColorImpl *pImpl;
};
#ifdef __cplusplus
}
#endif

#endif