#ifndef STEREOMATCH_H
#define STEREOMATCH_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#ifdef WITH_FILTER
#include <opencv2/ximgproc/disparity_filter.hpp>
#endif

#include "Thirdparty/elas/elas/elas.h"
#include "Thirdparty/TensorRTTemplate/TRTInfer/TRTinfer.h"

namespace ORB_SLAM3
{
    // base class for sterep matching
    class Stereo_Algorithm
    {
    public:
        enum AlgorithmType
        {
            ELAS = 0,
            SGBM = 1,
            IGEV = 2,
            LiteAnyStereo = 3
        };

    public:
        //
        virtual cv::Mat inference(const cv::Mat &left_rectified, const cv::Mat &right_rectified) = 0;

        // 原有工厂方法，保持向后兼容
        static std::shared_ptr<Stereo_Algorithm> create(double disp_min, double disp_max, AlgorithmType type, cv::Size input_size = cv::Size(),const std::string &model_path = "");
    };

    // disparity calculation
    class Elas_Algorithm : public Stereo_Algorithm
    {
    public:
        Elas_Algorithm(double disp_min, double disp_max);
        virtual cv::Mat inference(const cv::Mat &left_rectified, const cv::Mat &right_rectified);

    protected:
        Elas::parameters param;
        std::unique_ptr<Elas> model;
    };

    class SGBM_Algorithm : public Stereo_Algorithm
    {
    public:
        SGBM_Algorithm(double disp_min, double disp_max);
        virtual cv::Mat inference(const cv::Mat &left_rectified, const cv::Mat &right_rectified);

    public:
        cv::Ptr<cv::StereoSGBM> model;
#ifdef WITH_FILTER
        // filter
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wlsfilter;
        cv::Ptr<cv::StereoMatcher> model_right;
#endif
    };

    // TensorRT 立体匹配算法 - 支持 IGEV 和 LiteAnyStereo
    class TensorRT_Stereo_Algorithm : public Stereo_Algorithm
    {
    public:
        TensorRT_Stereo_Algorithm(
            const std::string &model_path,
            const cv::Size &input_size);
        virtual ~TensorRT_Stereo_Algorithm();

        virtual cv::Mat inference(const cv::Mat &left_rectified,
                                  const cv::Mat &right_rectified) override;

    private:
        // 预处理
        cv::Mat preprocess(const cv::Mat &img);
        // 后处理
        cv::Mat postprocess(const cv::Mat &disp);

    private:
        std::unique_ptr<TRTInfer> model_;
        cv::Size input_size_;
        cv::Size original_size_;
        double disp_min_, disp_max_;
        bool normalize_disp_;
    };

}

#endif