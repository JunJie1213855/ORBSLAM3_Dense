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
    // 参数结构化
    struct Stereo_Parameters
    {
        double disp_min;
        double disp_max;
        // Stereo_Algorithm::AlgorithmType type;
        cv::Size input_size = cv::Size();
        const std::string &model_path = "";
    };

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
        static std::shared_ptr<Stereo_Algorithm> create(double disp_min, double disp_max, AlgorithmType type, cv::Size input_size = cv::Size(), const std::string &model_path = "");
        // 参数结构化的方法
        // static std::shared_ptr<Stereo_Algorithm> create(Stereo_Parameters parameters);
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

    // TensorRT 立体匹配算法 -  IGEV
    class TensorRT_IGEV : public Stereo_Algorithm
    {
    public:
        TensorRT_IGEV(
            const std::string &model_path,
            const cv::Size &input_size);
        virtual ~TensorRT_IGEV();

        virtual cv::Mat inference(const cv::Mat &left_rectified,
                                  const cv::Mat &right_rectified) override;

    private:
        // 预处理
        cv::Mat preprocess(const cv::Mat &img);
        // 后处理
        cv::Mat postprocess(const cv::Mat &disp, const cv::Size &origin_size);

    private:
        std::unique_ptr<TRTInfer> model_;
        cv::Size input_size_;
        cv::Size original_size_;
        double disp_min_, disp_max_;
        bool normalize_disp_;
    };

    // TensorRT 立体匹配算法 - LiteAnyStereo
    class TensorRT_LiteAnyStereo : public Stereo_Algorithm
    {
    public:
        TensorRT_LiteAnyStereo(
            const std::string &model_path,
            const cv::Size &input_size)
        {
            std::cout << "todo, please wait" << std::endl;
        }
        virtual ~TensorRT_LiteAnyStereo() { std::cout << "todo, please wait" << std::endl; }

        virtual cv::Mat inference(const cv::Mat &left_rectified,
                                  const cv::Mat &right_rectified) override
        {
            std::cout << "todo, please wait" << std::endl;
            return cv::Mat();
        }

    private:
        // 预处理
        cv::Mat preprocess(const cv::Mat &img)
        {
            std::cout << "todo, please wait" << std::endl;
            return cv::Mat();
        }
        // 后处理
        cv::Mat postprocess(const cv::Mat &disp, const cv::Size &origin_size)
        {
            std::cout << "todo, please wait" << std::endl;
            return cv::Mat();
        }

    private:
        std::unique_ptr<TRTInfer> model_;
        cv::Size input_size_;
        cv::Size original_size_;
        double disp_min_, disp_max_;
        bool normalize_disp_;
    };
}

#endif