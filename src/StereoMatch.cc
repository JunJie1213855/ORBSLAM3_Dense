#include "StereoMatch.h"

namespace ORB_SLAM3
{

    std::shared_ptr<Stereo_Algorithm> Stereo_Algorithm::create(double disp_min, double disp_max, AlgorithmType type, cv::Size input_size, const std::string& model_path)
    {
        switch (type)
        {
        case Stereo_Algorithm::AlgorithmType::ELAS:
            std::cout << "create elas algorithm" << std::endl;
            return std::make_shared<Elas_Algorithm>(disp_min, disp_max);
        case Stereo_Algorithm::AlgorithmType::SGBM:
            std::cout << "create sgbm algorithm" << std::endl;
            return std::make_shared<SGBM_Algorithm>(disp_min, disp_max);
        case Stereo_Algorithm::AlgorithmType::IGEV:
            std::cout << "create igev algorithm" << std::endl;
            return std::make_shared<TensorRT_Stereo_Algorithm>(model_path, input_size);
        case Stereo_Algorithm::AlgorithmType::LiteAnyStereo:
            std::cerr << "create igev algorithm" << std::endl;
            return std::make_shared<TensorRT_Stereo_Algorithm>(model_path, input_size);
        default:
            return nullptr;
        }
    }

    // elas algorithm setting
    Elas_Algorithm::Elas_Algorithm(double disp_min, double disp_max)
    {
        param = Elas::parameters(Elas::setting::MIDDLEBURY);
        param.disp_min = disp_min;
        param.disp_max = disp_max;       
        // model
        model = std::make_unique<Elas>(param);
    }

    // disparity inference
    cv::Mat Elas_Algorithm::inference(const cv::Mat &left_rectified, const cv::Mat &right_rectified)
    {
        cv::Mat left = left_rectified.clone();
        cv::Mat right = right_rectified.clone();
        // 参数调整
        int height = left.rows;
        int width = left.cols;
        int dim[3] = {width, height, width};
        if (left.channels() == 3)
            cv::cvtColor(left, left, cv::COLOR_BGR2GRAY);
        if (right.channels() == 3)
            cv::cvtColor(right, right, cv::COLOR_BGR2GRAY);
        cv::Mat left_disp = cv::Mat::zeros(left.size(), CV_32FC1);
        cv::Mat right_disp = cv::Mat::zeros(right.size(), CV_32FC1);
        // 计算
        model->process(left.data, right.data, left_disp.ptr<float>(0), right_disp.ptr<float>(0), dim);

        return left_disp;
    }

    // ----------------------------------------------------------------
    // sgbm

    SGBM_Algorithm::SGBM_Algorithm(double disp_min, double disp_max)
    {
        int block_size = 8;
        int P1 = 8 * 3 * block_size * block_size;
        int P2 = 32 * 3 * block_size * block_size;
        // create the stereo
        model = cv::StereoSGBM::create(
            disp_min, disp_max, block_size, P1, P2, -1, 1, 10, 100, 1, cv::StereoSGBM::MODE_HH);

#ifdef WITH_FILTER
        wlsfilter = cv::ximgproc::createDisparityWLSFilter(model);
        model_right = cv::ximgproc::createRightMatcher(model);
        wlsfilter->setLambda(8000.0);
        wlsfilter->setSigmaColor(1.20);
#endif
    }
    cv::Mat SGBM_Algorithm::inference(const cv::Mat &left_rectified, const cv::Mat &right_rectified)
    {
        cv::Mat disp;
        model->compute(left_rectified, right_rectified, disp);
#ifdef WITH_FILTER
        cv::Mat right_disp;
        model_right->compute(right_rectified,left_rectified,right_disp);
        wlsfilter->filter(disp,left_rectified,disp,right_disp);
#endif
        return disp / 16;
    }

    // ----------------------------------------------------------------
    // TensorRT Stereo Algorithm (IGEV / LiteAnyStereo)

    TensorRT_Stereo_Algorithm::TensorRT_Stereo_Algorithm(
        const std::string& model_path,
        const cv::Size& input_size)
        : input_size_(input_size)
    {
        if (!model_path.empty()) {
            model_ = std::make_unique<TRTInfer>(model_path);
        }
    }

    TensorRT_Stereo_Algorithm::~TensorRT_Stereo_Algorithm()
    {
    }

    cv::Mat TensorRT_Stereo_Algorithm::inference(const cv::Mat &left_rectified, const cv::Mat &right_rectified)
    {
        // 1. 预处理
        cv::Mat left_rgb = preprocess(left_rectified);
        cv::Mat right_rgb = preprocess(right_rectified);

        // 2. 转换为 blob (NCHW)
        std::unordered_map<std::string, cv::Mat> input_blob;
        input_blob["left"] = cv::dnn::blobFromImage(left_rgb, 1.0/255.0, cv::Size(), cv::Scalar(), true, false);
        input_blob["right"] = cv::dnn::blobFromImage(right_rgb, 1.0/255.0, cv::Size(), cv::Scalar(), true, false);

        // 3. 推理
        auto output = (*model_)(input_blob);

        // 获取视差输出 - 支持 "disparity" 或 "output" 作为 tensor 名称
        cv::Mat disp_nchw;
        if (output.count("disparity")) {
            disp_nchw = output["disparity"];
        } else if (output.count("output")) {
            disp_nchw = output["output"];
        } else {
            // 默认取第一个输出
            disp_nchw = output.begin()->second;
        }

        // 4. 后处理
        cv::Mat disp = postprocess(disp_nchw);

        return disp;
    }

    cv::Mat TensorRT_Stereo_Algorithm::preprocess(const cv::Mat& img)
    {
        cv::Mat rgb;
        if (img.channels() == 1) {
            cv::cvtColor(img, rgb, cv::COLOR_GRAY2RGB);
        } else {
            cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);
        }

        original_size_ = rgb.size();

        // Padding 到 32 的倍数
        int pad_w = (32 - rgb.cols % 32) % 32;
        int pad_h = (32 - rgb.rows % 32) % 32;
        cv::copyMakeBorder(rgb, rgb, 0, pad_h, 0, pad_w, cv::BORDER_CONSTANT);

        // Resize 到模型输入尺寸
        cv::Mat resized;
        cv::resize(rgb, resized, input_size_);

        return resized;
    }

    cv::Mat TensorRT_Stereo_Algorithm::postprocess(const cv::Mat& disp_nchw)
    {
        // 从 NCHW 转换为 HWC
        cv::Mat disp_2d(input_size_.height, input_size_.width, CV_32F, disp_nchw.data);
        cv::Mat disp_orig;
        cv::resize(disp_2d, disp_orig, original_size_);

        // 视差范围转换
        if (normalize_disp_) {
            disp_orig = disp_orig * (disp_max_ - disp_min_);
        }

        return disp_orig;
    }
}
