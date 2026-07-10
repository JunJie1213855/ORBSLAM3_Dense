#include "TRTinfer.h"
#include "benchmark.h"
#include <opencv2/opencv.hpp>

std::unordered_map<std::string, cv::Mat> preprocess(cv::Mat &left, cv::Mat &right)
{
    std::unordered_map<std::string, cv::Mat> input_blob;
    // convert the H X W X 3 to 3 X H X W , and bgr to rgb
    input_blob["left"] = cv::dnn::blobFromImage(left, 1.0, cv::Size(512, 320), cv::Scalar(), true, false);
    input_blob["right"] = cv::dnn::blobFromImage(right, 1.0, cv::Size(512, 320), cv::Scalar(), true, false);
    return input_blob;
}

void postprocess(const cv::Mat &disp, cv::Mat &disp_vis)
{
    // to visualization
    cv::Mat disp_c = disp.clone();
    double min, max;
    cv::minMaxLoc(disp_c, &min, &max);
    cv::Mat disp_norm = ((disp_c - min) / (max - min)) * 255;
    disp_norm.convertTo(disp_vis, CV_8U);
    cv::applyColorMap(disp_vis, disp_vis, cv::COLORMAP_INFERNO);
}

int main(int argc, char *argv[])
{
    cv::Mat left = cv::imread("demo/image-1.png");
    cv::Mat right = cv::imread("demo/image.png");

    if (left.empty() || right.empty())
    {
        std::cerr << "Error: Could not load stereo images from demo/ directory" << std::endl;
        return -1;
    }

    // preprocess
    auto input_blob = preprocess(left, right);

    // model
    TRTInfer model("igev_320.engine");

    // Run benchmark with warmup
    std::cout << "\n=== IGEV Stereo Matching Benchmark ===" << std::endl;
    Benchmark::runModel(model, input_blob, 10, 100);
    std::cout << "\n=== Running single inference for visualization ===" << std::endl;

    // inference
    auto output_blob = model(input_blob);
    cv::Mat dst;
    // post process
    postprocess(output_blob["disparity"].reshape(1, 320), dst);
    cv::imshow("disp", dst);
    cv::waitKey();
    return 0;
}
