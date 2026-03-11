#include "TRTinfer.h"
#include "benchmark.h"
#include <opencv2/opencv.hpp>
#include <chrono>

std::unordered_map<std::string, cv::Mat> preprocess(cv::Mat &left, cv::Mat &right)
{
    std::unordered_map<std::string, cv::Mat> input_blob;

    // Step 1: BGR to RGB conversion (matches cv2.cvtColor(..., cv2.COLOR_BGR2RGB))
    cv::Mat left_rgb, right_rgb;
    cv::cvtColor(left, left_rgb, cv::COLOR_BGR2RGB);
    cv::cvtColor(right, right_rgb, cv::COLOR_BGR2RGB);

    // Step 2: Resize to multiple of 32 by cropping (not scaling)
    // Python: img_height = (img_height // 32) * 32
    int img_height = 736; // fixed height
    int img_width = 1280; // fixed width

    // Crop images to multiple of 32
    cv::Mat left_cropped = cv::Mat::zeros(cv::Size(img_width, img_height), CV_32FC3);
    cv::Mat right_cropped = cv::Mat::zeros(cv::Size(img_width, img_height), CV_32FC3);
    left_rgb(cv::Rect(0, 0, left_rgb.cols, left_rgb.rows)).copyTo(left_cropped(cv::Rect(0, 0, left_rgb.cols, left_rgb.rows)));
    right_rgb(cv::Rect(0, 0, right_rgb.cols, right_rgb.rows)).copyTo(right_cropped(cv::Rect(0, 0, right_rgb.cols, right_rgb.rows)));

    std::cout << "Original size: " << left_rgb.rows << " x " << left_rgb.cols << std::endl;
    std::cout << "Cropped size: " << img_height << " x " << img_width << std::endl;

    // Step 3: Convert to CHW format and add batch dimension
    // Python: torch.from_numpy(left).permute(-1, 0, 1).unsqueeze(0)
    // No normalization, no scaling, just dimension permutation

    // Convert to blob (automatically does: BGR->RGB, HWC->CHW, adds batch dim)
    // scalefactor=1.0 (no scaling), size (original size, no resize), swapRB=true (BGR->RGB)
    // mean (no mean subtraction), crop=false
    input_blob["input_left"] = cv::dnn::blobFromImage(left_cropped, 1.0, cv::Size(img_width, img_height),
                                                      cv::Scalar(), true, false);
    input_blob["input_right"] = cv::dnn::blobFromImage(right_cropped, 1.0, cv::Size(img_width, img_height),
                                                       cv::Scalar(), true, false);
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
    cv::Mat left = cv::imread("rect_left.png");
    cv::Mat right = cv::imread("rect_right.png");

    if (left.empty() || right.empty())
    {
        std::cerr << "Error: Could not load input images!" << std::endl;
        return -1;
    }

    // preprocess
    auto input_blob = preprocess(left, right);

    // model
    TRTInfer model("S2M2.engine");

    // Run benchmark with warmup
    std::cout << "\n=== S2M2 Stereo Matching Benchmark ===" << std::endl;
    Benchmark::runModel(model, input_blob, 10, 100);
    std::cout << "\n=== Running single inference for visualization ===" << std::endl;

    auto output_blob = model(input_blob); // inference

    cv::Mat dst, dst_conf;
    // post process
    postprocess(output_blob["output_disp"].reshape(1, 736), dst);
    postprocess(output_blob["output_conf"].reshape(1, 736), dst_conf);
    cv::imshow("disp", dst);
    cv::imshow("disp conf", dst_conf);
    cv::waitKey();
    return 0;
}
