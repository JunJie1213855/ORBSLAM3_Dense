#include "TRTinfer.h"
#include "benchmark.h"
#include <opencv2/opencv.hpp>
#include <random>

namespace SegFormer
{
    std::unordered_map<std::string, cv::Mat> preprocess(const cv::Mat &img)
    {
        cv::Mat imgc = img.clone();

        // Convert BGR to RGB as specified in config (bgr_to_rgb=True)
        cv::Mat rgb_img;
        cv::cvtColor(imgc, rgb_img, cv::COLOR_BGR2RGB);

        // Convert to float32
        cv::Mat float_img;
        rgb_img.convertTo(float_img, CV_32F);

        // Normalize with mean and std from config
        // mean=[123.675, 116.28, 103.53], std=[58.395, 57.12, 57.375]
        cv::Scalar mean(123.675, 116.28, 103.53);
        cv::Scalar std_dev(58.395, 57.12, 57.375);

        // Apply normalization: (pixel - mean) / std
        cv::Mat normalized_img;
        cv::subtract(float_img, mean, normalized_img);
        cv::divide(normalized_img, std_dev, normalized_img);

        // Create blob (NCHW format: batch_size=1, channels=3, height=256, width=256)
        cv::Mat blob = cv::dnn::blobFromImage(normalized_img, 1.0, cv::Size(512, 512), cv::Scalar(), false, false);

        std::unordered_map<std::string, cv::Mat> input_blob;
        input_blob["input"] = blob; // Adjust according to your model input name
        return input_blob;
    }
    cv::Mat visualizeSegmentation(const cv::Mat &segMap)
    {
        // Define color palette (BGR format)
        std::vector<cv::Vec3b> colorMap = {
            {0, 0, 0},       // 0: black - background
            {0, 0, 255},     // 1: red
            {0, 255, 0},     // 2: green
            {255, 0, 0},     // 3: blue
            {0, 255, 255},   // 4: yellow
            {255, 0, 255},   // 5: magenta
            {255, 255, 0},   // 6: cyan
            {128, 0, 128},   // 7: purple
            {255, 128, 0},   // 8: orange
            {0, 128, 255},   // 9: sky blue
            {128, 255, 0},   // 10: turquoise
            {255, 0, 128},   // 11: rose
            {128, 128, 0},   // 12: olive green
            {0, 128, 128},   // 13: teal
            {128, 0, 0},     // 14: dark red
            {0, 0, 128},     // 15: dark blue
            {255, 255, 255}, // 16: white
            {192, 192, 192}, // 17: silver
            {128, 128, 128}, // 18: gray
            {255, 165, 0}    // 19: orange red
        };

        cv::Mat coloredMap = cv::Mat(segMap.rows, segMap.cols, CV_8UC3);

        for (int i = 0; i < segMap.rows; i++)
        {
            for (int j = 0; j < segMap.cols; j++)
            {
                int label = segMap.at<uchar>(i, j);

                // If class exceeds palette size, use modulo
                label = label % colorMap.size();
                coloredMap.at<cv::Vec3b>(i, j) = colorMap[label];
            }
        }

        return coloredMap;
    }
}

int main(int argc, char *argv[])
{
    // image
    cv::Mat image = cv::imread("demo/image.png");

    if (image.empty())
    {
        std::cerr << "Error: Could not load image from demo/image.png" << std::endl;
        return -1;
    }

    // preprocess
    auto input_blob = SegFormer::preprocess(image);

    // model
    TRTInfer model("segformer.engine");

    // Run benchmark with warmup
    std::cout << "\n=== SegFormer Semantic Segmentation Benchmark ===" << std::endl;
    Benchmark::runModel(model, input_blob, 10, 100);
    std::cout << "\n=== Running single inference for visualization ===" << std::endl;

    // inference
    auto output_blob = model(input_blob);

    // reshape 1x1x512x512 to 512 x 512
    cv::Mat mat2d = output_blob["output"].reshape(0, 512).clone();

    // convert unsigned char type
    mat2d.convertTo(mat2d, CV_8U);

    cv::Mat viz, mask, image_512;
    // apply color map
    viz = SegFormer::visualizeSegmentation(mat2d);

    // for mask
    cv::resize(image, image_512, cv::Size(512, 512));
    cv::addWeighted(image_512, 0.7, viz, 0.3, 0, mask);

    // show result
    cv::imshow("output", viz);
    cv::imshow("mask", mask);
    cv::waitKey();

    return 0;
}
