#include "TRTinfer.h"
#include "benchmark.h"
#include <opencv2/opencv.hpp>
#include <random>

namespace YOLO
{
    struct Detection
    {
        int class_id{0};
        std::string className{};
        float confidence{0.0};
        cv::Scalar color{};
        cv::Rect box{};
    };
    std::unordered_map<std::string, cv::Mat> preprocess(const cv::Mat &img)
    {
        cv::Mat imgc = img.clone();
        if (imgc.size() != cv::Size(480, 640))
            cv::resize(imgc, imgc, cv::Size(480, 640));
        cv::Mat blob = cv::dnn::blobFromImage(imgc, 1 / 255.f, cv::Size(), cv::Scalar(), true, false);
        std::unordered_map<std::string, cv::Mat> input_blob;
        input_blob["images"] = blob;
        return input_blob;
    }
    cv::Mat postprocess(const cv::Mat &output_blob, const cv::Mat &img, const cv::Size2f &scale)
    {
        cv::Mat imgc = img.clone();
        // reshape
        cv::Mat output_blobc = output_blob.clone().reshape(1, 84);
        output_blobc.convertTo(output_blobc, CV_32F);
        cv::transpose(output_blobc, output_blobc);

        // data
        std::vector<cv::Rect> boxes;
        std::vector<float> scores_classs;
        std::vector<int> indices;

        // NMS
        float confidenceThreshold = 0.5;
        float nmsThreshold = 0.5;

        // convert data
        for (int i = 0; i < output_blobc.rows; i++)
        {
            float *classes_scores = (float *)output_blobc.row(i).data + 4;
            cv::Mat scores(cv::Size(80, 1), CV_32FC1, classes_scores);
            cv::Point class_id;
            double maxClassScore;
            // maximum and the location
            minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);
            if (maxClassScore > confidenceThreshold)
            {
                scores_classs.push_back(maxClassScore);
                indices.push_back(class_id.x);
                float x = output_blobc.at<float>(i, 0);
                float y = output_blobc.at<float>(i, 1);
                float w = output_blobc.at<float>(i, 2);
                float h = output_blobc.at<float>(i, 3);
                int left = int((x - 0.5 * w) * scale.width);
                int top = int((y - 0.5 * h) * scale.height);

                int width = int(w * scale.width);
                int height = int(h * scale.height);

                boxes.push_back(cv::Rect(left, top, width, height));
            }
            // break;
        }
        std::vector<int> nms_result;
        cv::dnn::NMSBoxes(boxes, scores_classs, confidenceThreshold, nmsThreshold, nms_result);
        for (unsigned long i = 0; i < nms_result.size(); ++i)
        {
            int idx = nms_result[i];

            Detection result;
            result.class_id = indices[idx];
            result.confidence = scores_classs[idx];

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis(100, 255);
            result.color = cv::Scalar(dis(gen),
                                      dis(gen),
                                      dis(gen));

            result.className = std::to_string(indices[idx]);
            result.box = boxes[idx];
            cv::rectangle(imgc, boxes[idx], result.color, 4);
            cv::putText(imgc, result.className, cv::Point(boxes[idx].x, boxes[idx].y), cv::FONT_HERSHEY_COMPLEX, 1.0, result.color);
        }
        return imgc;
    }

}

int main(int argc, char *argv[])
{
    // image
    cv::Mat image = cv::imread("./demo/bus.jpg");

    if (image.empty())
    {
        std::cerr << "Error: Could not load image from ./demo/bus.jpg" << std::endl;
        return -1;
    }

    // for rescale factor
    float scalew = static_cast<float>(image.size().width) / 480.f;
    float scaleh = static_cast<float>(image.size().height) / 640.f;
    cv::Size2f scale_factor(scalew, scaleh);

    // preprocess
    auto input_blob = YOLO::preprocess(image);

    // model
    TRTInfer model("./yolov8n.engine");

    // Run benchmark with warmup
    std::cout << "\n=== YOLOv8 Object Detection Benchmark ===" << std::endl;
    Benchmark::runModel(model, input_blob, 10, 100);
    std::cout << "\n=== Running single inference for visualization ===" << std::endl;

    // inference
    auto output_blob = model(input_blob);

    // post process
    cv::Mat result = YOLO::postprocess(output_blob["output0"], image, scale_factor);

    // show result
    cv::imshow("output", result);
    cv::waitKey();

    return 0;
}
