#include "TRTinfer.h"
#include "benchmark.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>

namespace LiteAnyStereo
{
    // Padding information structure
    struct PadInfo
    {
        int top, bottom, left, right;
    };

    /**
     * @brief Pad image to multiple of specified value
     * @param img Input image (H, W, C) RGB
     * @param multiple Multiple value (default 32)
     * @return Padded image and padding info
     */
    std::pair<cv::Mat, PadInfo> padToMultiple(const cv::Mat &img, int multiple = 32)
    {
        int H = img.rows;
        int W = img.cols;

        int pad_h = (multiple - H % multiple) % multiple;
        int pad_w = (multiple - W % multiple) % multiple;

        PadInfo pad_info;
        pad_info.top = pad_h / 2;
        pad_info.bottom = pad_h - pad_info.top;
        pad_info.left = pad_w / 2;
        pad_info.right = pad_w - pad_info.left;

        cv::Mat img_padded;
        // Use BORDER_REFLECT_101 similar to Python
        cv::copyMakeBorder(img, img_padded, pad_info.top, pad_info.bottom,
                          pad_info.left, pad_info.right, cv::BORDER_REFLECT_101);

        return {img_padded, pad_info};
    }

    /**
     * @brief Remove padding from image
     * @param img Padded image
     * @param pad_info Padding information
     * @param original_size Original image size before padding
     * @return Unpadded image
     */
    cv::Mat unpad(const cv::Mat &img, const PadInfo &pad_info, const cv::Size &original_size)
    {
        // If model output size matches original size, no unpadding needed
        if (img.rows == original_size.height && img.cols == original_size.width)
        {
            return img;
        }

        // If model output is larger than original, it was padded
        if (img.rows >= original_size.height && img.cols >= original_size.width)
        {
            int H = img.rows;
            int W = img.cols;
            return img(cv::Rect(pad_info.left, pad_info.top,
                               W - pad_info.left - pad_info.right,
                               H - pad_info.top - pad_info.bottom));
        }

        // Model output is smaller, resize to original size
        cv::Mat result;
        cv::resize(img, result, original_size);
        return result;
    }

    /**
     * @brief Load and preprocess image
     * @param img_path Path to image
     * @param target_size Target size (height, width), use cv::Size() for original size
     * @return Pair of (BGR image, RGB image, original size)
     */
    std::tuple<cv::Mat, cv::Mat, cv::Size> loadImage(const std::string &img_path,
                                                     const cv::Size &target_size = cv::Size())
    {
        cv::Mat img = cv::imread(img_path);
        if (img.empty())
        {
            throw std::runtime_error("Failed to load image: " + img_path);
        }

        cv::Size original_size(img.cols, img.rows);
        cv::Mat img_bgr = img.clone();
        cv::Mat img_rgb;

        // Resize if target_size is specified
        if (target_size.width > 0 && target_size.height > 0)
        {
            cv::resize(img, img, target_size);
        }

        // Convert BGR to RGB
        cv::cvtColor(img, img_rgb, cv::COLOR_BGR2RGB);

        return {img_bgr, img_rgb, original_size};
    }

    /**
     * @brief Preprocess stereo images for model input
     * Matches Python: left_input = left_rgb_pad.transpose(2, 0, 1)[np.newaxis, :, :, :].astype(np.float32)
     * @param left_rgb Left image in RGB format
     * @param right_rgb Right image in RGB format
     * @param normalize Whether to normalize to [0, 1] range (default: false, keep [0, 255])
     * @return Input blob map with "left_image" and "right_image" tensors
     */
    std::unordered_map<std::string, cv::Mat> preprocess(const cv::Mat &left_rgb,
                                                        const cv::Mat &right_rgb,
                                                        bool normalize = false)
    {
        std::unordered_map<std::string, cv::Mat> input_blob;

        // Convert RGB to blob (NCHW format)
        // swapRB=false (already RGB)
        if (normalize)
        {
            // Normalize to [0, 1] range
            input_blob["left_image"] = cv::dnn::blobFromImage(left_rgb, 1.0 / 255.0, cv::Size(), cv::Scalar(), false, false);
            input_blob["right_image"] = cv::dnn::blobFromImage(right_rgb, 1.0 / 255.0, cv::Size(), cv::Scalar(), false, false);
        }
        else
        {
            // Keep [0, 255] range
            input_blob["left_image"] = cv::dnn::blobFromImage(left_rgb, 1.0, cv::Size(), cv::Scalar(), false, false);
            input_blob["right_image"] = cv::dnn::blobFromImage(right_rgb, 1.0, cv::Size(), cv::Scalar(), false, false);
        }

        return input_blob;
    }

    /**
     * @brief Visualize disparity map with colormap
     * @param disp Disparity map (float)
     * @param invalid_thres Threshold for invalid pixels
     * @return Colorized disparity visualization (BGR)
     */
    cv::Mat visualizeDisparity(const cv::Mat &disp, float invalid_thres = std::numeric_limits<float>::infinity())
    {
        cv::Mat disp_c = disp.clone();
        cv::Mat invalid_mask = (disp_c >= invalid_thres);

        double min_val, max_val;
        cv::minMaxLoc(disp_c, &min_val, &max_val);

        cv::Mat vis;
        if (max_val > min_val)
        {
            vis = ((disp_c - min_val) / (max_val - min_val));
        }
        else
        {
            vis = cv::Mat::zeros(disp_c.size(), CV_32F);
        }

        vis.convertTo(vis, CV_8U, 255.0);

        // Apply colormap (TURBO matches Python)
        cv::Mat vis_color;
        cv::applyColorMap(vis, vis_color, cv::COLORMAP_TURBO);

        // Set invalid pixels to black
        if (cv::countNonZero(invalid_mask) > 0)
        {
            vis_color.setTo(cv::Scalar(0, 0, 0), invalid_mask);
        }

        return vis_color;
    }

    /**
     * @brief Concatenate images horizontally for visualization
     * @param images Vector of images to concatenate
     * @return Concatenated image
     */
    cv::Mat hconcat(const std::vector<cv::Mat> &images)
    {
        if (images.empty())
            return cv::Mat();

        // Filter out empty images
        std::vector<cv::Mat> valid_images;
        for (const auto &img : images)
        {
            if (!img.empty())
                valid_images.push_back(img);
        }

        if (valid_images.empty())
            return cv::Mat();

        if (valid_images.size() == 1)
            return valid_images[0];

        cv::Mat result;
        cv::hconcat(valid_images, result);
        return result;
    }

} // namespace LiteAnyStereo


void printUsage(const char *program_name)
{
    std::cout << "Usage: " << program_name << " [options]\n"
              << "\nOptions:\n"
              << "  --left_img <path>     Path to left rectified image (default: rect_left.png)\n"
              << "  --right_img <path>    Path to right rectified image (default: rect_right.png)\n"
              << "  --engine <path>       Path to TensorRT engine file (default: liteanystereo.engine)\n"
              << "  --output_dir <path>   Output directory (default: ./output)\n"
              << "  --target_size <h,w>   Target size for model input (default: original size)\n"
              << "  --benchmark           Enable benchmark mode\n"
              << "  --warmup <n>          Warmup iterations (default: 5)\n"
              << "  --runs <n>            Benchmark runs (default: 50)\n"
              << "  --no_display          Disable display window\n"
              << "  --debug, -d           Enable debug mode (save intermediate outputs)\n"
              << "  --normalize           Normalize input to [0, 1] range (default: [0, 255])\n"
              << "  --help                Show this help message\n"
              << "\nExample:\n"
              << "  " << program_name << " --left_img left.png --right_img right.png --engine model.engine\n"
              << std::endl;
}


int main(int argc, char *argv[])
{
    // Default parameters
    std::string left_img_path = "rect_left.png";
    std::string right_img_path = "rect_right.png";
    std::string engine_file = "liteanystereo.engine";
    std::string output_dir = "./output";
    cv::Size target_size;  // Empty means use original size
    bool benchmark_mode = false;
    int warmup_runs = 5;
    int benchmark_runs = 50;
    bool enable_display = true;
    bool debug_mode = false;  // Enable debug output
    bool normalize_input = false;  // Normalize input to [0, 1] range

    // Parse command line arguments
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h")
        {
            printUsage(argv[0]);
            return 0;
        }
        else if (arg == "--left_img" && i + 1 < argc)
        {
            left_img_path = argv[++i];
        }
        else if (arg == "--right_img" && i + 1 < argc)
        {
            right_img_path = argv[++i];
        }
        else if (arg == "--engine" && i + 1 < argc)
        {
            engine_file = argv[++i];
        }
        else if (arg == "--output_dir" && i + 1 < argc)
        {
            output_dir = argv[++i];
        }
        else if (arg == "--target_size" && i + 1 < argc)
        {
            std::string size_str = argv[++i];
            size_t comma_pos = size_str.find(',');
            if (comma_pos != std::string::npos)
            {
                int h = std::stoi(size_str.substr(0, comma_pos));
                int w = std::stoi(size_str.substr(comma_pos + 1));
                target_size = cv::Size(w, h);
            }
        }
        else if (arg == "--benchmark" || arg == "-b")
        {
            benchmark_mode = true;
        }
        else if (arg == "--warmup" && i + 1 < argc)
        {
            warmup_runs = std::stoi(argv[++i]);
        }
        else if (arg == "--runs" && i + 1 < argc)
        {
            benchmark_runs = std::stoi(argv[++i]);
        }
        else if (arg == "--no_display")
        {
            enable_display = false;
        }
        else if (arg == "--debug" || arg == "-d")
        {
            debug_mode = true;
        }
        else if (arg == "--normalize")
        {
            normalize_input = true;
        }
        else
        {
            std::cerr << "Unknown argument: " << arg << std::endl;
            printUsage(argv[0]);
            return -1;
        }
    }

    // Create output directory
    std::string mkdir_cmd = "mkdir -p " + output_dir;
    system(mkdir_cmd.c_str());

    // Print configuration
    std::cout << "========================================" << std::endl;
    std::cout << "LiteAnyStereo TensorRT Inference" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Left Image:   " << left_img_path << std::endl;
    std::cout << "  Right Image:  " << right_img_path << std::endl;
    std::cout << "  Engine:       " << engine_file << std::endl;
    std::cout << "  Output Dir:   " << output_dir << std::endl;
    std::cout << "  Benchmark:    " << (benchmark_mode ? "Yes" : "No") << std::endl;
    if (target_size.width > 0)
    {
        std::cout << "  Target Size:  " << target_size.height << "x" << target_size.width << std::endl;
    }
    std::cout << "========================================" << std::endl;

    try
    {
        // Load images
        std::cout << "\nLoading images..." << std::endl;
        cv::Mat left_bgr, left_rgb, right_bgr, right_rgb;
        cv::Size left_original_size, right_original_size;

        std::tie(left_bgr, left_rgb, left_original_size) = LiteAnyStereo::loadImage(left_img_path, target_size);
        std::tie(right_bgr, right_rgb, right_original_size) = LiteAnyStereo::loadImage(right_img_path, target_size);

        std::cout << "  Left image size:  " << left_rgb.cols << "x" << left_rgb.rows << std::endl;
        std::cout << "  Right image size: " << right_rgb.cols << "x" << right_rgb.rows << std::endl;

        // Pad to multiple of 32
        std::cout << "\nPadding images to multiple of 32..." << std::endl;
        cv::Mat left_rgb_pad, right_rgb_pad;
        LiteAnyStereo::PadInfo left_pad_info, right_pad_info;

        std::tie(left_rgb_pad, left_pad_info) = LiteAnyStereo::padToMultiple(left_rgb, 32);
        std::tie(right_rgb_pad, right_pad_info) = LiteAnyStereo::padToMultiple(right_rgb, 32);

        std::cout << "  Padded size: " << left_rgb_pad.cols << "x" << left_rgb_pad.rows << std::endl;

        // Save padded images for debugging
        if (debug_mode)
        {
            cv::Mat left_rgb_pad_bgr, right_rgb_pad_bgr;
            cv::cvtColor(left_rgb_pad, left_rgb_pad_bgr, cv::COLOR_RGB2BGR);
            cv::cvtColor(right_rgb_pad, right_rgb_pad_bgr, cv::COLOR_RGB2BGR);
            cv::imwrite(output_dir + "/debug_left_rgb_pad.png", left_rgb_pad_bgr);
            cv::imwrite(output_dir + "/debug_right_rgb_pad.png", right_rgb_pad_bgr);
            std::cout << "  Debug: Saved padded images to " << output_dir << std::endl;
        }

        // Preprocess
        auto input_blob = LiteAnyStereo::preprocess(left_rgb_pad, right_rgb_pad, normalize_input);

        if (debug_mode)
        {
            std::cout << "\nInput preprocessing:" << std::endl;
            std::cout << "  Normalization: " << (normalize_input ? "[0, 1]" : "[0, 255]") << std::endl;
        }

        // Debug: Print input tensor info
        std::cout << "\nInput tensor info:" << std::endl;
        for (const auto &pair : input_blob)
        {
            const cv::Mat &blob = pair.second;
            std::cout << "  " << pair.first << ": " << blob.size[0] << " x " << blob.size[1]
                      << " x " << blob.size[2] << " x " << blob.size[3] << std::endl;

            // Check data range (first few pixels)
            const float *ptr = blob.ptr<float>();
            float min_val = ptr[0], max_val = ptr[0];
            for (int i = 0; i < 100; ++i)
            {
                min_val = std::min(min_val, ptr[i]);
                max_val = std::max(max_val, ptr[i]);
            }
            std::cout << "    Data range (first 100 pixels): [" << min_val << ", " << max_val << "]" << std::endl;
        }

        // Load model
        std::cout << "\nLoading TensorRT engine..." << std::endl;
        TRTInfer model(engine_file);

        // Benchmark mode
        if (benchmark_mode)
        {
            std::cout << "\n=== Benchmark ===" << std::endl;
            Benchmark::runModel(model, input_blob, warmup_runs, benchmark_runs);
            std::cout << "\n=== Running inference for output ===" << std::endl;
        }

        // Single inference for output
        auto output_blob = model(input_blob);

        // Get disparity output
        cv::Mat disparity;
        if (output_blob.find("disparity") != output_blob.end())
        {
            disparity = output_blob["disparity"];
        }
        else if (output_blob.find("output") != output_blob.end())
        {
            disparity = output_blob["output"];
        }
        else
        {
            // Use first output
            disparity = output_blob.begin()->second;
        }

        // Reshape and unpad
        int n = disparity.size[0];
        int c = disparity.size[1];
        int model_h = disparity.size[2];
        int model_w = disparity.size[3];

        std::cout << "\nModel output info:" << std::endl;
        std::cout << "  Raw output shape: " << n << " x " << c << " x " << model_h << " x " << model_w << std::endl;

        // Create 2D Mat from 4D tensor (N, C, H, W) -> (H, W)
        // The data is stored in row-major order: N * C * H * W
        cv::Mat disp_2d(model_h, model_w, CV_32F, disparity.ptr<float>());

        std::cout << "  Reshaped to: " << disp_2d.rows << " x " << disp_2d.cols << std::endl;

        // Check disparity range before unpad
        double min_disp_raw, max_disp_raw;
        cv::minMaxLoc(disp_2d, &min_disp_raw, &max_disp_raw);
        std::cout << "  Disparity range (raw): [" << min_disp_raw << ", " << max_disp_raw << "]" << std::endl;

        // Unpad to original size (before padding)
        cv::Size original_size_before_padding(left_rgb.cols, left_rgb.rows);
        std::cout << "  Original size before padding: " << original_size_before_padding.width << " x " << original_size_before_padding.height << std::endl;
        std::cout << "  Padded size: " << left_rgb_pad.cols << " x " << left_rgb_pad.rows << std::endl;
        cv::Mat disp_unpadded = LiteAnyStereo::unpad(disp_2d, left_pad_info, original_size_before_padding);

        std::cout << "  Output disparity shape: " << disp_unpadded.cols << "x" << disp_unpadded.rows << std::endl;

        // Print disparity range
        double min_disp, max_disp;
        cv::minMaxLoc(disp_unpadded, &min_disp, &max_disp);
        std::cout << "  Disparity range (after unpad): [" << min_disp << ", " << max_disp << "]" << std::endl;

        // Debug: Save raw disparity float values
        if (debug_mode)
        {
            cv::Mat disp_vis_debug;
            if (max_disp > min_disp)
            {
                disp_vis_debug = ((disp_unpadded - min_disp) / (max_disp - min_disp) * 255.0);
            }
            else
            {
                disp_vis_debug = cv::Mat::zeros(disp_unpadded.size(), CV_32F);
            }
            disp_vis_debug.convertTo(disp_vis_debug, CV_8U);
            cv::imwrite(output_dir + "/debug_disparity_raw.png", disp_vis_debug);

            // Also save as float for analysis
            std::ofstream disp_file(output_dir + "/debug_disparity_float.txt");
            disp_file << "Rows: " << disp_unpadded.rows << ", Cols: " << disp_unpadded.cols << std::endl;
            disp_file << "Min: " << min_disp << ", Max: " << max_disp << std::endl;
            disp_file << "First 10x10 region values:" << std::endl;
            for (int y = 0; y < std::min(10, disp_unpadded.rows); ++y)
            {
                for (int x = 0; x < std::min(10, disp_unpadded.cols); ++x)
                {
                    disp_file << disp_unpadded.at<float>(y, x) << " ";
                }
                disp_file << std::endl;
            }
            disp_file.close();
            std::cout << "  Debug: Saved disparity analysis files" << std::endl;
        }

        // Visualize disparity
        cv::Mat disp_vis = LiteAnyStereo::visualizeDisparity(disp_unpadded);

        // Save outputs
        std::cout << "\nSaving outputs..." << std::endl;

        // Save colored disparity
        std::string disp_color_path = output_dir + "/disparity_color.png";
        cv::imwrite(disp_color_path, disp_vis);
        std::cout << "  Saved colored disparity to: " << disp_color_path << std::endl;

        // Save raw disparity (normalized)
        cv::Mat disp_raw;
        if (max_disp > min_disp)
        {
            disp_raw = ((disp_unpadded - min_disp) / (max_disp - min_disp) * 255.0);
        }
        else
        {
            disp_raw = cv::Mat::zeros(disp_unpadded.size(), CV_32F);
        }
        disp_raw.convertTo(disp_raw, CV_8U);
        std::string disp_raw_path = output_dir + "/disparity_raw.png";
        cv::imwrite(disp_raw_path, disp_raw);
        std::cout << "  Saved raw disparity to: " << disp_raw_path << std::endl;

        // Display
        if (enable_display)
        {
            // Create stereo image visualization (side by side)
            cv::Mat stereo_vis = LiteAnyStereo::hconcat({left_bgr, right_bgr});

            // Resize for display if too large
            int display_width = 1280;
            if (stereo_vis.cols > display_width)
            {
                float scale = static_cast<float>(display_width) / stereo_vis.cols;
                cv::resize(stereo_vis, stereo_vis, cv::Size(), scale, scale);
                cv::resize(disp_vis, disp_vis, cv::Size(), scale, scale);
            }

            cv::imshow("Stereo Images (Left | Right)", stereo_vis);
            cv::imshow("Disparity", disp_vis);

            std::cout << "\nPress any key to exit..." << std::endl;
            cv::waitKey(0);
            cv::destroyAllWindows();
        }

        std::cout << "\nDone!" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
