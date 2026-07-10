#ifndef REGISTER_HPP
#define REGISTER_HPP
#include <opencv2/opencv.hpp>
#include <depth_to_color.h>
#include <flash_bin_parser.h>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

class RGBRegister
{
public:
    RGBRegister(const std::string &ini_file, const std::string &yaml_file)
    {
        // load params
        d2c.LoadParameters(ini_file.c_str());
        cv::FileStorage file(yaml_file, cv::FileStorage::READ);
        cv::FileNode unit = file["unit"];
        if (!unit.empty())
            d2c.SetDepthUnit(unit.real());
        cv::FileNode width = file["width"];
        cv::FileNode height = file["height"];
        if (!(width.empty() || height.empty()))
        {
            d2c.PrepareDepthResolution(width, height);
            d2c.PrepareIrResolution(width, height);
        }
        cv::FileNode flagdistortion = file["flagdistortion"];
        if (!flagdistortion.empty())
        {
            d2c.EnableDistortion(bool(flagdistortion.real()));
        }
        cv::FileNode flagGapFill = file["flagGapFill"];
        if (!flagGapFill.empty())
        {
            d2c.EnableDistortion(bool(flagGapFill.real()));
        }
        d2c.SetDepthRange(200.0, 1e+4);

        cv::FileNode flagisMirror = file["flagisMirror"];
        if (!flagisMirror.empty())
        {
            d2c.SetMirrorMode(bool(flagisMirror.real()));
        }
        
    }

    cv::Mat depth2color(const cv::Mat &color, const cv::Mat &depth)
    {
        // 设置对齐图像
        cv::Mat aligned_depth(depth.size(),CV_16UC1);
        // 图像配准
        d2c.D2C(depth.ptr<uint16_t>(),depth.cols,depth.rows,
                       aligned_depth.ptr<uint16_t>(),color.cols,color.rows);
        return aligned_depth;
    }

private:
    DepthToColor d2c;
};

#endif