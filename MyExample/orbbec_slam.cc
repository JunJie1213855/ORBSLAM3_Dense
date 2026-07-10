#include <iostream>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <dirent.h>
#include <System.h>
#include <chrono>
#include <unistd.h>
#include <algorithm>
#include <OpenNI.h>
#include <register.hpp>
#include <OpenNI.h>
using namespace openni;

class StringSort
{
public:
    bool operator()(const std::string &a, const std::string &b)
    {
        return std::stoi(a.substr(0, a.size() - 4)) < std::stoi(b.substr(0, b.size() - 4));
    }
};

std::vector<std::string> listdir(std::string path)
{
    std::vector<std::string> files;
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir(path.c_str())) != nullptr)
    { // 打开当前文件夹
        while ((ent = readdir(dir)) != nullptr)
        { // 读取文件夹中的每个文件
            if (ent->d_type == DT_REG)
            { // 仅处理普通文件

                files.emplace_back(ent->d_name);
            }
        }
        closedir(dir); // 关闭文件夹
    }
    else
    {
        return {};
    }

    std::sort(files.begin(), files.end(), StringSort());
    return files;
}

std::string vocpath = "/home/robot/lib/ORB_Dense/Vocabulary/ORBvoc.txt";
std::string parampath = "/home/robot/lib/ORB_Dense/MyExample/rgbd_orb.yaml";
std::string ini_param = "/home/robot/lib/ORB_Dense/MyExample/orbbecparam/param.ini";
std::string yaml_param = "/home/robot/lib/ORB_Dense/MyExample/orbbecparam/param.yaml";


int main(int argc, char *argv[])
{
    // initialize openNI sdk
    Status rc = OpenNI::initialize();
    Status rcd;
    // open deivce
    Device device;
    rc = device.open(ANY_DEVICE);
    VideoStream color;
    VideoStream depth;
    // VideoStream ir;
    // create color stream
    rc = color.create(device, SENSOR_COLOR);
    rcd = depth.create(device, SENSOR_DEPTH);
    // start color stream
    rc = color.start();
    rcd = depth.start();
    double max, min;
    int index = 0;
    bool flagvideo = false;

    // register
    RGBRegister r(ini_param,yaml_param);

    ORB_SLAM3::System SLAM(vocpath, parampath, ORB_SLAM3::System::RGBD, true);
    cv::Mat frame;
    auto start = std::chrono::steady_clock::now();
    // cv::namedWindow("ORB-SLAM3: Current Frame",cv::WINDOW_NORMAL);
    while ( true)
    {
        VideoFrameRef colorFrame;
        color.readFrame(&colorFrame);
        cv::Mat colorImage(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3, (void *)colorFrame.getData());
        cv::cvtColor(colorImage, colorImage, cv::COLOR_RGB2BGR);

        VideoFrameRef depthFrame;
        depth.readFrame(&depthFrame);
        cv::Mat depthImage(depthFrame.getHeight(), depthFrame.getWidth(), CV_16SC1, (void *)depthFrame.getData());
        // register
        cv::Mat aligned_depth = r.depth2color(colorImage,depthImage);
        // start
        auto now = std::chrono::steady_clock::now();
        auto time = now - start;
        SLAM.TrackRGBD(colorImage, aligned_depth, static_cast<double>(time.count()) / 100.0);
        int key = cv::waitKey(50);
        if(key == 27) break; 
    }
    // system("pause");
    sleep(1);
    SLAM.Shutdown();

    return 0;
}