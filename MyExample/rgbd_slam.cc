#include <iostream>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <dirent.h>
#include <System.h>
#include <chrono>
#include <unistd.h>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

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

int main(int argc, char *argv[])
{
    std::string root_dir;
    boost::program_options::options_description desc("rgbd slam");
    desc.add_options()
    ("help", "produce help message")
    ("root_dir,r",boost::program_options::value<std::string>(&root_dir),"the root dir of dataset")
    ("param_path,p",boost::program_options::value<std::string>(&parampath)->default_value("/home/robot/lib/ORB_Dense/MyExample/rgbdslam.yaml"))
    ("voc_path,v",boost::program_options::value<std::string>(&vocpath)->default_value("/home/robot/lib/ORB_Dense/Vocabulary/ORBvoc.txt"))
    ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }
    boost::filesystem::path root_path(root_dir);
    auto rgb_sequence = listdir((root_path / "rgb").c_str());
    auto depth_sequence = listdir((root_path / "/depth/").c_str());
    ORB_SLAM3::System SLAM(vocpath, parampath, ORB_SLAM3::System::RGBD, true);
    cv::Mat frame;
    auto start = std::chrono::steady_clock::now();
    // cv::namedWindow("ORB-SLAM3: Current Frame",cv::WINDOW_NORMAL);
    for (int i = 0; i < rgb_sequence.size(); i++)
    {
        cv::Mat rgb = cv::imread( (root_path/ "rgb" / rgb_sequence[i]).c_str(), cv::IMREAD_UNCHANGED);
        cv::Mat depth = cv::imread((root_path/ "depth" / depth_sequence[i]).c_str(), cv::IMREAD_UNCHANGED);
        auto now = std::chrono::steady_clock::now();
        auto time = now - start;
        SLAM.TrackRGBD(rgb, depth, static_cast<double>(time.count()) / 1000.0);
    }
    // system("pause");
    sleep(1);
    SLAM.Shutdown();

    return 0;
}