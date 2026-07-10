#include <iostream>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <dirent.h>
#include <System.h>
#include <chrono>
#include <algorithm>
#include <unistd.h>
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


int main(int argc, char *argv[])
{
	std::string vocpath ;
	std::string parampath ;
	std::string left_dir;
	std::string right_dir;
    boost::program_options::options_description desc("stereo slam");
    desc.add_options()
    ("help", "produce help message")
    ("left_dir,l",boost::program_options::value<std::string>(&left_dir),"the root dir of left dataset")
	("right_dir,r",boost::program_options::value<std::string>(&right_dir),"the root dir of right dataset")
    ("param_path,p",boost::program_options::value<std::string>(&parampath)->default_value("./MyExample/stereoslam.yaml"))
    ("voc_path,v",boost::program_options::value<std::string>(&vocpath)->default_value("./Vocabulary/ORBvoc.txt"))
    ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

	boost::filesystem::path left_path(left_dir);
	boost::filesystem::path right_path(right_dir);
	auto l_ps = listdir(left_dir);
	auto r_ps = listdir(right_dir);
	ORB_SLAM3::System SLAM(vocpath, parampath, ORB_SLAM3::System::STEREO, true);
	auto start = std::chrono::steady_clock::now();
	std::cout << "image size : " << l_ps.size() << std::endl;
	for (int i = 0; i < l_ps.size(); i++)
	{
		cv::Mat left_img = cv::imread((left_path / l_ps[i]).c_str(), cv::IMREAD_UNCHANGED);
		cv::Mat right_img = cv::imread((right_path / r_ps[i]).c_str(), cv::IMREAD_UNCHANGED);
		if (left_img.empty() || right_img.empty())
		{
			std::cout << "image empty , exit !" << std::endl;
			break;
		}
		// cv::imshow("left",left_img);
		// cv::imshow("right",right_img);
		auto now = std::chrono::steady_clock::now();
		auto time = now - start;
		SLAM.TrackStereo(left_img, right_img, static_cast<double>(time.count()) / 1000.0);
		if (cv::waitKey(10) == 27)
			break;
	}
	sleep(3);
	SLAM.Shutdown();

	return 0;
}
