#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include "Settings.h"
#include "StereoMatch.h"

#include <opencv2/core.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>
#include <atomic>
#include <unordered_set>
#include <memory>
#include <cstdint>
#include <Eigen/Core>

namespace ORB_SLAM3
{
    // stereo map
    class PointCloudMapping
    {

        enum MappingSensor
        {
            RGBD = 0,
            STEREO
        };

        using Stereo_type = Stereo_Algorithm::AlgorithmType;

    public:
        using PointT = pcl::PointXYZRGB;
        using PointCloud = pcl::PointCloud<PointT>;
        // initial
        PointCloudMapping(double resolution_, double meank_, double stdthresh_, double unit_);

        // initial with the stereo matching algorithm
        PointCloudMapping(double resolution_, 
            double meank_, 
            double stdthresh_, 
            double unit_, 
            double mindisp_, 
            double maxdisp_, 
            Stereo_type type = Stereo_type::ELAS,
            cv::Size input_size = cv::Size(),
            const std::string& model_path = ""
        );

        // add the Coordinate which stand for the pose into the pcl viewer
        /**@brief
         * @param viewer the pcl viewer
         * @param pose the pose of the camera
         * @param prefix the Coordinate name
         */
        void addCoordinateSystem(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer, const Eigen::Matrix4f &pose, const std::string &prefix);

        // shut down the mapping
        // save the point cloud mappping
        void shutdown();

        // viewer thread (point cloud generation / processing)
        void viewer();

        // render thread (owns Pangolin GL context, ~30-60 Hz)
        void renderLoop();

        // save the point cloud to pcd
        void save();

    public:
        // -----------------------------------------------------------------------------------
        // 插入keyframe，并且更新地图
        // rgbd
        void insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth);

        // 插入keyframe，并且更新地图
        // stereo and elas
        void insertKeyFrame(KeyFrame *kf, cv::Mat &left, cv::Mat &right, cv::Mat &Q);

        // 插入keyframe，并且更新地图
        // stereo with the disp
        void insertKeyFrame(KeyFrame *kf, cv::Mat &left, cv::Mat &right, cv::Mat &disp, cv::Mat &Q);

    protected:
        // rgbd dense mapping
        //
        PointCloud::Ptr GetPointCloud(KeyFrame *kf, cv::Mat &left, cv::Mat &depth);

        // stereo dense mapping build
        // input could be RGB or Gray
        // deal with the disparity image by elas algorithm
        PointCloud::Ptr GetPointCloud(KeyFrame *kf, cv::Mat &left, cv::Mat &right, cv::Mat &Q);

        // stereo dense mapping build
        // input could be RGB or Gray
        // the difference bewteen the front one is that the input include the disparity image
        PointCloud::Ptr GetPointCloud(KeyFrame *kf, cv::Mat &left, cv::Mat &right, cv::Mat &disp, cv::Mat &Q);

        // Manual filters (bypass PCL bugs with in-place filtering)
        void VoxelFilter(PointCloud::Ptr& cloud);
        void OutlierFilter(PointCloud::Ptr& cloud);

    public:
        // to judge the sensor is rgbd or stereo
        MappingSensor mSensor;

    protected:
        // 视差计算
        std::shared_ptr<Stereo_Algorithm> stereo;
        int numDisp = 0;

        // global map
        PointCloud::Ptr globalMap;

        // 单一线程 , 用 std::move
        std::unique_ptr<thread> viewerThread;

        // 专用渲染线程 (拥有 Pangolin GL 上下文)
        std::unique_ptr<std::thread> renderThread;

        // 保护 globalMap 的互斥锁 (处理线程 <-> 渲染线程)
        std::mutex mMutexGlobalMap;

        // 点云脏标志: 处理线程更新 globalMap 后置 true, 渲染线程上传 VBO 后置 false
        std::atomic<bool> mbCloudUpdated{false};

        // 相机位姿快照 (Twc), 供渲染线程绘制坐标轴
        // aligned_allocator: Matrix4f 在 AVX (-march=native) 下需 32 字节对齐, 否则崩溃
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> mPoseSnapshot;
        std::mutex mMutexPoses;

        // 已占据体素集合, 用于增量体素降采样 (O(new points))
        std::unordered_set<uint64_t> mOccupiedVoxels;

        // 显示抽稀步长 (点云过大时按 stride 上传显示)
        int mDisplayStride = 1;

        // 视角跟随相机开关 (默认开启, 渲染窗口按 'f' 键切换)
        std::atomic<bool> mbFollowCamera{true};

        // 关机标志
        std::atomic<bool> shutDownFlag{false};

        // 线程阻塞变量
        std::condition_variable keyFrameUpdated;
        std::mutex keyFrameUpdateMutex;

        // 生成点云的数据
        std::vector<KeyFrame *> keyframes;
        std::vector<cv::Mat> colorImgs;
        std::vector<cv::Mat> depthImgs;
        // stereo
        std::vector<cv::Mat> rightImgs;
        std::vector<cv::Mat> dispImgs;
        cv::Mat Q;

        // 处理完的上一帧编号
        uint16_t lastKeyframeSize = 0;

        // 降采样
        double resolution = 0.04;
        pcl::VoxelGrid<PointT> voxel;

        // 离群点去除
        double meank = 10;
        double stdthresh = 1;
        pcl::StatisticalOutlierRemoval<PointT> sor;
        // 单位
        double unit = 1000; // defalut mm
    };
}

#endif
