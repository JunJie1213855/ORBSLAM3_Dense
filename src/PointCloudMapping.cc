#include "PointCloudMapping.h"
#include <pcl/common/projection_matrix.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <opencv2/highgui/highgui.hpp>
#include "Converter.h"
#include <KeyFrame.h>
#include <unordered_map>
#include <cmath>
#include <chrono>

namespace ORB_SLAM3
{

    pcl::PointXYZ pcl_transform(const Eigen::Matrix4f &pose, const Eigen::Vector3f &p)
    {
        pcl::PointXYZ pcl_p;
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.matrix() = pose;
        Eigen::Vector3f e_p = transform * p;
        return pcl::PointXYZ(e_p(0), e_p(1), e_p(2));
    }

    void PointCloudMapping::addCoordinateSystem(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer, const Eigen::Matrix4f &pose, const std::string &prefix)
    {

        Eigen::Vector3f o(0, 0, 0);
        Eigen::Vector3f x_axis(this->unit * 0.25, 0, 0);
        Eigen::Vector3f y_axis(0, this->unit * 0.25, 0);
        Eigen::Vector3f z_axis(0, 0, this->unit * 0.25);

        viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl_transform(pose, o), pcl_transform(pose, x_axis), 1.0, 0.0, 0.0, prefix + "_x");
        viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl_transform(pose, o), pcl_transform(pose, y_axis), 0.0, 1.0, 0.0, prefix + "_y");
        viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl_transform(pose, o), pcl_transform(pose, z_axis), 0.0, 0.0, 1.0, prefix + "_z");
    }
    // point cloud mapping
    PointCloudMapping::PointCloudMapping(double resolution_, double meank_, double stdthresh_, double unit_)
    {
        std::cout << "initializa with disp images !" << std::endl;
        // 体素采样
        std::cout << "the resolution of Point cloud Voxel filter : " << resolution_ << std::endl;
        this->resolution = resolution_;
        voxel.setLeafSize(resolution, resolution, resolution);
        // 离群滤波
        std::cout << "the Point cloud Outlier filter params :   \n"
                  << "meank : " << meank_ << std::endl
                  << "stdthresh :" << stdthresh_ << std::endl;
        this->meank = meank_;
        this->stdthresh = stdthresh_;
        sor.setMeanK(meank);
        sor.setStddevMulThresh(stdthresh);
        // 单位
        std::cout << "the Unit of Point cloud : " << unit_ << std::endl;
        this->unit = unit_;
        // 全局点云
        globalMap.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        // 线程
        viewerThread = make_unique<thread>(bind(&PointCloudMapping::viewer, this));
    }
    PointCloudMapping::PointCloudMapping(
        double resolution_,
        double meank_,
        double stdthresh_,
        double unit_,
        double mindisp_,
        double maxdisp_,
        Stereo_Algorithm::AlgorithmType type,
        cv::Size input_size,
        const std::string &model_path)
    {
        std::cout << "initializa without disp images ,so create a stereo match algorithm!" << std::endl;
        // 体素采样
        std::cout << "the resolution of Point cloud Voxel filter : " << resolution_ << std::endl;
        this->resolution = resolution_;
        voxel.setLeafSize(resolution, resolution, resolution);
        // 离群滤波
        std::cout << "the Point cloud Outlier filter params :   \n"
                  << "meank : " << meank_ << std::endl
                  << "stdthresh :" << stdthresh_ << std::endl;
        this->meank = meank_;
        this->stdthresh = stdthresh_;
        sor.setMeanK(meank);
        sor.setStddevMulThresh(stdthresh);
        // 单位
        std::cout << "the Unit of Point cloud : " << unit_ << std::endl;
        this->unit = unit_;
        // 全局点云
        globalMap.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        // 视差算法
        stereo = Stereo_Algorithm::create(mindisp_, maxdisp_, type, input_size, model_path);

        std::cout << "max disp : " << maxdisp_ << " min disp :" << mindisp_ << std::endl;
        numDisp = static_cast<int>(maxdisp_ - mindisp_);

        // 线程
        viewerThread = make_unique<thread>(bind(&PointCloudMapping::viewer, this));
    }

    void PointCloudMapping::shutdown()
    {
        // std::cout << "[DEBUG] shutdown requested, notifying viewer..." << std::endl;
        {
            shutDownFlag.store(true);
            keyFrameUpdated.notify_all();
        }
        // Wait with periodic re-notify to handle missed wakeups
        // std::cout << "[DEBUG] waiting for viewer thread to finish..." << std::endl;
        while (viewerThread->joinable())
        {
            keyFrameUpdated.notify_all();
            // Use timed join to avoid permanent hang
            auto start = std::chrono::steady_clock::now();
            // poll every 500ms
            while (viewerThread->joinable() &&
                   std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (viewerThread->joinable())
            {
                // std::cout << "[DEBUG] viewer thread still running, re-notifying..." << std::endl;
                keyFrameUpdated.notify_all();
            }
        }
        // std::cout << "[DEBUG] viewer thread joined, saving..." << std::endl;
        save();
    }

    // key frame insert function
    // rgbd
    void PointCloudMapping::insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
    {
        mSensor = MappingSensor::RGBD;
        cout << "receive a keyframe, id = " << kf->mnId << endl;
        unique_lock<mutex> lck(keyFrameUpdateMutex);
        // 数据
        keyframes.push_back(kf);
        colorImgs.push_back(color.clone());
        depthImgs.push_back(depth.clone());
        // 线程阻塞
        keyFrameUpdated.notify_one();
    }
    // stereo without the disparity
    void PointCloudMapping::insertKeyFrame(KeyFrame *kf, cv::Mat &left, cv::Mat &right, cv::Mat &Q)
    {
        mSensor = MappingSensor::STEREO;
        cout << "receive a keyframe, id = " << kf->mnId << endl;
        unique_lock<mutex> lck(keyFrameUpdateMutex);
        // 数据
        keyframes.push_back(kf);
        colorImgs.push_back(left.clone());
        rightImgs.push_back(right.clone());
        this->Q = Q.clone();
        // 线程阻塞
        keyFrameUpdated.notify_one();
    }
    // stereo with the disparity
    void PointCloudMapping::insertKeyFrame(KeyFrame *kf, cv::Mat &left, cv::Mat &right, cv::Mat &disp, cv::Mat &Q)
    {
        mSensor = MappingSensor::STEREO;
        cout << "receive a keyframe, id = " << kf->mnId << endl;
        unique_lock<mutex> lck(keyFrameUpdateMutex);
        // 数据
        keyframes.push_back(kf);
        colorImgs.push_back(left.clone());
        rightImgs.push_back(right.clone());
        dispImgs.push_back(disp);
        this->Q = Q.clone();
        // 线程阻塞
        keyFrameUpdated.notify_one();
    }

    // point cloud  generation function
    // rgbd
    pcl::PointCloud<PointCloudMapping::PointT>::Ptr PointCloudMapping::GetPointCloud(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
    {
        PointCloud::Ptr tmp(new PointCloud());
        // depth convert
        if (depth.type() != CV_32F)
            depth.convertTo(depth, CV_32F);

        // point cloud convert
        for (int m = 0; m < depth.rows; m += 1)
        {
            for (int n = 0; n < depth.cols; n += 1)
            {
                float d = depth.ptr<float>(m)[n];
                if (d / unit < 0.01 || d / unit > 10.0)
                    continue;
                PointT p;
                p.z = d / unit;
                p.x = (n - kf->cx) * p.z / kf->fx;
                p.y = (m - kf->cy) * p.z / kf->fy;

                p.b = color.ptr<uchar>(m)[n * 3];
                p.g = color.ptr<uchar>(m)[n * 3 + 1];
                p.r = color.ptr<uchar>(m)[n * 3 + 2];

                tmp->points.push_back(p);
            }
        }

        // get the transform point cloud
        PointCloud::Ptr cloud(new PointCloud());
        Eigen::Matrix4f transform = kf->GetPoseInverse().matrix();
        cloud->width = tmp->width;
        cloud->height = tmp->height;
        cloud->is_dense = tmp->is_dense;
        cloud->points.resize(tmp->points.size());

        for (size_t i = 0; i < tmp->points.size(); ++i)
        {
            const PointT &pt = tmp->points[i];
            Eigen::Vector4f v(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4f vt = transform * v;
            cloud->points[i].x = vt[0];
            cloud->points[i].y = vt[1];
            cloud->points[i].z = vt[2];
            cloud->points[i].r = pt.r;
            cloud->points[i].g = pt.g;
            cloud->points[i].b = pt.b;
        }
        // print
        // cout << "generate point cloud for kf " << kf->mnId << ", size=" << cloud->points.size() << endl;
        return cloud;
    }

    // stereo without the disparity image
    pcl::PointCloud<PointCloudMapping::PointT>::Ptr PointCloudMapping::GetPointCloud(KeyFrame *kf, cv::Mat &left, cv::Mat &right, cv::Mat &Q)
    {
        cv::Mat left_r = left.clone();
        cv::Mat right_r = right.clone();
        // 视差计算
        cv::Mat disp = stereo->inference(left_r, right_r);

        // 3D 点转换
        cv::Mat points_image;
        cv::reprojectImageTo3D(disp, points_image, Q);
        points_image.convertTo(points_image, CV_32F);

        // Diagnostic: check data quality
        {
            double dmin, dmax;
            cv::minMaxLoc(disp, &dmin, &dmax);
            // std::cout << "[DEBUG GetPC] disp range: " << dmin << " ~ " << dmax
            //           << ", disp type=" << disp.type() << " (CV_32F=" << CV_32F << " CV_16S=" << CV_16S << ")"
            //           << ", Q=" << Q << std::endl;
            // Check for NaN/Inf in points_image
            int nan_count = 0, inf_count = 0;
            float zmin = 1e9, zmax = -1e9;
            for (int r = 0; r < points_image.rows; r++) {
                const float* ptr = points_image.ptr<float>(r);
                for (int c = 0; c < points_image.cols; c++) {
                    float z = ptr[3*c + 2];
                    if (std::isnan(z)) nan_count++;
                    if (std::isinf(z)) inf_count++;
                    if (z > zmax) zmax = z;
                    if (z < zmin) zmin = z;
                }
            }
            // std::cout << "[DEBUG GetPC] 3D points Z range: " << zmin << " ~ " << zmax
                    //   << ", NaN=" << nan_count << " Inf=" << inf_count
                    //   << ", total pixels=" << (points_image.rows * points_image.cols) << std::endl;
        }

        PointCloud::Ptr tmp(new PointCloud());

        // double zmax = 0;
        for (int m = static_cast<int>(points_image.rows * 0); m < points_image.rows; m += 1)
        {
            for (int n = static_cast<int>(points_image.cols * 0); n < points_image.cols; n += 1)
            {
                // // std::cout<<points_image.ptr<float>(m)[3 * n + 2]<<std::endl;
                // 深度阈值, 单位为 unit, 0.25 m < z < 10 m
                if ((points_image.ptr<float>(m)[3 * n + 2] / unit) > 50 ||
                    (points_image.ptr<float>(m)[3 * n + 2] / unit) < 1.0)
                    continue;
                PointT p;
                p.x = points_image.ptr<float>(m)[3 * n];
                p.y = points_image.ptr<float>(m)[3 * n + 1];
                p.z = points_image.ptr<float>(m)[3 * n + 2];
                p.b = left_r.ptr<uchar>(m)[n * 3];
                p.g = left_r.ptr<uchar>(m)[n * 3 + 1];
                p.r = left_r.ptr<uchar>(m)[n * 3 + 2];
                tmp->points.push_back(p);
            }
        }

        PointCloud::Ptr cloud(new PointCloud());
        Eigen::Matrix4f transform = kf->GetPoseInverse().matrix();
        cloud->width = tmp->width;
        cloud->height = tmp->height;
        cloud->is_dense = tmp->is_dense;
        cloud->points.resize(tmp->points.size());

        for (size_t i = 0; i < tmp->points.size(); ++i)
        {
            const PointT &pt = tmp->points[i];
            Eigen::Vector4f v(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4f vt = transform * v;
            cloud->points[i].x = vt[0];
            cloud->points[i].y = vt[1];
            cloud->points[i].z = vt[2];
            cloud->points[i].r = pt.r;
            cloud->points[i].g = pt.g;
            cloud->points[i].b = pt.b;
        }
        // // std::cout << "z max " << zmax << std::endl;
        // Diagnostic: check world-frame point cloud
        {
            float cxmin=1e9, cxmax=-1e9, cymin=1e9, cymax=-1e9, czmin=1e9, czmax=-1e9;
            int cnan=0, cinf=0;
            for (size_t i=0; i<cloud->points.size(); i++) {
                float x=cloud->points[i].x, y=cloud->points[i].y, z=cloud->points[i].z;
                if(std::isnan(x)||std::isnan(y)||std::isnan(z)) cnan++;
                if(std::isinf(x)||std::isinf(y)||std::isinf(z)) cinf++;
                if(x<cxmin) cxmin=x; if(x>cxmax) cxmax=x;
                if(y<cymin) cymin=y; if(y>cymax) cymax=y;
                if(z<czmin) czmin=z; if(z>czmax) czmax=z;
            }
            // std::cout << "[DEBUG GetPC] world cloud: X["<<cxmin<<","<<cxmax<<"] Y["<<cymin<<","<<cymax<<"] Z["<<czmin<<","<<czmax<<"], NaN="<<cnan<<" Inf="<<cinf<< std::endl;
        }
        cout << "generate point cloud for kf " << kf->mnId << ", size=" << cloud->points.size() << endl;
        return cloud;
    }

    // stereo with the disparity image
    pcl::PointCloud<PointCloudMapping::PointT>::Ptr PointCloudMapping::GetPointCloud(KeyFrame *kf, cv::Mat &left, cv::Mat &right, cv::Mat &disp, cv::Mat &Q)
    {
        cv::Mat disp_c = disp.clone();
        cv::Mat left_r = left.clone();
        // convert the type
        if (disp_c.type() != CV_16S)
            disp_c.convertTo(disp_c, CV_16S);
        // 3D 点转换
        cv::Mat points_image;
        cv::reprojectImageTo3D(disp_c, points_image, Q, true, CV_32F);
        PointCloud::Ptr tmp(new PointCloud());
        // // 选取有效区域作为稠密重建
        for (int m = static_cast<int>(points_image.rows * 0.25); m < static_cast<int>(points_image.rows * 0.75); m += 1)
        {
            for (int n = static_cast<int>(points_image.cols * 0.25); n < static_cast<int>(points_image.cols * 0.75); n += 1)
            {

                // 深度阈值, 单位为 unit, 0.25 m < z < 10 m
                if ((points_image.ptr<float>(m)[3 * n + 2] / unit) > 10 ||
                    (points_image.ptr<float>(m)[3 * n + 2] / unit) < 0.3)
                    continue;
                PointT p;
                p.x = points_image.ptr<float>(m)[3 * n];
                p.y = points_image.ptr<float>(m)[3 * n + 1];
                p.z = points_image.ptr<float>(m)[3 * n + 2];
                p.b = left_r.ptr<uchar>(m)[n * 3];
                p.g = left_r.ptr<uchar>(m)[n * 3 + 1];
                p.r = left_r.ptr<uchar>(m)[n * 3 + 2];
                tmp->points.push_back(p);
            }
        }
        PointCloud::Ptr cloud(new PointCloud());
        Eigen::Matrix4f transform = kf->GetPoseInverse().matrix();
        cloud->width = tmp->width;
        cloud->height = tmp->height;
        cloud->is_dense = tmp->is_dense;
        cloud->points.resize(tmp->points.size());

        for (size_t i = 0; i < tmp->points.size(); ++i)
        {
            const PointT &pt = tmp->points[i];
            Eigen::Vector4f v(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4f vt = transform * v;
            cloud->points[i].x = vt[0];
            cloud->points[i].y = vt[1];
            cloud->points[i].z = vt[2];
            cloud->points[i].r = pt.r;
            cloud->points[i].g = pt.g;
            cloud->points[i].b = pt.b;
        }
        //
        // cout << "generate point cloud for kf " << kf->mnId << ", size=" << cloud->points.size() << endl;
        return cloud;
    }

    // mapping viewer thread
    void PointCloudMapping::viewer()
    {
        // // std::cout << "[DEBUG viewer] thread started" << std::endl;
        // pcl::visualization::CloudViewer viewer("check viewer");
        while (1)
        {
            try
            {
                // std::cout << "[DEBUG viewer] === loop begin, lastKeyframeSize=" << lastKeyframeSize << " ===" << std::endl;
                size_t N = 0;
                std::vector<KeyFrame *> local_kfs;
                std::vector<cv::Mat> local_colors, local_rights, local_depths, local_disps;
                cv::Mat local_Q;
                {
                    // std::cout << "[DEBUG viewer] waiting for keyframes..." << std::endl;
                    unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
                    keyFrameUpdated.wait(lck_keyframeUpdated, [this]()
                                         { return shutDownFlag.load() || lastKeyframeSize < keyframes.size(); });
                    if (shutDownFlag.load())
                    {
                        // std::cout << "[DEBUG viewer] shutdown flag set, exiting" << std::endl;
                        break;
                    }
                    N = keyframes.size();
                    // std::cout << "[DEBUG viewer] woke up, total KFs=" << N << ", new KFs=" << (N - lastKeyframeSize) << std::endl;
                    for (size_t i = lastKeyframeSize; i < N; i++)
                    {
                        // std::cout << "[DEBUG viewer] copying KF[" << i << "] id=" << keyframes[i]->mnId << std::endl;
                        local_kfs.push_back(keyframes[i]);
                        local_colors.push_back(colorImgs[i].clone());
                        if (mSensor == MappingSensor::RGBD)
                            local_depths.push_back(depthImgs[i].clone());
                        else if (mSensor == MappingSensor::STEREO)
                        {
                            local_rights.push_back(rightImgs[i].clone());
                            if (!dispImgs.empty())
                                local_disps.push_back(dispImgs[i].clone());
                        }
                    }
                    local_Q = Q.clone();
                    // std::cout << "[DEBUG viewer] data copied, lock released" << std::endl;
                }
                // Process from local copies
                PointCloud::Ptr p;
                if (mSensor == MappingSensor::RGBD)
                {
                    for (size_t i = 0; i < local_kfs.size(); i++)
                    {
                        // std::cout << "[DEBUG viewer] RGBD GetPointCloud for KF id=" << local_kfs[i]->mnId << std::endl;
                        p = GetPointCloud(local_kfs[i], local_colors[i], local_depths[i]);
                        // std::cout << "[DEBUG viewer] RGBD cloud size=" << p->points.size() << ", globalMap before merge=" << globalMap->points.size() << std::endl;
                        (*globalMap) += *p;
                        // std::cout << "[DEBUG viewer] RGBD after merge, globalMap size=" << globalMap->points.size() << std::endl;
                        p.reset();
                    }
                }
                else if (mSensor == MappingSensor::STEREO)
                {
                    for (size_t i = 0; i < local_kfs.size(); i++)
                    {
                        // std::cout << "[DEBUG viewer] STEREO GetPointCloud for KF id=" << local_kfs[i]->mnId << std::endl;
                        if (local_disps.empty())
                        {
                            p = GetPointCloud(local_kfs[i], local_colors[i], local_rights[i], local_Q);
                            // std::cout << "[DEBUG viewer] STEREO cloud size=" << p->points.size() << ", globalMap before merge=" << globalMap->points.size() << std::endl;
                            (*globalMap) += *p;
                            // std::cout << "[DEBUG viewer] STEREO after merge, globalMap size=" << globalMap->points.size() << std::endl;
                            p.reset();
                        }
                        else
                        {
                            p = GetPointCloud(local_kfs[i], local_colors[i], local_rights[i], local_disps[i], local_Q);
                            // std::cout << "[DEBUG viewer] STEREO(disp) cloud size=" << p->points.size() << ", globalMap before merge=" << globalMap->points.size() << std::endl;
                            (*globalMap) += *p;
                            // std::cout << "[DEBUG viewer] STEREO(disp) after merge, globalMap size=" << globalMap->points.size() << std::endl;
                            p.reset();
                        }
                    }
                }
                // Manual voxel filter + outlier removal (bypass PCL filter bugs)
                if (globalMap && !globalMap->points.empty())
                {
                    auto t0 = std::chrono::steady_clock::now();
                    auto t1 = t0;
                    // std::cout << "[DEBUG viewer] manual filter start, input=" << globalMap->points.size()
                            //   << " leaf=" << resolution << std::endl;

                    // Step 1: Simple voxel grid using hash map
                    {
                        auto voxel_hash = [this](const PointT& pt) -> uint64_t {
                            int64_t ix = static_cast<int64_t>(std::floor(pt.x / resolution));
                            int64_t iy = static_cast<int64_t>(std::floor(pt.y / resolution));
                            int64_t iz = static_cast<int64_t>(std::floor(pt.z / resolution));
                            // FNV-like hash
                            uint64_t h = 14695981039346656037ULL;
                            h = (h ^ static_cast<uint64_t>(ix)) * 1099511628211ULL;
                            h = (h ^ static_cast<uint64_t>(iy)) * 1099511628211ULL;
                            h = (h ^ static_cast<uint64_t>(iz)) * 1099511628211ULL;
                            return h;
                        };
                        auto voxel_equal = [this](const PointT& a, const PointT& b) -> bool {
                            int64_t ax = static_cast<int64_t>(std::floor(a.x / resolution));
                            int64_t ay = static_cast<int64_t>(std::floor(a.y / resolution));
                            int64_t az = static_cast<int64_t>(std::floor(a.z / resolution));
                            int64_t bx = static_cast<int64_t>(std::floor(b.x / resolution));
                            int64_t by = static_cast<int64_t>(std::floor(b.y / resolution));
                            int64_t bz = static_cast<int64_t>(std::floor(b.z / resolution));
                            return ax == bx && ay == by && az == bz;
                        };
                        std::unordered_map<PointT, bool, decltype(voxel_hash), decltype(voxel_equal)> voxel_map(
                            1024, voxel_hash, voxel_equal);
                        PointCloud::Ptr filtered(new PointCloud());
                        filtered->points.reserve(globalMap->points.size() / 10);
                        for (const auto& pt : globalMap->points) {
                            if (shutDownFlag.load()) break;
                            if (voxel_map.find(pt) == voxel_map.end()) {
                                voxel_map[pt] = true;
                                filtered->points.push_back(pt);
                            }
                        }
                        if (!shutDownFlag.load()) {
                            *globalMap = *filtered;
                        }
                        t1 = std::chrono::steady_clock::now();
                        // std::cout << "[DEBUG viewer] voxel done, points=" << globalMap->points.size()
                                //   << " (" << std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count() << "ms)" << std::endl;
                    }

                    // Step 2: Simple statistical outlier removal
                    if (globalMap->points.size() > static_cast<size_t>(meank)) {
                        PointCloud::Ptr filtered(new PointCloud());
                        filtered->points.reserve(globalMap->points.size());
                        // Build a simple spatial hash for neighbor search
                        float search_radius = resolution * 3.0f;
                        auto cell_hash = [search_radius](float x, float y, float z) -> uint64_t {
                            int64_t ix = static_cast<int64_t>(std::floor(x / search_radius));
                            int64_t iy = static_cast<int64_t>(std::floor(y / search_radius));
                            int64_t iz = static_cast<int64_t>(std::floor(z / search_radius));
                            uint64_t h = 14695981039346656037ULL;
                            h = (h ^ static_cast<uint64_t>(ix)) * 1099511628211ULL;
                            h = (h ^ static_cast<uint64_t>(iy)) * 1099511628211ULL;
                            h = (h ^ static_cast<uint64_t>(iz)) * 1099511628211ULL;
                            return h;
                        };
                        std::unordered_map<uint64_t, std::vector<size_t>> cells;
                        for (size_t i = 0; i < globalMap->points.size(); i++) {
                            const auto& pt = globalMap->points[i];
                            uint64_t key = cell_hash(pt.x, pt.y, pt.z);
                            cells[key].push_back(i);
                        }
                        // Filter: keep points with enough neighbors in adjacent cells
                        for (size_t i = 0; i < globalMap->points.size() && !shutDownFlag.load(); i++) {
                            const auto& pt = globalMap->points[i];
                            uint64_t base_key = cell_hash(pt.x, pt.y, pt.z);
                            int neighbors = 0;
                            // Check 3x3x3 cell neighborhood
                            for (int dx = -1; dx <= 1 && neighbors < meank; dx++) {
                                for (int dy = -1; dy <= 1 && neighbors < meank; dy++) {
                                    for (int dz = -1; dz <= 1 && neighbors < meank; dz++) {
                                        int64_t ix = static_cast<int64_t>(std::floor(pt.x / search_radius)) + dx;
                                        int64_t iy = static_cast<int64_t>(std::floor(pt.y / search_radius)) + dy;
                                        int64_t iz = static_cast<int64_t>(std::floor(pt.z / search_radius)) + dz;
                                        uint64_t h = 14695981039346656037ULL;
                                        h = (h ^ static_cast<uint64_t>(ix)) * 1099511628211ULL;
                                        h = (h ^ static_cast<uint64_t>(iy)) * 1099511628211ULL;
                                        h = (h ^ static_cast<uint64_t>(iz)) * 1099511628211ULL;
                                        auto it = cells.find(h);
                                        if (it != cells.end()) neighbors += it->second.size();
                                    }
                                }
                            }
                            if (neighbors >= meank) {
                                filtered->points.push_back(pt);
                            }
                        }
                        if (!shutDownFlag.load()) {
                            *globalMap = *filtered;
                            // viewer.showCloud(globalMap);
                        }
                        auto t2 = std::chrono::steady_clock::now();
                        // std::cout << "[DEBUG viewer] outlier done, points=" << globalMap->points.size()
                                //   << " (" << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() << "ms)" << std::endl;
                    }
                    else {
                        // std::cout << "[DEBUG viewer] too few points for outlier filter, skipped" << std::endl;
                    }
                }
                // std::cout << "[DEBUG viewer] tmp clouds released" << std::endl;
                lastKeyframeSize = N;
                // std::cout << "[DEBUG viewer] === loop end, lastKeyframeSize=" << lastKeyframeSize << " ===" << std::endl;
            }
            catch (const std::exception &e)
            {
                std::cerr << "[DEBUG viewer] EXCEPTION: " << e.what() << std::endl;
            }
            catch (...)
            {
                std::cerr << "[DEBUG viewer] UNKNOWN EXCEPTION" << std::endl;
            }
        }
        // std::cout << "[DEBUG viewer] thread exiting" << std::endl;
    }

    // save the point cloud to ply
    void PointCloudMapping::save()
    {
        if (this->globalMap != nullptr)
        {
            pcl::PLYWriter writer;
            writer.write<PointT>("./PointCloudmapping.ply", *globalMap, false);
        }
    }
}
