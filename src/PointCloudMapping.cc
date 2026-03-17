#include "PointCloudMapping.h"
#include <pcl/common/projection_matrix.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/highgui/highgui.hpp>
#include "Converter.h"
#include <KeyFrame.h>

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
    PointCloudMapping::PointCloudMapping(double resolution_, double meank_, double stdthresh_, double unit_, double mindisp_, double maxdisp_, Stereo_Algorithm::AlgorithmType type)
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
        stereo = Stereo_Algorithm::create(mindisp_, maxdisp_, type);
        std::cout << "max disp : " << maxdisp_ << " min disp :" << mindisp_ << std::endl;
        numDisp = static_cast<int>(maxdisp_ - mindisp_);
        // 线程
        viewerThread = make_unique<thread>(bind(&PointCloudMapping::viewer, this));
    }

    void PointCloudMapping::shutdown()
    {
        {
            shutDownFlag.store(true);
            keyFrameUpdated.notify_one();
        }
        viewerThread->join();
        save();
    }

    // key frame insert function
    // rgbd
    void PointCloudMapping::insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
    {
        mSensor = MappingSensor::RGBD;
        cout << "receive a keyframe, id = " << kf->mnId << endl;
        lock_guard<mutex> lck(keyFrameUpdateMutex);
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
        lock_guard<mutex> lck(keyFrameUpdateMutex);
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
        lock_guard<mutex> lck(keyFrameUpdateMutex);
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
        double min = 0;
        double max = 0;
        cv::minMaxLoc(disp, &min, &max);
        std::cout << "min disp : " << min << " max disp : " << max << std::endl;
        // disp = (disp - min) / (max - min)
        cv::Mat disp_vis = ((disp - min) / (max - min)) * 255;
        disp_vis.convertTo(disp_vis, CV_8U);
        cv::imshow("disp_vis", disp_vis);
        cv::waitKey(1);

        // 3D 点转换
        cv::Mat points_image;
        cv::reprojectImageTo3D(disp, points_image, Q);
        points_image.convertTo(points_image, CV_32F);
        PointCloud::Ptr tmp(new PointCloud());

        // double zmax = 0;
        for (int m = static_cast<int>(points_image.rows * 0); m < points_image.rows; m += 1)
        {
            for (int n = static_cast<int>(points_image.cols * 0); n < points_image.cols; n += 1)
            {
                // std::cout<<points_image.ptr<float>(m)[3 * n + 2]<<std::endl;
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
        // std::cout << "z max " << zmax << std::endl;
        //
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
        // std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer(new pcl::visualization::PCLVisualizer("Point cloud Viewer"));
        // std::shared_ptr<pcl::visualization::CloudViewer> cloud_viewer(new pcl::visualization::CloudViewer("Point Cloud"));
        while (1)
        {
            size_t N = 0;
            {
                unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
                // 这里等待 inseart key frame 函数的条件变量
                keyFrameUpdated.wait(lck_keyframeUpdated, [this]()
                                     { return shutDownFlag.load() || lastKeyframeSize < keyframes.size(); });

                if (shutDownFlag.load())
                {
                    break;
                }
                N = keyframes.size();
            }
            // PointCloud::Ptr all_p(new PointCloud());
            PointCloud::Ptr p;
            // check the sensor so that decide the Point cloud generation method
            // rgbd
            if (mSensor == MappingSensor::RGBD)
            {
                for (size_t i = lastKeyframeSize; i < N; i++)
                {
                    p = GetPointCloud(keyframes[i], colorImgs[i], depthImgs[i]);
                    // addCoordinateSystem(pcl_viewer, keyframes[i]->GetPoseInverse().matrix(), std::to_string(i));
                    (*globalMap) += *p;
                }
            }
            // stereo
            else if (mSensor == MappingSensor::STEREO)
            {
                for (size_t i = lastKeyframeSize; i < N; i++)
                {
                    if (dispImgs.size() == 0)
                    {
                        p = GetPointCloud(keyframes[i], colorImgs[i], rightImgs[i], Q);
                        // addCoordinateSystem(pcl_viewer, keyframes[i]->GetPoseInverse().matrix(), std::to_string(i));
                        (*globalMap) += *p;
                    }
                    else
                    {
                        p = GetPointCloud(keyframes[i], colorImgs[i], rightImgs[i], dispImgs[i], Q);
                        // addCoordinateSystem(pcl_viewer, keyframes[i]->GetPoseInverse().matrix(), std::to_string(i));
                        (*globalMap) += *p;
                    }
                }
            }
            // voxel filter
            // PointCloud::Ptr tmp(new PointCloud());
            voxel.setInputCloud(globalMap);
            voxel.filter(*globalMap);

            // outiler filter
            sor.setInputCloud(globalMap);
            sor.filter(*globalMap);
            // print
            std::cout << "the num of points : " << globalMap->points.size() << std::endl;
            // add point cloud
            // pcl_viewer->removePointCloud();
            // pcl_viewer->addPointCloud(globalMap);
            // pcl_viewer->spinOnce(1000);
            // cloud_viewer->showCloud(globalMap);
            lastKeyframeSize = N;
        }
    }

    // save the point cloud to pcd
    void PointCloudMapping::save()
    {
        if (this->globalMap != nullptr)
            // pcl::io::savePCDFile("./PointCloudmapping.pcd", *globalMap);
            pcl::io::savePLYFile("./PointCloudmapping.ply", *globalMap);
    }
}
