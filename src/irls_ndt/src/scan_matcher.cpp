#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <iostream>

#include "irls_ndt/ndt_scan_matcher.h"

using PointT = pcl::PointXYZI;

std::string target_pcd_file = "/home/plusai/cuda-pcl/cuda-icp/test_P.pcd";
std::string source_pcd_file = "/home/plusai/cuda-pcl/cuda-icp/test_Q.pcd";
double downsample_res = 0.01;

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_matcher");
    ros::Time::init();
    std::cout << "scan matcher";

    auto fast_ndt = irls_ndt::NDTScanMatcher::create(irls_ndt::NDTLocParam());

    pcl::PointCloud<PointT>::Ptr target_cloud(new pcl::PointCloud<PointT>());
    if (-1 == pcl::io::loadPCDFile(target_pcd_file, *target_cloud)) {
        std::cout << "failed to load " << target_pcd_file;
    }
    pcl::PointCloud<PointT>::Ptr source_cloud(new pcl::PointCloud<PointT>());
    if (-1 == pcl::io::loadPCDFile(source_pcd_file, *source_cloud)) {
        std::cout << "failed to load " << source_pcd_file;
    }
    // downsample for efficiency.
    pcl::PointCloud<PointT>::Ptr downsampled(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> voxelgrid;
    voxelgrid.setLeafSize(downsample_res, downsample_res, downsample_res);

    voxelgrid.setInputCloud(target_cloud);
    voxelgrid.filter(*downsampled);
    *target_cloud = *downsampled;

    voxelgrid.setInputCloud(source_cloud);
    voxelgrid.filter(*downsampled);
    source_cloud = downsampled;

    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();

    // set source point cloud
    Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity();
    init_guess.block<3, 3>(0, 0) = quat.toRotationMatrix();
    init_guess.block<3, 1>(0, 3) = pos;
    std::cout << "target v.s. source = " << target_cloud->points.size() << " "
              << source_cloud->points.size();
    int max_run = 10;
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    std::vector<double> v_latency;
    for (int nIter = 0; nIter < max_run; nIter++) {
        auto t0 = ros::Time::now();
        fast_ndt->setInputTarget(target_cloud);
        auto t1 = ros::Time::now();
        double latency_set_target = (t1 - t0).toSec();
        fast_ndt->align(source_cloud, aligned, init_guess);
        auto t2 = ros::Time::now();
        double latency_align = (t2 - t0).toSec();
        v_latency.push_back(latency_align);
        std::cout << "matching time = " << std::setprecision(5) << latency_align * 1000.0
                  << "ms, set_target = " << latency_set_target * 1000.0 << "ms";
    }
    // remove the first latency because it is unstable
    if (v_latency.size() >= 2) {
        v_latency.erase(v_latency.begin());
    }
    auto rstTf = fast_ndt->getFinalTransformation().cast<float>();
    std::cout << "rstTf = \n" << rstTf;
    // save point cloud.
    pcl::transformPointCloud(*source_cloud, *aligned, rstTf);
    // visualization.
    if (true) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
                new pcl::visualization::PCLVisualizer("Scan Matcher"));

        viewer->setBackgroundColor(0.1, 0.1, 0.1);
        viewer->resetCamera();
        viewer->setSize(900, 750);  // (800, 400)
        viewer->setCameraPosition(0, 0, 100, 0, 0, 0);

        viewer->addCoordinateSystem(1);
        viewer->setWindowBorders(true);

        pcl::visualization::PointCloudColorHandlerCustom<PointT> target_handler(
                target_cloud, 255.0, 0.0, 0.0);

        pcl::visualization::PointCloudColorHandlerCustom<PointT> source_handler(
                source_cloud, 0.0, 255.0, 0.0);

        pcl::visualization::PointCloudColorHandlerCustom<PointT> aligned_handler(
                aligned, 0.0, 0.0, 255.0);

        viewer->addPointCloud(target_cloud, target_handler, "target");
        // vis.addPointCloud(source_cloud, source_handler, "source");
        viewer->addPointCloud(aligned, aligned_handler, "aligned");
        viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "target");
        viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "aligned");
        // add text
        char cTmp[2000] = " ";
        double avg_latency = std::accumulate(v_latency.begin(), v_latency.end(), 0.0);
        avg_latency /= v_latency.size();
        sprintf(cTmp,
                "source/target = %04d/%04d, latency = %02dms",
                static_cast<int>(source_cloud->size()),
                static_cast<int>(target_cloud->size()),
                static_cast<int>(avg_latency * 1000.0));
        char cTextId[2000] = "text_id";
        if (!viewer->updateText(cTmp, 10, 100, 20, 1.0, 0.0, 0.0, cTextId)) {
            viewer->addText(cTmp, 10, 100, 20, 1.0, 0.0, 0.0, cTextId, 0);
        }
        viewer->addCoordinateSystem(1.0);
        viewer->spin();
    }
    ros::shutdown();
    return 0;
}
