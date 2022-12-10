/*
 * @Description: 删除地面点
 * @Autor: Li Rui
 * @Date: 2022-09-04 13:36:47
 * @LastEditTime: 2022-12-08 00:29:17
 */
#include "lidar_localization/models/cloud_filter/ground_filter_with_normal.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "glog/logging.h"

namespace lidar_localization {

NormalGroundFilter::NormalGroundFilter(const YAML::Node& node) {
    bool opt_coeff= node["opt_coeff"].as<int>();    
    float dist_threshold = node["dist_threshold"].as<float>();
    int     max_iter = node["max_iter"].as<int>();
    
    SetFilterParam(opt_coeff, dist_threshold, max_iter);
}

NormalGroundFilter::NormalGroundFilter(bool opt_coeff, float dist_threshold, int  max_iter) {
    SetFilterParam(opt_coeff, dist_threshold, max_iter);
}

bool NormalGroundFilter::SetFilterParam(bool opt_coeff, float dist_threshold, int  max_iter) {
    ground_filter_.setOptimizeCoefficients(opt_coeff);
    ground_filter_.setModelType(pcl::SACMODEL_PLANE);
    ground_filter_.setMethodType(pcl::SAC_RANSAC);
    ground_filter_.setMaxIterations(max_iter);
    ground_filter_.setDistanceThreshold(dist_threshold);

    LOG(INFO) << "Ground Filter 的参数为：" << std::endl
              << opt_coeff << ", "
              << max_iter << ", "
              << dist_threshold 
              << std::endl << std::endl;

    return true;
}

CloudData::CLOUD_PTR NormalGroundFilter::normal_filtering(const CloudData::CLOUD_PTR& cloud)  {
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    double normal_filter_thresh=55;
    pcl::search::KdTree<CloudData::POINT>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(10);
    double sensor_height=5;
    ne.setViewPoint(0.0f, 0.0f, sensor_height);
    ne.compute(*normals);

    CloudData::CLOUD_PTR filtered(new CloudData::CLOUD());
    filtered->reserve(cloud->size());

    for(int i = 0; i < cloud->size(); i++) {
    float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
    if(std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0)) {
        filtered->push_back(cloud->at(i));
        }
    }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    return filtered;
}

CloudData::CLOUD_PTR NormalGroundFilter::hill_points(const CloudData::CLOUD_PTR& cloud,const CloudData::CLOUD_PTR& cloud_ground){

    CloudData::CLOUD_PTR cloud_hill(new CloudData::CLOUD());
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeGround;
    kdtreeGround.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeGround->setInputCloud(cloud_ground);

    for(int i=0;i<cloud->points.size();i++)
    {
        std::vector<int> pointSearchInd;
        std::vector<int> pointNotRepeate;
        std::vector<float> pointSearchSqDis;

        pcl::PointXYZI point = cloud->points[i]; // 初始化一个查询点
        kdtreeGround->radiusSearch(point,0.5,pointSearchInd,pointSearchSqDis);

        if(pointSearchInd.size()==0)
        {
            cloud_hill->points.push_back(point);
        }

    }
    return cloud_hill;
}

bool NormalGroundFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
        CloudData::CLOUD_PTR cloud_Remove_hill(new CloudData::CLOUD());
        CloudData::CLOUD_PTR cloud_filtered(new CloudData::CLOUD());
        CloudData::CLOUD_PTR cloud_ground(new CloudData::CLOUD());
        CloudData::CLOUD_PTR cloud_tmp(new CloudData::CLOUD());
        CloudData::CLOUD_PTR cloud_outliers(new CloudData::CLOUD());
        CloudData::CLOUD_PTR cloud_out(new CloudData::CLOUD());

        double heightThreshold=1.5;
        double attach=0.05;
        pcl::PassThrough<pcl::PointXYZI> passthrough;
        //在使用ransac之前先把山体的点尽可能去掉
        Eigen::Vector4f centroid;					// 现在的地图的质心
        pcl::compute3DCentroid(*input_cloud_ptr, centroid);	// 齐次坐标
        passthrough.setInputCloud(input_cloud_ptr);//在入点云
        passthrough.setFilterFieldName("z");//滤波的字段，即滤波的方向。可以是XYZ也可以是BGR
        passthrough.setFilterLimits(centroid(2)-20*heightThreshold, centroid(2)+heightThreshold);//滤除在z轴方向上不在0.0-0.1范围内的所有点
        //Passthrough.setFilterLimitsNegative (true);//设置保留范围内部还是范围外部  默认false为内部
        passthrough.filter(*cloud_Remove_hill);//开始滤波，结果输出至cloud_filtered中

        cloud_filtered=normal_filtering(cloud_Remove_hill);        

        //use the ransac to remove the plane
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMaxIterations(10000);
        double Threshold=0.3+attach;
        // double Threshold=0.35;
        // double Threshold=0.3;
        // std::cout<<"Threshold: "<<Threshold<<"\n";
        seg.setDistanceThreshold(Threshold);

        seg.setInputCloud(cloud_filtered);

        seg.segment(*inliers, *coefficients);
        

        if (inliers->indices.size() == 0)
        {
            cout << "error! Could not found any inliers!" << endl;
            return false;
        }
        cloud_ground->reserve(inliers->indices.size());

        for(int i=0;i<inliers->indices.size();i++)
        {
            cloud_ground->points.push_back(cloud_filtered->points[inliers->indices[i]]);
        }

        cloud_tmp=hill_points(input_cloud_ptr,cloud_ground);

        //取逆，然后找到和地面点相反的点，从而起到去除的效果
        // pcl::ExtractIndices<pcl::PointXYZI> extractor;
        // extractor.setInputCloud(input_cloud_ptr);
        // extractor.setIndices(inliers);
        // extractor.setNegative(true);
        // std::cout<<"改变"<<"\n";
        // extractor.filter(*cloud_filtered);

        //use the radiusoutler filter to remove some outliers
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> radiusoutlier;
        radiusoutlier.setInputCloud(cloud_tmp);
        // radiusoutlier.setInputCloud(cloud_Remove_hill);
        float filter_radius=0.6;
        int filter_count=8;
        radiusoutlier.setRadiusSearch(filter_radius);
        radiusoutlier.setMinNeighborsInRadius(filter_count);
        radiusoutlier.filter(*cloud_outliers);

        cloud_out = cloud_outliers;
        filtered_cloud_ptr=cloud_out;

        // filtered_cloud_ptr=cloud_tmp;

        return true;
}
} 