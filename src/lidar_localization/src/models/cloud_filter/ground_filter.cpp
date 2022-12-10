/*
 * @Description: 删除地面点
 * @Autor: Li Rui
 * @Date: 2022-09-04 13:36:47
 * @LastEditTime: 2022-12-07 19:23:25
 */
#include "lidar_localization/models/cloud_filter/ground_filter.hpp"

#include "glog/logging.h"

namespace lidar_localization {

GroundFilter::GroundFilter(const YAML::Node& node) {
    bool opt_coeff= node["opt_coeff"].as<int>();    
    float dist_threshold = node["dist_threshold"].as<float>();
    int     max_iter = node["max_iter"].as<int>();
    
    SetFilterParam(opt_coeff, dist_threshold, max_iter);
}

GroundFilter::GroundFilter(bool opt_coeff, float dist_threshold, int  max_iter) {
    SetFilterParam(opt_coeff, dist_threshold, max_iter);
}

bool GroundFilter::SetFilterParam(bool opt_coeff, float dist_threshold, int  max_iter) {
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

bool GroundFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ground_filter_.setInputCloud(input_cloud_ptr);
    ground_filter_.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        LOG(ERROR) << "Ground Filter 的参数为：" << std::endl;
        return false;
    }

    extractor.setInputCloud(input_cloud_ptr);
    extractor.setIndices(inliers);
    extractor.setNegative(true);
    extractor.filter(*filtered_cloud_ptr);

    return true;
}
} 