/*
 * @Description: 删除地面点
 * @Autor: Li Rui
 * @Date: 2022-09-04 13:36:47
 * @LastEditTime: 2022-12-07 19:22:06
 */
#include "lidar_localization/models/cloud_filter/outliers_filter.hpp"

#include "glog/logging.h"

namespace lidar_localization {

OutliersFilter::OutliersFilter(const YAML::Node& node) {
    float  filter_radius=node["filter_radius"].as<float>();
    int    filter_count=node["filter_count"].as<int>();
    
    SetFilterParam(filter_radius,filter_count);
}

OutliersFilter::OutliersFilter(float filter_radius,int filter_count) {
    SetFilterParam(filter_radius,filter_count);
}

bool OutliersFilter::SetFilterParam(float filter_radius,int filter_count) {
    radius_outlier.setRadiusSearch(filter_radius);
    radius_outlier.setMinNeighborsInRadius(filter_count);


    LOG(INFO) << "OutliersFilter 的参数为：" << std::endl
              <<filter_radius<<", "
              <<filter_count<<", "
              << std::endl << std::endl;

    return true;
}

bool OutliersFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {

    // //添加去除噪声点的代码
    radius_outlier.setInputCloud(input_cloud_ptr);
    radius_outlier.filter(*filtered_cloud_ptr);

    return true;
}
} 