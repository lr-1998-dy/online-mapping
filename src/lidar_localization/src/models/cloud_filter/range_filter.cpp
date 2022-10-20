/*
 * @Description: 删除地面点
 * @Autor: Li Rui
 * @Date: 2022-09-04 13:36:47
 * @LastEditTime: 2022-10-18 14:10:33
 */
#include "lidar_localization/models/cloud_filter/range_filter.hpp"

#include "glog/logging.h"

namespace lidar_localization {

RangeFilter::RangeFilter(const YAML::Node& node) {
    float min_distance= node["min_distance"].as<float>();    
    float max_distance = node["max_distance"].as<float>();
    
    SetFilterParam(min_distance, max_distance);
}

RangeFilter::RangeFilter(float min_distance,float max_distance) {
    SetFilterParam(min_distance, max_distance);
}

bool RangeFilter::SetFilterParam(float min_distance,float max_distance) {
    min_distance_=min_distance;
    max_distance_=max_distance;

    LOG(INFO) << "Range Filter 的参数为：" << std::endl
              << min_distance << ", "
              << max_distance << std::endl;

    return true;
}

bool RangeFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    filtered_cloud_ptr->clear();
    for (uint32_t i = 0; i < input_cloud_ptr->points.size(); i++)
    {
        double x = input_cloud_ptr->points[i].x;
        double y = input_cloud_ptr->points[i].y;
        double s = sqrt(x * x + y * y);
        if ((s >= min_distance_ )&& (s <= max_distance_))
            filtered_cloud_ptr->points.push_back(input_cloud_ptr->points[i]);
    }

    filtered_cloud_ptr->height = 1;
    filtered_cloud_ptr->width = filtered_cloud_ptr->points.size();

    return true;
}
} 