/*
 * @Description: 不滤波
 * @Author: Li Rui
 * @Date: 2022-02-09 19:53:20
 */
#include "lidar_localization/models/cloud_filter/no_filter.hpp"
#include "glog/logging.h"

namespace lidar_localization {
NoFilter::NoFilter() {
}

bool NoFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    filtered_cloud_ptr.reset(new CloudData::CLOUD(*input_cloud_ptr));
    return true;
}
} 