/*
 * @Description: 删除地面点
 * @Autor: Li Rui
 * @Date: 2022-09-04 13:36:47
 * @LastEditTime: 2022-10-18 10:08:50
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_RANGE_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_RANGE_FILTER_HPP_

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization {
class RangeFilter: public CloudFilterInterface {
  public:
    RangeFilter(const YAML::Node& node);
    RangeFilter(float min_distance,float max_distance);

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(float min_distance,float max_distance);

  private:
    float min_distance_;
    float max_distance_;
};
}
#endif