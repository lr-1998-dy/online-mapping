/*
 * @Description: 删除地面点
 * @Autor: Li Rui
 * @Date: 2022-09-04 13:36:47
 * @LastEditTime: 2022-12-07 19:21:36
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_OUTLIERS_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_OUTLIERS_FILTER_HPP_

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization {
class OutliersFilter: public CloudFilterInterface {
  public:
    OutliersFilter(const YAML::Node& node);
    OutliersFilter(float filter_radius,
                                int filter_count);

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(float filter_radius,int filter_count);

  private:
    pcl::RadiusOutlierRemoval<CloudData::POINT> radius_outlier;

};
}
#endif