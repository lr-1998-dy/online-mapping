/*
 * @Description: 利用法向量删除地面点
 * @Autor: Li Rui
 * @Date: 2022-09-04 13:36:47
 * @LastEditTime: 2022-12-07 10:43:36
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_GROUND_FILTER_WITH_NORMAL_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_GROUND_FILTER_WITH_NORMAL_HPP_

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization {
class NormalGroundFilter: public CloudFilterInterface {
  public:
    NormalGroundFilter(const YAML::Node& node);
    NormalGroundFilter(bool opt_coeff, float dist_threshold, int  max_iter);

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(bool opt_coeff, float dist_threshold, int  max_iter);
    CloudData::CLOUD_PTR normal_filtering(const CloudData::CLOUD_PTR& cloud) ;
   CloudData::CLOUD_PTR hill_points(const CloudData::CLOUD_PTR& cloud,const CloudData::CLOUD_PTR& cloud_ground);
    // bool  PassThroughFilter();
    // bool  NormalFilter();
    // bool  RadiusOutlierRemoval();

  private:
    pcl::SACSegmentation<CloudData::POINT> ground_filter_;
    pcl::ExtractIndices<CloudData::POINT> extractor;
    CloudData::CLOUD_PTR ground_cloud_ptr;


};
}
#endif