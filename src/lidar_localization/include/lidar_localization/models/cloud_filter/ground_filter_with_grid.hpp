/*
 * @Description: 利用法向量删除地面点
 * @Autor: Li Rui
 * @Date: 2022-09-04 13:36:47
 * @LastEditTime: 2022-12-09 11:24:34
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_GROUND_FILTER_WITH_GRID_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_GROUND_FILTER_WITH_GRID_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization {
class GridGroundFilter: public CloudFilterInterface {
  public:
    GridGroundFilter(const YAML::Node& node);
    GridGroundFilter(int count_threshold,
                                        float height_threshold,
                                        float  map_resolution);

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(int count_threshold,
                                              float height_threshold,
                                              float  map_resolution);

    bool CreateInitialMap(const CloudData::CLOUD_PTR& input_cloud_ptr,cv::Mat& ogm_mat);
    bool WriteToCloud(const CloudData::CLOUD_PTR& input_cloud_ptr,CloudData::CLOUD_PTR& filtered_cloud_ptr,const cv::Mat& ogm_mat);

  private:
    int count_threshold_;
    float height_threshold_;
    float  map_resolution_;

    float  x_min_;
    float  y_min_;

};
}
#endif