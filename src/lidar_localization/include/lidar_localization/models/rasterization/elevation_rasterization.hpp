/*
 * @Description: 利用高程生成栅格地图
 * @Author: Li Rui
 * @Date: 2022-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_RASTERIZATION_NDT_RASTERIZATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_RASTERIZATION_NDT_RASTERIZATION_HPP_

#include <string>
#include<vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "lidar_localization/models/rasterization/rasterization_interface.hpp"
#include"lidar_localization/models/cloud_filter/voxel_filter.hpp"

namespace lidar_localization {
class ElevationRasterization: public RasterizationInterface {
  public:
    ElevationRasterization(const YAML::Node& node,const std::string &key_frames_path_);
    ElevationRasterization(int count_threshold,float height_threshold,int inflation_map_x,int inflation_map_y,float  map_resolution);

    bool CreateGridMap(const CloudData::CLOUD_PTR& cloud_map) override;
    nav_msgs::OccupancyGrid GetGridMap();

  private:
    bool SetRasterizationParam(int count_threshold,float height_threshold,int inflation_map_x,int inflation_map_y,float  map_resolution);
    bool CreateInitialMap(const CloudData::CLOUD_PTR& cloud_map,nav_msgs::OccupancyGrid &gridmap);
    bool Erode_Dilate(nav_msgs::OccupancyGrid &gridmap);
    bool InflateGradMap(const nav_msgs::OccupancyGrid &gridmap,nav_msgs::OccupancyGrid &inflated_gridmap);
    bool WriteToJson(const nav_msgs::OccupancyGrid &inflated_gridmap);


  private:
    int count_threshold_;
    float height_threshold_;
    int inflation_map_x_;
    int inflation_map_y_;
    float  map_resolution_;

    nav_msgs::OccupancyGrid inflated_gridmap_;
    std::string json_path_;
};
}

#endif