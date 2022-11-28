/*
 * @Description: 生成栅格地图的基类
 * @Author: Li Rui
 * @Date: 2022-02-08 21:25:11
 */
#ifndef LIDAR_LOCALIZATION_MODELS_RASTERIZATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_RASTERIZATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <nav_msgs/OccupancyGrid.h>
#include <json/json.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include"lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
class RasterizationInterface {
  public:
    virtual ~RasterizationInterface() = default;

    virtual bool CreateGridMap(const CloudData::CLOUD_PTR& cloud_map) = 0;

    virtual nav_msgs::OccupancyGrid GetGridMap()=0;
};
} 

#endif