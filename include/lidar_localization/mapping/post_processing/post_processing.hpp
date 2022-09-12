/*
 * @Author: Li Rui
 * @LastEditTime: 2022-09-12 15:41:30
 * @LastEditors: lr 2012227985@qq.com
 * @Description: 获得被优化之后的位姿信息，后处理点云，存储点云，生成全局地图
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_POST_PROCESSING_POST_PROCESSING_HPP_
#define LIDAR_LOCALIZATION_MAPPING_POST_PROCESSING_POST_PROCESSING_HPP_

#include <deque>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>

#include "lidar_localization/sensor_data/key_frame.hpp"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"
#include "lidar_localization/models/rasterization/rasterization_interface.hpp"
// tools
#include <lidar_localization/tools/get_filenames.hpp>

#include"lidar_localization/sensor_data/cloud_data.hpp"


namespace lidar_localization {
class Post_Processing {
  public:
    Post_Processing();

    bool Update(std::deque<KeyFrame>  key_frame_buff_);

    bool SaveMap(const std::deque<KeyFrame>& optimized_odom_buff_);
    bool HasNewGlobalMap();
    nav_msgs::OccupancyGrid CreateGridMap();
    nav_msgs::OccupancyGrid GetGridMap();

  private:
    bool InitWithConfig();
    bool InitParam(const YAML::Node& config_node);
    bool InitDataPath(const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool InitRasterization(std::string rasterization_user, std::shared_ptr<RasterizationInterface>& filter_ptr, const YAML::Node& config_node);
    bool SaveKeyCloud(int id,CloudData::CLOUD_PTR cloud_ptr);
    bool JointMap(const std::deque<KeyFrame>& optimized_odom_buff_,CloudData::CLOUD_PTR &global_map_without_ground_);

  private:
    std::string key_frames_path_ = "";

    std::shared_ptr<CloudFilterInterface> map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> map_without_ground_filter_ptr_;


    std::deque<KeyFrame> all_key_frames_;

    bool has_new_global_map_ = false;
    int last_id=0;

    FileGetter  file_names_get_;
    std::vector<std::string> file_names_;

    std::shared_ptr<RasterizationInterface> map_rasterization_ptr_;
    nav_msgs::OccupancyGrid inflated_gridmap_;
};
}

#endif