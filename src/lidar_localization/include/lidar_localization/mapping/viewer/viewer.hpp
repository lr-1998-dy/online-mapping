/*
 * @Description: 实时显示，主要是点云
 * @Author: Li Rui
 * @Date: 2022-02-29 03:19:45
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_VIEWER_VIEWER_HPP_
#define LIDAR_LOCALIZATION_MAPPING_VIEWER_VIEWER_HPP_

#include <string>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/key_frame.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"

#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"
#include "lidar_localization/models/rasterization/rasterization_interface.hpp"

#include <nav_msgs/OccupancyGrid.h>

namespace lidar_localization {
class Viewer {
  public:
    Viewer();

    bool UpdateWithOptimizedKeyFrames(std::deque<KeyFrame>& optimized_key_frames);
    bool UpdateWithNewKeyFrame(std::deque<KeyFrame>& new_key_frames,
                               PoseData transformed_data,
                               CloudData cloud_data);

    bool SaveMap();
    Eigen::Matrix4f& GetCurrentPose();
    CloudData::CLOUD_PTR& GetCurrentScan();
    bool GetLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
    bool GetGlobalMap(CloudData::CLOUD_PTR& local_map_ptr,nav_msgs::OccupancyGrid& inflated_gridmap);
    bool HasNewLocalMap();
    bool HasNewGlobalMap();

  private:
    bool InitWithConfig();
    bool InitParam(const YAML::Node& config_node);
    std::string GetDate();
    bool InitDataPath(const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, 
                    std::shared_ptr<CloudFilterInterface>& filter_ptr, 
                    const YAML::Node& config_node);
  bool InitRasterization(std::string rasterization_user, std::shared_ptr<RasterizationInterface>& filter_ptr, const YAML::Node& config_node);


    bool OptimizeKeyFrames();
    bool JointGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
    bool JointLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
    bool JointCloudMap(const std::deque<KeyFrame>& key_frames, 
                             CloudData::CLOUD_PTR& map_cloud_ptr,int save_map);

  private:
    std::string data_path_ = "";
    int local_frame_num_ = 20;

    std::string key_frames_path_ = "";
    std::string post_key_frames_path_ = "";
    std::string map_path_ = "";

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> global_map_ground_filter_ptr_;

    std::shared_ptr<RasterizationInterface> map_rasterization_ptr_;

    Eigen::Matrix4f pose_to_optimize_ = Eigen::Matrix4f::Identity();
    PoseData optimized_odom_;
    CloudData optimized_cloud_;
    std::deque<KeyFrame> optimized_key_frames_;
    std::deque<KeyFrame> all_key_frames_;


    bool has_new_global_map_ = false;
    bool has_new_local_map_ = false;
};
}

#endif