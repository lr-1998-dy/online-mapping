/*
 * @Author: Li Rui
 * @LastEditTime: 2022-10-17 15:37:24
 * @LastEditors: lr 2012227985@qq.com
 * @Description: 所有模块的最前端，数据预处理模块；包括坐标系对齐（GNSS转为lidar系）、时间同步、点云去畸变等
 */
#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_haibo_subscriber.hpp"
#include "lidar_localization/subscriber/velocity_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
// publisher
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
// models
#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"

#include <yaml-cpp/yaml.h>
#include<vector>

namespace lidar_localization {
class DataPretreatFlow {
  public:
    DataPretreatFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool TransformData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<IMUHaiboSubscriber> imu_sub_ptr_;
    // std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    // std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    // std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    // models
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    std::deque<CloudData> cloud_data_buff_;
    std::deque<IMUHaiboData> imu_data_buff_;
    // std::deque<VelocityData> velocity_data_buff_;
    // std::deque<GNSSData> gnss_data_buff_;

    CloudData current_cloud_data_;
    IMUHaiboData current_imu_data_;
    // VelocityData current_velocity_data_;
    // GNSSData current_gnss_data_;

    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};
}

#endif