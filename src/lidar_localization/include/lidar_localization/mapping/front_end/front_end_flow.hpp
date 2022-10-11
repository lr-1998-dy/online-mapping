/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Li Rui
 * @Date: 2022-02-10 08:31:22
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
#include "lidar_localization/subscriber/key_frame_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/mapping/front_end/front_end.hpp"

namespace lidar_localization {
class FrontEndFlow {
  public:
    FrontEndFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool InitWithConfig();
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateLaserOdometry();
    bool PublishData();

  private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<FrontEnd> front_end_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> gnss_pose_data_buff_;

    CloudData current_cloud_data_;
    PoseData current_gnss_pose_data_;

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();

    bool Prior_information_;
};
}

#endif