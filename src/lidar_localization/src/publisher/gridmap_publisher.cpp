/*
 * @Description: 通过ros发布点云
 * @Author: Li Rui
 * @Date: 2022-02-05 02:27:30
 */

#include "lidar_localization/publisher/gridmap_publisher.hpp"
#include "glog/logging.h"

namespace lidar_localization {
GridMapPublisher::GridMapPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(topic_name, buff_size);
}

void GridMapPublisher::Publish(const nav_msgs::OccupancyGrid&  gridmap_ptr_input, double time) {
    ros::Time ros_time((float)time);
    PublishData(gridmap_ptr_input, ros_time);
}

void GridMapPublisher::Publish(const nav_msgs::OccupancyGrid&  gridmap_ptr_input) {
    ros::Time time = ros::Time::now();
    PublishData(gridmap_ptr_input, time);
}

void GridMapPublisher::PublishData(const nav_msgs::OccupancyGrid&  gridmap_ptr_input, ros::Time time) {
    publisher_.publish(gridmap_ptr_input);
}

bool GridMapPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} // namespace lidar_localization