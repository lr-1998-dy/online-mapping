/*
 * @Description: 订阅imu数据
 * @Author: Li Rui
 * @Date: 2022-06-14 16:44:18
 */
#include "lidar_localization/subscriber/imu_haibo_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_localization{
IMUHaiboSubscriber::IMUHaiboSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUHaiboSubscriber::msg_callback, this);
}

void IMUHaiboSubscriber::msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    IMUHaiboData imu_data;
    imu_data.time = imu_msg_ptr->header.stamp.toSec();

    imu_data

    new_imu_data_.push_back(imu_data);
}

void IMUHaiboSubscriber::ParseData(std::deque<IMUHaiboData>& imu_data_buff) {
    if (new_imu_data_.size() > 0) {
        imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
        new_imu_data_.clear();
    }
}
}