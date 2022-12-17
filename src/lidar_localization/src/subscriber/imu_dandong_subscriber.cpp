/*
 * @Description: 订阅imu数据
 * @Author: Li Rui
 * @Date: 2022-06-14 16:44:18
 */
#include "lidar_localization/subscriber/imu_dandong_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_localization{
IMUDandongSubscriber::IMUDandongSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUDandongSubscriber::msg_callback, this);
}

void IMUDandongSubscriber::msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    IMUDandongData imu_data;
    // time
    imu_data.time = imu_msg_ptr->linear_acceleration_covariance[0];
    // quaternion
    // imu_data.orientation.x() = imu_msg_ptr->orientation.x;
    // imu_data.orientation.y() = imu_msg_ptr->orientation.y;
    // imu_data.orientation.z() = imu_msg_ptr->orientation.z;
    // imu_data.orientation.w() = imu_msg_ptr->orientation.w;
    // euler_angles
    imu_data.euler_angles.roll=imu_msg_ptr->angular_velocity_covariance[0];
    imu_data.euler_angles.pitch=imu_msg_ptr->angular_velocity_covariance[1];
    imu_data.euler_angles.yall=imu_msg_ptr->angular_velocity_covariance[2];
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity(3, 3);

    matrix = Eigen::AngleAxisd(imu_data.euler_angles.yall , Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(imu_data.euler_angles.roll , Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(imu_data.euler_angles.pitch , Eigen::Vector3d::UnitY());

    imu_data.orientation=matrix;
    // position
    imu_data.position[0] = imu_msg_ptr->linear_acceleration_covariance[1] - 569880;
    imu_data.position[1] = imu_msg_ptr->linear_acceleration_covariance[2] - 4501440;
    imu_data.position[2] = imu_msg_ptr->linear_acceleration_covariance[3] - 360;

    new_imu_data_.push_back(imu_data);
}

void IMUDandongSubscriber::ParseData(std::deque<IMUDandongData>& imu_data_buff) {
    if (new_imu_data_.size() > 0) {
        imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
        new_imu_data_.clear();
    }
}
}
