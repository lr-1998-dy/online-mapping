/*
 * @Description: 订阅imu数据
 * @Author: Li Rui
 * @Date: 2022-08-19 19:22:17
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_IMU_DANDONG_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_IMU_DANDONG_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "lidar_localization/sensor_data/imu_dandong_data.hpp"

namespace lidar_localization {
class IMUDandongSubscriber {
  public:
    IMUDandongSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    IMUDandongSubscriber() = default;
    void ParseData(std::deque<IMUDandongData>& deque_imu_data);

  private:
    void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<IMUDandongData> new_imu_data_; 
};
}
#endif
