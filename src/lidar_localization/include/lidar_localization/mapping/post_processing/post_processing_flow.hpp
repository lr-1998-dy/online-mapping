/*
 * @Author: Li Rui
 * @LastEditTime: 2022-09-12 10:08:57
 * @LastEditors: lr 2012227985@qq.com
 * @Description: 地图的后处理，接收GNSS的关键帧位姿，
 * 计算出来有多少帧数据，去除地面点，读取优化之后的位姿，拼接，变成栅格
 */

#ifndef LIDAR_LOCALIZATION_MAPPING_POST_PROCESSING_POST_PROCESSING_FLOW_HPP_
#define LIDAR_LOCALIZATION_MAPPING_POST_PROCESSING_POST_PROCESSING_FLOW_HPP_

#include<vector>
#include<string>
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
// subscriber
#include "lidar_localization/subscriber/key_frames_subscriber.hpp"
#include "lidar_localization/subscriber/key_frame_subscriber.hpp"
// publisher
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/mapping/post_processing/post_processing.hpp"

namespace lidar_localization {
class PostProcessingFlow {
  public:
    PostProcessingFlow(ros::NodeHandle& nh);

    bool SaveMap();
    bool Run();
    bool GetGridMap();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<KeyFramesSubscriber> optimized_odom_sub_ptr_;
    std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> map_without_ground_pub_ptr_;

    std::deque<KeyFrame> optimized_odom_buff_;
    std::deque<KeyFrame> key_frame_buff_;
    std::shared_ptr<Post_Processing>  post_processing_ptr_;
    
    int key_frames_size_=0;

    nav_msgs::OccupancyGrid inflated_gridmap_;
};
}

#endif