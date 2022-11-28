/*
 * @Description: 在ros中发布栅格地图
 * @Author: Li Rui
 * @Date: 2022-02-05 02:27:30
 */

#ifndef LIDAR_LOCALIZATION_PUBLISHER_GRIDMAP_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_GRIDMAP_PUBLISHER_HPP_

#include <nav_msgs/OccupancyGrid.h>
#include <json/json.h>

#include <ros/ros.h>

namespace lidar_localization {
class GridMapPublisher {
  public:
    GridMapPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   std::string frame_id,
                   size_t buff_size);
    GridMapPublisher() = default;

    void Publish(const nav_msgs::OccupancyGrid& gridmap_ptr_input, double time);
    void Publish(const nav_msgs::OccupancyGrid& gridmap_ptr_input);

    bool HasSubscribers();
  
  private:
    void PublishData(const nav_msgs::OccupancyGrid& cloud_ptr_input, ros::Time time);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif