/*
 * @Description: 
 * @Author: Li Rui
 * @Date: 2022-02-10 08:38:42
 */
#include "lidar_localization/mapping/post_processing/post_processing_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
PostProcessingFlow::PostProcessingFlow(ros::NodeHandle& nh) {
    // subscriber
    optimized_odom_sub_ptr_ = std::make_shared<KeyFramesSubscriber>(nh, "/optimized_key_frames", 100000);
    key_frame_sub_ptr_= std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 100000);
    // publisher
    map_without_ground_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/map_without_ground", "/map", 100);
    post_processing_ptr_=std::make_shared<Post_Processing>();
}


bool PostProcessingFlow::Run() {
    if (!ReadData())
        return false;

    if (HasData())
    {
        if (ValidData())
        {
            post_processing_ptr_->Update(key_frame_buff_);
            key_frames_size_=key_frame_buff_.size();
        }
    }
    return true;
}

/**
 * @description: 判断是否有新的关键帧数据传进来
 * @return {*}
 */
bool PostProcessingFlow::ReadData() {
    optimized_odom_sub_ptr_->ParseData(optimized_odom_buff_);
    key_frame_sub_ptr_->ParseData(key_frame_buff_);

    return true;
}

bool PostProcessingFlow::HasData() {
    if (key_frame_buff_.size() == 0)
        return false;

    return true;
}
/**
 * @description: 确定是否有新的关键帧产生
 * @return {*}
 */
bool PostProcessingFlow::ValidData() {
    if (key_frames_size_>=key_frame_buff_.size())
    {
        return false;
    }
    return true;
}

bool PostProcessingFlow::SaveMap(){
    post_processing_ptr_->SaveMap(optimized_odom_buff_);
    return true;
}

bool PostProcessingFlow::GetGridMap(){
    inflated_gridmap_=post_processing_ptr_->GetGridMap();
    return true;
}

bool PostProcessingFlow::PublishData() {
    return true;
}
}