/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Li Rui
 * @Date: 2022-02-10 08:38:42
 */
#include "lidar_localization/mapping/front_end/front_end_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
    gnss_pose_sub_ptr_=std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_odom", "/map", "/lidar", 100);

    front_end_ptr_ = std::make_shared<FrontEnd>();
    //确定先验信息
    InitWithConfig();
}

bool FrontEndFlow::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/front_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    std::string prior_information = config_node["prior_information"].as<std::string>();
    std::cout << "前端配准的先验信息是：" << prior_information << std::endl;

    if (prior_information == "Predict") {
        Prior_information_=0;
        return true;
    } 
    if (prior_information == "GNSS") {
        Prior_information_=1;
        return true;
    } 

    LOG(ERROR) << "没找到与 " << prior_information << " 相对应的先验信息!";
    return false;
}


bool FrontEndFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        if (UpdateLaserOdometry()) {
            PublishData();
        }
    }

    return true;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);
    return true;
}

bool FrontEndFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (gnss_pose_data_buff_.size() == 0)
        return false;

    return true;
}

bool FrontEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_gnss_pose_data_=gnss_pose_data_buff_.front();

    double diff_gnss_time = current_cloud_data_.time - current_gnss_pose_data_.time;

    //说明这个点云过于靠前，丢掉
    if (diff_gnss_time < -0.05 ) {
        cloud_data_buff_.pop_front();
        return false;
    }

    //说明这个点云过于靠前，丢掉
    if (diff_gnss_time > 0.05) {
        gnss_pose_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_pose_data_buff_.pop_front();

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
    static bool odometry_inited = false;
    if (!odometry_inited) {
        odometry_inited = true;
        front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());        
        if (Prior_information_==1)
        {
            front_end_ptr_->SetGNSSInitPose(current_gnss_pose_data_.pose);
            return front_end_ptr_->Update(current_cloud_data_, laser_odometry_,current_gnss_pose_data_.pose);
        }
        else
            return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
    }
    
    if (Prior_information_==1)
    {
        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_,current_gnss_pose_data_.pose);
    }
    else
        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
    }

bool FrontEndFlow::PublishData() {
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);

    return true;
}
}