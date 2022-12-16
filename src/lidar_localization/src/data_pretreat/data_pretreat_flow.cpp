/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Li Rui
 * @Date: 2022-02-10 08:38:42
 */
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"
#include<vector>

//model
#include"lidar_localization/models/cloud_filter/range_filter.hpp"

namespace lidar_localization {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh) {
    // subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/livox_undistort", 100000);
    imu_sub_ptr_ = std::make_shared<IMUDandongSubscriber>(nh, "/livox/loc", 1000000);
    // publisher
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/synced_cloud", "/velo_link", 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "/velo_link", 100);

}

bool DataPretreatFlow::Run() {
    if (!InitCalibration()) 
        return false;

    if (!InitGNSS())
        return false;

    if (!InitFilter())
        return false;


    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        TransformData();
        PublishData();

    }

    return true;
}

bool DataPretreatFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    static std::deque<IMUDandongData> unsynced_imu_;

    imu_sub_ptr_->ParseData(unsynced_imu_);

    if (cloud_data_buff_.size() == 0)
        return false;

    double cloud_time = cloud_data_buff_.front().time;
    bool valid_imu = IMUDandongData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);

    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu) {
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

bool DataPretreatFlow::InitCalibration() {
    static bool calibration_received = false;

    if (!calibration_received)
    {
        std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/data_pretreat.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        std::vector<float> lidar_to_imu= config_node["lidar2imu_"].as<std::vector<float>>();
      
        for (int row = 0; row < lidar_to_imu_.rows(); row++)
        {
            for (int col = 0; col < lidar_to_imu_.cols(); col++)
            {
                lidar_to_imu_(row, col)=lidar_to_imu[col + row * lidar_to_imu_.cols()];
            }
        }
        calibration_received=true;
    }
    
    return calibration_received;
}

bool DataPretreatFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/data_pretreat.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);

        double ori_longitude_=config_node["map"]["ori_longitude_"].as<double>();
        double ori_latitude_=config_node["map"]["ori_latitude_"].as<double>();
        double ori_altitude_=config_node["map"]["ori_altitude_"].as<double>();

        IMUDandongData imu_data=imu_data_buff_.front();
        imu_data.InitOriginPosition(ori_latitude_,ori_longitude_,ori_altitude_);

        gnss_inited = true;
    }

    return gnss_inited;
}

bool DataPretreatFlow::InitFilter() {
    static bool filter_inited = false;
    if (!filter_inited) {
        std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/data_pretreat.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);

        //model
        range_filter_ptr_=std::make_shared<RangeFilter>(config_node["range_filter"]);

        filter_inited = true;
    }

    return filter_inited;
}

bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;

    return true;
}

bool DataPretreatFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    if (diff_imu_time < -0.05 ) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::TransformData() {
    gnss_pose_ = Eigen::Matrix4f::Identity();

    gnss_pose_(0,3) = current_imu_data_.position[0];
    gnss_pose_(1,3) = current_imu_data_.position[1];
    gnss_pose_(2,3) = current_imu_data_.position[2];
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    gnss_pose_ *= lidar_to_imu_;

    range_filter_ptr_->Filter(current_cloud_data_.cloud_ptr,filtered_cloud_data_.cloud_ptr);

    return true;
}

bool DataPretreatFlow::PublishData() {
    cloud_pub_ptr_->Publish(filtered_cloud_data_.cloud_ptr, current_cloud_data_.time);
    // cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    gnss_pub_ptr_->Publish(gnss_pose_, current_imu_data_.time);

    return true;
}
}
