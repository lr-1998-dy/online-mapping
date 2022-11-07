/*
 * @Description: imu data
 * @Author: Li Rui
 * @Date: 2022-02-23 22:20:41
 */
#include "lidar_localization/sensor_data/imu_haibo_data.hpp"
#include <cmath>
#include "glog/logging.h"

bool lidar_localization::IMUHaiboData::origin_position_inited = false;
GeographicLib::LocalCartesian lidar_localization::IMUHaiboData::geo_converter;
int lidar_localization::IMUHaiboData::sensor_check = 0;

namespace lidar_localization {
void IMUHaiboData::InitOriginPosition(double latitude,double longitude,double altitude) {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
}

void IMUHaiboData::GetENUPosition(){
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity(3, 3);

    // double d_yall=euler_angles.yall;
    // if (d_yall>270)
    // {
    //     d_yall=(360.0+90.0-d_yall);
    // }else
    // {
    //     d_yall=(90.0-d_yall);
    // }
    // euler_angles.yall=d_yall;
    
    matrix = Eigen::AngleAxisd(euler_angles.yall * torad_, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(euler_angles.roll * torad_, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(euler_angles.pitch * torad_, Eigen::Vector3d::UnitX());

    orientation=matrix.inverse();
    
    geo_converter.Forward(llh.lat, llh.lon, llh.alt, position[0], position[1],position[2]);
}

Eigen::Matrix3f IMUHaiboData::GetOrientationMatrix() {
    Eigen::Matrix3f matrix = orientation.matrix().cast<float>();
    return matrix;
}

bool IMUHaiboData::SyncData(std::deque<IMUHaiboData>& UnsyncedData, std::deque<IMUHaiboData>& SyncedData, double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time) 
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }

    if (UnsyncedData.size() < 2){
        if (sensor_check++>40)
        {
            LOG(WARNING)<<"传感器丢帧情况严重，请检查传感器"<<sensor_check;
        }
        return false;
    }

    IMUHaiboData front_data = UnsyncedData.at(0);
    IMUHaiboData back_data = UnsyncedData.at(1);
    IMUHaiboData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    // synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    // synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    // synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    // synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    // synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    // synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
      
    // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
    // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
    // synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    // synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    // synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    // synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
    // 线性插值之后要归一化
    // synced_data.orientation.Normlize();

    synced_data.orientation = front_data.orientation.slerp(back_scale, back_data.orientation);
    synced_data.position = back_scale * (back_data.position - front_data.position) + front_data.position;

    SyncedData.push_back(synced_data);

    return true;
}
}