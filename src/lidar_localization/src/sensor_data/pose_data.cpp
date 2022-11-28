/*
 * @Description: 
 * @Author: Li Rui
 * @Date: 2022-02-28 18:50:16
 */
#include "lidar_localization/sensor_data/pose_data.hpp"

namespace lidar_localization {
Eigen::Quaternionf PoseData::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}

}