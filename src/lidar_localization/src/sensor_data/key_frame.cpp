/*
 * @Description: 
 * @Author: Li Rui
 * @Date: 2022-02-28 19:17:00
 */
#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
Eigen::Quaternionf KeyFrame::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}
}