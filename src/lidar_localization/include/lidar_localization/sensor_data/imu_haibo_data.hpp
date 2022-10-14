/*
 * @Description: 
 * @Author: Li Rui
 * @Date: 2019-07-17 18:27:40
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_IMU_HAIBO_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_IMU_HAIBO_DATA_HPP_

#include <deque>
#include <cmath>
#include <Eigen/Dense>

namespace lidar_localization {
class IMUHaiboData {
  public:
    struct LinearAcceleration {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct Position {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };


    class EulerAngles {
      public:
        double roll = 0.0;
        double yall = 0.0;
        double pitch = 0.0;
    };

    class EulerAngles {
      public:
        double roll = 0.0;
        double yall = 0.0;
        double pitch = 0.0;
    };


    double time = 0.0;
    double torad_=M_PI / 180;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Position position;
    EulerAngles euler_angles;

  public:
    // 把四元数转换成旋转矩阵送出去
    Eigen::Matrix3f GetEulerAnglesMatrix();
    static bool SyncData(std::deque<IMUHaiboData>& UnsyncedData, std::deque<IMUHaiboData>& SyncedData, double sync_time);
};
}
#endif