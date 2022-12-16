/*
 * @Description: 
 * @Author: Li Rui
 * @Date: 2022-07-17 18:27:40
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DANDONG_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DANDONG_DATA_HPP_

#include <deque>
#include <cmath>
#include <Eigen/Dense>
#include "Geocentric/LocalCartesian.hpp"

namespace lidar_localization {
class IMUDandongData {
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

    struct EulerAngles {
      public:
        double roll = 0.0;
        double yall = 0.0;
        double pitch = 0.0;
    };

    struct LLH{
      double lat = 0.0;
      double lon = 0.0;
      double alt = 0.0;
    };

    double time = 0.0;
    double torad_=M_PI / 180;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    LLH llh;
    Eigen::Vector3d position;
    EulerAngles euler_angles;
    Eigen::Quaterniond orientation;

  private:
  static GeographicLib::LocalCartesian geo_converter;
  static bool origin_position_inited;
  static int sensor_check;

  public:
    void InitOriginPosition(double latitude,double longitude,double altitude);
    void GetENUPosition();
    // 把四元数转换成旋转矩阵送出去
    Eigen::Matrix3f GetOrientationMatrix();
    static bool SyncData(std::deque<IMUDandongData>& UnsyncedData, std::deque<IMUDandongData>& SyncedData, double sync_time);
};
}
#endif
