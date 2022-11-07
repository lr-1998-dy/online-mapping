/*
 * @Description: 点云滤波模块的接口
 * @Author: Li Rui
 * @Date: 2022-02-09 19:29:50
 */
#ifndef LIDAR_LOCALIZATION_MODELS_ASSESS_TRAJECTORY_ASSESS_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_ASSESS_TRAJECTORY_ASSESS_INTERFACE_HPP_

#include <deque>

#include "lidar_localization/models/assess/assess_interface.hpp"

namespace lidar_localization {
class TrajectoryAssess :public AssessInterface{
  public:
    TrajectoryAssess(const YAML::Node& node);
    TrajectoryAssess(double coincidence_degree);
    // void InputTrajectory(const std::deque<Eigen::Matrix4f>& optimized_pose,const std::deque<Eigen::Matrix4f>& gnss_pose);

    bool Assess(const std::deque<Eigen::Matrix4f>& optimized_pose,const std::deque<Eigen::Matrix4f>& gnss_pose) override;

  private:
    bool SetAssessParam(double coincidence_degree);

  private:
    double coincidence_degree_;
    std::deque<Eigen::Matrix4f> optimized_pose_;
    std::deque<Eigen::Matrix4f> gnss_pose_;

};
}

#endif