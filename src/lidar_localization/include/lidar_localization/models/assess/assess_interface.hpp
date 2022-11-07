/*
 * @Description: 建图质量评价模块的接口
 * @Author: Li Rui
 * @Date: 2022-02-09 19:29:50
 */
#ifndef LIDAR_LOCALIZATION_MODELS_ASSESS_ASSESS_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_ASSESS_ASSESS_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

namespace lidar_localization {
class AssessInterface {
  public:
    virtual ~AssessInterface() = default;

    virtual bool Assess(const std::deque<Eigen::Matrix4f>& optimized_pose,const std::deque<Eigen::Matrix4f>& gnss_pose) = 0;
};
}

#endif