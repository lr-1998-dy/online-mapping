/*
 * @Description: NDT-IRLS匹配模块
 * @Author: Li Rui
 * @Date: 2022-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_IRLS_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_IRLS_REGISTRATION_HPP_

#include "irls_ndt/ndt_scan_matcher.h"
#include "lidar_localization/models/registration/registration_interface.hpp"
#include "irls_ndt/ndt_scan_matcher.h"

namespace lidar_localization {
class NDTIrlsRegistration: public RegistrationInterface {
  public:
    NDTIrlsRegistration(const YAML::Node& node);
    NDTIrlsRegistration(int max_irls,
                                              int max_iterations,
                                              int min_neighbor_num,
                                              float min_score,
                                              float ndt_resolution,
                                              float outlier_ratio,
                                              int search_neighbor_num,
                                              float transformation_epsilon);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;
  
  private:
    bool SetRegistrationParam(irls_ndt::NDTLocParam params);

  private:
    // pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
    irls_ndt::NDTScanMatcher::Ptr ndt_ptr_;
};
}

#endif