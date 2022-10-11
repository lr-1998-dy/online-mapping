/*
 * @Description: NDT-IRLS匹配模块
 * @Author: Li Rui
 * @Date: 2022-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_IRLS_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_IRLS_REGISTRATION_HPP_

#include "irls_ndt/ndt_scan_matcher.h"
#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {
class NDTIrlsRegistration: public RegistrationInterface {
  public:
    NDTIrlsRegistration(const YAML::Node& node);
    NDTIrlsRegistration(float res, float step_size, float trans_eps, int max_iter,int threads);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;
  
  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter,int threads);

  private:
    // pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
    irls_ndt::NDTScanMatcher::Ptr ndt_ptr_;
};
}

#endif