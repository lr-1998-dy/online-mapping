/*
 * @Description: NDT 匹配模块
 * @Author: Li Rui
 * @Date: 2022-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_OMP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_OMP_REGISTRATION_HPP_

#include "pclomp/ndt_omp.h"
#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {
class NDTOmpRegistration: public RegistrationInterface {
  public:
    NDTOmpRegistration(const YAML::Node& node);
    NDTOmpRegistration(float res, float step_size, float trans_eps, int max_iter,int threads);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;
  
  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter,int threads);

  private:
    pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
};
}

#endif