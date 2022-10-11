/*
 * @Description: NDT-irls匹配模块
 * @Author: Li Rui
 * @Date: 2022-02-08 21:46:45
 */
#include "lidar_localization/models/registration/ndt_irls_registration.hpp"
#include "glog/logging.h"

// #define    NDT_IRLS

namespace lidar_localization {

NDTIrlsRegistration::NDTIrlsRegistration(const YAML::Node& node)
    :ndt_ptr_(irls_ndt::NDTScanMatcher::create(node)) {

}

NDTIrlsRegistration::NDTIrlsRegistration(float res, float step_size, float trans_eps, int max_iter,int threads)
{

}

bool NDTIrlsRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter,int threads) {

    return true;
}

bool NDTIrlsRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    #ifdef NDT_IRLS
        ndt_ptr_->setInputTarget(input_target);
   #else  

  #endif  

    return true;
}

bool NDTIrlsRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    // ndt_ptr_->align(input_source, result_cloud_ptr, predict_pose.cast<double>());
    // result_pose = ndt_ptr_->getFinalTransformation().cast<float>();
    // pcl::transformPointCloud(*input_source, *result_cloud_ptr, result_pose);

    #ifdef NDT_IRLS
        ndt_ptr_->align(input_source, result_cloud_ptr, predict_pose.cast<double>());
        result_pose = ndt_ptr_->getFinalTransformation().cast<float>();
        pcl::transformPointCloud(*input_source, *result_cloud_ptr, result_pose);
   #else  

  #endif  


    return true;
}

float NDTIrlsRegistration::GetFitnessScore() {
    return 0;
}
}