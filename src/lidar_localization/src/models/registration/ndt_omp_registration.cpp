/*
 * @Description: NDT 匹配模块
 * @Author: Li Rui
 * @Date: 2022-02-08 21:46:45
 */
#include "lidar_localization/models/registration/ndt_omp_registration.hpp"
#include "glog/logging.h"

namespace lidar_localization {

NDTOmpRegistration::NDTOmpRegistration(const YAML::Node& node)
    :ndt_ptr_(new pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {
    
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();
    int  threads=node["threads"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter,threads);
}

NDTOmpRegistration::NDTOmpRegistration(float res, float step_size, float trans_eps, int max_iter,int threads)
    :ndt_ptr_(new pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(res, step_size, trans_eps, max_iter,threads);
}

bool NDTOmpRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter,int threads) {
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);
    ndt_ptr_->setNumThreads(threads);
    ndt_ptr_->setNeighborhoodSearchMethod(pclomp::DIRECT7);

    std::cout << "NDT 的匹配参数为：" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter  << ", "
              <<"threads: "<<threads<< ", "
              << std::endl << std::endl;

    return true;
}

bool NDTOmpRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    ndt_ptr_->setInputTarget(input_target);

    return true;
}

bool NDTOmpRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    ndt_ptr_->setInputSource(input_source);
    ndt_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = ndt_ptr_->getFinalTransformation();

    return true;
}

float NDTOmpRegistration::GetFitnessScore() {
    return ndt_ptr_->getFitnessScore();
}
}