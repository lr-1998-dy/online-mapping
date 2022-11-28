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
{
    irls_ndt::NDTLocParam params;
    params.max_irls=node["max_irls"].as<int>();
    params.max_iterations=node["max_iterations"].as<int>();
    params.min_neighbor_num=node["min_neighbor_num"].as<int>();
    params.min_score=node["min_score"].as<float>();
    params.ndt_resolution=node["ndt_resolution"].as<float>();
    params.outlier_ratio=node["outlier_ratio"].as<float>();
    params.search_neighbor_num=node["search_neighbor_num"].as<int>();
    params.transformation_epsilon=node["transformation_epsilon"].as<float>();
    SetRegistrationParam(params);
}

NDTIrlsRegistration::NDTIrlsRegistration(int max_irls,
                                                                                    int max_iterations,
                                                                                    int min_neighbor_num,
                                                                                    float min_score,
                                                                                    float ndt_resolution,
                                                                                    float outlier_ratio,
                                                                                    int search_neighbor_num,
                                                                                    float transformation_epsilon)
{
    irls_ndt::NDTLocParam params;
    params.max_irls=max_irls;
    params.max_iterations=max_iterations;
    params.min_neighbor_num=min_neighbor_num;
    params.min_score=min_score;
    params.ndt_resolution=ndt_resolution;
    params.outlier_ratio=outlier_ratio;
    params.search_neighbor_num=search_neighbor_num;
    params.transformation_epsilon=transformation_epsilon;
    SetRegistrationParam(params);
}

bool NDTIrlsRegistration::SetRegistrationParam(irls_ndt::NDTLocParam params) {
    ndt_ptr_=irls_ndt::NDTScanMatcher::create(params);
    return true;
}

bool NDTIrlsRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    ndt_ptr_->setInputTarget(input_target);

    return true;
}

bool NDTIrlsRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    // ndt_ptr_->align(input_source, result_cloud_ptr, predict_pose.cast<double>());
    // result_pose = ndt_ptr_->getFinalTransformation().cast<float>();
    // pcl::transformPointCloud(*input_source, *result_cloud_ptr, result_pose);

    ndt_ptr_->align(input_source, result_cloud_ptr, predict_pose.cast<double>());
    result_pose = ndt_ptr_->getFinalTransformation().cast<float>();

    return true;
}

float NDTIrlsRegistration::GetFitnessScore() {

    return 0;
}
}