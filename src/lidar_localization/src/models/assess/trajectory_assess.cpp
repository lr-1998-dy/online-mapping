/*
 * @Description: 利用高程生成栅格地图
 * @Author: Li Rui
 * @Date: 2022-02-08 21:46:45
 */
#include "lidar_localization/models/assess/trajectory_assess.hpp"

#include "glog/logging.h"

namespace lidar_localization {

TrajectoryAssess::TrajectoryAssess(const YAML::Node& node){
    
    double coincidence_degree = node["coincidence_degree"].as<double>();

    SetAssessParam(coincidence_degree);
}

TrajectoryAssess::TrajectoryAssess(double coincidence_degree){

    SetAssessParam(coincidence_degree);
}

bool TrajectoryAssess::SetAssessParam(double coincidence_degree){
    coincidence_degree_=coincidence_degree;

    LOG(INFO) << "轨迹评价的参数为" << std::endl
              << "coincidence_degree_: " << coincidence_degree
              << std::endl << std::endl;

    return true;
}

// void TrajectoryAssess::InputTrajectory(const std::deque<Eigen::Matrix4f>& optimized_pose,const std::deque<Eigen::Matrix4f>& gnss_pose){
//     optimized_pose_=optimized_pose;
//     gnss_pose_=gnss_pose;
// }

/**
 * @description: 通过轨迹重合度来评价建图质量
 * @return bool
 */
bool TrajectoryAssess::Assess(const std::deque<Eigen::Matrix4f>& optimized_pose,const std::deque<Eigen::Matrix4f>& gnss_pose) {

    optimized_pose_=optimized_pose;
    gnss_pose_=gnss_pose;
    int error_count=0;
    for (int i = 0; i < optimized_pose_.size(); i++)
    {
        Eigen::Vector3d optimized_pose(optimized_pose_.at(i)(0,3),optimized_pose_.at(i)(1,3),optimized_pose_.at(i)(2,3));
        Eigen::Vector3d gnss_pose(gnss_pose_.at(i)(0,3),gnss_pose_.at(i)(1,3),gnss_pose_.at(i)(2,3));
        double error=(gnss_pose-optimized_pose).norm();
        if (error>1)
        {
            error_count++;
        }
    }

    if (error_count>5)
    {
        LOG(WARNING)<<"建图质量评价不合格";
        return false;
    }
    LOG(WARNING)<<"建图质量评价合格";
    return true;
}

}

