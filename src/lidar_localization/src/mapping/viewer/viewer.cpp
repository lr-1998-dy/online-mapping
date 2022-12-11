/*
 * @Description: 
 * @Author: Li Rui
 * @Date: 2022-02-29 03:49:12
 */
#include "lidar_localization/mapping/viewer/viewer.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"
#include <unistd.h>

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"
#include"lidar_localization/models/rasterization/elevation_rasterization.hpp"
#include "lidar_localization/models/cloud_filter/ground_filter.hpp"
#include "lidar_localization/models/cloud_filter/outliers_filter.hpp"
#include "lidar_localization/models/cloud_filter/ground_filter_with_normal.hpp"
#include "lidar_localization/models/cloud_filter/ground_filter_with_grid.hpp"


#define SAVE_MAP 1
#define SAVE_POST_MAP 0

namespace lidar_localization {
Viewer::Viewer() {
    InitWithConfig();
}

bool Viewer::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/viewer.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------显示模块初始化-------------------" << std::endl;
    InitParam(config_node);
    InitDataPath(config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("global_map", global_map_filter_ptr_, config_node);
    InitFilter("global_map_ground", global_map_ground_filter_ptr_, config_node);
    InitRasterization("global_map", map_rasterization_ptr_, config_node);

    return true;
}

bool Viewer::InitParam(const YAML::Node& config_node) {
    local_frame_num_ = config_node["local_frame_num"].as<int>();
    return true;
}

std::string Viewer::GetDate(){
    std::string date;
    time_t now = time(0);
    
    tm *ltm = localtime(&now);

    //补零位
	std::stringstream fillmonzero;
	fillmonzero << std::setw(2) << std::setfill('0') << (1 + ltm->tm_mon) ;
	std::string mon;
	fillmonzero >> mon;     

	std::stringstream filldayzero;
    filldayzero << std::setw(2) << std::setfill('0') << (ltm->tm_mday) ;
	std::string day;
	filldayzero >> day;     

    // 输出 tm 结构的各个组成部分
    date=std::to_string(1900 + ltm->tm_year)+mon+day;
    return date;
}

bool Viewer::InitDataPath(const YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    std::string map_path = config_node["map_path"].as<std::string>();
    if (data_path == "./") {
        data_path = WORK_SPACE_PATH;
    }

    key_frames_path_ = data_path + "/slam_data/key_frames";
    post_key_frames_path_=data_path + "/slam_data/post_key_frames";

    if (map_path == "./") {
        map_path_ = data_path + "/slam_data/map";
    }else{
        map_path_=map_path+"/"+GetDate();
    }


    while (!(FileManager::IsDirectory(data_path+"/slam_data")))
    {
        sleep(1);
    }

    if (!FileManager::InitDirectory(map_path_, "点云地图文件"))
        return false;

    if (!FileManager::InitDirectory(data_path + "/slam_data/post_key_frames","后处理之后的点云"))
        return false;

    return true;
}

bool Viewer::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "显示模块" << filter_user << "选择的滤波方法为：" << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
        return true;
    } 

    if (filter_mothod == "ground_filter") {
        filter_ptr = std::make_shared<GroundFilter>(config_node[filter_mothod]);
        return true;
    } 

    if (filter_mothod == "ground_filter_with_normal") {
        filter_ptr = std::make_shared<NormalGroundFilter>(config_node[filter_mothod]);
        return true;
    } 

    if (filter_mothod == "ground_filter_with_grid") {
        filter_ptr = std::make_shared<GridGroundFilter>(config_node[filter_mothod]);
        return true;
    } 

    if (filter_mothod == "outlier_filter") {
        filter_ptr = std::make_shared<OutliersFilter>(config_node[filter_mothod]);
        return true;
    } 


    LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
    return false;
}

bool Viewer::InitRasterization(std::string rasterization_user, std::shared_ptr<RasterizationInterface>& filter_ptr, const YAML::Node& config_node){
    std::string rasterization_mothod = config_node[rasterization_user + "_rasterization"].as<std::string>();
    std::cout << "栅格化" << rasterization_user << "选择的栅格方法为：" << rasterization_mothod << std::endl;

    if (rasterization_mothod=="elevation_rasterization")
    {
        map_rasterization_ptr_=std::make_shared<ElevationRasterization>(config_node[rasterization_mothod],map_path_);
        return true;
    }

    LOG(ERROR) << "没有为 " << rasterization_user << " 找到与 " << rasterization_mothod << " 相对应的栅格化方法!";
    return false;
}

bool Viewer::UpdateWithOptimizedKeyFrames(std::deque<KeyFrame>& optimized_key_frames) {
    has_new_global_map_ = false;
    
    if (optimized_key_frames.size() > 0) {
        optimized_key_frames_ = optimized_key_frames;
        optimized_key_frames.clear();
        OptimizeKeyFrames();
        has_new_global_map_ = true;
    }

    return has_new_global_map_;
}

bool Viewer::UpdateWithNewKeyFrame(std::deque<KeyFrame>& new_key_frames,
                                   PoseData transformed_data,
                                   CloudData cloud_data) {
    has_new_local_map_ = false;

    if (new_key_frames.size() > 0) {
        KeyFrame key_frame;
        for (size_t i = 0; i < new_key_frames.size(); ++i) {
            key_frame = new_key_frames.at(i);
            key_frame.pose = pose_to_optimize_ * key_frame.pose;
            all_key_frames_.push_back(key_frame);
        }
        new_key_frames.clear();
        has_new_local_map_ = true;
    }

    optimized_odom_ = transformed_data;
    optimized_odom_.pose = pose_to_optimize_ * optimized_odom_.pose;

    optimized_cloud_ = cloud_data;
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *optimized_cloud_.cloud_ptr, optimized_odom_.pose);

    return true;
}

bool Viewer::OptimizeKeyFrames() {
    size_t optimized_index = 0;
    size_t all_index = 0;
    while (optimized_index < optimized_key_frames_.size() && all_index < all_key_frames_.size()) {
        if (optimized_key_frames_.at(optimized_index).index < all_key_frames_.at(all_index).index) {
            optimized_index ++;
        } else if (optimized_key_frames_.at(optimized_index).index < all_key_frames_.at(all_index).index) {
            all_index ++;
        } else {
            pose_to_optimize_ = optimized_key_frames_.at(optimized_index).pose * all_key_frames_.at(all_index).pose.inverse();
            all_key_frames_.at(all_index) = optimized_key_frames_.at(optimized_index);
            optimized_index ++;
            all_index ++;
        }
    }

    while (all_index < all_key_frames_.size()) {
        all_key_frames_.at(all_index).pose = pose_to_optimize_ * all_key_frames_.at(all_index).pose;
        all_index ++;
    }

    return true;
}

bool Viewer::JointGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    JointCloudMap(optimized_key_frames_, global_map_ptr,SAVE_POST_MAP);
    return true;
}

bool Viewer::JointLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    size_t begin_index = 0;
    if (all_key_frames_.size() > (size_t)local_frame_num_)
        begin_index = all_key_frames_.size() - (size_t)local_frame_num_;

    std::deque<KeyFrame> local_key_frames;
    for (size_t i = begin_index; i < all_key_frames_.size(); ++i) {
        local_key_frames.push_back(all_key_frames_.at(i));
    }

    JointCloudMap(local_key_frames, local_map_ptr,SAVE_MAP);
    return true;
}

bool Viewer::JointCloudMap(const std::deque<KeyFrame>& key_frames, CloudData::CLOUD_PTR& map_cloud_ptr, int save_map) {
    map_cloud_ptr.reset(new CloudData::CLOUD());

    CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
    std::string file_path = "";

    for (size_t i = 0; i < key_frames.size(); ++i) {
        if (save_map==1)
        {
            file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames.at(i).index) + ".pcd";
            pcl::io::loadPCDFile(file_path, *cloud_ptr);
            pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, key_frames.at(i).pose);
            *map_cloud_ptr += *cloud_ptr;
        }else{
            CloudData::CLOUD_PTR cloud_out_ptr(new CloudData::CLOUD());
            file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames.at(i).index) + ".pcd";
            pcl::io::loadPCDFile(file_path, *cloud_ptr);
            pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, key_frames.at(i).pose);
            global_map_ground_filter_ptr_->Filter(cloud_ptr,cloud_out_ptr);
            
            std::string post_file_path = post_key_frames_path_ + "/post_key_frame_" + std::to_string(key_frames.at(i).index) + ".pcd";
            pcl::io::savePCDFileBinary(post_file_path, *cloud_out_ptr);
            *map_cloud_ptr += *cloud_out_ptr;
        }
    }
    return true;
}

bool Viewer::SaveMap() {
    LOG(INFO)<<"Viewer::SaveMap";
    if (optimized_key_frames_.size() == 0)
        return false;

    CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
    JointCloudMap(optimized_key_frames_, global_map_ptr,SAVE_MAP);
    global_map_filter_ptr_->Filter(global_map_ptr, global_map_ptr);

    std::string map_file_path = map_path_ + "/map.pcd";
    pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr);

    LOG(INFO) << "地图保存完成，地址是：" << std::endl << map_file_path << std::endl << std::endl;

    CloudData::CLOUD_PTR global_map_without_ground_ptr(new CloudData::CLOUD());
    JointCloudMap(optimized_key_frames_, global_map_without_ground_ptr,SAVE_POST_MAP);
    // global_map_ground_filter_ptr_->Filter(global_map_without_ground_ptr,global_map_without_ground_ptr);

    std::string map_without_ground_file_path = map_path_ + "/post_map.pcd";
    pcl::io::savePCDFileBinary(map_without_ground_file_path, *global_map_without_ground_ptr);

    LOG(INFO) << "无地面的地图保存完成，地址是：" << std::endl << map_without_ground_file_path << std::endl << std::endl;

    return true;
}

Eigen::Matrix4f& Viewer::GetCurrentPose() {
    return optimized_odom_.pose;
}

CloudData::CLOUD_PTR& Viewer::GetCurrentScan() {
    frame_filter_ptr_->Filter(optimized_cloud_.cloud_ptr, optimized_cloud_.cloud_ptr);
    return optimized_cloud_.cloud_ptr;
}

bool Viewer::GetLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    JointLocalMap(local_map_ptr);
    local_map_filter_ptr_->Filter(local_map_ptr, local_map_ptr);
    return true;
}

bool Viewer::GetGlobalMap(CloudData::CLOUD_PTR& global_map_ptr,nav_msgs::OccupancyGrid& inflated_gridmap) {
    JointGlobalMap(global_map_ptr);
    // global_map_filter_ptr_->Filter(global_map_ptr, global_map_ptr);
    map_rasterization_ptr_->CreateGridMap(global_map_ptr);
    inflated_gridmap=map_rasterization_ptr_->GetGridMap();
    return true;
}

bool Viewer::HasNewLocalMap() {
    return has_new_local_map_;
}

bool Viewer::HasNewGlobalMap() {
    return has_new_global_map_;
}
}