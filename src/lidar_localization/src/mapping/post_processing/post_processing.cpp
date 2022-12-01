/*
 * @Description: 后处理算法
 * @Author: Li Rui
 * @Date: 2022-02-04 18:53:06
 */

#include <cmath>
#include <ctime>
#include <sstream>
#include <algorithm>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"
#include <unistd.h>

#include "lidar_localization/mapping/post_processing/post_processing.hpp"
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/ground_filter.hpp"
#include "lidar_localization/tools/print_info.hpp"
#include"lidar_localization/models/rasterization/elevation_rasterization.hpp"
#include <lidar_localization/tools/file_manager.hpp>

namespace lidar_localization {
Post_Processing::Post_Processing() {
    InitWithConfig();
}

bool Post_Processing::InitWithConfig() {
    std::cout << "-----------------后处理初始化-------------------" << std::endl;
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/post_processing.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    InitParam(config_node);
    InitDataPath(config_node);
    InitFilter("map", map_filter_ptr_, config_node);
    InitFilter("global_map", map_without_ground_filter_ptr_, config_node);//这个是被调用去除关键帧地面的
    InitFilter("global_map", post_map_without_ground_filter_ptr_, config_node);
    InitRasterization("global_map", map_rasterization_ptr_, config_node);
    global_map_with_out_ground_.reset(new CloudData::CLOUD());
    return true;
}

bool Post_Processing::InitParam(const YAML::Node& config_node) {
    return true;
}

std::string Post_Processing::GetDate(){
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

bool Post_Processing::InitDataPath(const YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    std::string map_path = config_node["map_path"].as<std::string>();
    if (data_path == "./") {
        data_path = WORK_SPACE_PATH;
    }

    key_frames_path_ = data_path + "/slam_data";
    std::string key_frames_path=key_frames_path_+"/key_frames";

    if (map_path == "./") {
        map_path_=key_frames_path_+"/map";
    }else{
        map_path_=map_path+"/"+GetDate();
    }

    //保证所依赖的文件夹在次之前就建好了
    while (!(FileManager::IsDirectory(key_frames_path_)&&FileManager::IsDirectory(key_frames_path)))
    {
        sleep(1);
    }
    
    file_names_get_.ScanFiles(key_frames_path);

    if (!FileManager::InitDirectory(key_frames_path_ + "/post_key_frames","后处理之后的点云"))
        return false;

    if (!FileManager::InitDirectory(map_path_, "后处理之后的点云地图文件"))
        return false;


    return true;
}

bool Post_Processing::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "闭环的" << filter_user << "选择的滤波方法为：" << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
        return true;
    } 

    if (filter_mothod == "ground_filter") {
        filter_ptr = std::make_shared<GroundFilter>(config_node[filter_mothod]);
        return true;
    } 

    if (filter_mothod == "post_ground_filter") {
        filter_ptr = std::make_shared<GroundFilter>(config_node[filter_mothod]);
        return true;
    } 

    LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
    return false;
}

bool Post_Processing::InitRasterization(std::string rasterization_user, std::shared_ptr<RasterizationInterface>& filter_ptr, const YAML::Node& config_node){
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

/**
 * @description: 该函数用于更新key_frames_without_ground，在确定有新的关键帧产生的时候运行
 * @param {KeyFrame} key_frame
 * @param {KeyFrame} key_gnss
 * @return {*}
 */
bool Post_Processing::Update(std::deque<KeyFrame>  key_frame_buff_) {

    has_new_global_map_ = false;
    CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());

    // LOG(ERROR) << "找到的文件夹之内的文件数量：   " << file_names_.size() ;

    for (int i = last_id; i <= key_frame_buff_.back().index; i++)
    {
        SaveKeyCloud(i,cloud_ptr);
    }
    last_id=key_frame_buff_.back().index;

    // //第一次开始初始化
    // if (file_names_.size()==0)
    // {
    //     file_names_get_.GetFileNames(file_names_);
    //     for (int i = 0; i < file_names_.size(); i++)
    //     {
    //         SaveKeyCloud(file_names_[i],cloud_ptr);
    //     }
    // }
    // else
    // {
    //     std::vector<std::string> file_names;
    //     file_names_get_.GetFileNames(file_names);
    //     for (int i = file_names_.size(); i < file_names.size(); i++)
    //     {
    //         SaveKeyCloud(file_names[i],cloud_ptr);
    //     }
    // }
    has_new_global_map_ = true;
    return true;
}
/**
 * @description: load之前的关键帧数据，去除地面点之后，存储起来
 * @param {string} file_name
 * @param {CLOUD_PTR} cloud_ptr
 * @return {*}
 */
bool Post_Processing::SaveKeyCloud(int id,CloudData::CLOUD_PTR cloud_ptr) {
        CloudData::CLOUD_PTR cloud_out_ptr(new CloudData::CLOUD());
        std::string file_path = key_frames_path_ +"/key_frames/"+  "key_frame_"+std::to_string(id)+".pcd";
        pcl::io::loadPCDFile(file_path, *cloud_ptr);
        map_without_ground_filter_ptr_->Filter(cloud_ptr,cloud_out_ptr);

        std::string post_file_path = key_frames_path_ +"/post_key_frames/"+ "post_key_frame_"+std::to_string(id)+".pcd" ;
        pcl::io::savePCDFileBinary(post_file_path, *cloud_out_ptr);    
        return true;
}

/**
 * @description: 生成全局地图
 * @param {int} key_frame_index
 * @param {CLOUD_PTR&} map_cloud_ptr
 * @param {Matrix4f&} map_pose
 * @return {*}
 */
bool Post_Processing::JointMap(const std::deque<KeyFrame>& optimized_odom_buff_,CloudData::CLOUD_PTR  &global_map_without_ground) 
{
    global_map_without_ground.reset(new CloudData::CLOUD());
    LOG(INFO)<<"optimized_odom_buff_.size():   "<<optimized_odom_buff_.size();
    for (int i = 0; i < optimized_odom_buff_.size(); i++)
    {
        Eigen::Matrix4f current_optimized_odom=optimized_odom_buff_.at(i).pose;
        int id=optimized_odom_buff_.at(i).index;

        CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
        std::string  file_path=key_frames_path_ +"/post_key_frames/"+ "post_key_frame_"+std::to_string(id)+".pcd" ;
        pcl::io::loadPCDFile(file_path, *cloud_ptr);
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, current_optimized_odom);

        *global_map_without_ground+=*cloud_ptr;
    }
    return true;
}

bool Post_Processing::SaveMap(const std::deque<KeyFrame>& optimized_odom_buff_){
    if (optimized_odom_buff_.size() == 0)
        return false;
    CloudData::CLOUD_PTR map_with_out_ground(new CloudData::CLOUD());
    CloudData::CLOUD_PTR global_map_with_out_ground(new CloudData::CLOUD());
    JointMap(optimized_odom_buff_,map_with_out_ground);
    post_map_without_ground_filter_ptr_->Filter(map_with_out_ground,global_map_with_out_ground);
    std::string map_file_path=map_path_+"/global_map_with_out_ground.pcd";
    pcl::io::savePCDFileBinary(map_file_path, *global_map_with_out_ground);
    LOG(INFO)<<"去除地面点的地图保存为："<<map_file_path;
    pcl::copyPointCloud(*global_map_with_out_ground,*global_map_with_out_ground_);
    //创建栅格地图并且存储
    map_rasterization_ptr_->CreateGridMap(global_map_with_out_ground);
    inflated_gridmap_=map_rasterization_ptr_->GetGridMap();
    return true;
}

nav_msgs::OccupancyGrid Post_Processing::GetGridMap(){
    return inflated_gridmap_;
}

CloudData::CLOUD_PTR Post_Processing::GetGlobalMap(){
    return global_map_with_out_ground_;
}


}

