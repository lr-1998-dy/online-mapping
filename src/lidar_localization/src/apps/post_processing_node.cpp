/*
 * @Description: 前端里程计的node文件
 * @Author: Li Rui
 * @Date: 2022-02-05 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include <lidar_localization/savePostMap.h>
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/mapping/post_processing/post_processing_flow.hpp"


using namespace lidar_localization;

std::shared_ptr<PostProcessingFlow> _post_processing_flow_ptr;
bool _need_post_map = false;

bool save_postmap_callback(savePostMap::Request &request, savePostMap::Response &response) {
    _need_post_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "post_processing_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("save_postmap", save_postmap_callback);
    _post_processing_flow_ptr = std::make_shared<PostProcessingFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        _post_processing_flow_ptr->Run();

        if (_need_post_map) {
            _post_processing_flow_ptr->SaveMap();
            _post_processing_flow_ptr->PublishData();
            _need_post_map = false;
        }

        rate.sleep();
    }

    return 0;
}