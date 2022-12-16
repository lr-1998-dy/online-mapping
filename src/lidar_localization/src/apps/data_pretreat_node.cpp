/*
 * @Description: 数据预处理的node文件
 * @Author: Li Rui
 * @Date: 2022-02-05 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/stopMapping.h"
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

using namespace lidar_localization;

bool _need_stop_map = false;

bool stop_map_callback(stopMapping::Request &request, stopMapping::Response &response) {
    _need_stop_map = true;
    response.succeed = true;
    return response.succeed;
}


int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "data_pretreat_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("stop_mapping", stop_map_callback);
    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        data_pretreat_flow_ptr->Run();
        if(_need_stop_map==true){
            break;
        }

        rate.sleep();
    }

    return 0;
}
