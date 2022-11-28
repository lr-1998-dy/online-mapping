/*
 * @Author: error: git config user.name && git config user.email & please set dead value or install git
 * @Date: 2022-07-24 11:18:19
 * @LastEditors: error: git config user.name && git config user.email & please set dead value or install git
 * @LastEditTime: 2022-09-02 21:23:21
 * @FilePath: /remove/src/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "remove.h"
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "create_ogm");
    ros::NodeHandle n;
    Remove remover(n);
    // remover.inflate();
    // std::string path="/home/lr/lyh/map/uB_BB95_xy.csv";
    // remover.loadmainroute();
    // string frame_id = "base";
    // nav_msgs::OccupancyGrid test_map;
    // test_map.header.frame_id = "base_link";
    // test_map.header.stamp = ros::Time::now();
    // test_map.info.height = 40;
    // test_map.info.width = 40;
    // test_map.info.resolution = 1;
    // test_map.info.origin.position.x = 0;
    // test_map.info.origin.position.y = 0;
    // test_map.info.origin.position.z = 0;
    // test_map.data.resize(test_map.info.height * test_map.info.width);

    // for (int i = 0; i < test_map.data.size(); i++)
    // {
    //     test_map.data[i] = 0;
    //     if (i % 10 == 0)
    //         test_map.data[i] = 100;
    // }

    // ros::Publisher pub_testMap = n.advertise<nav_msgs::OccupancyGrid>("/test_map", 1);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        remover.pubOgm();
        loop_rate.sleep();
    }

    return 0;
}