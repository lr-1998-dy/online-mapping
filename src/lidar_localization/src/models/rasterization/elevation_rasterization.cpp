/*
 * @Description: 利用高程生成栅格地图
 * @Author: Li Rui
 * @Date: 2022-02-08 21:46:45
 */
#include "lidar_localization/models/rasterization/elevation_rasterization.hpp"

#include "glog/logging.h"

namespace lidar_localization {

ElevationRasterization::ElevationRasterization(const YAML::Node& node,const std::string &map_path){
    
    int count_threshold = node["count_threshold"].as<int>();
    float height_threshold = node["height_threshold"].as<float>();
    int inflation_map_x = node["inflation_map_x"].as<int>();
    int inflation_map_y = node["inflation_map_y"].as<int>();
    float map_resolution=node["map_resolution"].as<float>();
    float outliers_dis=node["outliers_dis"].as<float>();

    json_path_=map_path+"/global_map.json";

    SetRasterizationParam(count_threshold, height_threshold, inflation_map_x, inflation_map_y,map_resolution,outliers_dis);
}

ElevationRasterization::ElevationRasterization(int count_threshold,float height_threshold,int inflation_map_x,int inflation_map_y,float  map_resolution,float outliers_dis){

    SetRasterizationParam(count_threshold, height_threshold, inflation_map_x, inflation_map_y,map_resolution,outliers_dis);
}

bool ElevationRasterization::SetRasterizationParam(int count_threshold,float height_threshold,int inflation_map_x,int inflation_map_y,float  map_resolution,float outliers_dis){
    count_threshold_=count_threshold;
    height_threshold_=height_threshold;
    inflation_map_x_=inflation_map_x;
    inflation_map_y_=inflation_map_y;
    map_resolution_=map_resolution;
    outliers_dis_=outliers_dis;

    std::cout << "高程栅格化的参数为" << std::endl
              << "count_threshold_: " << count_threshold_ << ", "
              << "height_threshold_: " << height_threshold_ << ", "
              << "inflation_map_x_: " << inflation_map_x_ << ", "
              << "inflation_map_y_: " << inflation_map_y_ << ", "
              << "map_resolution_: " << map_resolution_<<", "
              <<"outliers_dis_: "<<outliers_dis_
              << std::endl << std::endl;

    return true;
}

/**
 * @description: 构造栅格，其中包括初步构造，腐蚀，膨胀三个步骤
 * @param {CLOUD_PTR&} cloud_map
 * @param {OccupancyGrid} &gridmap
 * @return {*}
 */
bool ElevationRasterization::CreateGridMap(const CloudData::CLOUD_PTR& cloud_map) {
    if (cloud_map->points.empty())
    {
        LOG(WARNING)<<"点云地图为空，不能构建栅格地图";
        return false;
    }
    nav_msgs::OccupancyGrid gridmap;
    cv::Mat cv_gridmap;
    cv::Mat cv_pointscount;
    CreateInitialMap(cloud_map,gridmap,cv_gridmap,cv_pointscount);
    Erode(cv_gridmap);
    Dilate(cv_gridmap);
    WriteToGrid(cv_gridmap,gridmap);
    InflateGradMap(gridmap,inflated_gridmap_);
    WriteToNewJson(inflated_gridmap_);

    return true;
}

/**
 * @description: 构造初始的地图
 * @param {CLOUD_PTR&} cloud_map
 * @param {OccupancyGrid} &gridmap
 * @return {*}
 */
bool ElevationRasterization::CreateInitialMap(const CloudData::CLOUD_PTR& cloud_map,nav_msgs::OccupancyGrid &gridmap,cv::Mat &cv_gridmap,cv::Mat &cv_pointscount) {

    gridmap.header.seq = 0;
    gridmap.header.stamp = ros::Time::now();
    gridmap.header.frame_id = "map";

    gridmap.info.map_load_time = ros::Time::now();
    gridmap.info.resolution = map_resolution_;

    double x_min, x_max, y_min, y_max, z_min, z_max;

    //calculate the max and min x and y
    for (int i = 0; i < cloud_map->points.size() - 1; i++)
    {
        if (i == 0)
        {
            x_min = x_max = cloud_map->points[i].x;
            y_min = y_max = cloud_map->points[i].y;
            z_min = z_max = cloud_map->points[i].z;
        }

        double x = cloud_map->points[i].x;
        double y = cloud_map->points[i].y;
        double z = cloud_map->points[i].z;

        if (x < x_min)
            x_min = x;
        if (x > x_max)
            x_max = x;

        if (y < y_min)
            y_min = y;
        if (y > y_max)
            y_max = y;

        if (z < z_min)
            z_min = z;
        if (z > z_max)
            z_max = z;
    }

    gridmap.info.origin.position.x =  x_min;
    gridmap.info.origin.position.y =  y_min;
    gridmap.info.origin.position.z = 0.0;
    gridmap.info.origin.orientation.x = 0.0;
    gridmap.info.origin.orientation.y = 0.0;
    gridmap.info.origin.orientation.z = 0.0;
    gridmap.info.origin.orientation.w = 1.0;

    gridmap.info.width = int((x_max - x_min) / map_resolution_);
    gridmap.info.height = int((y_max - y_min) / map_resolution_);

    //initialize the two vectors
    std::vector<std::vector<float>> maxheight(gridmap.info.height);
    std::vector<std::vector<float>> minheight(gridmap.info.height);
    std::vector<std::vector<float>> count(gridmap.info.height);

    for (int i = 0; i < gridmap.info.height; i++)
    {
        maxheight[i].resize(gridmap.info.width);
        minheight[i].resize(gridmap.info.width);
        count[i].resize(gridmap.info.width);

        for (int j = 0; j < gridmap.info.width; j++)
        {
            maxheight[i][j] = std::numeric_limits<float>::min();
            minheight[i][j] = std::numeric_limits<float>::max();
            count[i][j] = 0;
        }
    }

    for (int i = 0; i < cloud_map->points.size(); i++)
    {
        if (cloud_map->points[i].x > x_min + 1.1 && cloud_map->points[i].x < x_max - 1.1 && cloud_map->points[i].y > y_min + 1.1 && cloud_map->points[i].y < y_max - 1.1)
        {
            int x = int((cloud_map->points[i].x - x_min) / map_resolution_);
            int y = int((cloud_map->points[i].y - y_min) / map_resolution_);

            count[y][x]++;
            if (cloud_map->points[i].z > maxheight[y][x])
            {
                maxheight[y][x] = cloud_map->points[i].z;
            }
            if (cloud_map->points[i].z < minheight[y][x])
            {
                minheight[y][x] = cloud_map->points[i].z;
            }
        }
    }

    gridmap.data.resize(gridmap.info.width * gridmap.info.height);
    gridmap.data.assign(gridmap.info.width * gridmap.info.height, 0);
    cv_gridmap=cv::Mat(gridmap.info.height, gridmap.info.width, CV_8UC1);
    cv_pointscount=cv::Mat(gridmap.info.height, gridmap.info.width, CV_8UC1);

    for (int i = 0; i < gridmap.info.height; i++)
    {
        for (int j = 0; j < gridmap.info.width; j++)
        {
            cv_gridmap.at<uint8_t>(i, j)=0;
            cv_pointscount.at<uint8_t>(i, j)=0;
            if (maxheight[i][j] - minheight[i][j] > height_threshold_ && count[i][j] > count_threshold_)
            {
                int index = j + i * gridmap.info.width;
                gridmap.data[index] = 100;//(maxheight[i][j] - minheight[i][j]) * 100;
                cv_gridmap.at<uint8_t>(i, j)=100; 
                cv_pointscount.at<uint8_t>(i, j)=count[i][j];
            }
        }
    }

    return true;
}

void ElevationRasterization::AddFalseNegatives(cv::Mat& ogm_mat,int i,int j){
    int count=0;

    for (int col=i-1; col < i+2; col++)
    {
        for (int row=j-1; row< j+2; row++)
        {
            if (ogm_mat.at<uint8_t>(col, row)==100)
            {
                count ++;
            }
        }
    }

    if (count>=1)
    {
        ogm_mat.at<uint8_t>(i, j) =50;
    }
}

void ElevationRasterization::RemoveOutliers(cv::Mat& ogm_mat,int i,int j){
    int count=0;
    for (int col=i-outliers_dis_; col < i+outliers_dis_; col++)
    {
        for (int row=j-outliers_dis_; row< j+outliers_dis_; row++)
        {
            if ((abs(col-i)+abs(row-j))==outliers_dis_)
            {
                if (ogm_mat.at<uint8_t>(col, row)==100)
                {
                    count ++;
                }            
            }
        }
    }

    if (count<=3)
    {
        ogm_mat.at<uint8_t>(i, j) =0;
    }
}

bool ElevationRasterization::Erode(cv::Mat &cv_gridmap){
    // cv::namedWindow("腐蚀膨胀之前地图", CV_WINDOW_NORMAL);
    // cv::imshow("腐蚀膨胀之前地图", cv_gridmap);
    // cv::waitKey(0);

    //利用opencv进行腐蚀与膨胀
    cv::Mat erode_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::erode(cv_gridmap, cv_gridmap, erode_element);

    // cv::namedWindow("腐蚀后地图", CV_WINDOW_NORMAL);
    // cv::imshow("腐蚀后地图", cv_gridmap);
    // cv::waitKey(0);

    for (int i = 0; i < cv_gridmap.rows; i++)
    {
        for (int j = 0; j < cv_gridmap.cols; j++)
        {
            if (cv_gridmap.at<uint8_t>(i, j) == 100&&(i>outliers_dis_)&&(i<cv_gridmap.rows-outliers_dis_)&&(j>outliers_dis_)&&(j<cv_gridmap.cols-outliers_dis_))
            {
                RemoveOutliers(cv_gridmap,i,j);
            }   
        }
    }

    // cv::namedWindow("去除离群点后地图", CV_WINDOW_NORMAL);
    // cv::imshow("去除离群点后地图", cv_gridmap);
    // cv::waitKey(0);

    for (int i = 0; i < cv_gridmap.rows; i++)
    {
        for (int j = 0; j < cv_gridmap.cols; j++)
        {
            if (cv_gridmap.at<uint8_t>(i, j) == 100&&(i>15)&&(i<cv_gridmap.rows-15)&&(j>15)&&(j<cv_gridmap.cols-15))
            {
                RemoveOutliers(cv_gridmap,i,j);
            }   
        }
    }

    // cv::namedWindow("2次去除离群点后地图", CV_WINDOW_NORMAL);
    // cv::imshow("2次去除离群点后地图", cv_gridmap);
    // cv::waitKey(0);


    return true;
}

bool ElevationRasterization::Dilate(cv::Mat &cv_gridmap){

    cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::dilate(cv_gridmap, cv_gridmap, dilate_element);

    // cv::namedWindow("膨胀后地图", CV_WINDOW_NORMAL);
    // cv::imshow("膨胀后地图", cv_gridmap);
    // cv::waitKey(0);

    //未完待续
    // for (int i = 0; i < cv_gridmap.rows; i++)
    // {
    //     for (int j = 0; j < cv_gridmap.cols; j++)
    //     {
    //         if (cv_gridmap.at<uint8_t>(i, j) == 100&&(i>15)&&(i<cv_gridmap.rows-15)&&(j>15)&&(j<cv_gridmap.cols-15))
    //         {
    //             RemoveOutliers(cv_gridmap,i,j);
    //         }   
    //     }
    // }

    // cv::namedWindow("去除离群点后地图", CV_WINDOW_NORMAL);
    // cv::imshow("去除离群点后地图", cv_gridmap);
    // cv::waitKey(0);


    return true;
}

bool ElevationRasterization::WriteToGrid(const cv::Mat &cv_gridmap,nav_msgs::OccupancyGrid &gridmap){
    for (int i = 0; i < gridmap.info.height; i++)
    {
        for (int j = 0; j < gridmap.info.width; j++)
        {
            int index = j + i * gridmap.info.width;
            gridmap.data[index] = (cv_gridmap.at<uint8_t>(i, j) > 100) ? 100 : cv_gridmap.at<uint8_t>(i, j);
        }
    }
    return true;
}


bool ElevationRasterization::InflateGradMap(const nav_msgs::OccupancyGrid &gridmap,nav_msgs::OccupancyGrid &inflated_gridmap){
    LOG(INFO)<<"ElevationRasterization::InflateGradMap !!!! \n";
    inflated_gridmap.header.seq = 0;
    inflated_gridmap.header.stamp = ros::Time::now();
    inflated_gridmap.header.frame_id = "map";

    inflated_gridmap.info.map_load_time = ros::Time::now();
    inflated_gridmap.info.resolution = map_resolution_;

    inflated_gridmap.info.origin.position.x =  gridmap.info.origin.position.x - inflation_map_x_;
    inflated_gridmap.info.origin.position.y =  gridmap.info.origin.position.y- inflation_map_y_;
    inflated_gridmap.info.origin.position.z = 0.0;
    inflated_gridmap.info.origin.orientation.x = 0.0;
    inflated_gridmap.info.origin.orientation.y = 0.0;
    inflated_gridmap.info.origin.orientation.z = 0.0;
    inflated_gridmap.info.origin.orientation.w = 1.0;


    inflated_gridmap.info.width = int((gridmap.info.width + 2 * inflation_map_x_)/map_resolution_);
    inflated_gridmap.info.height = int((gridmap.info.height + 2 * inflation_map_y_)/map_resolution_);

    inflated_gridmap.data.resize(inflated_gridmap.info.width * inflated_gridmap.info.height);
    inflated_gridmap.data.assign(inflated_gridmap.info.width * inflated_gridmap.info.height, 0);
    // inflated_occupancy_grid_map_data.resize(inflated_msg.info.width * inflated_msg.info.height);
    // inflated_occupancy_grid_map_data.assign(inflated_msg.info.width * inflated_msg.info.height, 0);
    for (int i = 0; i < gridmap.info.height; i++)
    {
        for (int j = 0; j < gridmap.info.width; j++)
        {
            int new_index = (j + inflation_map_x_) + (i + inflation_map_y_) * inflated_gridmap.info.width;
            int index = j + i * gridmap.info.width;
            inflated_gridmap.data[new_index] = gridmap.data[index];
        }
    }
    return true;
}

bool ElevationRasterization::WriteToJson(const nav_msgs::OccupancyGrid &inflated_gridmap){
    Json::Value root;

    for (int i = 0; i < (inflated_gridmap.info.width * inflated_gridmap.info.height); i++)
    {
        root["data"].append(inflated_gridmap.data[i]);
        // root["data"].append(0);
    }
    root["header.frame_id"] = Json::Value("map");
    //时间戳写入
    // ros::Time json_map_time=inflated_gridmap.info.map_load_time;
    root["header.stamp"] = Json::Value(ros::Time::now().toSec());
    root["origin.x"] = Json::Value(inflated_gridmap.info.origin.position.x);
    root["origin.y"] = Json::Value(inflated_gridmap.info.origin.position.y);
    root["origin.z"] = Json::Value(0.0);
    root["origin.heading"] = Json::Value(0.0);
    root["width"] = Json::Value(inflated_gridmap.info.width);
    root["height"] = Json::Value(inflated_gridmap.info.height);
    root["length"] = Json::Value(20.0);
    root["resolution"] = Json::Value(1.0);

    Json::StyledWriter sw;
    std::ofstream os;
    os.open(json_path_, std::ios::out);
    if (!os.is_open())
        std::cout << "error:can not find or create the file which named \" demo.json\"." << std::endl;
    os << sw.write(root);
    os.close();
    LOG(INFO)<<"created json file !!!! \n";

    return true;
}

bool ElevationRasterization::WriteToNewJson(const nav_msgs::OccupancyGrid &inflated_gridmap){
    Json::Value root;

    for (int i = 0; i < (inflated_gridmap.info.width * inflated_gridmap.info.height); i++)
    {
        if (inflated_gridmap.data[i]>20)
        {
            root["data"].append(i);
        }
        // root["data"].append(0);
    }
    root["header.frame_id"] = Json::Value("map");
    //时间戳写入
    // ros::Time json_map_time=inflated_gridmap.info.map_load_time;
    root["header.stamp"] = Json::Value(ros::Time::now().toSec());
    root["origin.x"] = Json::Value(inflated_gridmap.info.origin.position.x);
    root["origin.y"] = Json::Value(inflated_gridmap.info.origin.position.y);
    root["origin.z"] = Json::Value(0.0);
    root["origin.heading"] = Json::Value(0.0);
    root["width"] = Json::Value(inflated_gridmap.info.width);
    root["height"] = Json::Value(inflated_gridmap.info.height);
    root["length"] = Json::Value(20.0);
    root["resolution"] = Json::Value(1.0);

    Json::StyledWriter sw;
    std::ofstream os;
    os.open(json_path_, std::ios::out);
    if (!os.is_open())
        std::cout << "error:can not find or create the file which named \" demo.json\"." << std::endl;
    os << sw.write(root);
    os.close();
    LOG(INFO)<<"created json file !!!! \n";

    return true;
}

nav_msgs::OccupancyGrid ElevationRasterization::GetGridMap(){
    return inflated_gridmap_;
}

}

