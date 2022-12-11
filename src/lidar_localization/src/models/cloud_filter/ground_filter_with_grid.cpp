/*
 * @Description: 利用栅格单帧删除地面点
 * @Autor: Li Rui
 * @Date: 2022-09-04 13:36:47
 * @LastEditTime: 2022-12-11 20:40:18
 */

#include "lidar_localization/models/cloud_filter/ground_filter_with_grid.hpp"

#include "glog/logging.h"

namespace lidar_localization {

GridGroundFilter::GridGroundFilter(const YAML::Node& node) {
    int count_threshold= node["count_threshold"].as<int>();    
    float height_threshold = node["height_threshold"].as<float>();
    float     map_resolution = node["map_resolution"].as<float>();
    
    SetFilterParam(count_threshold, height_threshold, map_resolution);
}

GridGroundFilter::GridGroundFilter(int count_threshold,float height_threshold,float  map_resolution) {
    SetFilterParam(count_threshold, height_threshold, map_resolution);
}

bool GridGroundFilter::SetFilterParam(int count_threshold,float height_threshold,float  map_resolution) {
    count_threshold_=count_threshold;
    height_threshold_=height_threshold;
    map_resolution_=map_resolution;
    LOG(INFO) << "Grid GroundFilter 的参数为：" << std::endl
              << "count_threshold" << ", "<<count_threshold<< ", "
              << "height_threshold" << ", "<<height_threshold<< ", "
              << "map_resolution"<<map_resolution 
              << std::endl << std::endl;

    return true;
}

bool GridGroundFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr){
    cv::Mat ogm_mat;
    CreateInitialMap(input_cloud_ptr,ogm_mat);    
    LOG(ERROR) << "CreateInitialMap&WriteToCloud ";
    WriteToCloud(input_cloud_ptr,filtered_cloud_ptr,ogm_mat);
    return true;
}

bool GridGroundFilter::CreateInitialMap(const CloudData::CLOUD_PTR& input_cloud_ptr,cv::Mat& ogm_mat){
    double x_min, x_max, y_min, y_max, z_min, z_max;

    //calculate the max and min x and y
    for (int i = 0; i < input_cloud_ptr->points.size() - 1; i++)
    {
        if (i == 0)
        {
            x_min = x_max = input_cloud_ptr->points[i].x;
            y_min = y_max = input_cloud_ptr->points[i].y;
            z_min = z_max = input_cloud_ptr->points[i].z;
        }

        double x = input_cloud_ptr->points[i].x;
        double y = input_cloud_ptr->points[i].y;
        double z = input_cloud_ptr->points[i].z;

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

    x_min_=x_min;
    y_min_=y_min;
    int ogm_mat_width=int((x_max - x_min) / map_resolution_)+1;
    int ogm_mat_height=int((y_max - y_min) / map_resolution_)+1;
    ogm_mat=cv::Mat(ogm_mat_height, ogm_mat_width, CV_8UC1);

    //initialize the two vectors
    cv::Mat maxheight(ogm_mat_height, ogm_mat_width, CV_8UC1);
    cv::Mat minheight(ogm_mat_height, ogm_mat_width, CV_8UC1);
    cv::Mat count(ogm_mat_height, ogm_mat_width, CV_8UC1);

    // std::vector<std::vector<float>> maxheight(ogm_mat_height);
    // std::vector<std::vector<float>> minheight(ogm_mat_height);
    // std::vector<std::vector<float>> count(ogm_mat_height);

    for (int i = 0; i < ogm_mat_height; i++)
    {
        // maxheight[i].resize(ogm_mat_width);
        // minheight[i].resize(ogm_mat_width);
        // count[i].resize(ogm_mat_width);

        for (int j = 0; j < ogm_mat_width; j++)
        {
            maxheight.at<uint8_t>(i, j)= std::numeric_limits<float>::min();
            minheight.at<uint8_t>(i, j)= std::numeric_limits<float>::max();
            count.at<uint8_t>(i, j)= 0;
        }
    }

    for (int i = 0; i < input_cloud_ptr->points.size(); i++)
    {
        int x = int((input_cloud_ptr->points[i].x - x_min) / map_resolution_);
        int y = int((input_cloud_ptr->points[i].y - y_min) / map_resolution_);

        count.at<uint8_t>(y, x)++;
        if (input_cloud_ptr->points[i].z > maxheight.at<uint8_t>(y, x))
        {
            maxheight.at<uint8_t>(y, x) = input_cloud_ptr->points[i].z;
        }
        if (input_cloud_ptr->points[i].z < minheight.at<uint8_t>(y, x))
        {
            minheight.at<uint8_t>(y, x) = input_cloud_ptr->points[i].z;
        }
    }

    // cv::namedWindow("count", CV_WINDOW_NORMAL);
    // cv::imshow("count", count);
    // cv::waitKey(0);
    int count_cv=0;
    for (int i = 0; i < ogm_mat_height; i++)
    {
        for (int j = 0; j < ogm_mat_width; j++)
        {
            ogm_mat.at<uint8_t>(i, j)=0;
            if (count.at<uint8_t>(i, j)>= count_threshold_&&maxheight.at<uint8_t>(i, j) - minheight.at<uint8_t>(i, j) >= height_threshold_ )
            {
                ogm_mat.at<uint8_t>(i, j)=100; 
                count_cv++;
            }
        }
    }

    // cv::namedWindow("腐蚀膨胀之前地图", CV_WINDOW_NORMAL);
    // cv::imshow("腐蚀膨胀之前地图", ogm_mat);
    // cv::waitKey(0);

    LOG(ERROR) << "error "<<count_cv;

    return true;
}

bool GridGroundFilter::WriteToCloud(const CloudData::CLOUD_PTR& input_cloud_ptr,CloudData::CLOUD_PTR& filtered_cloud_ptr,const cv::Mat& ogm_mat){
    LOG(ERROR) << "error ";
    for (int i = 0; i < input_cloud_ptr->points.size(); i++)
    {
        int x = int((input_cloud_ptr->points[i].x - x_min_) / map_resolution_);
        int y = int((input_cloud_ptr->points[i].y - y_min_) / map_resolution_);
       
        filtered_cloud_ptr->points.push_back(input_cloud_ptr->points[i]);

        for (size_t i = 0; i < count; i++)
        {
            if (ogm_mat.at<uint8_t>(y, x)==100)
            {
                
            }
        }
        
        
    }
    return true;
}

} 