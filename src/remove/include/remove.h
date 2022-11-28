#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <fstream>
 
#include <json/json.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <datatype.h>

class Remove
{
public:
    Remove(ros::NodeHandle &nh){
        nh.getParam("json_file_path", json_file_path);
        nh.getParam("OccVal", OccVal);

        //main load
        nh.getParam("up_path", up_path);
        nh.getParam("down_path", down_path);

        json_file.open(json_file_path);
        if (!json_reader.parse(json_file, root_new))
        {
            std::cout << "Error opening json file : " << json_file_path << std::endl;
        }
        //init

        //先把新格式代码转换为旧格式 root为旧格式，只有在最后的时候才转换为新格式存储
        root["width"]=root_new["width"];
        root["height"]=root_new["height"];
        root["origin.x"]=root_new["origin.x"];
        root["origin.y"]=root_new["origin.y"];
        root["resolution"]=root_new["resolution"];


        width=root["width"].asDouble();
        height=root["height"].asDouble();
        origin_x=root["origin.x"].asDouble();
        origin_y=root["origin.y"].asDouble();
        resolution=root["resolution"].asFloat();


        //可视化
        global_map.header.frame_id = "/map";
        global_map.info.width = root["width"].asInt();
        global_map.info.height = root["height"].asInt();
        global_map.info.resolution = root["resolution"].asFloat();
        global_map.info.origin.position.x = root["origin.x"].asDouble();
        global_map.info.origin.position.y = root["origin.y"].asDouble();
        global_map.info.origin.position.z = 0.0;
        size = global_map.info.width*global_map.info.height;
        global_map.data.resize(size,0);
        route_map.data.resize(size,0);
        std::cout<<"root:  "<<root["width"].asInt()<<"   "<<root["height"].asInt()<<"    "<<root["data"].size()<<std::endl;        

        for (int i = 0; i < size; i++)
        {
            global_map.data[i]=0;
        }

        for (int i = 0; i < root_new["data"].size(); i++)
        {
            global_map.data[root_new["data"][i].asInt()]=100;
        }
        

        for (int i = 0; i < size; i++)
        {
            root["data"].append(global_map.data[i]);
        }

        //道路信息地图初始化
        route_map.header.frame_id = "/map";
        route_map.info.width = root["width"].asInt();
        route_map.info.height = root["height"].asInt();
        route_map.info.resolution = root["resolution"].asFloat();
        route_map.info.origin.position.x = root["origin.x"].asDouble();
        route_map.info.origin.position.y = root["origin.y"].asDouble();
        route_map.info.origin.position.z = 0.0;
        
        
        os.open(json_file_path, std::ios::out);
        if (!os.is_open())
            std::cout << "error:can not find or create the file which named \" demo.json\"." << std::endl;

        getRemovePoint = nh.subscribe("/clicked_point",1,&Remove::removePoint,this);
        getAddPoint = nh.subscribe("/initialpose",1,&Remove::addPoint,this);

        occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/ogm_json", 1);
        road_pub=nh.advertise<nav_msgs::OccupancyGrid>("/ogm_json1", 1);
        // blank_grid_pub=nh.advertise<nav_msgs::OccupancyGrid>("/ogm_json1", 1);
    }

    void loadmainroute(){
        std::ifstream up_file(up_path);
        std::ifstream down_file(down_path);
        std::string one_line;
        RoutePoint line;
        int up_count=0;
        int down_count=0;

        std::cout<<"dead"<<std::endl;
        //up road
        std::getline(up_file, one_line);
        int sparse=0;
        std::vector<std::string> fields(6);
        while (std::getline(up_file, one_line))
        {
            if (!one_line.size())
            {
                break;
            }
            up_count++;
            std::istringstream sin(one_line);
            std::string field;
            int column=0;
            while (getline(sin, field, ','))
            { // 将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
                fields[column]=field;
                column++;
                // fields.push_back(field); // 将刚刚读取的字符串添加到向量fields中
            }
            line.x = std::stof(fields[0]);
            line.y = std::stof(fields[1]);
            line.z = std::stof(fields[2]);
            // std::cout<<"up_count:   "<<up_count<<"   "<<line.x<<"   "<<line.y<<"   "<<line.z<<std::endl;
            // std::vector<std::string>().swap(fields);
            // fields.clear();


            //init route_map
            point_x=line.x;
            point_y=line.y;
            // int index_x=int((point_x)/resolution);
            // int index_y=int((point_y)/resolution);
            int index_x=int((point_x-origin_x)/resolution);
            int index_y=int((point_y-origin_y)/resolution);

            int index = index_x + index_y * width;

            for (int i = -1; i < 1; i++)
            {
                for (int j = -1; j < 1; j++)
                {
                    index = (index_x+i) + (index_y+j) * width;
                    route_map.data[index]=100;
                }
            }

        }
        std::cout << "up road size:" << up_count << std::endl;

        //down road
        std::getline(down_file, one_line);
        std::cout<<"one_line:  "<<one_line<<std::endl;
        while (std::getline(down_file, one_line))
        {
            if (!one_line.size())
            {
                break;
            }
            down_count++;
            std::istringstream sin(one_line);
            std::string field;
            // std::cout<<fields.size()<<std::endl;
            int column=0;
            while (getline(sin, field, ','))
            { // 将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
                // fields.push_back(field); // 将刚刚读取的字符串添加到向量fields中
                fields[column]=field;
                column++;

            }

            line.x = std::stof(fields[0]);
            line.y = std::stof(fields[1]);
            line.z = std::stof(fields[2]);
            // std::cout << "line.heading" <<line.heading <<  std::endl;
            // std::vector<std::string>().swap(fields);
            // fields.clear();

            //init route_map
            point_x=line.x;
            point_y=line.y;
            // int index_x=int((point_x)/resolution);
            // int index_y=int((point_y)/resolution);
            int index_x=int((point_x-origin_x)/resolution);
            int index_y=int((point_y-origin_y)/resolution);

            int index = index_x + index_y * width;

            for (int i = -1; i < 1; i++)
            {
                for (int j = -1; j < 1; j++)
                {
                    index = (index_x+i) + (index_y+j) * width;
                    route_map.data[index]=100;
                }
            }

        }

        std::cout << "down road size:" << down_count << std::endl;

        if ((!up_count)||(!down_count))
        {
            ROS_ERROR("main road infomation fail to get");
            std::cout << "up road size:" << up_count << std::endl;
            std::cout << "down road size:" << down_count << std::endl;

        }
        std::cout << "success:"<< std::endl;
    }

    void removePoint(const geometry_msgs::PointStamped &msg){
        point_x=msg.point.x;
        point_y=msg.point.y;
        int index_x=int((point_x-origin_x)/resolution);
        int index_y=int((point_y-origin_y)/resolution);


        int index = index_x + index_y * width;

        for (int i = -2; i < 2; i++)
        {
            for (int j = -2; j < 2; j++)
            {
                index = (index_x+i) + (index_y+j) * width;
                root["data"][index]=0;
                std::cout<<index_x<<"  "<<index_y<<"  "<<index_y<<"   "<<index<<std::endl;

            }
        }

        //更新rviz的地图
        for (int i = 0; i < size; i++)
        {
            global_map.data[i]=root["data"][i].asInt();
        }
    }

    //添加点
    void addPoint(const geometry_msgs::PoseWithCovarianceStamped &msg){
        point_x=msg.pose.pose.position.x;
        point_y=msg.pose.pose.position.y;
        int index_x=int((point_x-origin_x)/resolution);
        int index_y=int((point_y-origin_y)/resolution);

        int index = index_x + index_y * width;
        for (int i = -3; i < 3; i++)
        {
            for (int j = -3; j < 3; j++)
            {
                index = (index_x+i) + (index_y+j) * width;
                root["data"][index]=100;
                // std::cout<<index_x<<"  "<<index_y<<"  "<<index_y<<"   "<<index<<std::endl;

            }
        }

        std::cout << global_map.data.size()<<"    "<<root["data"].size()<< std::endl;
        //更新rviz的地图
        std::cout<<size<<std::endl;

        // for (int i = 449150; i < size; i++)
        // {
        //     std::cout<<global_map.data[i]<<std::endl;
        // }

        std::cout<<root["width"].asInt()<<"   "<<root["height"].asInt()<<"    "<<root["data"].size()<<std::endl;

        for (int i = root["data"].size() - 1; i >0; i--)
        {
            std::cout<<root["data"][i].asInt()<<"   "<<i<<std::endl;
            std::cout<<"dead"<<std::endl;
            global_map.data[i]=root["data"][i].asInt();
        }
    }

    void inflate(){
        root["width"]=740;
        root["height"]=1000;
        root["origin.x"]=60;
        root["origin.y"]=-600;

        root["data"].resize(740000);
        for (int i = root["data"].size() - 1; i >0; i--)
        {
            root["data"][i]=0;
        }
    }

    void pubOgm() { 
        // std::cout<<"dead"<<std::endl;
        occupancy_grid_pub.publish(global_map); 
        // std::cout<<"dead1"<<std::endl;
        road_pub.publish(route_map);
        // std::cout<<"dead2"<<std::endl;
    }

    ~Remove() { 
        std::cout<<"析构函数被调用"<<std::endl;
        root_new["data"].clear();
        for (int i = 0; i < root["data"].size(); i++)
        {
            if (root["data"][i]==0)
            {
                continue;
            }
            root_new["data"].append(i);
        }
        

        os << sw.write(root_new);
        os.close();
    }

public:
    std::string json_file_path;
    double point_x;
    double point_y;
    double origin_x;
    double origin_y;
    nav_msgs::OccupancyGrid global_map;
    nav_msgs::OccupancyGrid route_map;
    // nav_msgs::OccupancyGrid temp;
    Json::Reader json_reader;
    std::ifstream json_file;
    Json::Value root;
    Json::Value root_new;
    Json::StyledWriter sw;
    std::ofstream os;
    int width, height, size;
    float resolution;
    int OccVal;

    //主路信息
    std::string up_path;
    std::string down_path;
    // std::vector<RoutePoint> up_road_;
    // std::vector<RoutePoint> down_road_;

    ros::Subscriber getRemovePoint;
    ros::Subscriber getAddPoint;

    ros::Publisher occupancy_grid_pub;
    ros::Publisher road_pub;

    // ros::Publisher blank_grid_pub;

};


