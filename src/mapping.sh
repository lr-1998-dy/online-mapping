root_dir1="/home/mdc/tool/grid_map_record"
###
 # @Author: lr 2012227985@qq.com
 # @Date: 2022-11-24 22:13:01
 # @LastEditors: lr 2012227985@qq.com
 # @LastEditTime: 2022-11-27 10:36:40
 # @FilePath: /online-mapping/src/mapping.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 
child_dir=$(date +%Y%m%d)
data_map_path="/disk3/ConchFile/gridmap/"
grid_map_path=$data_map_path$child_dir"/global_map.json"
file_path="/home/mdc/tool/grid_map_record/src/lidar_localization/time.txt"

case $1 in
    pointcloud)
    cd $root_dir1
    source devel/setup.bash
    rm -rf $file_path
    touch $file_path
    timer_start=`date "+%Y-%m-%d %H:%M:%S"`
    start_seconds=$(date --date="$timer_start" +%s);
    echo $start_seconds > $file_path
    roslaunch lidar_localization mapping.launch
   ;;
  savemap)
    cd $root_dir1
    read start_seconds <$file_path
    timer_end=`date "+%Y-%m-%d %H:%M:%S"`
    end_seconds=$(date --date="$timer_end" +%s);
    source devel/setup.bash
    rosservice call /stop_mapping
    echo -e "正在建点云地图与栅格地图，请耐心等待" "5分钟..."
    sleep $((end_seconds-start_seconds))
    rosservice call /optimize_map
    sleep 5
    rosservice call /save_map
    sleep 20
    echo -e "已经建完地图，等10秒钟将自动启动擦除障碍物程序"
    pkill ros
    pkill rviz
    sleep 10
    roslaunch remove remove.launch arg_json_file_path:=$grid_map_path
   ;;
esac