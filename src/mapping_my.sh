root_dir1="/home/lirui/online-mapping"

case $1 in
    pointcloud)
    cd $root_dir1
    source devel/setup.bash
    roslaunch lidar_localization mapping.launch
   ;;
  savemap)
    cd $root_dir1
    source devel/setup.bash
    sleep 30
    rosservice call /optimize_map
    sleep 30
    rosservice call /save_map
   ;;
esac

