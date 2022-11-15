root_dir1="/home/mdc/Func_test_Platform/lirui"

    
if [ $# -eq 0 ]; then
echo -e "Usage: bash $0 [options] $Y"
echo "       record:   record ros bag and press ctrl-c to stop"
echo "       mapping:  build a global map, send out pointcloud and gird map"
exit -1
elif [ $# -gt 1 ]; then
echo "wrong number of arguments."
else
    case $1 in
        pointcloud)
        cd root_dir1
        source $root_dir1/devel/setup.bash
        roslaunch lidar_localization mapping.launch
        exit 1
        ;;
        savemap)
        sleep 30
        rosservice call /optimize_map
        sleep 30
        rosservice call /save_map
        exit 0
fi
