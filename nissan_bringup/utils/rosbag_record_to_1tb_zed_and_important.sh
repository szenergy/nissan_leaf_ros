cd /mnt/storage_1tb
mkdir -p $(date -I)
cd $(date -I)
TEXT1="$1"
TIME1="$(date +"%Y-%m-%d_%H-%M")"
FILE1="$TEXT1$TIME1.bag"
FILE2="x_rosparam_dump_$TEXT1$TIME1.txt"
PWD1="$(pwd)"
echo "Writing to file: $PWD1/$FILE1"
#rosparam dump $FILE2
rosbag record -O $FILE1 -b 4096 /zed_node/left/image_rect_color/compressed /zed_node/left/camera_info /cloud /current_pose /diagnostics /gps/duro/current_pose /gps/nova/current_pose /gps/duro/fix /gps/nova/fix /gps/duro/imu /gps/nova/imu /gps/duro/mag /left_os1/os1_cloud_node/imu /left_os1/os1_cloud_node/points  /right_os1/os1_cloud_node/imu /right_os1/os1_cloud_node/points /scan /tf /vehicle_status /velodyne_left/velodyne_points /velodyne_right/velodyne_points  /vehicle_speed_kmph 



 




