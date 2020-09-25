cd /mnt/storage_1tb
mkdir -p $(date -I)
cd $(date -I)
TEXT1="$1"
TIME1="$(date +"%Y-%m-%d-%H-%M")"
FILE1="$TEXT1$TIME1.bag"
FILE2="$TEXT1$TIME1.txt"
PWD1="$(pwd)"
echo "Writing to file: $PWD1/$FILE1"
#rosparam dump $FILE2
rosbag record -a -O $FILE1 -x "(.*)_packets|(.*)/velodyne_packets|/velodyne_right/velodyne_right_driver/(.*)|/velodyne_right/cloud_nodelet/(.*)|/velodyne_right/lidar_nodelet_manager/bond|/velodyne_left/velodyne_left_driver/(.*)|/velodyne_left/cloud_nodelet/(.*)|/velodyne_left/lidar_nodelet_manager/bond|/zed_node/depth(.*)|/zed_node/left/image_rect_color(.*)|/zed_node/left/image_rect_gray(.*)|/zed_node/left_raw(.*)|/zed_node/rgb/image_rect_gray(.*)|/zed_node/rgb_raw(.*)|/zed_node/right/image_rect_color(.*)|/zed_node/right/image_rect_gray(.*)|/zed_node/stereo_raw(.*)/zed_node/rgb/image_rect_color(.*)|/zed_node/rgb/camera_info|/points_concat|/filtered_points|/zed_node/parameter_descriptions|/(.*)compressed(.*)|/(.*)theora(.*)|/zed_node/parameter_updates|/zed_node/pose_with_covariance|/zed_node/pose_with_covariance|/zed_node/path_odom|/zed_node/path_map|/zed_node/confidence/confidence_map|/zed_node/disparity/disparity_image|/zed_node/point_cloud/cloud_registered|/zed_node/pose|/sick_lms_1xx/parameter(.*)" -b 4096
# or:  record -a -O $FILE1  -x (no compress)
# or:  record -a -O $FILE1 --bz2 -x (compress)
# exclude ouster and velodyne packets topics, and 
# exclude most of the zed topics, keep /zed_node/stereo/image_rect_color
# and /zed_node/odom /zed_node/*/camera_info and some others
