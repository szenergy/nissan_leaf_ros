cd /mnt/storage_1tb
mkdir -p $(date -I)
cd $(date -I)
TEXT1="$1"
TIME1="$(date +"%Y-%m-%d-%H-%M")"
FILE1="$TEXT1$TIME1.bag"
PWD1="$(pwd)"
echo "Writing to file: $PWD1/$FILE1"
rosbag record -a -O $FILE1 -x "(.*)_packets|(.*)/velodyne_packets|/zed_node/depth(.*)|/zed_node/left(.*)|/zed_node/left_raw(.*)|/zed_node/rgb/image_rect_gray(.*)|/zed_node/rgb_raw(.*)|/zed_node/right(.*)|/zed_node/stereo_raw(.*)"
# exclude ouster and velodyne packets topics, and 
# exclude most of the zed topics, keep /zed_node/stereo/image_rect_color
# and /zed_node/rgb_raw/image_raw_color/compressed and some others