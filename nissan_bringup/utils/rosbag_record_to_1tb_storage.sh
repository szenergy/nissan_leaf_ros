cd /mnt/storage_1tb
mkdir -p $(date -I)
cd $(date -I)
TEXT1="$1"
TIME1="$(date +"%Y-%m-%d-%H-%M")"
FILE1="$TEXT1$TIME1.bag"
PWD1="$(pwd)"
echo "Writing to file: $PWD1/$FILE1"
rosbag record -a -O $FILE1 -x "(.*)_packets|(.*)/velodyne_packets"