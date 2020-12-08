cd /mnt/storage_1tb
mkdir -p $(date -I)
cd $(date -I)
TEXT1="$1"
TIME1="$(date +"%Y-%m-%d-%H-%M")"
FILE1="$TEXT1$TIME1.bag"
FILE2="$TEXT1$TIME1.txt"
PWD1="$(pwd)"
echo "Writing to file: $PWD1/$FILE1"
echo "Writing to file: $PWD1/$FILE2"
rosparam dump $FILE2
rosbag record -a -O $FILE1 -x "(.*)imu(.*)|(.*)cloud(.*)|(.*)scan(.*)|(.*)ublx(.*)|(.*)detected(.*)|(.*)velodyne(.*)|(.*)os1(.*)|(.*)zed(.*)|(.*)image(.*)|/sick_lms_1xx/parameter(.*)|(.*)_ground(.*)" -b 4096
# or:  record -a -O $FILE1  -x (no compress)
# or:  record -a -O $FILE1 --bz2 -x (compress)
