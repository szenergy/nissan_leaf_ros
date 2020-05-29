# Utils

This directory contains various utils which comes handy when saving rosbag, mounting an HDD etc.

## Rosbag exclude

In a complex vehicular ROS system many topics are present, however not all of them are needed in the bag files. The solution for that can be the use of `-x` exclusion when rosbag record.

E.g.:

```
rosbag record -a -O myrosbag.bag -x "(.*)/velodyne_packets"
```

```
rosbag record -a -O $FILE1 -x "(.*)_packets|(.*)/velodyne_packets|/zed_node/depth(.*)|/zed_node/left/image_rect_color(.*)|/zed_node/left/image_rect_gray(.*)|/zed_node/left_raw(.*)|/zed_node/rgb/image_rect_gray(.*)|/zed_node/rgb_raw(.*)|/zed_node/right/image_rect_color(.*)|/zed_node/right/image_rect_gray(.*)|/zed_node/stereo_raw(.*)/zed_node/rgb/image_rect_color(.*)|/zed_node/rgb/camera_info|/zed_node/parameter_descriptions|/zed_node/parameter_updates|/zed_node/pose_with_covariance|/zed_node/pose_with_covariance|/zed_node/path_odom|/zed_node/path_map|/zed_node/confidence/confidence_map|/zed_node/disparity/disparity_image|/zed_node/point_cloud/cloud_registered|/zed_node/pose"
```

## Saved zed topics

When delaing with the zed stereo camera, there are [lot of zed topics](#all-zed-topics). For our purposes the following ones are required. (The [`rosbag_record_to_1tb_storage.sh`](rosbag_record_to_1tb_storage.sh) shell script saves only the required ones.)

```
/zed_node/stereo/image_rect_color/compressed/parameter_updates
/zed_node/stereo/image_rect_color/theora/parameter_descriptions
/zed_node/stereo/image_rect_color/compressed
/zed_node/stereo/image_rect_color/compressedDepth
/zed_node/stereo/image_rect_color/compressed/parameter_descriptions
/zed_node/stereo/image_rect_color/theora/parameter_updates
/zed_node/stereo/image_rect_color/compressedDepth/parameter_descriptions
/zed_node/stereo/image_rect_color
/zed_node/stereo/image_rect_color/compressedDepth/parameter_updates
/zed_node/stereo/image_rect_color/theora
/zed_node/odom
/zed_node/left/camera_info
/zed_node/right/camera_info
```

## All zed topics

```
/zed_node/confidence/confidence_map
/zed_node/depth/camera_info
/zed_node/depth/depth_registered
/zed_node/depth/depth_registered/compressed
/zed_node/depth/depth_registered/compressed/parameter_descriptions
/zed_node/depth/depth_registered/compressed/parameter_updates
/zed_node/depth/depth_registered/compressedDepth
/zed_node/depth/depth_registered/compressedDepth/parameter_descriptions
/zed_node/depth/depth_registered/compressedDepth/parameter_updates
/zed_node/depth/depth_registered/theora
/zed_node/depth/depth_registered/theora/parameter_descriptions
/zed_node/depth/depth_registered/theora/parameter_updates
/zed_node/disparity/disparity_image
/zed_node/left/camera_info
/zed_node/left/image_rect_color
/zed_node/left/image_rect_color/compressed
/zed_node/left/image_rect_color/compressed/parameter_descriptions
/zed_node/left/image_rect_color/compressed/parameter_updates
/zed_node/left/image_rect_color/compressedDepth
/zed_node/left/image_rect_color/compressedDepth/parameter_descriptions
/zed_node/left/image_rect_color/compressedDepth/parameter_updates
/zed_node/left/image_rect_color/theora
/zed_node/left/image_rect_color/theora/parameter_descriptions
/zed_node/left/image_rect_color/theora/parameter_updates
/zed_node/left/image_rect_gray
/zed_node/left/image_rect_gray/compressed
/zed_node/left/image_rect_gray/compressed/parameter_descriptions
/zed_node/left/image_rect_gray/compressed/parameter_updates
/zed_node/left/image_rect_gray/compressedDepth
/zed_node/left/image_rect_gray/compressedDepth/parameter_descriptions
/zed_node/left/image_rect_gray/compressedDepth/parameter_updates
/zed_node/left/image_rect_gray/theora
/zed_node/left/image_rect_gray/theora/parameter_descriptions
/zed_node/left/image_rect_gray/theora/parameter_updates
/zed_node/left_raw/camera_info
/zed_node/left_raw/image_raw_color
/zed_node/left_raw/image_raw_color/compressed
/zed_node/left_raw/image_raw_color/compressed/parameter_descriptions
/zed_node/left_raw/image_raw_color/compressed/parameter_updates
/zed_node/left_raw/image_raw_color/compressedDepth
/zed_node/left_raw/image_raw_color/compressedDepth/parameter_descriptions
/zed_node/left_raw/image_raw_color/compressedDepth/parameter_updates
/zed_node/left_raw/image_raw_color/theora
/zed_node/left_raw/image_raw_color/theora/parameter_descriptions
/zed_node/left_raw/image_raw_color/theora/parameter_updates
/zed_node/left_raw/image_raw_gray
/zed_node/left_raw/image_raw_gray/compressed
/zed_node/left_raw/image_raw_gray/compressed/parameter_descriptions
/zed_node/left_raw/image_raw_gray/compressed/parameter_updates
/zed_node/left_raw/image_raw_gray/compressedDepth
/zed_node/left_raw/image_raw_gray/compressedDepth/parameter_descriptions
/zed_node/left_raw/image_raw_gray/compressedDepth/parameter_updates
/zed_node/left_raw/image_raw_gray/theora
/zed_node/left_raw/image_raw_gray/theora/parameter_descriptions
/zed_node/left_raw/image_raw_gray/theora/parameter_updates
/zed_node/odom
/zed_node/parameter_descriptions
/zed_node/parameter_updates
/zed_node/path_map
/zed_node/path_odom
/zed_node/point_cloud/cloud_registered
/zed_node/pose
/zed_node/pose_with_covariance
/zed_node/rgb/camera_info
/zed_node/rgb/image_rect_color
/zed_node/rgb/image_rect_color/compressed
/zed_node/rgb/image_rect_color/compressed/parameter_descriptions
/zed_node/rgb/image_rect_color/compressed/parameter_updates
/zed_node/rgb/image_rect_color/compressedDepth
/zed_node/rgb/image_rect_color/compressedDepth/parameter_descriptions
/zed_node/rgb/image_rect_color/compressedDepth/parameter_updates
/zed_node/rgb/image_rect_color/theora
/zed_node/rgb/image_rect_color/theora/parameter_descriptions
/zed_node/rgb/image_rect_color/theora/parameter_updates
/zed_node/rgb/image_rect_gray
/zed_node/rgb/image_rect_gray/compressed
/zed_node/rgb/image_rect_gray/compressed/parameter_descriptions
/zed_node/rgb/image_rect_gray/compressed/parameter_updates
/zed_node/rgb/image_rect_gray/compressedDepth
/zed_node/rgb/image_rect_gray/compressedDepth/parameter_descriptions
/zed_node/rgb/image_rect_gray/compressedDepth/parameter_updates
/zed_node/rgb/image_rect_gray/theora
/zed_node/rgb/image_rect_gray/theora/parameter_descriptions
/zed_node/rgb/image_rect_gray/theora/parameter_updates
/zed_node/rgb_raw/camera_info
/zed_node/rgb_raw/image_raw_color
/zed_node/rgb_raw/image_raw_color/compressed
/zed_node/rgb_raw/image_raw_color/compressed/parameter_descriptions
/zed_node/rgb_raw/image_raw_color/compressed/parameter_updates
/zed_node/rgb_raw/image_raw_color/compressedDepth
/zed_node/rgb_raw/image_raw_color/compressedDepth/parameter_descriptions
/zed_node/rgb_raw/image_raw_color/compressedDepth/parameter_updates
/zed_node/rgb_raw/image_raw_color/theora
/zed_node/rgb_raw/image_raw_color/theora/parameter_descriptions
/zed_node/rgb_raw/image_raw_color/theora/parameter_updates
/zed_node/rgb_raw/image_raw_gray
/zed_node/rgb_raw/image_raw_gray/compressed
/zed_node/rgb_raw/image_raw_gray/compressed/parameter_descriptions
/zed_node/rgb_raw/image_raw_gray/compressed/parameter_updates
/zed_node/rgb_raw/image_raw_gray/compressedDepth
/zed_node/rgb_raw/image_raw_gray/compressedDepth/parameter_descriptions
/zed_node/rgb_raw/image_raw_gray/compressedDepth/parameter_updates
/zed_node/rgb_raw/image_raw_gray/theora
/zed_node/rgb_raw/image_raw_gray/theora/parameter_descriptions
/zed_node/rgb_raw/image_raw_gray/theora/parameter_updates
/zed_node/right/camera_info
/zed_node/right/image_rect_color
/zed_node/right/image_rect_color/compressed
/zed_node/right/image_rect_color/compressed/parameter_descriptions
/zed_node/right/image_rect_color/compressed/parameter_updates
/zed_node/right/image_rect_color/compressedDepth
/zed_node/right/image_rect_color/compressedDepth/parameter_descriptions
/zed_node/right/image_rect_color/compressedDepth/parameter_updates
/zed_node/right/image_rect_color/theora
/zed_node/right/image_rect_color/theora/parameter_descriptions
/zed_node/right/image_rect_color/theora/parameter_updates
/zed_node/right/image_rect_gray
/zed_node/right/image_rect_gray/compressed
/zed_node/right/image_rect_gray/compressed/parameter_descriptions
/zed_node/right/image_rect_gray/compressed/parameter_updates
/zed_node/right/image_rect_gray/compressedDepth
/zed_node/right/image_rect_gray/compressedDepth/parameter_descriptions
/zed_node/right/image_rect_gray/compressedDepth/parameter_updates
/zed_node/right/image_rect_gray/theora
/zed_node/right/image_rect_gray/theora/parameter_descriptions
/zed_node/right/image_rect_gray/theora/parameter_updates
/zed_node/right_raw/camera_info
/zed_node/right_raw/image_raw_color
/zed_node/right_raw/image_raw_color/compressed
/zed_node/right_raw/image_raw_color/compressed/parameter_descriptions
/zed_node/right_raw/image_raw_color/compressed/parameter_updates
/zed_node/right_raw/image_raw_color/compressedDepth
/zed_node/right_raw/image_raw_color/compressedDepth/parameter_descriptions
/zed_node/right_raw/image_raw_color/compressedDepth/parameter_updates
/zed_node/right_raw/image_raw_color/theora
/zed_node/right_raw/image_raw_color/theora/parameter_descriptions
/zed_node/right_raw/image_raw_color/theora/parameter_updates
/zed_node/right_raw/image_raw_gray
/zed_node/right_raw/image_raw_gray/compressed
/zed_node/right_raw/image_raw_gray/compressed/parameter_descriptions
/zed_node/right_raw/image_raw_gray/compressed/parameter_updates
/zed_node/right_raw/image_raw_gray/compressedDepth
/zed_node/right_raw/image_raw_gray/compressedDepth/parameter_descriptions
/zed_node/right_raw/image_raw_gray/compressedDepth/parameter_updates
/zed_node/right_raw/image_raw_gray/theora
/zed_node/right_raw/image_raw_gray/theora/parameter_descriptions
/zed_node/right_raw/image_raw_gray/theora/parameter_updates
/zed_node/stereo/image_rect_color
/zed_node/stereo/image_rect_color/compressed
/zed_node/stereo/image_rect_color/compressed/parameter_descriptions
/zed_node/stereo/image_rect_color/compressed/parameter_updates
/zed_node/stereo/image_rect_color/compressedDepth
/zed_node/stereo/image_rect_color/compressedDepth/parameter_descriptions
/zed_node/stereo/image_rect_color/compressedDepth/parameter_updates
/zed_node/stereo/image_rect_color/theora
/zed_node/stereo/image_rect_color/theora/parameter_descriptions
/zed_node/stereo/image_rect_color/theora/parameter_updates
/zed_node/stereo_raw/image_raw_color
/zed_node/stereo_raw/image_raw_color/compressed
/zed_node/stereo_raw/image_raw_color/compressed/parameter_descriptions
/zed_node/stereo_raw/image_raw_color/compressed/parameter_updates
/zed_node/stereo_raw/image_raw_color/compressedDepth
/zed_node/stereo_raw/image_raw_color/compressedDepth/parameter_descriptions
/zed_node/stereo_raw/image_raw_color/compressedDepth/parameter_updates
/zed_node/stereo_raw/image_raw_color/theora
/zed_node/stereo_raw/image_raw_color/theora/parameter_descriptions
/zed_node/stereo_raw/image_raw_color/theora/parameter_updates
```

