# nissan_leaf_ros
SZE Nissan Leaf ROS specific files (generated and so on)

# Install other required packages
Just create an src folder, and issue the following command:

```bash
mkdir src && cd src
vcs -w1 import < ../nissan_packages.repos
```
# Install Python requirements
There are some Python packages required to run some simulation setups (like direct-gps localization):

- UTM (>=0.5.0)

If you would do all this with a command, issue the following command:
```bash
sudo -H pip install -r requirements.txt
```

# Autoware specific files
The test cases rely on Autoware, you need to make Autoware paths available on your system.

The following Autoware packages are required for full functionality:
- __waypoint_maker (waypoint_marker_publisher)__: to publish Lanes as RViz paths.