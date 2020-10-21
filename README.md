# nissan_leaf_ros
SZE Nissan Leaf ROS specific files. The most important package here is the `nissan_bringup`.

![img](nissan_bringup/meshes/Nissan_Leaf_Simulation_02_06.png)

## nissan_bringup
This package is responsible for the basic functionality of the vehicle (drivers, simple demo requirements, etc).

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