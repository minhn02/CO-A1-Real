source /opt/ros/melodic/setup.bash
source /home/hrg/Documents/minh/real_a1/devel_isolated/setup.bash
export ROS_PACKAGE_PATH=/home/hrg/Documents/minh/real_a1:${ROS_PACKAGE_PATH}
export LD_LIBRARY_PATH=/home/hrg/Documents/minh/real_a1/devel_isolated/lib:${LD_LIBRARY_PATH}
export UNITREE_SDK_VERSION=3_2
export UNITREE_LEGGED_SDK_PATH=/home/hrg/Documents/minh/unitree_legged_sdk-3.2
export UNITREE_PLATFORM="amd64"

export ROS_MASTER_URI=http://192.168.1.239:11311
export ROS_IP=192.168.1.239
