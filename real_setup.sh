source /opt/ros/melodic/setup.bash
source /home/shiraori/Documents/minh/real_a1/devel/setup.bash
export ROS_PACKAGE_PATH=/home/shiraori/Documents/minh/real_a1:${ROS_PACKAGE_PATH}
export LD_LIBRARY_PATH=/home/shiraori/Documents/minh/real_a1/devel/lib:${LD_LIBRARY_PATH}
export UNITREE_SDK_VERSION=3_2
export UNITREE_LEGGED_SDK_PATH=/home/shiraori/Documents/minh/unitree_legged_sdk-3.2
export UNITREE_PLATFORM="amd64"

export ROS_MASTER_URI=http://192.168.1.239:11311
export ROS_IP=192.168.1.239

#source ../catkin_space/devel/setup.bash
