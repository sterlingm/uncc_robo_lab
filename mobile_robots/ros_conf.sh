# If grabbing this from the uncc_robo_lab repo, note that the settings below probably need to be changed



# Retup the ROS environment variables
export ROS_DISTRO=kinetic
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source the catkin workspace(s)
source ~/ros_workspace/devel/setup.bash

# Source the rosbuild setup
source ~/rosbuild_workspace/setup.bash

# Setup the ROS network variables
export ROS_PORT=11311

export ROS_IP=192.168.1.107

export ROS_MASTER_IP=192.168.1.107

#==========================
#Local testing
#export ROS_IP=127.0.0.1
#export ROS_MASTER_IP=127.0.0.1
#==========================

export ROS_MASTER_URI=http://${ROS_MASTER_IP}:${ROS_PORT}
export ROSLAUNCH_SSH_UNKNOWN=1


#export TURTLEBOT_BASE=create
#export TURTLEBOT_STACKS=circle
#export TURTLEBOT_3D_SENSOR=kinect
export TURTLEBOT_SERIAL_PORT=/dev/ttyUSB0
