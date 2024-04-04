# Rover navigation

- **install python packages to work in ROS**

sudo apt install python3-catkin-tools python3-osrf-pycommon

sudo apt install python3-wstool

- **install ros related packages**

wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh

sudo apt install ros-$ROS_DISTRO-catkin python3-catkin-tools\
   
sudo apt install ros-noetic-moveit

sudo apt install ros-noetic-rqt-controller-manager

sudo apt install ros-noetic-rqt-joint-trajectory-controller

sudo apt install ros-noetic-rqt-multiplot

sudo apt install husky_gazebo

sudo apt-get install ros-noetic-husky-simulator

cd ~/
  
mkdir --parents catkin_ws/src (put files in this folder)
    
cd catkin_ws

catkin_make

- **to run the code**

export ROS_PACKAGE_PATH=/home/sx/catkin_ws/src/:$ROS_PACKAGE_PATH

roslaunch  husky_mpc_datadriven  world1_1.launch

python3 fullK1_1.py


- **The following is not needed, just leave here as a record...**

(echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc
   
python3-osrf-pycommon\

sudo apt install ros-$ROS_DISTRO-moveit\

sudo apt install ros-$ROS_DISTRO-rqt-controller-manager\

ros-$ROS_DISTRO-rqt-joint-trajectory-controller\

ros-$ROS_DISTRO-rqt-multiplot\)
