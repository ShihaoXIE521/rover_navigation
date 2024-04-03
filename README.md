# Rover navigation

1. wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh

2. sudo apt install ros-$ROS_DISTRO-catkin python3-catkin-tools\

3. sudo apt install python3-catkin-tools python3-osrf-pycommon

4. sudo apt install python3-wstool

5. sudo apt install ros-noetic-moveit

6. sudo apt install ros-noetic-rqt-controller-manager

7. sudo apt install ros-noetic-rqt-joint-trajectory-controller

8. sudo apt install ros-noetic-rqt-multiplot

9. sudo apt install husky_gazebo

10. sudo apt-get install ros-noetic-husky-simulator

11. cd ~/
  
12. mkdir --parents catkin_ws/src (put files in this folder)
    
13. cd catkin_ws

14. catkin_make

15. export ROS_PACKAGE_PATH=/home/sx/catkin_ws/src/:$ROS_PACKAGE_PATH

16. roslaunch  husky_mpc_datadriven  world1_1.launch
    
17. python3 fullK1_1.py


The following might be optional...

(2. echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

3. source ~/.bashrc
   
17. python3-osrf-pycommon\

19. sudo apt install ros-$ROS_DISTRO-moveit\

20. sudo apt install ros-$ROS_DISTRO-rqt-controller-manager\

21. ros-$ROS_DISTRO-rqt-joint-trajectory-controller\

22. ros-$ROS_DISTRO-rqt-multiplot\)
