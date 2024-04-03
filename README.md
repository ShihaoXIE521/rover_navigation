# Rover navigation

1. wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh

2. echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

3. source ~/.bashrc

4. sudo apt install ros-$ROS_DISTRO-catkin python3-catkin-tools\

5. sudo apt install python3-catkin-tools python3-osrf-pycommon

6. sudo apt install python3-wstool

7. sudo apt install ros-noetic-moveit

8. sudo apt install ros-noetic-rqt-controller-manager

9. sudo apt install ros-noetic-rqt-joint-trajectory-controller

10. sudo apt install ros-noetic-rqt-multiplot

11. catkin_make

12. export ROS_PACKAGE_PATH=/home/sx/catkin_ws/src/:$ROS_PACKAGE_PATH

13. roslaunch  husky_mpc_datadriven  collision_avoidance_real_husky.launch x:=-4 y:=-4 z:=0 yaw:=0

14. sudo apt install husky_gazebo

15. sudo apt-get install ros-noetic-husky-simulator

16. python3-osrf-pycommon\

17. sudo apt install python3-wstool\

18. sudo apt install ros-$ROS_DISTRO-moveit\

19. sudo apt install ros-$ROS_DISTRO-rqt-controller-manager\

20. ros-$ROS_DISTRO-rqt-joint-trajectory-controller\

21. ros-$ROS_DISTRO-rqt-multiplot\
