# multi_robot_formation
需预先安装Turtlebot3仿真环境功能包

无障碍物环境下编队

roslaunch turtlebot3_gazebo multi_empty_world.launch 

roslaunch formation empty_line.launch 

ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

障碍物环境下编队

roslaunch turtlebot3_gazebo multi_turtlebot3.launch 

roslaunch turtlebot3_navigation multi_turtlebot3_navigation.launch 

roslaunch formation line.launch
