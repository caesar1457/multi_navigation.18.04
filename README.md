to bring up different name space:
ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_bringup turtlebot3_robot.launch
ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_bringup turtlebot3_robot.launch

to launch the main:
export TURTLEBOT3_MODEL=waffle_pi
roslaunch multi_navigation multi_navigation.launch

an example to send the multi targets:
rostopic pub /task_allocation std_msgs/String "{data: '{\"tb3_0\": [19,21,27,19], \"tb3_1\": [22,26,28,21]}'}"
