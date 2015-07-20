export ROS_MASTER_URI=http://momobot:11311
export ROS_IP=192.168.130.35
gnome-terminal -x sh -c "rosrun joystick_turtle joystick_node"
gnome-terminal -x sh -c "rosrun joystick_turtle joystick_amibot_transmitter"
gnome-terminal -x sh -c "rosrun joystick_turtle obstacle_manager"
