export ROS_MASTER_URI=http://momobot:11311
export ROS_IP=`ifconfig wlan2 | grep 'inet addr:' | cut -d: -f2 | cut -d' ' -f1`
gnome-terminal -x sh -c "rosrun joystick_turtle joystick_node"
gnome-terminal -x sh -c "rosrun joystick_turtle joystick_amibot_transmitter"
gnome-terminal -x sh -c "rosrun joystick_turtle obstacle_manager"
