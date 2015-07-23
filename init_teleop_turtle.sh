export ROS_MASTER_URI=http://localhost:11311
gnome-terminal -x sh -c "rosrun joystick_a4h joystick_node"
rosnode list | grep /turtlesim >/dev/null
if [ $? -eq 1 ]
then gnome-terminal -x sh -c "rosrun turtlesim turtlesim_node >/dev/null"
	 sleep 0.5
	 rosservice call spawn 0 7.6 0 tmp >/dev/null
	 rosservice call /tmp/set_pen 255 0 0 5 0 >/dev/null
	 rosservice call /tmp/teleport_relative 11.05 0 >/dev/null
	 rosservice call kill tmp >/dev/null
fi
gnome-terminal -x sh -c "rosrun joystick_a4h joystick_turtlesim_server"
