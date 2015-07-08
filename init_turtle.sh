source devel/setup.bash
rosnode list | grep /turtlesim >/dev/null
if [ $? -eq 1 ]
then rosrun turtlesim turtlesim_node >/dev/null &
	 sleep 0.5
	 rosservice call spawn 0 7.6 0 tmp >/dev/null
	 rosservice call /tmp/set_pen 255 0 0 5 0 >/dev/null
	 rosservice call /tmp/teleport_relative 11.05 0 >/dev/null
	 rosservice call kill tmp >/dev/null
fi
gnome-terminal &
rosrun joystick_turtle joystick_node
