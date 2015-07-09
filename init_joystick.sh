source devel/setup.bash
if [ $# -eq 1 ]
then
	if [[ $1 == "momobot" ]]
	then export ROS_IP=192.168.130.35
	fi
export ROS_MASTER_URI=http://$1:11311
else export ROS_MASTER_URI=http://localhost:11311
fi
gnome-terminal &
rosrun joystick_turtle joystick_node
