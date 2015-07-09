source devel/setup.bash
if [[ $ROS_MASTER_URI == "http://momobot:11311" ]]
then rosrun joystick_turtle joystick_turtlesim_server amibot
else rosrun joystick_turtle joystick_turtlesim_server
fi
