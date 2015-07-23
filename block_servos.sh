rosrun joystick_a4h block_servos 1>/dev/null 2>/dev/null
if [ $? -eq 1 ]
then echo "
	/!\\ Connection to the servos lost. Can't block them... /!\\
"
else echo "
	Servos successfully blocked.
"
fi
