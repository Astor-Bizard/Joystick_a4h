~/SmartServoFramework-master/build/stop_servos 1>/dev/null 2>/dev/null
if [ $? -eq 1 ]
then echo "
	/!\\ Connection to the servos lost. Can't stop them... /!\\
"
else echo " Motors successfully stopped."
fi
