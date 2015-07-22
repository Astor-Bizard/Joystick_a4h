export ROS_MASTER_URI=http://momobot:11311
# In case of connection issues, try replacing following line by
# export ROS_IP=<your IP connected to the robot>
# or just replacing "wlan2" by the name of this connection (e.g. eth0, wlan1...)
export ROS_IP=`ifconfig wlan2 | grep 'inet addr:' | cut -d: -f2 | cut -d' ' -f1`
gnome-terminal -x sh -c "roslaunch rplidar_ros view_rplidar.launch"
