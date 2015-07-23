Joystick teleoperating module on Amibot with obstacles detection and feedback
=============================================================================

## Introduction

This project is a C++ module to drive a robot equipped with a rplidar laser scanner. The purpose is to give the user a feedback on the joystick of the several obstacles the robot may encounter.  
This is using [ROS](http://www.ros.org/) (Robot Operating System) to communicate with the robot, and [SmartServoFramework](https://github.com/emericg/SmartServoFramework) from Emeric Grange ( <emeric.grange@gmail.com> ) to drive the servos of the joystick.

## Prerequisites

### ROS
To build and execute the programs, you will need [ROS](http://www.ros.org/install/) installed (version indigo will be pretty nice, hydro or jade should work. For earlier versions, changes in code will be needed).  
It is recommended to follow the [ROS Beginner Tutorial](http://wiki.ros.org/ROS/Tutorials#Beginner_Level) if you don't know it aldready.

### Joystick

You will need [these prerequisite](https://github.com/emericg/SmartServoFramework#prerequisite), and two Dynamixel servos from [Robotis](http://www.robotis.com/) for the joystick.

## Building the module

> $ cd Joystick_a4h/src/  
> $ catkin_init_workspace  
> $ cd ../  
> $ catkin_make  

You will also need to update your .bashrc so ROS can find your packages :
> $ cd ~/  
> $ echo "source /opt/ros/\<indigo-hydro-jade-...>/setup.bash" >> .bashrc  
> $ echo "source Joystick_a4h/devel/setup.bash" >> .bashrc  

## Given scripts :

* stop/block_servos: If for some reason you need to free or block the joystick.

A virtual test is given, using the ROS Turtlesim :
* init_roscore: It will open a new terminal, and run roscore. It is necessary for ROS to run other programs.
* init_teleop_turtle: It will open several new terminals, and a turtle simulator. You can then move the turtle with the joystick.  

> \# Run these two commands in the same terminal  
> $ ./init_roscore.sh  
> $ ./init_teleop_turtle.sh  

Please take care to wait roscore is completely initialized before running the teleop, unless you may experience crash issues.

To drive Amibot :
* launch_rviz_amibot: It will open a new terminal and a window, where you will see the scanning of the robot. Until ROSbridge isn't used in this module, you will need to be connected at the same network than the robot (A4H_smart_home).
* init_teleop_amibot: It will open several new terminals. You can then move the robot with the joystick.  

> \# Run these two commands in the same terminal  
> $ ./launch_rviz_amibot.sh  
> $ ./init_teleop_amibot.sh  

Please take care to wait rviz is completely initialized before running the teleop, unless you may experience crash issues.

## Nodes and Topics
![NODES](http://i.imgur.com/wVb8Q3M.png)

## Possible issues
Since this project is still experimental, you may experience some connection issues between PC and robot. Try then modifying ROS_IP in scripts of amibot.  
Don't forget to modify ~/.bashrc unless the package won't be found.

The servos IDs are default 1 and 10. If your servos have different IDs, modify it in src/joystick_turtle/joystick_node.cpp.

If for some reasons you experience issues while using serial link with your USB ports, modify in src/joystick_turtle/joystick_node.cpp the name of the port you are using for serial link.

## Get involved !

### Users

You can help us finding bugs, proposing new functionalities and more directly on this website ! Click on the "New issue" button in the menu to do that.

### Developers

You can browse the git repository here on GitHub, submit patches and push requests!  
[Here is a link to SmartServoFramework](https://github.com/emericg/SmartServoFramework) from Emeric Grange ( <emeric.grange@gmail.com> ) that is used in this project.

## Licensing

Joystick_a4h is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.
[Consult the licence on the FSF website](http://www.gnu.org/licenses/lgpl-3.0.txt).

Please read the Joystick_a4h/src/rplidar_ros/[LICENSE](https://github.com/Astor-Bizard/Joystick_a4h/blob/master/src/rplidar_ros/LICENSE)

Copyright (c) 2015, INRIA, All rights reserved.  
Astor Bizard <astord.bizard@inria.fr>  
Emeric Grange <emeric.grange@gmail.com>  
Dominique Vaufreydaz <dominique.vaufreydaz@inria.fr>  
