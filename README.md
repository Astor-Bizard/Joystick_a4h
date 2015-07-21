Joystick teleoperating module on Amibot with obstacles detection and feedback
=============================================================================

## Introduction



## Prerequisites

### ROS
To build and execute the programs, you will need [ROS](http://www.ros.org/install/) installed, version hydro, indigo or jade. (For earlier versions, changes in code will be needed).

### Joystick

## Building the module

> $ cd Joystick_a4h/src/
> $ catkin_init_workspace
> $ cd ../
> $ catkin_make

You will also need to update your .bahsrc so ROS can find your packages :
> $ cd ~/
> $ echo "source /<path>/Joystick_a4h/devel/setup.bash" >> .bashrc

## Given scripts :

* stop/block_servos: If for some reason you need to free or block the joystick.

To test the joystick on the Turtlesim module from ROS :
* init_roscore: It will open a new terminal, and run roscore. It is necessary for ROS to run other programs.
* init_teleop_turtle: It will open several new terminals, and a turtle simulator. You can then move the turtle with the joystick.
> $ ./init_roscore.sh			} Run these in
> $ ./init_teleop_turtle.sh		} the same terminal

To drive Amibot :
* launch_rviz_amibot: It will open a new terminal and a window, where you will see the scanning of the robot. Until ROSBridge isn't used in this module, you will need to be connected at the same network than the robot (A4H_smart_home).
* init_teleop_turtle: It will open several new terminals. You can then move the robot with the joystick.
> $ ./launch_rviz_amibot.sh		} Run these in
> $ ./init_teleop_amibot.sh		} the same terminal
Please take care to wait rviz is completely initialized before running the teleop, unless you may experience crash issues.

## Get involved!

### Users

You can help us finding bugs, proposing new functionalities and more directly on this website! Click on the "New issue" button in the menu to do that.

### Developers

You can browse the git repository here on GitHub, submit patches and push requests!
Here is a link to [SmartServoFramework](https://github.com/emericg/SmartServoFramework) from Emeric Grange <emeric.grange@gmail.com> that is used in this project.

## Licensing

Joystick_a4h is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.
[Consult the licence on the FSF website](http://www.gnu.org/licenses/lgpl-3.0.txt).

Copyright (c) 2015, INRIA, All rights reserved.
Emeric Grange <emeric.grange@gmail.com>  
Dominique Vaufreydaz <dominique.vaufreydaz@inria.fr>  