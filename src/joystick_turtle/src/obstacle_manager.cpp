
// ROS dependencies
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"			// Input
#include "sensor_msgs/JoyFeedbackArray.h"	// Output
#include "Topics.h"

// C++ standard library
#include <iostream>
#include <cmath>
#define PI				3.14159

/* ************************************************************************** */

#define FORCE_MAX		1023

// Macros for ROS
#define LOOP_RATE		100
#define PUB_QUEUE_SIZE	1
#define SUB_QUEUE_SIZE	1

#define DEF_TURTLE_NAME	"amibot"

/* ************************************************************************** */

sensor_msgs::JoyFeedbackArray msg_feedback;

/*float min (const float tab[], const int i, const int j){
	float minimum=tab[i];
	for(int k=i+1;k<=j;k++){
		if (tab[k]<minimum) minimum=tab[k];
	}
	return minimum;
}*/

void manage_obstacles (const sensor_msgs::LaserScan obstacles){
	float minimum=obstacles.ranges[170];
	int k_min=171;
	for(int k=171;k<=190;k++){
		if (obstacles.ranges[k]<minimum){
			minimum=obstacles.ranges[k];
			k_min=k;
		}
	}
	if (minimum<1){
		msg_feedback.array[0].type=sensor_msgs::JoyFeedback::TYPE_RUMBLE;
		msg_feedback.array[0].id=ID_LIN_FOR;
		msg_feedback.array[0].intensity=float(FORCE_MAX)*(1.+obstacles.range_min-minimum);
	}
	else{
		msg_feedback.array[0].type=sensor_msgs::JoyFeedback::TYPE_LED;
	}
	msg_feedback.array[1].type=sensor_msgs::JoyFeedback::TYPE_LED;
	msg_feedback.array[1].id=ID_LIN_BACK;
	msg_feedback.array[1].intensity=0.0;
	msg_feedback.array[2].type=sensor_msgs::JoyFeedback::TYPE_LED;
	msg_feedback.array[2].id=ID_ANG_LEFT;
	msg_feedback.array[2].intensity=0.0;
	msg_feedback.array[3].type=sensor_msgs::JoyFeedback::TYPE_LED;
	msg_feedback.array[3].id=ID_ANG_RIGHT;
	msg_feedback.array[3].intensity=0.0;
}

int main(int argc, char *argv[])
{
    std::cout << std::endl << "================== Joystick teleoperating module on Turtlesim ==================" << std::endl << std::endl;
    std::cout << std::endl << std::endl << "============================== [OBSTACLE MANAGER] ==============================" << std::endl;

    std::cout << std::endl << "> Move the turtle with the joystick !" << std::endl;

	ros::init(argc, argv, "obstacle_manager");
	ros::NodeHandle n;
	ros::Subscriber scan_sub;

	// Init array
	sensor_msgs::JoyFeedback tmp;
	msg_feedback.array.push_back(tmp);msg_feedback.array.push_back(tmp);msg_feedback.array.push_back(tmp);msg_feedback.array.push_back(tmp);

	if (argc >= 2){
		scan_sub = n.subscribe("/"+std::string(argv[1])+"/scan", SUB_QUEUE_SIZE, manage_obstacles);
	}
	else{
		scan_sub = n.subscribe("/"+std::string(DEF_TURTLE_NAME)+"/scan", SUB_QUEUE_SIZE, manage_obstacles);
	}

	ros::Publisher feedback_pub = n.advertise<sensor_msgs::JoyFeedbackArray>(std::string(FEEDBACK_TOPIC), PUB_QUEUE_SIZE);

  	ros::Rate loop_rate(LOOP_RATE);

    // Run main loop
    while (ros::ok()){
		feedback_pub.publish(msg_feedback);
		ros::spinOnce();
		loop_rate.sleep();
	}

    std::cout << "\r  \n\n=================================== EXITING ====================================" << std::endl << std::endl;

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
