
// ROS dependencies
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"			// Input
#include "sensor_msgs/JoyFeedbackArray.h"	// Output
#include "Topics.h"

// C++ standard library
#include <iostream>
#define PI				3.14159

/* ************************************************************************** */

#define FORCE_MAX		1023	// Maximum feedback intensity

#define DIST_DETECT_1	0.5		// Detect obstacles in front of the robot up to 50cm
#define DIST_DETECT_2	0.35	// Detect obstacles on the sides of the robot up to 35cm

// Macros for ROS
#define LOOP_RATE		100		// Send messages at a rate of 100 Hz
#define PUB_QUEUE_SIZE	10
#define SUB_QUEUE_SIZE	10

#define DEF_TURTLE_NAME	"amibot"

/* ************************************************************************** */

sensor_msgs::JoyFeedbackArray msg_feedback;

float ind_min (const std::vector<float> tab, const int i, const int j){
	float minimum=tab[std::min(i,j)];
	int k_min=std::min(i,j);
	for(int k=std::min(i,j)+1;k<=std::max(i,j);k++){
		if (tab[k]<minimum){
			minimum=tab[k];
			k_min=k;
		}
	}
	return k_min;
}

void manage_obstacles (const sensor_msgs::LaserScan obstacles){
	msg_feedback.array[0].id=ID_LIN_FOR;
	msg_feedback.array[1].id=ID_LIN_BACK;
	msg_feedback.array[2].id=ID_ANG_LEFT;
	msg_feedback.array[3].id=ID_ANG_RIGHT;
	float minimum=obstacles.ranges[ind_min(obstacles.ranges,170,190)];
	if (minimum<DIST_DETECT_1){
		msg_feedback.array[0].type=sensor_msgs::JoyFeedback::TYPE_RUMBLE;
		msg_feedback.array[0].intensity=(1.0-((minimum-obstacles.range_min)/(DIST_DETECT_1-obstacles.range_min)))*float(FORCE_MAX);
	}
	else{
		minimum=obstacles.ranges[ind_min(obstacles.ranges,145,215)];
		if (minimum<DIST_DETECT_2){
			msg_feedback.array[0].type=sensor_msgs::JoyFeedback::TYPE_RUMBLE;
			msg_feedback.array[0].intensity=(1.0-((minimum-obstacles.range_min)/(DIST_DETECT_2-obstacles.range_min)))*float(FORCE_MAX);
		}
		else{
			msg_feedback.array[0].type=sensor_msgs::JoyFeedback::TYPE_LED;
		}
	}
	minimum=std::min(obstacles.ranges[ind_min(obstacles.ranges,0,10)],obstacles.ranges[ind_min(obstacles.ranges,350,359)]);
	if (minimum<DIST_DETECT_1){
		msg_feedback.array[1].type=sensor_msgs::JoyFeedback::TYPE_RUMBLE;
		msg_feedback.array[1].intensity=(1.0-((minimum-obstacles.range_min)/(DIST_DETECT_1-obstacles.range_min)))*float(FORCE_MAX);
	}
	else{
		minimum=std::min(obstacles.ranges[ind_min(obstacles.ranges,0,35)],obstacles.ranges[ind_min(obstacles.ranges,325,359)]);
		if (minimum<DIST_DETECT_2){
			msg_feedback.array[1].type=sensor_msgs::JoyFeedback::TYPE_RUMBLE;
			msg_feedback.array[1].intensity=(1.0-((minimum-obstacles.range_min)/(DIST_DETECT_2-obstacles.range_min)))*float(FORCE_MAX);
		}
		else{
			msg_feedback.array[1].type=sensor_msgs::JoyFeedback::TYPE_LED;
		}
	}
	msg_feedback.array[2].type=sensor_msgs::JoyFeedback::TYPE_LED;
	msg_feedback.array[2].intensity=0.0;
	msg_feedback.array[3].type=sensor_msgs::JoyFeedback::TYPE_LED;
	msg_feedback.array[3].intensity=0.0;
}

int main(int argc, char *argv[])
{
    std::cout << std::endl << "======================== Joystick teleoperating module =========================" << std::endl << std::endl;
    std::cout << std::endl << std::endl << "============================== [OBSTACLE MANAGER] ==============================" << std::endl;

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
