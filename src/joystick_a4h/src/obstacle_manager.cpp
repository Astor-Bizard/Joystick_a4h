
// ROS dependencies
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"			// Input
#include "sensor_msgs/JoyFeedbackArray.h"	// Output
#include "Topics.h"
#include <boost/ref.hpp>

// C++ standard library
#include <iostream>

/* ************************************************************************** */

#define FORCE_MAX		1023	// Maximum feedback intensity

#define DIST_DETECT_1	0.5		// Detect obstacles in front of the robot up to 50cm
#define DIST_DETECT_2	0.35	// Detect obstacles on the sides of the robot up to 35cm

// Macros for ROS
#define LOOP_RATE		100		// Send messages at a rate of 100 Hz
#define PUB_QUEUE_SIZE	10
#define SUB_QUEUE_SIZE	10

/* ************************************************************************** */

// Returns the indice of the minimum value of tab[i;j]
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


// Calculates intensity in [0;FORCE_MAX] while dist is in [range_min;dist_detect]
float calc_intensity (const float dist, const float dist_detect, const float range_min){
	return (1.0-((dist-range_min)/(dist_detect-range_min)))*float(FORCE_MAX);
}


// ROS callback function
void manage_obstacles (const sensor_msgs::LaserScan::ConstPtr& obstacles, sensor_msgs::JoyFeedbackArray* msg_feedback){

	// Arbitrarily define IDs
	msg_feedback->array[0].id=ID_LIN_FOR;
	msg_feedback->array[1].id=ID_LIN_BACK;
	msg_feedback->array[2].id=ID_ANG_LEFT;
	msg_feedback->array[3].id=ID_ANG_RIGHT;

	// First search obstacles right in front of the robot
	float minimum=obstacles->ranges[ind_min(obstacles->ranges,170,190)];
	if (minimum<DIST_DETECT_1){
		msg_feedback->array[0].type=sensor_msgs::JoyFeedback::TYPE_RUMBLE;
		msg_feedback->array[0].intensity=calc_intensity(minimum,DIST_DETECT_1,obstacles->range_min);
	}
	else{
		// Then search obstacles on the front sides
		minimum=obstacles->ranges[ind_min(obstacles->ranges,145,215)];
		if (minimum<DIST_DETECT_2){
			msg_feedback->array[0].type=sensor_msgs::JoyFeedback::TYPE_RUMBLE;
			msg_feedback->array[0].intensity=calc_intensity(minimum,DIST_DETECT_2,obstacles->range_min);
		}
		else{
			msg_feedback->array[0].type=sensor_msgs::JoyFeedback::TYPE_LED;
		}
	}

	// Search obstacles right in the back of the robot
	minimum=std::min(obstacles->ranges[ind_min(obstacles->ranges,0,10)],obstacles->ranges[ind_min(obstacles->ranges,350,359)]);
	if (minimum<DIST_DETECT_1){
		msg_feedback->array[1].type=sensor_msgs::JoyFeedback::TYPE_RUMBLE;
		msg_feedback->array[1].intensity=calc_intensity(minimum,DIST_DETECT_1,obstacles->range_min);
	}
	else{
		// Then search obstacles on the back sides
		minimum=std::min(obstacles->ranges[ind_min(obstacles->ranges,0,35)],obstacles->ranges[ind_min(obstacles->ranges,325,359)]);
		if (minimum<DIST_DETECT_2){
			msg_feedback->array[1].type=sensor_msgs::JoyFeedback::TYPE_RUMBLE;
			msg_feedback->array[1].intensity=calc_intensity(minimum,DIST_DETECT_2,obstacles->range_min);
		}
		else{
			msg_feedback->array[1].type=sensor_msgs::JoyFeedback::TYPE_LED;
		}
	}
	// Feedback on turning servo not implemented yet ##TODO##
	msg_feedback->array[2].type=sensor_msgs::JoyFeedback::TYPE_LED;
	msg_feedback->array[2].intensity=0.0;
	msg_feedback->array[3].type=sensor_msgs::JoyFeedback::TYPE_LED;
	msg_feedback->array[3].intensity=0.0;
}

int main(int argc, char *argv[])
{
    std::cout << std::endl << "======================== Joystick teleoperating module =========================" << std::endl << std::endl;
    std::cout << std::endl << std::endl << "============================== [OBSTACLE MANAGER] ==============================" << std::endl;

	
	sensor_msgs::JoyFeedbackArray msg_feedback;
	// Init array
	sensor_msgs::JoyFeedback tmp;
	tmp.id=ID_LIN_FOR;
	tmp.type=sensor_msgs::JoyFeedback::TYPE_LED;
	tmp.intensity=0.0;
	msg_feedback.array.push_back(tmp);msg_feedback.array.push_back(tmp);msg_feedback.array.push_back(tmp);msg_feedback.array.push_back(tmp);


	// ROS initialization
	ros::init(argc, argv, "obstacle_manager");
	ros::NodeHandle n;
	ros::Subscriber scan_sub;
	ros::Publisher feedback_pub = n.advertise<sensor_msgs::JoyFeedbackArray>(std::string(FEEDBACK_TOPIC), PUB_QUEUE_SIZE);
  	ros::Rate loop_rate(LOOP_RATE);

	// Possibility to give another robot name in arg
	if (argc >= 2) scan_sub = n.subscribe<sensor_msgs::LaserScan>("/"+std::string(argv[1])+"/scan", SUB_QUEUE_SIZE, boost::bind(manage_obstacles, _1, &msg_feedback));
	else scan_sub = n.subscribe<sensor_msgs::LaserScan>("/"+std::string(DEF_BOT_NAME)+"/scan", SUB_QUEUE_SIZE, boost::bind(manage_obstacles, _1, &msg_feedback));


	std::cout << std::endl << "> Obstacle manager ready." << std::endl;

    // Run main loop
    while (ros::ok()){
		// Publish feedback message
		feedback_pub.publish(msg_feedback);
		ros::spinOnce();
		loop_rate.sleep();
	}

    std::cout << "\r  \n\n=================================== EXITING ====================================" << std::endl << std::endl;

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
