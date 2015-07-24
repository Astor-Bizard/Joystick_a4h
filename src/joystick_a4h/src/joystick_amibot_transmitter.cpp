
// ROS dependencies
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"				// Input
#include "geometry_msgs/Twist.h"			// Output
#include "Topics.h"

// C++ standard library
#include <iostream>

/* ************************************************************************** */

// Macros for ROS
#define LOOP_RATE		100		// Send messages at a rate of 100 Hz
#define PUB_QUEUE_SIZE	10
#define SUB_QUEUE_SIZE	10

/* ************************************************************************** */


// ROS callback function
void transmit_cmd (const sensor_msgs::Joy::ConstPtr& position, geometry_msgs::Twist* msg_vel){
	msg_vel->linear.x=float(position->axes[0])*-0.5;
	msg_vel->angular.z=float(position->axes[1])*1.5;
	if (position->header.seq % 500 == 0) std::cout << "Recieved the " << position->header.seq << "th packet sent by the joystick !" << std::endl;
}

int main(int argc, char *argv[])
{
    std::cout << std::endl << "==================== Command Transmitter Joystick -> Amibot ====================" << std::endl << std::endl;
    std::cout << std::endl << std::endl << "================================ [TRANSMITTER] =================================" << std::endl;

	// ROS initialization
	ros::init(argc, argv, "joystick_amibot_transmitter");

	ros::NodeHandle n;
	ros::Publisher cmd_vel_pub;
	geometry_msgs::Twist msg_vel;

	// Possibility to give another robot name in arg
	if (argc >= 2){
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/"+std::string(argv[1])+"/cmd_vel", PUB_QUEUE_SIZE);
	}
	else{
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/"+std::string(DEF_BOT_NAME)+"/cmd_vel", PUB_QUEUE_SIZE);
	}

	ros::Subscriber joystick_position_sub = n.subscribe<sensor_msgs::Joy>(std::string(JOYSTICK_TOPIC), SUB_QUEUE_SIZE, boost::bind(transmit_cmd, _1, &msg_vel));

  	ros::Rate loop_rate(LOOP_RATE);

    std::cout << std::endl << "> Move the robot with the joystick !" << std::endl;

    // Run main loop
    while (ros::ok()){
		// Publish cmd_vel message
		cmd_vel_pub.publish(msg_vel);
		ros::spinOnce();
		loop_rate.sleep();
	}

    std::cout << "\r  \n\n=================================== EXITING ====================================" << std::endl << std::endl;

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
