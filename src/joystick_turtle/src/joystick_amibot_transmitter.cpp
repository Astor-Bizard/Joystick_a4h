
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

#define DEF_BOT_NAME	"amibot"

/* ************************************************************************** */

geometry_msgs::Twist msg_vel;

void transmit_cmd (const sensor_msgs::Joy position){
	msg_vel.linear.x=float(position.axes[0])*-0.5;
	msg_vel.angular.z=float(position.axes[1])*1.5;
	if (position.header.seq % 500 == 0) std::cout << "Recieved the " << position.header.seq << "th packet sent by the joystick !\n";
}

int main(int argc, char *argv[])
{
    std::cout << std::endl << "==================== Command Transmitter Joystick -> Amibot ====================" << std::endl << std::endl;
    std::cout << std::endl << std::endl << "================================ [TRANSMITTER] =================================" << std::endl;

    std::cout << std::endl << "> Move the robot with the joystick !" << std::endl;

	ros::init(argc, argv, "joystick_amibot_transmitter");
	ros::NodeHandle n;
	ros::Publisher cmd_vel_pub;

	if (argc >= 2){
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/"+std::string(argv[1])+"/cmd_vel", PUB_QUEUE_SIZE);
	}
	else{
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/"+std::string(DEF_BOT_NAME)+"/cmd_vel", PUB_QUEUE_SIZE);
	}

	ros::Subscriber joystick_position_sub = n.subscribe(std::string(JOYSTICK_TOPIC), SUB_QUEUE_SIZE, transmit_cmd);
  	ros::Rate loop_rate(LOOP_RATE);

    // Run main loop
    while (ros::ok()){
		cmd_vel_pub.publish(msg_vel);
		ros::spinOnce();
		loop_rate.sleep();
	}

    std::cout << "\r  \n\n=================================== EXITING ====================================" << std::endl << std::endl;

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
