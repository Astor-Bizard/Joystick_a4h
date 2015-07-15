
// Smart Servo Framework
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "sensor_msgs/Joy.h"
#include "joystick_turtle/Cmd_feedback.h"

// C++ standard library
#include <iostream>
#include <cmath>

/* ************************************************************************** */

// Vibration types

#define TYPE_JERK		1
#define TYPE_TREMBLING	2

#define FORCE_MAX		1023

// Macros for ROS
#define LOOP_RATE		100
#define PUB_QUEUE_SIZE	1
#define SUB_QUEUE_SIZE	1

#define DEF_TURTLE_NAME	"turtle1"
#define JOY_SUB_TOPIC	"joystick_position"
#define FEED_PUB_TOPIC	"feedback"

/* ************************************************************************** */

joystick_turtle::Cmd_feedback msg_feedback;
geometry_msgs::Twist msg_vel;

int block_forwards=0;
int block_backwards=0;

int min (const int a, const int b){
	if (a<b) return a; else return b;
}

int max (const int a, const int b, const int c, const int d){
	if (a>b){
		if (a>c) if (a>d) return a; else return d;
		else if (c>d) return c; else return d;
	}
	else{
		if (b>c) if (b>d) return b; else return d;
		else if (c>d) return c; else return d;
	}
}

float max (const float a, const float b){
	if (a>b) return a; else return b;
}

float abs_float (const float x){
	if (x>=0) return x; else return -x;
}

void manage_walls (const turtlesim::Pose position){

	int fx11=0,fx0=0,fy11=0,fy0=0,bx11=0,bx0=0,by11=0,by0=0;
	float dist;
	float norm_theta;
	
	if (position.y < 3.6 && position.y > 3.4){
		msg_feedback.vib_forwards=true;
		msg_feedback.vib_backwards=true;
		msg_feedback.vib_type=TYPE_JERK;
		msg_feedback.vib_force_forwards=FORCE_MAX;
		msg_feedback.vib_force_backwards=FORCE_MAX;
	}
	else{
		if (position.theta >= 0) norm_theta=position.theta; else norm_theta = position.theta + 6.28;
		if (position.x > 9){
			if (cos(norm_theta)!=0){
				dist=(11.1-position.x)/std::abs(cos(norm_theta));
				if (cos(norm_theta)>0){
					fx11=int(100.0*max(10.0-(5.0*dist),0.0));
					bx11=0;
				}
				else{
					bx11=int(100.0*max(10.0-(5.0*dist),0.0));
					fx11=0;
				}
			}
		}
		if (position.x < 2){
			if (cos(norm_theta)!=0){
				dist=(position.x+0.1)/std::abs(cos(norm_theta));
				if (cos(norm_theta)<0){
					fx0=int(100.0*max(10.0-(5.0*dist),0.0));
					bx0=0;
				}
				else{
					bx0=int(100.0*max(10.0-(5.0*dist),0.0));
					fx0=0;
				}
			}
		}

		if (sin(norm_theta)!=0){
			norm_theta=norm_theta-1.57;
			if (norm_theta < 0) norm_theta = norm_theta + 6.28;
			if (position.y > 9){
				dist=(11.1-position.y)/std::abs(cos(norm_theta));
				if (cos(norm_theta)>0){
					fy11=int(100.0*max(10.0-(5.0*dist),0.0));
					by11=0;
				}
				else{
					by11=int(100.0*max(10.0-(5.0*dist),0.0));
					fy11=0;
				}
			} 
			if (position.y < 2){
			
				dist=(position.y+0.1)/std::abs(cos(norm_theta));
				if (cos(norm_theta)<0){
					fy0=int(100.0*max(10.0-(5.0*dist),0.0));
					by0=0;
				}
				else{
					by0=int(100.0*max(10.0-(5.0*dist),0.0));
					fy0=0;
				}
			}
		}
		block_forwards=min(max(fx11,fx0,fy11,fy0),823);
		block_backwards=min(max(bx11,bx0,by11,by0),823);

		msg_feedback.vib_forwards=(block_forwards!=0);
		msg_feedback.vib_backwards=(block_backwards!=0);
		msg_feedback.vib_type=TYPE_TREMBLING;
		msg_feedback.vib_force_forwards=block_forwards;
		msg_feedback.vib_force_backwards=block_backwards;
	}

}

void transmit_cmd (const sensor_msgs::Joy position){

	msg_vel.linear.x=float(position.axes[1])*-0.5;
	//msg_vel.linear.x=float(position.axes[1])*4.0;
	msg_vel.angular.z=float(position.axes[3])*1.5;
	//msg_vel.angular.z=float(position.axes[3])*4.0;
	if (position.header.seq % 500 == 0) std::cout << "Recieved " << position.header.seq << " packets from the joystick so far.\n";

}

int main(int argc, char *argv[])
{
    std::cout << std::endl << "================== Joystick teleoperating module on Turtlesim ==================" << std::endl << std::endl;
    std::cout << std::endl << std::endl << "================================== Let's go ! ==================================" << std::endl;

    std::cout << std::endl << "> Move the turtle with the joystick !" << std::endl;

	ros::init(argc, argv, "joystick_turtlesim_server");
	ros::NodeHandle n;
	ros::Publisher cmd_vel_pub;
	ros::Subscriber pose_sub;

	if (argc >= 2){
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/"+std::string(argv[1])+"/cmd_vel", PUB_QUEUE_SIZE);
		pose_sub = n.subscribe("/"+std::string(argv[1])+"/pose", SUB_QUEUE_SIZE, manage_walls);
	}
	else{
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/"+std::string(DEF_TURTLE_NAME)+"/cmd_vel", PUB_QUEUE_SIZE);
		pose_sub = n.subscribe("/"+std::string(DEF_TURTLE_NAME)+"/pose", SUB_QUEUE_SIZE, manage_walls);
	}

	ros::Publisher feedback_pub = n.advertise<joystick_turtle::Cmd_feedback>(std::string(FEED_PUB_TOPIC), PUB_QUEUE_SIZE);

	ros::Subscriber joystick_position_sub = n.subscribe(std::string(JOY_SUB_TOPIC), SUB_QUEUE_SIZE, transmit_cmd);
  	ros::Rate loop_rate(LOOP_RATE);

    // Run main loop
    while (ros::ok()){
		feedback_pub.publish(msg_feedback);
		cmd_vel_pub.publish(msg_vel);
		ros::spinOnce();
		loop_rate.sleep();
	}

    std::cout << "\r  \n\n=================================== EXITING ====================================" << std::endl << std::endl;

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
