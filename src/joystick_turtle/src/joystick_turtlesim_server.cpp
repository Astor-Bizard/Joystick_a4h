
// ROS dependencies
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"				// Input
#include "geometry_msgs/Twist.h"			// Output
#include "turtlesim/Pose.h"					// Input
#include "sensor_msgs/JoyFeedbackArray.h"	// Output
#include "Topics.h"

// C++ standard library
#include <iostream>

#define PI				3.14159

/* ************************************************************************** */

#define FORCE_MAX		1023

// Macros for ROS
#define LOOP_RATE		100
#define PUB_QUEUE_SIZE	10
#define SUB_QUEUE_SIZE	10

#define DEF_TURTLE_NAME	"turtle1"

/* ************************************************************************** */

sensor_msgs::JoyFeedbackArray msg_feedback;
geometry_msgs::Twist msg_vel;

int max4 (const int a, const int b, const int c, const int d){
	return std::max(a,std::max(b,std::max(c,d)));
}

float abs_float (const float a){if (a<0) return -a; else return a;}

void manage_walls (const turtlesim::Pose position){
	msg_feedback.array[0].id=ID_LIN_FOR;
	msg_feedback.array[1].id=ID_LIN_BACK;
	msg_feedback.array[2].id=ID_ANG_LEFT;
	msg_feedback.array[3].id=ID_ANG_RIGHT;
	int fx11=0,fx0=0,fy11=0,fy0=0,bx11=0,bx0=0,by11=0,by0=0;
	int lx11=0,lx0=0,ly11=0,ly0=0,rx11=0,rx0=0,ry11=0,ry0=0;
	float dist;
	float norm_theta,cos_theta;
	int force;
	int block_forwards=0,block_backwards=0,block_left=0,block_right=0;
	if (position.y < 3.6 && position.y > 3.4){
		msg_feedback.array[0].type=sensor_msgs::JoyFeedback::TYPE_BUZZER;
		msg_feedback.array[0].intensity=float(FORCE_MAX);
	}
	else{
		//Normalize theta into ]-Pi,Pi]
		if (position.theta > PI) norm_theta = position.theta-2.0*PI;
		else if (position.theta <= -PI) norm_theta = position.theta+2.0*PI;
		else norm_theta = position.theta;
		cos_theta=cos(norm_theta);
		if (cos_theta != 0) {
			if (position.x > 9){
				dist=(11.1-position.x)/abs_float(cos_theta);
				force=int(100.0*std::max(10.0-(5.0*dist),0.0));
				// if theta in [-86°,86°]
				if (abs_float(cos_theta)>=cos(PI/2.1)){
					if (cos_theta>0){
						fx11=force;
					}
					else{
						bx11=force;
					}

					// if theta in [-86°,-10°[ U ]10°,86°]
					if (cos_theta<cos(PI/18.0) && cos_theta>0){
						if (norm_theta>0){
							rx11=int(force/3.0);
						}
						else if (norm_theta<0){
							lx11=int(force/3.0);
						}
					}
				}
			}
			else if (position.x < 2){
				dist=(position.x+0.1)/abs_float(cos_theta);
				force=int(100.0*std::max(10.0-(5.0*dist),0.0));
				// if theta in [-86°,86°]
				if (abs_float(cos_theta)>=cos(PI/2.1)){
					if (cos_theta<0){
						fx0=force;
					}
					else{
						bx0=force;
					}

					// if theta in [-86°,-10°[ U ]10°,86°]
					if (cos_theta>-cos(PI/18.0) && cos_theta<0){
						if (norm_theta>0){
							lx0=int(force/3.0);
						}
						else if (norm_theta<0){
							rx0=int(force/3.0);
						}
					}
				}
			}
		}

		if (sin(norm_theta)!=0){
			//"Turn" theta for 90°
			norm_theta=norm_theta-PI/2.0;
			//Normalize theta into ]-Pi,Pi]
			if (norm_theta<=-PI) norm_theta = norm_theta+2.0*PI;
			cos_theta=cos(norm_theta);
			if (position.y > 9){
				dist=(11.1-position.y)/abs_float(cos_theta);
				force=int(100.0*std::max(10.0-(5.0*dist),0.0));
				// if theta in [-86°,86°]
				if (abs_float(cos_theta)>=cos(PI/2.1)){
					if (cos_theta>0){
						fy11=force;
					}
					else{
						by11=force;
					}

					// if theta in [-86°,-10°[ U ]10°,86°]
					if (cos_theta<cos(PI/18.0) && cos_theta>0){
						if (norm_theta>0){
							ry11=int(force/3.0);
						}
						else if (norm_theta<0){
							ly11=int(force/3.0);
						}
					}
				}
			}
			else if (position.y < 2){
				dist=(position.y+0.1)/abs_float(cos_theta);
				force=int(100.0*std::max(10.0-(5.0*dist),0.0));
				// if theta in [-86°,86°]
				if (abs_float(cos_theta)>=cos(PI/2.1)){
					if (cos_theta<0){
						fy0=force;
					}
					else{
						by0=force;
					}

					// if theta in [-86°,-10°[ U ]10°,86°]
					if (cos_theta>-cos(PI/18.0) && cos_theta<0){
						if (norm_theta>0){
							ly0=int(force/3.0);
						}
						else if (norm_theta<0){
							ry0=int(force/3.0);
						}
					}
				}
			}
		}
		block_forwards=std::min(max4(fx11,fx0,fy11,fy0),FORCE_MAX);
		block_backwards=std::min(max4(bx11,bx0,by11,by0),FORCE_MAX);
		block_left=std::min(max4(lx11,lx0,ly11,ly0),FORCE_MAX);
		block_right=std::min(max4(rx11,rx0,ry11,ry0),FORCE_MAX);
		if (block_forwards+block_backwards+block_left+block_right!=0){
			msg_feedback.array[0].type=sensor_msgs::JoyFeedback::TYPE_RUMBLE;
			msg_feedback.array[0].intensity=float(block_forwards);
			msg_feedback.array[1].type=sensor_msgs::JoyFeedback::TYPE_RUMBLE;
			msg_feedback.array[1].intensity=float(block_backwards);
			msg_feedback.array[2].type=sensor_msgs::JoyFeedback::TYPE_RUMBLE;
			msg_feedback.array[2].intensity=float(block_left);
			msg_feedback.array[3].type=sensor_msgs::JoyFeedback::TYPE_RUMBLE;
			msg_feedback.array[3].intensity=float(block_right);
		}
		else msg_feedback.array[0].type=sensor_msgs::JoyFeedback::TYPE_LED;
	}

}

void transmit_cmd (const sensor_msgs::Joy position){

	msg_vel.linear.x=float(position.axes[1])*4.0;
	msg_vel.angular.z=float(position.axes[3])*4.0;
	if (position.header.seq % 500 == 0) std::cout << "Recieved the " << position.header.seq << "th packet sent by the joystick !\n";

}

int main(int argc, char *argv[])
{
    std::cout << std::endl << "================== Joystick teleoperating module on Turtlesim ==================" << std::endl << std::endl;
    std::cout << std::endl << std::endl << "=================================== [SERVER] ===================================" << std::endl;

    std::cout << std::endl << "> Move the turtle with the joystick !" << std::endl;

	ros::init(argc, argv, "joystick_turtlesim_server");
	ros::NodeHandle n;
	ros::Publisher cmd_vel_pub;
	ros::Subscriber pose_sub;

	// Init array
	sensor_msgs::JoyFeedback tmp;
	msg_feedback.array.push_back(tmp);msg_feedback.array.push_back(tmp);msg_feedback.array.push_back(tmp);msg_feedback.array.push_back(tmp);

	if (argc >= 2){
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/"+std::string(argv[1])+"/cmd_vel", PUB_QUEUE_SIZE);
		pose_sub = n.subscribe("/"+std::string(argv[1])+"/pose", SUB_QUEUE_SIZE, manage_walls);
	}
	else{
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/"+std::string(DEF_TURTLE_NAME)+"/cmd_vel", PUB_QUEUE_SIZE);
		pose_sub = n.subscribe("/"+std::string(DEF_TURTLE_NAME)+"/pose", SUB_QUEUE_SIZE, manage_walls);
	}

	ros::Publisher feedback_pub = n.advertise<sensor_msgs::JoyFeedbackArray>(std::string(FEEDBACK_TOPIC), PUB_QUEUE_SIZE);

	ros::Subscriber joystick_position_sub = n.subscribe(std::string(JOYSTICK_TOPIC), SUB_QUEUE_SIZE, transmit_cmd);
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
