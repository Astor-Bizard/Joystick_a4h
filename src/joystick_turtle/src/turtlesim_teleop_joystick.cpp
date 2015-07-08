
// Smart Servo Framework
#include "./SmartServoFramework-master/src/DynamixelSimpleAPI.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

// C++ standard library
#include <iostream>
#include <cmath>

/* ************************************************************************** */

// Macros for the 2 servos
#define ID_ANGULAR		1
#define ID_LINEAR		10

#define POS_INIT		2047

#define STEP			24

#define NORMAL_SPEED	512
#define HIGH_SPEED		1023

#define TORQUE_MAX		1023
#define BLOCK_DIST		512

// Macros for ROS
#define LOOP_RATE		10
#define PUB_QUEUE_SIZE	1000
#define SUB_QUEUE_SIZE	1000

/* ************************************************************************** */

int block_forwards=0;
int block_backwards=0;
DynamixelSimpleAPI dxl;

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

void manage_walls (const turtlesim::Pose position){
	int fx11=0,fx0=0,fy11=0,fy0=0,bx11=0,bx0=0,by11=0,by0=0;
	float dist;
	float norm_theta;
	if (position.theta >= 0) norm_theta=position.theta; else norm_theta = position.theta + 6.28;
	if (position.x > 9){
		if (cos(norm_theta)!=0){
			dist=(11.1-position.x)/std::abs(cos(norm_theta));
			if (cos(norm_theta)>0){
				fx11=int(100.0*max(10.0-(3.0*dist),0.0));
				bx11=0;
			}
			else{
				bx11=int(100.0*max(10.0-(3.0*dist),0.0));
				fx11=0;
			}
		}
	}
	if (position.x < 2){
		if (cos(norm_theta)!=0){
			dist=(position.x+0.1)/std::abs(cos(norm_theta));
			if (cos(norm_theta)<0){
				fx0=int(100.0*max(10.0-(3.0*dist),0.0));
				bx0=0;
			}
			else{
				bx0=int(100.0*max(10.0-(3.0*dist),0.0));
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
				fy11=int(100.0*max(10.0-(3.0*dist),0.0));
				by11=0;
			}
			else{
				by11=int(100.0*max(10.0-(3.0*dist),0.0));
				fy11=0;
			}
		} 
		if (position.y < 2){
			
			dist=(position.y+0.1)/std::abs(cos(norm_theta));
			if (cos(norm_theta)<0){
				fy0=int(100.0*max(10.0-(3.0*dist),0.0));
				by0=0;
			}
			else{
				by0=int(100.0*max(10.0-(3.0*dist),0.0));
				fy0=0;
			}
		}
	}
	block_forwards=min(max(fx11,fx0,fy11,fy0),823);
	block_backwards=min(max(bx11,bx0,by11,by0),823);



	if (position.y < 3.6 && position.y > 3.4){
		dxl.setTorqueLimit(ID_LINEAR,TORQUE_MAX);
		dxl.setGoalPosition(ID_LINEAR,POS_INIT-BLOCK_DIST,HIGH_SPEED);
		dxl.setGoalPosition(ID_LINEAR,POS_INIT+BLOCK_DIST,HIGH_SPEED);
		dxl.setGoalSpeed(ID_LINEAR,NORMAL_SPEED);
	}
}

int main(int argc, char *argv[])
{
    std::cout << std::endl << "================== Joystick teleoperating module on Turtlesim ==================" << std::endl << std::endl;

    // Initialize a Dynamixel "Simple API" instance
    //DynamixelSimpleAPI dxl;

    // Initialize a serial link for the SimpleAPI
    // You can specify the serial port path directly if you know it. Ex: "/dev/ttyUSB0" for a Linux system; "//./COM1" for a Windows system.
    // Note: serial port "auto-detection" will only work if a single serial port adapter is connected to your computer, or if the fisrt one detected is the one connected to your devices.
    std::string deviceName = "/dev/ttyUSB0";
    if (dxl.connect(deviceName, 1) == 0)
    {
        std::cerr << "> Failed to open a serial link for our SimpleAPI! Exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << std::endl << std::endl << "================================== Let's go ! ==================================" << std::endl;

    std::cout << std::endl << "> Move the turtle with the joystick !" << std::endl;

    // Init servo position
    int pos_1,pos_2;
    int torque_1,torque_2;
	int diff_1,diff_2;
    dxl.setTorqueEnabled(ID_ANGULAR,1);
    dxl.setTorqueEnabled(ID_LINEAR,1);
    dxl.setMaxTorque(ID_ANGULAR,TORQUE_MAX);
    dxl.setMaxTorque(ID_LINEAR,TORQUE_MAX);
    dxl.setTorqueLimit(ID_ANGULAR,TORQUE_MAX);
    dxl.setTorqueLimit(ID_LINEAR,TORQUE_MAX);
    dxl.setGoalPosition(ID_ANGULAR,POS_INIT,NORMAL_SPEED);
    dxl.setGoalPosition(ID_LINEAR,POS_INIT,NORMAL_SPEED);

	dxl.setSetting(ID_LINEAR,REG_P_GAIN,8,REGISTER_RAM,SERVO_MX12W);
	dxl.setSetting(ID_LINEAR,REG_I_GAIN,5,REGISTER_RAM,SERVO_MX12W);
	dxl.setSetting(ID_LINEAR,REG_D_GAIN,8,REGISTER_RAM,SERVO_MX12W);

    while(dxl.isMoving(ID_ANGULAR) || dxl.isMoving(ID_LINEAR));

	

	ros::init(argc, argv, "turtlesim_teleop_joystick");
  	ros::NodeHandle n;

	std::string turtle_name="turtle1";
  	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/"+turtle_name+"/cmd_vel", PUB_QUEUE_SIZE);
	ros::Subscriber pose_sub = n.subscribe("/"+turtle_name+"/pose", SUB_QUEUE_SIZE, manage_walls);
  	ros::Rate loop_rate(LOOP_RATE);

    // Run main loop
    while (ros::ok()){
		geometry_msgs::Twist msg;
		pos_1 = dxl.readCurrentPosition(ID_ANGULAR);
		pos_2 = dxl.readCurrentPosition(ID_LINEAR);
		dxl.setGoalPosition(ID_ANGULAR,POS_INIT,NORMAL_SPEED);
		dxl.setGoalPosition(ID_LINEAR,POS_INIT,NORMAL_SPEED);

		diff_1 = abs(pos_1-POS_INIT);
		//torque_1 = diff_1*2+32;
		torque_1=150;
		if (diff_1 <= STEP){
			msg.angular.z=0.0;
		}
		else{
			msg.angular.z=float(pos_1-POS_INIT)/128.0;
		}
		dxl.setTorqueLimit(ID_ANGULAR,torque_1);



		diff_2 = abs(pos_2-POS_INIT);
		//torque_2 = diff_2*2+32;
		torque_2=200;
		dxl.setTorqueLimit(ID_LINEAR,torque_2);
		if (dxl.readCurrentLoad(ID_LINEAR)>1023){
			if (block_backwards != 0){
				dxl.setGoalSpeed(ID_LINEAR,HIGH_SPEED);
				dxl.setTorqueLimit(ID_LINEAR,torque_2+block_backwards);
				dxl.setTorqueLimit(ID_ANGULAR,TORQUE_MAX);
				dxl.setTorqueLimit(ID_LINEAR,0);
				dxl.setTorqueLimit(ID_ANGULAR,0);
				dxl.setTorqueLimit(ID_LINEAR,torque_2+block_backwards);
				dxl.setTorqueLimit(ID_ANGULAR,torque_1);
			}
			else dxl.setGoalSpeed(ID_LINEAR,NORMAL_SPEED);
		}
		else{
			if (block_forwards != 0){
				dxl.setGoalSpeed(ID_LINEAR,HIGH_SPEED);
				dxl.setTorqueLimit(ID_LINEAR,torque_2+block_forwards);
				dxl.setTorqueLimit(ID_ANGULAR,TORQUE_MAX);
				dxl.setTorqueLimit(ID_LINEAR,0);
				dxl.setTorqueLimit(ID_ANGULAR,0);
				dxl.setTorqueLimit(ID_LINEAR,torque_2+block_forwards);
				dxl.setTorqueLimit(ID_ANGULAR,torque_1);
			}
			else dxl.setGoalSpeed(ID_LINEAR,NORMAL_SPEED);
		}

		if (diff_2 <= STEP){
			msg.linear.x=0.0;
		}
		else{
			msg.linear.x=float(pos_2-POS_INIT)/128.0;
		}


		cmd_vel_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

    std::cout << "\r  \n\n=================================== EXITING ====================================" << std::endl << std::endl;
    // Close device(s)
    dxl.setTorqueEnabled(ID_ANGULAR,0);
    dxl.setTorqueEnabled(ID_LINEAR,0);
    dxl.setTorqueLimit(ID_ANGULAR,1023);
    dxl.setTorqueLimit(ID_LINEAR,1023);
    dxl.disconnect();

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
