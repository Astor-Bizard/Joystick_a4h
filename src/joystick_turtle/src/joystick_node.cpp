
// Smart Servo Framework
#include "./SmartServoFramework-master/src/DynamixelSimpleAPI.h"

// ROS dependencies
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"				// Output
#include "sensor_msgs/JoyFeedbackArray.h"	// Input
#include "Topics.h"

/* ************************************************************************** */

// Servo for right-left moves
#define ID_ANGULAR		1
#define POS_INIT_ANG	2100
#define TORQUE_ANGULAR	80

// Servo for forward-backward moves
#define ID_LINEAR		10
#define POS_INIT_LIN	1800
#define TORQUE_LINEAR	80

// Macros for the 2 servos
#define NORMAL_SPEED	512
#define HIGH_SPEED		1023

#define TORQUE_MAX		1023
#define BLOCK_DIST		512

#define STEP			32

// Macros for ROS
#define LOOP_RATE		100		// Send messages at a rate of 100 Hz
#define PUB_QUEUE_SIZE	10
#define SUB_QUEUE_SIZE	10

/* ************************************************************************** */

int limit (const int a, const int l1, const int l2){
	return std::max(std::min(a,std::max(l1,l2)),std::min(l1,l2));
}

float limit (const float a, const float l1, const float l2){
	return std::max(std::min(a,std::max(l1,l2)),std::min(l1,l2));
}

DynamixelSimpleAPI dxl;
bool recieved_feedback;

void block_servo (const int ID, const int diff){
	int torque=(ID==ID_LINEAR)?TORQUE_LINEAR:TORQUE_ANGULAR;
	dxl.setTorqueLimit(ID,std::min(TORQUE_MAX,abs(diff-BLOCK_DIST)*5+torque));
}

void manage_feedback (const sensor_msgs::JoyFeedbackArray cmd){
	recieved_feedback=true;
	int pos_angular = dxl.readCurrentPosition(ID_ANGULAR);
	int pos_linear = dxl.readCurrentPosition(ID_LINEAR);
	int load_lin=dxl.readCurrentLoad(ID_LINEAR);
	int load_ang;
	int diff_linear = abs(pos_linear-POS_INIT_LIN);
	int diff_angular = abs(pos_angular-POS_INIT_ANG);
	if (cmd.array[0].type==sensor_msgs::JoyFeedback::TYPE_BUZZER){
		dxl.setTorqueLimit(ID_LINEAR,cmd.array[0].intensity);
		if (load_lin > 1023){
			dxl.setGoalPosition(ID_LINEAR,POS_INIT_LIN-BLOCK_DIST,HIGH_SPEED);
			dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG-BLOCK_DIST);
			dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG+BLOCK_DIST);
			dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG);
		}
		else{
			dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG+BLOCK_DIST);
			dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG-BLOCK_DIST);
			dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG);
			dxl.setGoalPosition(ID_LINEAR,POS_INIT_LIN+BLOCK_DIST,HIGH_SPEED);
		}
		dxl.setGoalPosition(ID_LINEAR,POS_INIT_LIN,NORMAL_SPEED);
	}
	else{
		load_lin=dxl.readCurrentLoad(ID_LINEAR);
		load_ang=dxl.readCurrentLoad(ID_ANGULAR);
		for (int i=0;i<4;i++){
			switch (cmd.array[i].id){
				case ID_LIN_FOR:
					if (diff_linear >= BLOCK_DIST) block_servo(ID_LINEAR,diff_linear);
					else if (load_lin<=1023){
						if (cmd.array[i].type==sensor_msgs::JoyFeedback::TYPE_LED) dxl.setTorqueLimit(ID_LINEAR,TORQUE_LINEAR);
						else dxl.setTorqueLimit(ID_LINEAR,limit(int(cmd.array[i].intensity),TORQUE_MAX,TORQUE_LINEAR));
					}
					break;
				case ID_LIN_BACK:
					if (diff_linear >= BLOCK_DIST) block_servo(ID_LINEAR,diff_linear);
					else if (load_lin>1023){
						if (cmd.array[i].type==sensor_msgs::JoyFeedback::TYPE_LED) dxl.setTorqueLimit(ID_LINEAR,TORQUE_LINEAR);
						else dxl.setTorqueLimit(ID_LINEAR,limit(int(cmd.array[i].intensity),TORQUE_MAX,TORQUE_LINEAR));
					}
					break;
				case ID_ANG_LEFT:
					if (diff_angular >= BLOCK_DIST) block_servo(ID_ANGULAR,diff_angular);
					else if (load_ang<=1023){
						if (cmd.array[i].type==sensor_msgs::JoyFeedback::TYPE_LED) dxl.setTorqueLimit(ID_ANGULAR,TORQUE_ANGULAR);
						else dxl.setTorqueLimit(ID_ANGULAR,limit(int(cmd.array[i].intensity),TORQUE_MAX,TORQUE_ANGULAR));
					}
					break;
				case ID_ANG_RIGHT:
					if (diff_angular >= BLOCK_DIST) block_servo(ID_ANGULAR,diff_angular);
					else if (load_ang>1023){
						if (cmd.array[i].type==sensor_msgs::JoyFeedback::TYPE_LED) dxl.setTorqueLimit(ID_ANGULAR,TORQUE_ANGULAR);
						else dxl.setTorqueLimit(ID_ANGULAR,limit(int(cmd.array[i].intensity),TORQUE_MAX,TORQUE_ANGULAR));
					}
					break;
				default:
					std::cerr << "Some data are unreadable ! (ID : " << cmd.array[i].id << ")" << std::endl;
					break;
			}
		}
	}
}

int main(int argc, char *argv[])
{
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

    // Init servo position
    int pos_angular=POS_INIT_ANG,pos_linear=POS_INIT_LIN;
	int diff_angular=0,diff_linear=0;
	recieved_feedback=false;
    dxl.setTorqueEnabled(ID_ANGULAR,1);
    dxl.setTorqueEnabled(ID_LINEAR,1);
    dxl.setMaxTorque(ID_ANGULAR,TORQUE_MAX);
    dxl.setMaxTorque(ID_LINEAR,TORQUE_MAX);
    dxl.setTorqueLimit(ID_ANGULAR,TORQUE_ANGULAR);
    dxl.setTorqueLimit(ID_LINEAR,TORQUE_LINEAR);
    dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG,NORMAL_SPEED);
    dxl.setGoalPosition(ID_LINEAR,POS_INIT_LIN,NORMAL_SPEED);

	dxl.setSetting(ID_LINEAR,REG_P_GAIN,20,REGISTER_RAM,SERVO_MX12W);
	dxl.setSetting(ID_LINEAR,REG_I_GAIN,5,REGISTER_RAM,SERVO_MX12W);
	dxl.setSetting(ID_LINEAR,REG_D_GAIN,8,REGISTER_RAM,SERVO_MX12W);
	dxl.setSetting(ID_ANGULAR,REG_P_GAIN,20,REGISTER_RAM,SERVO_MX12W);
	dxl.setSetting(ID_ANGULAR,REG_I_GAIN,5,REGISTER_RAM,SERVO_MX12W);
	dxl.setSetting(ID_ANGULAR,REG_D_GAIN,8,REGISTER_RAM,SERVO_MX12W);

	std::cout << std::endl << "> Joystick ready." << std::endl;


	ros::init(argc, argv, "joystick_node");
  	ros::NodeHandle n;

  	ros::Publisher joy_pub = n.advertise<sensor_msgs::Joy>(std::string(JOYSTICK_TOPIC), PUB_QUEUE_SIZE);
	ros::Subscriber feedback_sub = n.subscribe(std::string(FEEDBACK_TOPIC), SUB_QUEUE_SIZE, manage_feedback);

  	ros::Rate loop_rate(LOOP_RATE);
	int i=1;
	
    // Run main loop
    while (ros::ok()){
		sensor_msgs::Joy pose;
		pose.header.frame_id="joystick_a4h/joystick";
		pos_angular = dxl.readCurrentPosition(ID_ANGULAR);
		pos_linear = dxl.readCurrentPosition(ID_LINEAR);

		diff_angular = abs(pos_angular-POS_INIT_ANG);
		diff_linear = abs(pos_linear-POS_INIT_LIN);
		if (!recieved_feedback){
			if (diff_angular < BLOCK_DIST) dxl.setTorqueLimit(ID_ANGULAR,TORQUE_ANGULAR);
			else block_servo(ID_ANGULAR,diff_angular);
			if (diff_linear < BLOCK_DIST) dxl.setTorqueLimit(ID_LINEAR,TORQUE_LINEAR);
			else block_servo(ID_LINEAR,diff_linear);
		}

		// Return axes position in [-1,1]
		if (diff_linear<STEP) pos_linear=POS_INIT_LIN;
		if (diff_angular<STEP) pos_angular=POS_INIT_ANG;
		pose.axes.push_back(float(pos_linear-POS_INIT_LIN)/float(BLOCK_DIST));
		pose.axes.push_back(float(pos_angular-POS_INIT_ANG)/float(BLOCK_DIST));

		pose.header.seq=i;
		i++;

		pose.header.stamp=ros::Time::now();
		recieved_feedback=false;
		joy_pub.publish(pose);
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "\r  \n";
    // Close device(s)
    dxl.setTorqueEnabled(ID_ANGULAR,0);
    dxl.setTorqueEnabled(ID_LINEAR,0);
    dxl.setTorqueLimit(ID_ANGULAR,1023);
    dxl.setTorqueLimit(ID_LINEAR,1023);
    dxl.disconnect();

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
