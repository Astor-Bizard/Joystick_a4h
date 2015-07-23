
// Smart Servo Framework
#include "./SmartServoFramework-master/src/DynamixelSimpleAPI.h"

// ROS dependencies
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"				// Output
#include "sensor_msgs/JoyFeedbackArray.h"	// Input
#include "Topics.h"

/* ************************************************************************** */

// Servo for forward-backward moves
#define ID_LINEAR		10
#define POS_INIT_LIN	1800
#define TORQUE_LINEAR	80
#define STEP_LIN		32	// Error margin to consider servo in neutral position
#define P_GAIN_LIN		20	// }
#define I_GAIN_LIN		5	// } PID parameters
#define D_GAIN_LIN		8	// }

// Servo for right-left moves
#define ID_ANGULAR		1
#define POS_INIT_ANG	2100
#define TORQUE_ANGULAR	80
#define STEP_ANG		32	// Error margin to consider servo in neutral position
#define P_GAIN_ANG		20	// }
#define I_GAIN_ANG		5	// } PID parameters
#define D_GAIN_ANG		8	// }

// Macros for the 2 servos
#define NORMAL_SPEED	512		// Usual speed for the joystick to return to initial position
#define HIGH_SPEED		1023	// Speed when buzz feedback

#define TORQUE_MAX		1023
#define BLOCK_DIST		512

// Macros for ROS
#define LOOP_RATE		100		// Send messages at a rate of 100 Hz
#define PUB_QUEUE_SIZE	10
#define SUB_QUEUE_SIZE	10

/* ************************************************************************** */


int limit (const int a, const int l1, const int l2){
	// returns a if in [l1;l2], returns the nearest limit else [INT version].
	return std::max(std::min(a,std::max(l1,l2)),std::min(l1,l2));
}

float limit (const float a, const float l1, const float l2){
	// returns a if in [l1;l2], returns the nearest limit else [FLOAT version].
	return std::max(std::min(a,std::max(l1,l2)),std::min(l1,l2));
}




// Block the servos, but not too sharply to avoid rumble
void block_servo (const int ID, const unsigned int diff, DynamixelSimpleAPI* dxl){
	int torque=(ID==ID_LINEAR)?TORQUE_LINEAR:TORQUE_ANGULAR;
	dxl->setTorqueLimit(ID,std::min(TORQUE_MAX,abs(diff-BLOCK_DIST)*5+torque));
}


// ROS callback function
void manage_feedback (const sensor_msgs::JoyFeedbackArray::ConstPtr& cmd, DynamixelSimpleAPI* dxl, const unsigned int* diff_linear, const unsigned int* diff_angular, bool* recieved_feedback){
	*recieved_feedback=true;
	const int load_lin = dxl->readCurrentLoad(ID_LINEAR);
	const int load_ang = dxl->readCurrentLoad(ID_ANGULAR);
	if (cmd->array[0].type==sensor_msgs::JoyFeedback::TYPE_BUZZER){
		// Force the joystick to move quickly
		dxl->setTorqueLimit(ID_LINEAR,cmd->array[0].intensity);
		if (load_lin > 1023){
			dxl->setGoalPosition(ID_LINEAR,POS_INIT_LIN-BLOCK_DIST,HIGH_SPEED);
			dxl->setGoalPosition(ID_ANGULAR,POS_INIT_ANG-BLOCK_DIST);
			dxl->setGoalPosition(ID_ANGULAR,POS_INIT_ANG+BLOCK_DIST);
			dxl->setGoalPosition(ID_ANGULAR,POS_INIT_ANG);
		}
		else{
			dxl->setGoalPosition(ID_ANGULAR,POS_INIT_ANG+BLOCK_DIST);
			dxl->setGoalPosition(ID_ANGULAR,POS_INIT_ANG-BLOCK_DIST);
			dxl->setGoalPosition(ID_ANGULAR,POS_INIT_ANG);
			dxl->setGoalPosition(ID_LINEAR,POS_INIT_LIN+BLOCK_DIST,HIGH_SPEED);
		}
		dxl->setGoalPosition(ID_LINEAR,POS_INIT_LIN,NORMAL_SPEED);
	}
	else{
		// Apply resistance to joystick
		for (int i=0;i<4;i++){
			switch (cmd->array[i].id){
				case ID_LIN_FOR:
					if (*diff_linear >= BLOCK_DIST) block_servo(ID_LINEAR,*diff_linear,dxl);
					else if (load_lin<=1023){	// Only apply resistance if user is trying to move it
						if (cmd->array[i].type==sensor_msgs::JoyFeedback::TYPE_LED) dxl->setTorqueLimit(ID_LINEAR,TORQUE_LINEAR);
						else dxl->setTorqueLimit(ID_LINEAR,limit(int(cmd->array[i].intensity),TORQUE_MAX,TORQUE_LINEAR));
					}
					break;
				case ID_LIN_BACK:
					if (*diff_linear >= BLOCK_DIST) block_servo(ID_LINEAR,*diff_linear,dxl);
					else if (load_lin>1023){	// Only apply resistance if user is trying to move it
						if (cmd->array[i].type==sensor_msgs::JoyFeedback::TYPE_LED) dxl->setTorqueLimit(ID_LINEAR,TORQUE_LINEAR);
						else dxl->setTorqueLimit(ID_LINEAR,limit(int(cmd->array[i].intensity),TORQUE_MAX,TORQUE_LINEAR));
					}
					break;
				case ID_ANG_LEFT:
					if (*diff_angular >= BLOCK_DIST) block_servo(ID_ANGULAR,*diff_angular,dxl);
					else if (load_ang<=1023){	// Only apply resistance if user is trying to move it
						if (cmd->array[i].type==sensor_msgs::JoyFeedback::TYPE_LED) dxl->setTorqueLimit(ID_ANGULAR,TORQUE_ANGULAR);
						else dxl->setTorqueLimit(ID_ANGULAR,limit(int(cmd->array[i].intensity),TORQUE_MAX,TORQUE_ANGULAR));
					}
					break;
				case ID_ANG_RIGHT:
					if (*diff_angular >= BLOCK_DIST) block_servo(ID_ANGULAR,*diff_angular,dxl);
					else if (load_ang>1023){	// Only apply resistance if user is trying to move it
						if (cmd->array[i].type==sensor_msgs::JoyFeedback::TYPE_LED) dxl->setTorqueLimit(ID_ANGULAR,TORQUE_ANGULAR);
						else dxl->setTorqueLimit(ID_ANGULAR,limit(int(cmd->array[i].intensity),TORQUE_MAX,TORQUE_ANGULAR));
					}
					break;
				default:
					std::cerr << "Some data are unreadable ! (ID : " << cmd->array[i].id << ")" << std::endl;
					break;
			}
		}
	}
}

// Build a message to publish containing joystick position
sensor_msgs::Joy build_joy_message (const int pos_linear, const int pos_angular, const int seq){
	sensor_msgs::Joy pose;
	pose.header.frame_id="joystick_a4h/joystick";
	pose.header.seq=seq;
	pose.header.stamp=ros::Time::now();
	// Return axes position in [-1,1]
	pose.axes.push_back(std::min(std::max(float(pos_linear-POS_INIT_LIN)/float(BLOCK_DIST),float(-1)),float(1)));
	pose.axes.push_back(std::min(std::max(float(pos_angular-POS_INIT_ANG)/float(BLOCK_DIST),float(-1)),float(1)));
	return pose;
}

int main(int argc, char *argv[])
{
	// Initialize a Dynamixel instance
	DynamixelSimpleAPI dxl;
    // Initialize a serial link for the SimpleAPI
    // You can specify the serial port path directly if you know it. Ex: "/dev/ttyUSB0" for a Linux system; "//./COM1" for a Windows system.
    // Note: serial port "auto-detection" will only work if a single serial port adapter is connected to your computer, or if the fisrt one detected is the one connected to your devices.
    std::string deviceName = "auto";
    if (dxl.connect(deviceName, 1) == 0)
    {
        std::cerr << "> Failed to open a serial link for our SimpleAPI! Exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }

    // Init servos position
    int pos_linear=POS_INIT_LIN, pos_angular=POS_INIT_ANG;
	unsigned int diff_linear=0, diff_angular=0;
	

	// This is to know if we recieve feedback messages
	bool recieved_feedback=false;


	// Set parameters for the servos
    dxl.setTorqueEnabled(ID_LINEAR,1);
    dxl.setTorqueEnabled(ID_ANGULAR,1);
    dxl.setMaxTorque(ID_LINEAR,TORQUE_MAX);
    dxl.setMaxTorque(ID_ANGULAR,TORQUE_MAX);
    dxl.setTorqueLimit(ID_LINEAR,TORQUE_LINEAR);
    dxl.setTorqueLimit(ID_ANGULAR,TORQUE_ANGULAR);
    dxl.setGoalPosition(ID_LINEAR,POS_INIT_LIN,NORMAL_SPEED);
    dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG,NORMAL_SPEED);

	// Set PID parameters
	dxl.setSetting(ID_LINEAR,REG_P_GAIN,P_GAIN_LIN,REGISTER_RAM,SERVO_MX12W);
	dxl.setSetting(ID_LINEAR,REG_I_GAIN,I_GAIN_LIN,REGISTER_RAM,SERVO_MX12W);
	dxl.setSetting(ID_LINEAR,REG_D_GAIN,D_GAIN_LIN,REGISTER_RAM,SERVO_MX12W);
	dxl.setSetting(ID_ANGULAR,REG_P_GAIN,P_GAIN_ANG,REGISTER_RAM,SERVO_MX12W);
	dxl.setSetting(ID_ANGULAR,REG_I_GAIN,I_GAIN_ANG,REGISTER_RAM,SERVO_MX12W);
	dxl.setSetting(ID_ANGULAR,REG_D_GAIN,D_GAIN_ANG,REGISTER_RAM,SERVO_MX12W);


	// ROS initialization
	ros::init(argc, argv, "joystick_node");
  	ros::NodeHandle n;

  	ros::Publisher joy_pub = n.advertise<sensor_msgs::Joy>(std::string(JOYSTICK_TOPIC), PUB_QUEUE_SIZE);
	ros::Subscriber feedback_sub = n.subscribe<sensor_msgs::JoyFeedbackArray>(std::string(FEEDBACK_TOPIC), SUB_QUEUE_SIZE, boost::bind(manage_feedback, _1, &dxl, &diff_linear, &diff_angular, &recieved_feedback));

  	ros::Rate loop_rate(LOOP_RATE);
	int seq=1;	// seq number for sending messages
	

	std::cout << std::endl << "> Joystick ready." << std::endl;

    // Run main loop
    while (ros::ok()){
		// Read joystick position
		pos_linear = dxl.readCurrentPosition(ID_LINEAR);
		pos_angular = dxl.readCurrentPosition(ID_ANGULAR);
		diff_linear = abs(pos_linear-POS_INIT_LIN);
		diff_angular = abs(pos_angular-POS_INIT_ANG);

		// If no feedback have been recieved, then we must manage here the servos
		if (!recieved_feedback){
			if (diff_linear < BLOCK_DIST) dxl.setTorqueLimit(ID_LINEAR,TORQUE_LINEAR);
			else block_servo(ID_LINEAR,diff_linear,&dxl);
			if (diff_angular < BLOCK_DIST) dxl.setTorqueLimit(ID_ANGULAR,TORQUE_ANGULAR);
			else block_servo(ID_ANGULAR,diff_angular,&dxl);
		}

		// Consider servos in neutral position with a little error margin
		if (diff_linear<STEP_LIN) pos_linear=POS_INIT_LIN;
		if (diff_angular<STEP_ANG) pos_angular=POS_INIT_ANG;

		// Build and publish Joy message
		joy_pub.publish(build_joy_message(pos_linear,pos_angular,seq));
		seq++;

		// Spin once to recieve incoming feedback message
		recieved_feedback=false;
		ros::spinOnce();
		loop_rate.sleep();
	}

	std::cout << "\r  \n";

    // Close device(s)
    dxl.setTorqueEnabled(ID_LINEAR,0);
    dxl.setTorqueEnabled(ID_ANGULAR,0);
    dxl.setTorqueLimit(ID_LINEAR,1023);
    dxl.setTorqueLimit(ID_ANGULAR,1023);
    dxl.disconnect();

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
