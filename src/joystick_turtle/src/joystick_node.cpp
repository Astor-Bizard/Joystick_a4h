
// Smart Servo Framework
#include "./SmartServoFramework-master/src/DynamixelSimpleAPI.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JoyFeedbackArray.h"

/* ************************************************************************** */

#define ID_ANG_LEFT		11
#define ID_ANG_RIGHT	12
#define ID_LIN_FOR		101
#define ID_LIN_BACK		102

// Macros for the 2 servos
#define ID_ANGULAR		1
#define ID_LINEAR		10

#define POS_INIT_LIN	2047
#define POS_INIT_ANG	2047

#define NORMAL_SPEED	512
#define HIGH_SPEED		1023

#define TORQUE_ANGULAR	150
#define TORQUE_LINEAR	80

#define TORQUE_MAX		1023
#define BLOCK_DIST		512

#define STEP			32

// Macros for ROS
#define LOOP_RATE		100
#define PUB_QUEUE_SIZE	10
#define SUB_QUEUE_SIZE	10

#define JOY_PUB_TOPIC	"joystick_position"
#define FEED_SUB_TOPIC	"feedback"

/* ************************************************************************** */

int max (const int a, const int b){if (a>b) return a; else return b;}
int min (const int a, const int b){if (a<b) return a; else return b;}

DynamixelSimpleAPI dxl;

void manage_feedback (const sensor_msgs::JoyFeedbackArray cmd){
	int pos_angular = dxl.readCurrentPosition(ID_ANGULAR);
	int pos_linear = dxl.readCurrentPosition(ID_LINEAR);
	int load_lin,load_ang;
	if (cmd.array[0].type==sensor_msgs::JoyFeedback::TYPE_BUZZER){
		dxl.setTorqueLimit(ID_LINEAR,cmd.array[0].intensity);
		if (dxl.readCurrentLoad(ID_LINEAR) > 1023){
			dxl.setGoalPosition(ID_LINEAR,POS_INIT_LIN-BLOCK_DIST,HIGH_SPEED);
			dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG-BLOCK_DIST);
			dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG+BLOCK_DIST);
			dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG);
		}
		dxl.setGoalPosition(ID_LINEAR,POS_INIT_LIN+BLOCK_DIST,HIGH_SPEED);
		dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG+BLOCK_DIST);
		dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG-BLOCK_DIST);
		dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG);
		dxl.setGoalPosition(ID_LINEAR,POS_INIT_LIN-BLOCK_DIST,HIGH_SPEED);
		if (dxl.readCurrentLoad(ID_LINEAR) <= 1023){
			dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG+BLOCK_DIST);
			dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG-BLOCK_DIST);
			dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG);
			dxl.setGoalPosition(ID_LINEAR,POS_INIT_LIN+BLOCK_DIST,HIGH_SPEED);
		}
		dxl.setGoalSpeed(ID_LINEAR,NORMAL_SPEED);
	}

	else if (cmd.array[0].type==sensor_msgs::JoyFeedback::TYPE_RUMBLE){
		if (abs(pos_linear-POS_INIT_LIN) < BLOCK_DIST || abs(pos_angular-POS_INIT_ANG) < BLOCK_DIST){
			load_lin=dxl.readCurrentLoad(ID_LINEAR);
			load_ang=dxl.readCurrentLoad(ID_ANGULAR);
			for (int i=0;i<4;i++){
				switch (cmd.array[i].id){
					case ID_LIN_FOR:
						if (load_lin<=1023){
							dxl.setTorqueLimit(ID_LINEAR,min(TORQUE_LINEAR+cmd.array[i].intensity,TORQUE_MAX));
						}
						break;
					case ID_LIN_BACK:
						if (load_lin>1023){
							dxl.setTorqueLimit(ID_LINEAR,min(TORQUE_LINEAR+cmd.array[i].intensity,TORQUE_MAX));
						}
						break;
					case ID_ANG_LEFT:
						if (load_ang<=1023){
							dxl.setTorqueLimit(ID_ANGULAR,min(TORQUE_ANGULAR+cmd.array[i].intensity,TORQUE_MAX));
						}
						break;
					case ID_ANG_RIGHT:
						if (load_ang>1023){
							dxl.setTorqueLimit(ID_ANGULAR,min(TORQUE_ANGULAR+cmd.array[i].intensity,TORQUE_MAX));
						}
						break;
					default:
						std::cerr << "Some data are unreadable ! (ID : " << cmd.array[i].id << ")" << std::endl;
						break;
				}
			}	
		}
		else{
			dxl.setTorqueLimit(ID_LINEAR,TORQUE_MAX);
			dxl.setTorqueLimit(ID_ANGULAR,TORQUE_MAX);
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
    int pos_angular,pos_linear;
    int torque_angular,torque_linear;
	int diff_angular,diff_linear;
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

  	ros::Publisher joy_pub = n.advertise<sensor_msgs::Joy>(std::string(JOY_PUB_TOPIC), PUB_QUEUE_SIZE);
	ros::Subscriber feedback_sub = n.subscribe(std::string(FEED_SUB_TOPIC), SUB_QUEUE_SIZE, manage_feedback);

  	ros::Rate loop_rate(LOOP_RATE);
	int i=1;
    // Run main loop
    while (ros::ok()){
		sensor_msgs::Joy pose;
		pos_angular = dxl.readCurrentPosition(ID_ANGULAR);
		pos_linear = dxl.readCurrentPosition(ID_LINEAR);
		dxl.setGoalPosition(ID_ANGULAR,POS_INIT_ANG,NORMAL_SPEED);
		dxl.setGoalPosition(ID_LINEAR,POS_INIT_LIN,NORMAL_SPEED);

		diff_angular = abs(pos_angular-POS_INIT_ANG);
		//torque_angular = diff_angular*2+32;
		if (diff_angular < BLOCK_DIST) dxl.setTorqueLimit(ID_ANGULAR,TORQUE_ANGULAR);
		else dxl.setTorqueLimit(ID_ANGULAR,TORQUE_MAX);

		diff_linear = abs(pos_linear-POS_INIT_LIN);
		//torque_linear = diff_linear*2+32;
		if (diff_linear < BLOCK_DIST) dxl.setTorqueLimit(ID_LINEAR,TORQUE_LINEAR);
		else dxl.setTorqueLimit(ID_LINEAR,TORQUE_MAX);
		//if (abs(pos_linear-POS_INIT) < BLOCK_DIST) dxl.setTorqueLimit(ID_LINEAR,TORQUE_LINEAR);
		//else dxl.setTorqueLimit(ID_LINEAR,TORQUE_MAX);

		// Return axes position in [-1,1]
		if (diff_linear<STEP) pos_linear=POS_INIT_LIN;
		if (diff_angular<STEP) pos_angular=POS_INIT_ANG;
		pose.axes.push_back(0.0);
		pose.axes.push_back(float(pos_linear-POS_INIT_LIN)/float(BLOCK_DIST));
		pose.axes.push_back(0.0);
		pose.axes.push_back(float(pos_angular-POS_INIT_ANG)/float(BLOCK_DIST));

		pose.header.seq=i;
		i++;

		pose.header.stamp=ros::Time::now();

		pose.header.frame_id="joystick_turtle/joystick";

		joy_pub.publish(pose);
		ros::spinOnce();
		loop_rate.sleep();
	}

    // Close device(s)
    dxl.setTorqueEnabled(ID_ANGULAR,0);
    dxl.setTorqueEnabled(ID_LINEAR,0);
    dxl.setTorqueLimit(ID_ANGULAR,1023);
    dxl.setTorqueLimit(ID_LINEAR,1023);
    dxl.disconnect();

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
