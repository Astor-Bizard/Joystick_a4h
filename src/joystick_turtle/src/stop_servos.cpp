
// Smart Servo Framework
#include "./SmartServoFramework-master/src/DynamixelSimpleAPI.h"

#include <iostream>
/* ************************************************************************** */

// ID of the servo you want to use in this program:
#define ID_1     1
#define ID_2     10

/* ************************************************************************** */

int main(int argc, char *argv[])
{
    DynamixelSimpleAPI dxl;
    //"/dev/ttyUSB0"

    std::string deviceName = "auto";
    if (dxl.connect(deviceName, 1) == 0)
    {
        std::cerr << "> Failed to open a serial link for the Dynamixel servos ! Exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }

    dxl.setTorqueEnabled(ID_1,0);
    dxl.setTorqueEnabled(ID_2,0);
    dxl.setTorqueLimit(ID_1,1023);
    dxl.setTorqueLimit(ID_2,1023);
    dxl.disconnect();

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
