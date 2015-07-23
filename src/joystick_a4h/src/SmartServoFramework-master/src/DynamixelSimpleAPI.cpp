/*!
 * This file is part of SmartServoFramework.
 * Copyright (c) 2014, INRIA, All rights reserved.
 *
 * SmartServoFramework is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/lgpl-3.0.txt>.
 *
 * \file DynamixelSimpleAPI.cpp
 * \date 11/04/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "DynamixelSimpleAPI.h"
#include "ControlTables.h"
#include "ControlTablesDynamixel.h"
#include "ControlTablesHerkuleX.h"

// C++ standard libraries
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>

DynamixelSimpleAPI::DynamixelSimpleAPI(int servos)
{
    if (servos != SERVO_UNKNOWN)
    {
        std::cout << std::endl;

        if (servos >= SERVO_HERKULEX)
        {
            ackPolicy = 1;
            maxId = 253;

            protocolVersion = 1;
            servoSerie = SERVO_DRS;

            if (servos == SERVO_DRS_0402 || servos == SERVO_DRS_0602)
            {
                ct = DRS0x02_control_table;
            }
            else if (servos == SERVO_DRS_0401 || servos == SERVO_DRS_0601)
            {
                ct = DRS0x01_control_table;
            }
            else
            {
                ct = DRS0101_control_table;
            }

            std::cout << "- Using HerkuleX communication protocol" << std::endl;
        }
        else //if (servos >= SERVO_DYNAMIXEL)
        {
            ackPolicy = 2;
            maxId = 252;

            if (servos >= SERVO_PRO)
            {
                protocolVersion = 2;
                servoSerie = SERVO_PRO;
                ct = PRO_control_table;
            }
            else if (servos >= SERVO_XL)
            {
                protocolVersion = 2;
                servoSerie = SERVO_XL;
                ct = XL320_control_table;
            }
            else // SERVO AX to MX
            {
                // We set the servo serie to 'MX' which is the more capable of the Dynamixel v1 serie
                protocolVersion = 1;
                servoSerie = SERVO_MX;
                ct = MX_control_table;

                if (serialDevice == SERIAL_USB2AX)
                {
                    // The USB2AX device uses the ID 253 for itself
                    maxId = 252;
                }
                else
                {
                    maxId = 253;
                }
            }

            if (protocolVersion == 2)
            {
                std::cout << "- Using Dynamixel communication protocol version 2" << std::endl;
            }
            else
            {
                std::cout << "- Using Dynamixel communication protocol version 1" << std::endl;
            }
        }
    }
    else
    {
        std::cerr << "Warning: Unknown servo serie!" << std::endl;
    }
}

DynamixelSimpleAPI::~DynamixelSimpleAPI()
{
    disconnect();
}

int DynamixelSimpleAPI::connect(std::string &devicePath, const int baud, const int serialDevice)
{
    this->serialDevice = serialDevice;
    return serialInitialize(devicePath, baud);
}

void DynamixelSimpleAPI::disconnect()
{
    serialTerminate();
}

bool DynamixelSimpleAPI::checkId(const int id, const bool broadcast)
{
    bool status = false;

    if ((id >= 0 && id <= maxId) ||
        (id == BROADCAST_ID && broadcast == true))
    {
        status = true;
    }
    else
    {
        if (id == BROADCAST_ID && broadcast == false)
        {
            std::cerr << "Error: Broadcast ID is disabled for the current instruction." << std::endl;
        }
        else
        {
            std::cerr << "Error: ID '" << id << "' is out of [0;" << maxId << "] boundaries." << std::endl;
        }
    }

    return status;
}

std::vector <int> DynamixelSimpleAPI::servoScan(int start, int stop)
{
    // Check start/stop boundaries
    if (start < 0 || start > (maxId - 1))
        start = 0;

    if (stop < 1 || stop > maxId || stop < start)
        stop = maxId;

    std::cout << std::endl << "> Scanning for Dynamixel devices on '" << serialGetCurrentDevice() << "'... Range is [" << start << "," << stop << "]" << std::endl;

    // A vector of Dynamixel IDs found during the scan
    std::vector <int> ids;

    for (int id = start; id <= stop; id++)
    {
        PingResponse pingstats;

        // If the ping gets a response, then we have found a servo
        if (dxl_ping(id, &pingstats) == true)
        {
            setLed(id, 1, LED_GREEN);

            ids.push_back(id);

            std::cout << std::endl;
            std::cout << "[#" << id << "] DYNAMIXEL servo found!" << std::endl;
            std::cout << "[#" << id << "] model: " << pingstats.model_number << " (" << dxl_get_model_name(pingstats.model_number) << ")" << std::endl;

            // Other informations, not printed by default:
            //std::cout << "[#" << id << "] firmware: " << pingstats.firmware_version << std::endl;
            //std::cout << "[#" << id << "] position: " << dxl_read_current_position(id) << std::endl;
            //std::cout << "[#" << id << "] speed: " << dxl_read_current_speed(id) << std::endl;
            //std::cout << "[#" << id << "] torque: " << dxl_read_current_torque(id) << std::endl;
            //std::cout << "[#" << id << "] load: " << dxl_read_current_load(id) << std::endl;
            //std::cout << "[#" << id << "] baudrate: " << dxl_get_baud(id) << std::endl;

            setLed(id, 0);
        }
        else
        {
            std::cout << ".";
        }
    }

    std::cout << std::endl;
    return ids;
}

bool DynamixelSimpleAPI::ping(const int id, PingResponse *status)
{
    return dxl_ping(id, status);
}

void DynamixelSimpleAPI::action(const int id)
{
    if (checkId(id) == true)
    {
        dxl_action(id);
    }
}

void DynamixelSimpleAPI::reboot(const int id)
{
    if (checkId(id) == true)
    {
        dxl_reboot(id);
    }
}

void DynamixelSimpleAPI::reset(const int id, const int setting)
{
    if (checkId(id) == true)
    {
        dxl_reset(id, setting);
    }
}

int DynamixelSimpleAPI::readModelNumber(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_MODEL_NUMBER);

        value = dxl_read_word(id, addr);
        if (dxl_print_error() != 0)
        {
            value = -1;
        }
    }

    return value;
}

int DynamixelSimpleAPI::readFirmwareVersion(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_FIRMWARE_VERSION);

        value = dxl_read_byte(id, addr);
        if (dxl_print_error() != 0)
        {
            value = -1;
        }
    }

    return value;
}

int DynamixelSimpleAPI::changeId(const int old_id, const int new_id)
{
    int status = 0;

    // Check 'old' ID
    if (checkId(old_id, false) == true)
    {
        // Check 'new' ID // Valid IDs are in range [0:maxId]
        if ((new_id >= 0) && (new_id <= maxId))
        {
            // If the ping get a response, then we already have a servo on the new id
            dxl_ping(new_id);

            if (dxl_get_com_status() == COMM_RXSUCCESS)
            {
                std::cout << "Cannot set new ID '" << new_id << "' for this servo: already in use" << std::endl;
            }
            else
            {
                int addr = getRegisterAddr(ct, REG_ID);

                dxl_write_byte(old_id, addr, new_id);
                if (dxl_print_error() == 0)
                {
                    status = 1;
                }
            }
        }
        else
        {
            std::cerr << "Cannot set new ID '" << new_id << "' for this servo: out of range" << std::endl;
        }
    }

    return status;
}

int DynamixelSimpleAPI::changeBaudRate(const int id, const int baudnum)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // Valid baudnums are in range [0:254]
        if ((baudnum >= 0) && (baudnum <= 254))
        {
            int addr = getRegisterAddr(ct, REG_BAUD_RATE);

            dxl_write_byte(id, addr, baudnum);
            if (dxl_print_error() == 0)
            {
                status = 1;
            }
        }
        else
        {
            std::cerr << "Cannot set new baudnum '" << baudnum << "' for this servo: out of range" << std::endl;
        }
    }

    return status;
}

void DynamixelSimpleAPI::getMinMaxPositions(const int id, int &min, int &max)
{
    if (checkId(id, false) == true)
    {
        int addr_min = getRegisterAddr(ct, REG_MIN_POSITION);
        int addr_max = getRegisterAddr(ct, REG_MAX_POSITION);

        min = dxl_read_word(id, addr_min);
        if (dxl_print_error() == 0)
        {
            // Valid positions are in range [0:1023] for most servo series, and [0:4095] for high-end servo series
            if ((min < 0) || (min > 4095))
            {
                min = -1;
            }
        }
        else
        {
            min = -1;
        }

        max = dxl_read_word(id, addr_max);
        if (dxl_print_error() == 0)
        {
            // Valid positions are in range [0:1023] for most servo series, and [0:4095] for high-end servo series
            if ((max < 0) || (max > 4095))
            {
                max = -1;
            }
        }
        else
        {
            max = -1;
        }
    }
}

int DynamixelSimpleAPI::setMinMaxPositions(const int id, const int min, const int max)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // Valid positions are in range [0:1023] for most servo series, and [0:4095] for high-end servo series
        if ((min < 0) || (min > 4095) || (max < 0) || (max > 4095))
        {
            std::cerr << "Cannot set new min/max positions: '" << min << "/" << max << "' for this servo: out of range" << std::endl;
        }
        else
        {
            int addr_min = getRegisterAddr(ct, REG_MIN_POSITION);
            int addr_max = getRegisterAddr(ct, REG_MAX_POSITION);

            // Write min value
            dxl_write_word(id, addr_min, min);
            if (dxl_print_error() == 0)
            {
                status = 1;
            }

            // Write max value
            dxl_write_word(id, addr_max, max);
            if (dxl_print_error() == 0)
            {
                status = 1;
            }
        }
    }

    return status;
}

int DynamixelSimpleAPI::getMaxTorque(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_MAX_TORQUE);
        value = dxl_read_word(id, addr);

        if (dxl_print_error() == 0)
        {
            // Valid torques are in range [0:1023]
            if ((value < 0) || (value > 1023))
            {
                value = -1;
            }
        }
        else
        {
            value = -1;
        }
    }

    return value;
}

int DynamixelSimpleAPI::setMaxTorque(const int id, const int torque)
{
    int status = 0;

    if (checkId(id, false) == true)
    {
        // Valid torques are in range [0:1023]
        if ((torque >= 0) && (torque <= 1023))
        {
            int addr = getRegisterAddr(ct, REG_MAX_TORQUE);

            dxl_write_word(id, addr, torque);
            if (dxl_print_error() == 0)
            {
                status = 1;
            }
        }
    }

    return status;
}

int DynamixelSimpleAPI::getTorqueLimit(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_TORQUE_LIMIT);
        value = dxl_read_word(id, addr);

        if (dxl_print_error() == 0)
        {
            // Valid torques are in range [0:1023]
            if ((value < 0) || (value > 1023))
            {
                value = -1;
            }
        }
        else
        {
            value = -1;
        }
    }

    return value;
}

int DynamixelSimpleAPI::setTorqueLimit(const int id, const int torque)
{
    int status = 0;

    if (checkId(id, false) == true)
    {
        // Valid torques are in range [0:1023]
        if ((torque >= 0) && (torque <= 1023))
        {
            int addr = getRegisterAddr(ct, REG_TORQUE_LIMIT);

            dxl_write_word(id, addr, torque);
            if (dxl_print_error() == 0)
            {
                status = 1;
            }
        }
    }

    return status;
}

int DynamixelSimpleAPI::getTorqueEnabled(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_TORQUE_ENABLE);
        value = dxl_read_byte(id, addr);

        if (dxl_print_error() != 0)
        {
            value = -1;
        }
    }

    return value;
}

int DynamixelSimpleAPI::setTorqueEnabled(const int id, int torque)
{
    int status = 0;

    if (checkId(id) == true)
    {
        int addr = getRegisterAddr(ct, REG_TORQUE_ENABLE);

        // Valid torque are in range [0:1]
        if (torque != 0)
        {
            torque = 1;
        }

        dxl_write_byte(id, addr, torque);
        if (dxl_print_error() == 0)
        {
            status = 1;
        }
    }

    return status;
}

int DynamixelSimpleAPI::getLed(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_LED);
        value = dxl_read_byte(id, addr);

        if (dxl_print_error() != 0)
        {
            value = -1;
        }
    }

    return value;
}

int DynamixelSimpleAPI::setLed(const int id, int value, const int color)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // Normalize value
        if (value >= 1)
        {
            (servoSerie == SERVO_XL) ? value = color : value = 1;
        }
        else
        {
            value = 0;
        }

        // Execute command
        int addr = getRegisterAddr(ct, REG_LED);
        if (addr >= 0)
        {
            dxl_write_byte(id, addr, value);

            // Check for error
            if (dxl_print_error() == 0)
            {
                status = 1;
            }
        }
    }

    return status;
}

int DynamixelSimpleAPI::turn(const int id, const int velocity)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // TODO
    }

    return status;
}

int DynamixelSimpleAPI::getGoalPosition(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_GOAL_POSITION);
        value = dxl_read_word(id, addr);

        if (dxl_print_error() == 0)
        {
            // Valid positions are in range [0:1023] for most servo series, and [0:4095] for high-end servo series
            if ((value < 0) || (value > 4095))
            {
                value = -1;
            }
        }
        else
        {
            value = -1;
        }
    }

    return value;
}

int DynamixelSimpleAPI::setGoalPosition(const int id, const int position)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // Valid positions are in range [0:1023] for most servo series, and [0:4095] for high-end servo series
        if ((position >= 0) && (position <= 4095))
        {
            int addr = getRegisterAddr(ct, REG_GOAL_POSITION);

            dxl_write_word(id, addr, position);
            if (dxl_print_error() == 0)
            {
                status = 1;
            }
        }
        else
        {
            std::cerr << "Cannot set goal position: '" << position << "' for this servo: out of range" << std::endl;
        }
    }

    return status;
}

int DynamixelSimpleAPI::setGoalPosition(const int id, const int position, const int speed)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // Set goal speed, and if success proceed to set goal position
        if (setGoalSpeed(id, speed) != -1)
        {
            // Valid positions are in range [0:1023] for most servo series, and [0:4095] for high-end servo series
            if ((position >= 0) && (position <= 4095))
            {
                int addr = getRegisterAddr(ct, REG_GOAL_POSITION);

                dxl_write_word(id, addr, position);
                if (dxl_print_error() == 0)
                {
                    status = 1;
                }
            }
            else
            {
                std::cerr << "Cannot set goal position: '" << position << "' for this servo: out of range" << std::endl;
            }
        }
    }

    return status;
}
/*
int DynamixelSimpleAPI::setGoalPosition_synced(const int id1, const int position1, const int id2, const int position2)
{
    while(commLock);

    txPacket[PKT_ID] = BROADCAST_ID;
    txPacket[PKT_LENGTH] = 10;
    txPacket[PKT_INSTRUCTION] = INST_SYNC_WRITE;
    txPacket[PKT_PARAMETER] = CT_GOAL_POSITION_L; // Start address to write Data
    txPacket[PKT_PARAMETER+1] = 2; // Length of Data to write

    // 1
    txPacket[PKT_PARAMETER+2] = static_cast<unsigned char>(id1 & 0xFF);
    txPacket[PKT_PARAMETER+3] = static_cast<unsigned char>(position1 & 0xFF);
    txPacket[PKT_PARAMETER+4] = static_cast<unsigned char>((position1 >> 8) & 0xFF);

    // 2
    txPacket[PKT_PARAMETER+5] = static_cast<unsigned char>(id2 & 0xFF);
    txPacket[PKT_PARAMETER+6] = static_cast<unsigned char>(position2 & 0xFF);
    txPacket[PKT_PARAMETER+7] = static_cast<unsigned char>((position2 >> 8) & 0xFF);

    dxl_txrx_packet();

    return -1;
}
*/
int DynamixelSimpleAPI::getGoalSpeed(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_GOAL_SPEED);
        value = dxl_read_word(id, addr);

        if (dxl_print_error() == 0)
        {
            // Valid speeds are in range [0:1023] for "Join Mode", [0:2047] for "Wheel Mode"
            if ((value < 0) || (value > 2047))
            {
                value = -1;
            }
        }
        else
        {
            value = -1;
        }
    }

    return value;
}

int DynamixelSimpleAPI::setGoalSpeed(const int id, const int speed)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // Valid speeds are in range [0:1023] for "Join Mode", [0:2047] for "Wheel Mode"
        if ((speed > -1) && (speed < 2048))
        {
            int addr = getRegisterAddr(ct, REG_GOAL_SPEED);
            dxl_write_word(id, addr, speed);
            if (dxl_print_error() == 0)
            {
                status = 1;
            }
        }
        else
        {
            std::cerr << "Cannot set goal speed: '" << speed << "' for this servo: out of range" << std::endl;
        }
    }

    return status;
}

int DynamixelSimpleAPI::readCurrentPosition(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_CURRENT_POSITION);
        value = dxl_read_word(id, addr);

        if (dxl_print_error() == 0)
        {
            // Valid positions are in range [0:1023] for most servo series, and [0:4095] for high-end servo series
            if ((value < 0) || (value > 4095))
            {
                value = -1;
            }
        }
        else
        {
            value = -1;
        }
    }

    return value;
}

int DynamixelSimpleAPI::readCurrentSpeed(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_CURRENT_SPEED);
        value = dxl_read_word(id, addr);

        if (dxl_print_error() == 0)
        {
            // Valid speeds are in range [0:1023] for "Join Mode", [0:2047] for "Wheel Mode"
            if ((value < 0) || (value > 2047))
            {
                value = -1;
            }
        }
        else
        {
            value = -1;
        }
    }

    return value;
}

int DynamixelSimpleAPI::readCurrentLoad(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_CURRENT_LOAD);
        value = dxl_read_word(id, addr);

        if (dxl_print_error() == 0)
        {
            // Valid loads are in range [0:2047]
            if ((value < 0) || (value > 2047))
            {
                value = -1;
            }
        }
        else
        {
            value = -1;
        }
    }

    return value;
}

double DynamixelSimpleAPI::readCurrentVoltage(const int id)
{
    double value = 0.0;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_CURRENT_VOLTAGE);
        value = dxl_read_byte(id, addr);

        // Valid voltages are in range [9;12], but there is not much point in validating this value
        if (dxl_print_error() == 0)
        {
            value /= 10.0;
        }
        else
        {
            value = 0.0;
        }
    }

    return value;
}

double DynamixelSimpleAPI::readCurrentTemperature(const int id)
{
    double value = 0.0;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_CURRENT_TEMPERATURE);
        value = dxl_read_byte(id, addr);

        // Valid temperatures are in range [-5;+70], but there is not much point in validating this value
        if (dxl_print_error() == 0)
        {
            value /= 10.0;
        }
        else
        {
            value = 0.0;
        }
    }

    return value;
}

bool DynamixelSimpleAPI::isMoving(const int id)
{
    bool moving = false;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_MOVING);
        if (dxl_read_byte(id, addr) > 0)
        {
            if (dxl_print_error() == 0)
            {
                moving = true;
            }
        }
    }

    return moving;
}

int DynamixelSimpleAPI::getSetting(const int id, const int reg_name, int reg_type, const int device)
{
    int value = -1;

    if (checkId(id) == true)
    {
        // Device detection
        const int (*cctt)[8] = getRegisterTable(device);

        if (cctt == NULL)
        {
            // Using default control table from this SimpleAPI isntance
            cctt = ct;
        }

        if (cctt)
        {
            // Find register's informations (addr, size...)
            RegisterInfos infos;
            if (getRegisterInfos(cctt, reg_name, infos) == 1)
            {
                // Read value
                if (infos.reg_size == 1)
                {
                    value = dxl_read_byte(id, infos.reg_addr);
                }
                else if (infos.reg_size == 2)
                {
                    value = dxl_read_word(id, infos.reg_addr);
                }

                // Check value
                if (value < infos.reg_value_min && value > infos.reg_value_max)
                {
                    value = -1;
                }
            }
            else
            {
                std::cerr << "[#" << id << "] getSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER NAME ERROR]" << std::endl;
            }
        }
        else
        {
            std::cerr << "[#" << id << "] getSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [CONTROL TABLE ERROR]" << std::endl;
        }
    }
    else
    {
        std::cerr << "[#" << id << "] getSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [DEVICE ID ERROR]" << std::endl;
    }

    return value;
}

int DynamixelSimpleAPI::setSetting(const int id, const int reg_name, const int reg_value, int reg_type, const int device)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // Device detection
        const int (*cctt)[8] = getRegisterTable(device);

        if (cctt == NULL)
        {
            // Using default control table from this SimpleAPI isntance
            cctt = ct;
        }

        if (cctt)
        {
            // Find register's informations (addr, size...)
            RegisterInfos infos;
            if (getRegisterInfos(cctt, reg_name, infos) == 1)
            {
                // Check if we have permission to write into this register
                if (infos.reg_access_mode == READ_WRITE)
                {
                    // Check value
                    if (reg_value >= infos.reg_value_min && reg_value <= infos.reg_value_max)
                    {
                        // Write value
                        if (infos.reg_size == 1)
                        {
                            dxl_write_byte(id, infos.reg_addr, reg_value);
                        }
                        else if (infos.reg_size == 2)
                        {
                            dxl_write_word(id, infos.reg_addr, reg_value);
                        }

                        // Check for error
                        if (dxl_print_error() == 0)
                        {
                            status = 1;
                        }
                    }
                    else
                    {
                        std::cerr << "[#" << id << "] setSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << " to '" << reg_value
                                  << "')  [VALUE ERROR]! min/max(" << infos.reg_value_min << "/" << infos.reg_value_max << ")" << std::endl;
                    }
                }
                else
                {
                    std::cerr << "[#" << id << "] setSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER ACCESS ERROR]" << std::endl;
                }
            }
            else
            {
                std::cerr << "[#" << id << "] setSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER NAME ERROR]" << std::endl;
            }
        }
        else
        {
            std::cerr << "[#" << id << "] setSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [CONTROL TABLE ERROR]" << std::endl;
        }
    }
    else
    {
        std::cerr << "[#" << id << "] setSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [DEVICE ID ERROR]" << std::endl;
    }

    return status;
}
