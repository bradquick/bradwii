/* 
Copyright 2013 Brad Quick

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// This file contains options that can be set in the config.h file.  

#pragma once

// CONTROL_BOARD_TYPE's
#define CONTROL_BOARD_HK_MULTIWII_PRO_2 1
#define CONTROL_BOARD_HK_MULTIWII_328P 2
#define CONTROL_BOARD_HK_NANOWII 3
#define CONTROL_BOARD_HK_POCKET_QUAD 4
#define CONTROL_BOARD_SIRIUS_AIR 5
#define CONTROL_BOARD_SIRIUS_AIR_GPS 6
#define CONTROL_BOARD_SIRIUS_PARIS_V4 7
#define CONTROL_BOARD_SIRIUS_PARIS_V4_GPS 8
#define CONTROL_BOARD_WITESPY_FLIP 9
#define CONTROL_BOARD_WITESPY_MULTIWII_PRO_2 10
#define CONTROL_BOARD_WITESPY_MULTIWII_PRO_2_GPS 11

// MICROCONTROLLER_TYPE's
#define MEGA2560 0
#define MEGA328P 1
#define MEGA32U4 2

// AIRCRAFT_TYPE's (need to stay in this order for multi-wii config)
#define NOTYPE 0
#define TRI 1
#define QUADP 2
#define QUADX 3
#define BI 4
#define GIMBAL 5
#define Y6 6
#define HEX6 7
#define FLYING_WING 8
#define Y4 9
#define HEX6X 10
#define OCTOX8 11
#define OCTOFLATP 12
#define OCTOFLATX 13
#define AIRPLANE 14
#define HELI_120_CCPM 15
#define HELI_90_DEG 16
#define VTAIL4 17

// GYRO_TYPE's
#define ITG3200 1
#define MPU6050 2

// ACCELEROMETER_TYPE's
#define BMA180 1
#define MPU6050 2

// GPS_TYPE's
#define NO_GPS 0
#define SERIAL_GPS 1
#define I2C_GPS 2

// COMPASS_TYPE's
#define NO_COMPASS 0
#define HMC5883 1
#define HMC5883_VIA_MPU6050 2
#define HMC5843 3
#define MAG3110 4

// BAROMETER_TYPE's
#define NO_BAROMETER 0
#define BMP085 1
#define MS5611 2

// MULTIWII_CONFIG_SERIAL_PORTS
// These can be added (or or'ed together) to choose muliple ports
#define NOSERIALPORT 0
#define SERIALPORT0 1
#define SERIALPORT1 2
#define SERIALPORT2 4
#define SERIALPORT3 8
#define SERIALPORTUSB 16

// RX receiver types
#define RX_NORMAL 0
#define RX_DSM2_1024 1
#define RX_DSM2_2048 2
#define RX_CPPM 3

// Stick Command stick positions
#define STICK_COMMAND_ROLL_LOW (1<<0)
#define STICK_COMMAND_ROLL_HIGH (1<<1)
#define STICK_COMMAND_PITCH_LOW (1<<2)
#define STICK_COMMAND_PITCH_HIGH (1<<3)
#define STICK_COMMAND_YAW_LOW (1<<4)
#define STICK_COMMAND_YAW_HIGH (1<<5)

// general yes/no (used for MOTOR_STOP)
#define YES 1
#define NO 0
