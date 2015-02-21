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

#include "config.h"
#include "lib_digitalio.h"
#include "output.h"

// This file takes the settings from config.h and creates all of the definitions needed for the rest of the code.

// set control board dependant defines here
#if (CONTROL_BOARD_TYPE==CONTROL_BOARD_HK_MULTIWII_PRO_2)
   #define MICROCONTROLLER_TYPE MEGA2560
   #define GYRO_TYPE ITG3200 // gyro
   #define GYRO_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX] =  Y; VALUES[PITCHINDEX] = -X; VALUES[YAWINDEX] = -Z;}
   #define ACCELEROMETER_TYPE BMA180 // accelerometer
   #define ACC_ORIENTATION(VALUES,X, Y, Z)  {VALUES[ROLLINDEX]  = -X; VALUES[PITCHINDEX]  = -Y; VALUES[YAWINDEX]  =  Z;}
   #ifndef COMPASS_TYPE
      #define COMPASS_TYPE HMC5883 // compass
   #endif
   #define COMPASS_ORIENTATION(VALUES,X, Y, Z)  {VALUES[ROLLINDEX]  =  Y; VALUES[PITCHINDEX]  = -X; VALUES[YAWINDEX]  = -Z;}
   #ifndef BAROMETER_TYPE
      #define BAROMETER_TYPE BMP085 // baro
   #endif
   #if (TELEMETRYMODE!=NOTELEMTRY && !defined(TELEMETRY_SERIAL_PORT))
      #define TELEMETRY_SERIAL_PORT 3
   #endif
   #ifndef MULTIWII_CONFIG_SERIAL_PORTS
      #if (TELEMETRY_SERIAL_PORT==3)
         #define MULTIWII_CONFIG_SERIAL_PORTS SERIALPORT0
      #else
         #define MULTIWII_CONFIG_SERIAL_PORTS SERIALPORT0+SERIALPORT3
      #endif
   #endif
   #ifndef GPS_TYPE
      #define GPS_TYPE SERIAL_GPS
   #endif
   #if ((RX_TYPE==RX_DSM2_1024 || RX_TYPE==RX_DSM2_2048 || RX_TYPE==RX_SBUS) && !defined(RX_SERIAL_PORT))
      #define RX_SERIAL_PORT 1
   #endif

#elif (CONTROL_BOARD_TYPE==CONTROL_BOARD_WITESPY_MULTIWII_PRO_2 || CONTROL_BOARD_TYPE==CONTROL_BOARD_WITESPY_MULTIWII_PRO_2_GPS)
   #define MICROCONTROLLER_TYPE MEGA2560
   #define GYRO_TYPE MPU6050 // gyro
   #define GYRO_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX] =  Y; VALUES[PITCHINDEX] = -X; VALUES[YAWINDEX] = -Z;}
   #define ACCELEROMETER_TYPE MPU6050 // accelerometer
   #define ACC_ORIENTATION(VALUES,X, Y, Z)  {VALUES[ROLLINDEX]  = -X; VALUES[PITCHINDEX]  = -Y; VALUES[YAWINDEX]  =  Z;}
   #ifndef COMPASS_TYPE
      #define COMPASS_TYPE HMC5883 // compass
   #endif
   #define COMPASS_ORIENTATION(VALUES,X, Y, Z)  {VALUES[ROLLINDEX]  =  X; VALUES[PITCHINDEX]  = Y; VALUES[YAWINDEX]  = -Z;}
   #ifndef BAROMETER_TYPE
      #define BAROMETER_TYPE MS5611
   #endif
   #if (TELEMETRYMODE!=NOTELEMTRY && !defined(TELEMETRY_SERIAL_PORT))
      #define TELEMETRY_SERIAL_PORT 3
   #endif
   #ifndef MULTIWII_CONFIG_SERIAL_PORTS
      #if (TELEMETRY_SERIAL_PORT==3)
         #define MULTIWII_CONFIG_SERIAL_PORTS SERIALPORT0
      #else
         #define MULTIWII_CONFIG_SERIAL_PORTS SERIALPORT0+SERIALPORT3
      #endif
   #endif
   #ifndef GPS_TYPE
      #if (CONTROL_BOARD_TYPE==CONTROL_BOARD_WITESPY_MULTIWII_PRO_2)
         #define GPS_TYPE NO_GPS
      #else
         #define GPS_TYPE UBLOX_GPS
      #endif
   #endif
   #if ((RX_TYPE==RX_DSM2_1024 || RX_TYPE==RX_DSM2_2048 || RX_TYPE==RX_SBUS) && !defined(RX_SERIAL_PORT))
      #define RX_SERIAL_PORT 1
   #endif

#elif (CONTROL_BOARD_TYPE==CONTROL_BOARD_HK_MULTIWII_328P)
   #define MICROCONTROLLER_TYPE MEGA328P
   #define GYRO_TYPE ITG3200 // gyro
   #define GYRO_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX] =  Y; VALUES[PITCHINDEX] = -X; VALUES[YAWINDEX] = -Z;}
   #define ACCELEROMETER_TYPE BMA180 // accelerometer
   #define ACC_ORIENTATION(VALUES,X, Y, Z)  {VALUES[ROLLINDEX]  = -X; VALUES[PITCHINDEX]  = -Y; VALUES[YAWINDEX]  =  Z;}
   #ifndef COMPASS_TYPE
      #define COMPASS_TYPE HMC5883 // compass
   #endif
   #define COMPASS_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX]  =  X; VALUES[PITCHINDEX]  = Y; VALUES[YAWINDEX]  = -Z;}
   #ifndef BAROMETER_TYPE
      #define BAROMETER_TYPE BMP085 // baro
   #endif
   #ifndef MULTIWII_CONFIG_SERIAL_PORTS
      #define MULTIWII_CONFIG_SERIAL_PORTS SERIALPORT0
   #endif
   #ifndef GPS_TYPE
      #define GPS_TYPE NO_GPS
   #endif
   #if ((RX_TYPE==RX_DSM2_1024 || RX_TYPE==RX_DSM2_2048 || RX_TYPE==RX_SBUS) && !defined(RX_SERIAL_PORT))
      #define RX_SERIAL_PORT 0
   #endif

#elif (CONTROL_BOARD_TYPE==CONTROL_BOARD_HK_NANOWII)
   #define MICROCONTROLLER_TYPE MEGA32U4
   #define GYRO_TYPE MPU6050 // gyro
   #define GYRO_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX] =  -X; VALUES[PITCHINDEX] = -Y; VALUES[YAWINDEX] = -Z;}
   #define ACCELEROMETER_TYPE MPU6050 // accelerometer
   #define ACC_ORIENTATION(VALUES,X, Y, Z)  {VALUES[ROLLINDEX]  = -Y; VALUES[PITCHINDEX]  = X; VALUES[YAWINDEX]  =  Z;}
   #ifndef COMPASS_TYPE
      #define COMPASS_TYPE NO_COMPASS // compass
   #endif
   #ifndef BAROMETER_TYPE
      #define BAROMETER_TYPE NO_BAROMETER // baro
   #endif
   #ifndef MULTIWII_CONFIG_SERIAL_PORTS
      #define MULTIWII_CONFIG_SERIAL_PORTS SERIALPORTUSB
   #endif
   #ifndef GPS_TYPE
      #define GPS_TYPE NO_GPS
   #endif
   #if ((RX_TYPE==RX_DSM2_1024 || RX_TYPE==RX_DSM2_2048 || RX_TYPE==RX_SBUS) && !defined(RX_SERIAL_PORT))
      #define RX_SERIAL_PORT 1
   #endif

#elif (CONTROL_BOARD_TYPE==CONTROL_BOARD_HK_POCKET_QUAD)
   #define MICROCONTROLLER_TYPE MEGA32U4
   #define DC_MOTORS
   #define GYRO_TYPE MPU6050 // gyro
   #define GYRO_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX] =  Y; VALUES[PITCHINDEX] = -X; VALUES[YAWINDEX] = -Z;}
   #define ACCELEROMETER_TYPE MPU6050 // accelerometer
   #define ACC_ORIENTATION(VALUES,X, Y, Z)  {VALUES[ROLLINDEX]  = -X; VALUES[PITCHINDEX]  = -Y; VALUES[YAWINDEX]  =  Z;}
   #ifndef COMPASS_TYPE
      #define COMPASS_TYPE NO_COMPASS // compass
   #endif
   #define COMPASS_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX]  =  X; VALUES[PITCHINDEX]  = Y; VALUES[YAWINDEX]  = -Z;}
   #ifndef BAROMETER_TYPE
      #define BAROMETER_TYPE NO_BAROMETER // baro
   #endif
   #ifndef MULTIWII_CONFIG_SERIAL_PORTS
      #define MULTIWII_CONFIG_SERIAL_PORTS SERIALPORTUSB
   #endif
   #ifndef GPS_TYPE
      #define GPS_TYPE NO_GPS
   #endif
   #if (RX_TYPE==RX_NORMAL)
      #define RXNUMCHANNELS 5
   #elif ((RX_TYPE==RX_DSM2_1024 || RX_TYPE==RX_DSM2_2048 || RX_TYPE==RX_SBUS) && !defined(RX_SERIAL_PORT))
      #define RX_SERIAL_PORT 1
   #endif
   #ifndef ARMED_MIN_MOTOR_OUTPUT
      #define ARMED_MIN_MOTOR_OUTPUT 1000 // motors don't spin slowly when armed
   #endif
   #ifndef THROTTLE_TO_MOTOR_OFFSET
      #define THROTTLE_TO_MOTOR_OFFSET -60 // allow the motors to stop when armed
   #endif
   #ifndef MOTORS_STOP
      #define MOTORS_STOP YES // allow the motors to stop when armed if not in acro or semi acro mode
   #endif
   #ifndef GYRO_LOW_PASS_FILTER
      #define GYRO_LOW_PASS_FILTER 2
   #endif
   #ifndef GAIN_SCHEDULING_FACTOR
      #define GAIN_SCHEDULING_FACTOR 0
   #endif


#elif (CONTROL_BOARD_TYPE==CONTROL_BOARD_SIRIUS_AIR || CONTROL_BOARD_TYPE==CONTROL_BOARD_SIRIUS_AIR_GPS)
   #define MICROCONTROLLER_TYPE MEGA32U4
   #define GYRO_TYPE MPU6050 // gyro
   #define GYRO_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX] =  -Y; VALUES[PITCHINDEX] = X; VALUES[YAWINDEX] = -Z;}
   #define ACCELEROMETER_TYPE MPU6050 // accelerometer
   #define ACC_ORIENTATION(VALUES,X, Y, Z)  {VALUES[ROLLINDEX]  = X; VALUES[PITCHINDEX]  = Y; VALUES[YAWINDEX]  =  Z;}
   #ifndef MULTIWII_CONFIG_SERIAL_PORTS
      #define MULTIWII_CONFIG_SERIAL_PORTS SERIALPORTUSB
   #endif
   #define BAROMETER_TYPE BMP085 // baro
   #if (CONTROL_BOARD_TYPE==CONTROL_BOARD_SIRIUS_AIR_GPS)
      #define COMPASS_TYPE HMC5883 // compass
      #define COMPASS_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX]  =  -X; VALUES[PITCHINDEX]  = Y; VALUES[YAWINDEX]  = Z;}
      #define GPS_TYPE I2C_GPS
   #else
      #define COMPASS_TYPE NO_COMPASS // compass
      #define GPS_TYPE NO_GPS
   #endif
   #define AUX2_RX_INPUT (DIGITALPORTB | 0)
   #define PCINTERRUPT0PORTANDPIN AUX2_RX_INPUT
   
#elif (CONTROL_BOARD_TYPE==CONTROL_BOARD_SIRIUS_PARIS_V4)
   #define MICROCONTROLLER_TYPE MEGA328P
   #define GYRO_TYPE ITG3200 // gyro
   #define GYRO_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX] =  Y; VALUES[PITCHINDEX] = -X; VALUES[YAWINDEX] = -Z;}
   #define ACCELEROMETER_TYPE BMA180 // accelerometer
   #define ACC_ORIENTATION(VALUES,X, Y, Z)  {VALUES[ROLLINDEX]  = -X; VALUES[PITCHINDEX]  = -Y; VALUES[YAWINDEX]  =  Z;}
   #ifndef COMPASS_TYPE
      #define COMPASS_TYPE HMC5883 // compass
   #endif
   #define COMPASS_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX]  =  X; VALUES[PITCHINDEX]  = Y; VALUES[YAWINDEX]  = -Z;}
   #ifndef BAROMETER_TYPE
      #define BAROMETER_TYPE BMP085 // baro
   #endif
   #ifndef MULTIWII_CONFIG_SERIAL_PORTS
      #define MULTIWII_CONFIG_SERIAL_PORTS SERIALPORT0
   #endif
   #define AUX2_RX_INPUT (DIGITALPORTB | 0)
   #define PCINTERRUPT0PORTANDPIN AUX2_RX_INPUT

#elif (CONTROL_BOARD_TYPE==CONTROL_BOARD_SIRIUS_PARIS_V4_GPS)
   #define MICROCONTROLLER_TYPE MEGA328P
   #define GYRO_TYPE ITG3200 // gyro
   #define GYRO_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX] =  Y; VALUES[PITCHINDEX] = -X; VALUES[YAWINDEX] = -Z;}
   #define ACCELEROMETER_TYPE BMA180 // accelerometer
   #define ACC_ORIENTATION(VALUES,X, Y, Z)  {VALUES[ROLLINDEX]  = -X; VALUES[PITCHINDEX]  = -Y; VALUES[YAWINDEX]  =  Z;}
   #ifndef COMPASS_TYPE
      #define COMPASS_TYPE HMC5883 // compass
   #endif
   #define COMPASS_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX]  =  -X; VALUES[PITCHINDEX]  = Y; VALUES[YAWINDEX]  = Z;}
   #ifndef BAROMETER_TYPE
      #define BAROMETER_TYPE BMP085 // baro
   #endif
   #ifndef MULTIWII_CONFIG_SERIAL_PORTS
      #define MULTIWII_CONFIG_SERIAL_PORTS SERIALPORT0
   #endif
   #define AUX2_RX_INPUT (DIGITALPORTB | 0)
   #define PCINTERRUPT0PORTANDPIN AUX2_RX_INPUT

#elif (CONTROL_BOARD_TYPE==CONTROL_BOARD_WITESPY_FLIP)
   #define MICROCONTROLLER_TYPE MEGA328P
   #define GYRO_TYPE MPU6050 // gyro
   #define GYRO_ORIENTATION(VALUES,X, Y, Z) {VALUES[ROLLINDEX] =  -Y; VALUES[PITCHINDEX] = X; VALUES[YAWINDEX] = -Z;}
   #define ACCELEROMETER_TYPE MPU6050 // accelerometer
   #define ACC_ORIENTATION(VALUES,X, Y, Z)  {VALUES[ROLLINDEX]  = X; VALUES[PITCHINDEX]  = Y; VALUES[YAWINDEX]  =  Z;}
   #ifndef COMPASS_TYPE
      #define COMPASS_TYPE NO_COMPASS // compass
   #endif
   #ifndef BAROMETER_TYPE
      #define BAROMETER_TYPE NO_BAROMETER // baro
   #endif
   #ifndef MULTIWII_CONFIG_SERIAL_PORTS
      #if (RX_TYPE!=RX_DSM2_1024 && RX_TYPE!=RX_DSM2_2048 && RX_TYPE!=RX_SBUS)
         #define MULTIWII_CONFIG_SERIAL_PORTS SERIALPORT0
      #else
         #define MULTIWII_CONFIG_SERIAL_PORTS NOSERIALPORT
      #endif
   #endif
   #ifndef GPS_TYPE
      #define GPS_TYPE NO_GPS
   #endif
   #if ((RX_TYPE==RX_DSM2_1024 || RX_TYPE==RX_DSM2_2048 || RX_TYPE==RX_SBUS) && !defined(RX_SERIAL_PORT))
      #define RX_SERIAL_PORT 0
   #endif
   #define AUX2_RX_INPUT (DIGITALPORTB | 4)
   #define PCINTERRUPT4PORTANDPIN AUX2_RX_INPUT


#else
   You need to define a Control Board in config.h!
#endif

#ifndef RXNUMCHANNELS
   #if (RX_TYPE==RX_DSM2_1024 || RX_TYPE==RX_DSM2_2048 || RX_TYPE==RX_CPPM || RX_TYPE==RX_SBUS)
      #define RXNUMCHANNELS 8
   #else
      #define RXNUMCHANNELS 6
   #endif
#endif

// if no channel order was chosen, choose one
#ifndef RX_CHANNEL_ORDER
   #define RX_CHANNEL_ORDER THROTTLEINDEX,ROLLINDEX,PITCHINDEX,YAWINDEX,AUX1INDEX,AUX2INDEX,AUX3INDEX,AUX4INDEX,8,9,10,11 //For Graupner/Spektrum
#endif

// set microcontroller dependant defines here
#if (MICROCONTROLLER_TYPE==MEGA2560)
   #define THROTTLE_RX_INPUT (DIGITALPORTK | 0)
   #define PCINTERRUPT16PORTANDPIN THROTTLE_RX_INPUT
   #if (RX_TYPE!=RX_CPPM)
      #define ROLL_RX_INPUT (DIGITALPORTK | 1)
      #define PCINTERRUPT17PORTANDPIN ROLL_RX_INPUT
      #define PITCH_RX_INPUT (DIGITALPORTK | 2)
      #define PCINTERRUPT18PORTANDPIN PITCH_RX_INPUT
      #define YAW_RX_INPUT (DIGITALPORTK | 3)
      #define PCINTERRUPT19PORTANDPIN YAW_RX_INPUT
      #define AUX1_RX_INPUT (DIGITALPORTK | 4)
      #define PCINTERRUPT20PORTANDPIN AUX1_RX_INPUT
      #define AUX2_RX_INPUT (DIGITALPORTK | 5)
      #define PCINTERRUPT21PORTANDPIN AUX2_RX_INPUT
      #if (RXNUMCHANNELS>6)
         #define AUX3_RX_INPUT (DIGITALPORTK | 6)
         #define PCINTERRUPT22PORTANDPIN AUX3_RX_INPUT
         #define AUX4_RX_INPUT (DIGITALPORTK | 7)
         #define PCINTERRUPT23PORTANDPIN AUX4_RX_INPUT
      #endif
   #endif

   // LED Outputs
   #define LED1_OUTPUT (DIGITALPORTB | 7)
   #ifndef LED1_ON
      #define LED1_ON DIGITALON
   #endif

   #define MOTOR_0_CHANNEL (OUTPUT_TIMER3 | OUTPUT_CHANNELC)
   #define MOTOR_0_PIN (DIGITALPORTE | 3)
   #define MOTOR_1_CHANNEL (OUTPUT_TIMER3 | OUTPUT_CHANNELA)
   #define MOTOR_1_PIN (DIGITALPORTE | 4)
   #define MOTOR_2_CHANNEL (OUTPUT_TIMER4 | OUTPUT_CHANNELA)
   #define MOTOR_2_PIN (DIGITALPORTH | 3)
   #define MOTOR_3_CHANNEL (OUTPUT_TIMER3 | OUTPUT_CHANNELB)
   #define MOTOR_3_PIN (DIGITALPORTE | 5)

   #define MOTOR_4_CHANNEL (OUTPUT_TIMER4 | OUTPUT_CHANNELB)
   #define MOTOR_4_PIN (DIGITALPORTH | 4)
   #define MOTOR_5_CHANNEL (OUTPUT_TIMER4 | OUTPUT_CHANNELC)
   #define MOTOR_5_PIN (DIGITALPORTH | 5)

#elif (MICROCONTROLLER_TYPE==MEGA328P)
   #define THROTTLE_RX_INPUT (DIGITALPORTD | 2)
   #define PCINTERRUPT18PORTANDPIN THROTTLE_RX_INPUT
   #if (RX_TYPE!=RX_CPPM)
      #define ROLL_RX_INPUT (DIGITALPORTD | 4)
      #define PCINTERRUPT20PORTANDPIN ROLL_RX_INPUT
      #define PITCH_RX_INPUT (DIGITALPORTD | 5)
      #define PCINTERRUPT21PORTANDPIN PITCH_RX_INPUT
      #define YAW_RX_INPUT (DIGITALPORTD | 6)
      #define PCINTERRUPT22PORTANDPIN YAW_RX_INPUT
      #define AUX1_RX_INPUT (DIGITALPORTD | 7)
      #define PCINTERRUPT23PORTANDPIN AUX1_RX_INPUT
      #ifndef AUX2_RX_INPUT
         #define AUX2_RX_INPUT (DIGITALPORTB | 5)
         #define PCINTERRUPT5PORTANDPIN AUX2_RX_INPUT
      #endif
      #if (RXNUMCHANNELS>6)
         #define AUX3_RX_INPUT (DIGITALPORTC | 0)
         #define PCINTERRUPT8PORTANDPIN AUX3_RX_INPUT
         #define AUX4_RX_INPUT (DIGITALPORTC | 1)
         #define PCINTERRUPT9PORTANDPIN AUX4_RX_INPUT
      #endif
   #endif
   
   // LED Outputs
   #define LED1_OUTPUT (DIGITALPORTB | 5)
   #define LED1_ON DIGITALON

   #define MOTOR_0_CHANNEL (OUTPUT_TIMER1 | OUTPUT_CHANNELA)
   #define MOTOR_0_PIN (DIGITALPORTB | 1)
   #define MOTOR_1_CHANNEL (OUTPUT_TIMER1 | OUTPUT_CHANNELB)
   #define MOTOR_1_PIN (DIGITALPORTB | 2)
   #define MOTOR_2_CHANNEL (OUTPUT_TIMER2 | OUTPUT_CHANNELA)
   #define MOTOR_2_PIN (DIGITALPORTB | 3)
   #define MOTOR_3_CHANNEL (OUTPUT_TIMER2 | OUTPUT_CHANNELB)
   #define MOTOR_3_PIN (DIGITALPORTD | 3)

#elif (MICROCONTROLLER_TYPE==MEGA32U4)
   #define THROTTLE_RX_INPUT (DIGITALPORTE | 6)
   #define INTERRUPT6PORTANDPIN THROTTLE_RX_INPUT
   #if (RX_TYPE!=RX_CPPM)
      #define ROLL_RX_INPUT (DIGITALPORTB | 2)
      #define PCINTERRUPT2PORTANDPIN ROLL_RX_INPUT
      #define PITCH_RX_INPUT (DIGITALPORTB | 3)
      #define PCINTERRUPT3PORTANDPIN PITCH_RX_INPUT
      #define YAW_RX_INPUT (DIGITALPORTB | 1)
      #define PCINTERRUPT1PORTANDPIN YAW_RX_INPUT
      #define AUX1_RX_INPUT (DIGITALPORTB | 4)
      #define PCINTERRUPT4PORTANDPIN AUX1_RX_INPUT
      #ifndef AUX2_RX_INPUT
         #define AUX2_RX_INPUT (DIGITALPORTB | 7)
         #define PCINTERRUPT7PORTANDPIN AUX2_RX_INPUT
      #endif
      #if (RXNUMCHANNELS>6)
         #define AUX3_RX_INPUT (DIGITALPORTD | 3)
         #define INTERRUPT3PORTANDPIN AUX3_RX_INPUT
         #define AUX4_RX_INPUT (DIGITALPORTD | 2)
         #define INTERRUPT2PORTANDPIN AUX4_RX_INPUT
      #endif
   #endif
      
   // LED Outputs
   #define LED1_OUTPUT (DIGITALPORTD | 5)
   #define LED1_ON DIGITALOFF

   #define MOTOR_0_CHANNEL (OUTPUT_TIMER1 | OUTPUT_CHANNELA)
   #define MOTOR_0_PIN (DIGITALPORTB | 5)
   #define MOTOR_1_CHANNEL (OUTPUT_TIMER1 | OUTPUT_CHANNELB)
   #define MOTOR_1_PIN (DIGITALPORTB | 6)
   #define MOTOR_2_CHANNEL (OUTPUT_TIMER3 | OUTPUT_CHANNELA)
   #define MOTOR_2_PIN (DIGITALPORTC | 6)
   #define MOTOR_3_CHANNEL (OUTPUT_TIMER4 | OUTPUT_CHANNELD)
   #define MOTOR_3_PIN (DIGITALPORTD | 7)

   #define MOTOR_4_CHANNEL (OUTPUT_TIMER1 | OUTPUT_CHANNELC)
   #define MOTOR_4_PIN (DIGITALPORTB | 7)
   #define MOTOR_5_CHANNEL (OUTPUT_TIMER4 | OUTPUT_CHANNELA)
   #define MOTOR_5_PIN (DIGITALPORTC | 1)

#endif

// default to QUADX if no configuration was chosen
#ifndef AIRCRAFT_CONFIGURATION
#define AIRCRAFT_CONFIGURATION QUADX
#endif

// set aircraft type dependant defines here
#if (AIRCRAFT_CONFIGURATION==QUADX)
   #define NUMMOTORS 4
#endif

// set configuration port baud rates to defaults if none have been set
#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT0)
   #ifndef SERIAL_0_BAUD
      #define SERIAL_0_BAUD 115200
   #endif
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT1)
   #ifndef SERIAL_1_BAUD
      #define SERIAL_1_BAUD 115200
   #endif
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT2)
   #ifndef SERIAL_2_BAUD
      #define SERIAL_2_BAUD 115200
   #endif
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT3)
   #ifndef SERIAL_3_BAUD
      #define SERIAL_3_BAUD 115200
   #endif
#endif

#if (GPS_TYPE==SERIAL_GPS)
   #ifndef GPS_SERIAL_PORT
      #define GPS_SERIAL_PORT 2
   #endif
   #ifndef GPS_BAUD
      #define GPS_BAUD 115200
   #endif
#endif

#if (GPS_TYPE==UBLOX_GPS)
   #ifndef GPS_SERIAL_PORT
      #define GPS_SERIAL_PORT 2
   #endif
   #ifndef GPS_BAUD
      #define GPS_BAUD 57600
   #endif
#endif

// use default values if not set anywhere else
#ifndef ARMED_MIN_MOTOR_OUTPUT
   #define ARMED_MIN_MOTOR_OUTPUT 1067 // motors spin slowly when armed
#endif
#ifndef THROTTLE_TO_MOTOR_OFFSET
   #define THROTTLE_TO_MOTOR_OFFSET 0 // motors spin slowly when armed
#endif

// by default don't allow the motors to stop when armed if not in acro or semi acro mode
#ifndef MOTORS_STOP
   #define MOTORS_STOP NO 
#endif

// default low pass filter
#ifndef GYRO_LOW_PASS_FILTER
   #define GYRO_LOW_PASS_FILTER 0
#endif

// default gain scheduling
#ifndef GAIN_SCHEDULING_FACTOR
   #define GAIN_SCHEDULING_FACTOR 1.0
#endif

