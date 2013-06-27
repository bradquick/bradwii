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

// this is the file that is used to configure the software.  Uncomment the appropriate lines by removing the // in front of them.
// Configuration works with a lot of defaults.  The only thing you really need to choose is the control board.  After that,
// all defaults will be chosen for that board.  If you want to use other than defaults, then uncomment the things you want
// to change.  To see what the defaults are, look in defs.h
// The options are defined in options.h

#include "options.h"

// Choose your control board:
#define CONTROL_BOARD_TYPE CONTROL_BOARD_MULTIWII_PRO_2
//#define CONTROL_BOARD_TYPE CONTROL_BOARD_MULTIWII_328P
//#define CONTROL_BOARD_TYPE CONTROL_BOARD_NANOWII

// Choose an aircraft configuration (defaults to QUADX)
//#define AIRCRAFT_CONFIGURATION QUADX

// Choose which serial ports will be used to transfer data to a configuration device.
// Multiple serial channels can be configured. (i.e. one for computer, one for bluetooth).
// Be sure to uncomment and set the baud rate for any enabled serial ports.
// note: two examples are given below, but any combination of ports can be added together.

//#define MULTIWII_CONFIG_SERIAL_PORTS NOSERIALPORT
//#define MULTIWII_CONFIG_SERIAL_PORTS SERIALPORT1
//#define MULTIWII_CONFIG_SERIAL_PORTS SERIALPORT1+SERIALPORT3

//#define SERIAL_0_BAUD 115200
//#define SERIAL_1_BAUD 9600
//#define SERIAL_2_BAUD 9600
//#define SERIAL_3_BAUD 115200

// Choose whether to include code for a GPS and set parameters for the GPS, otherwise it will default o what the control board come with
//#define GPS_TYPE NO_GPS // select if no GPS is going to be used
//#define GPS_TYPE SERIAL_GPS   // select if a serial GPS (NMEA) is going to be used
//#define GPS_SERIAL_PORT 2
//#define GPS_BAUD 115200

// Choose a multiplier for high rotation rates when in acro mode
#define HIGH_RATES_MULTILIER 2.0

// Choose maximum tilt angles when in level mode
#define LEVEL_MODE_MAX_TILT 45 // 45 degrees
#define LEVEL_MODE_MAX_TILT_HIGH_ANGLE 80 // 80 degrees when high angle checkbox active

// Choose maximum tilt angles owhile navigating. This will determine how fast it moves from point to point.
#define NAVIGATION_MAX_TILT 15 //15 degrees

// Choose output ranges (in microseconds)
#define MIN_MOTOR_OUTPUT 1000
#define MAX_MOTOR_OUTPUT 2000
#define ARMED_MIN_MOTOR_OUTPUT 1067 // motors spin slowly when armed
#define FAILSAFE_MOTOR_OUTPUT 1200 // throttle setting for bringing the aircraft down at a safe speed

// Divide the Aux inputs into low, medium, and high using the following divisions
#define AUX_MID_RANGE_LOW 1300
#define AUX_MID_RANGE_HIGH 1700

// un-comment if you don't want to include code for a compass, otherwise it will default to what the control board has on it
//#define COMPASS_TYPE NO_COMPASS

// un-comment if you don't want to include code for a barometer, otherwise it will default to what the control board has on it
//#define BAROMETER_TYPE NO_BAROMETER

// Get your magnetic decliniation from here : http://magnetic-declination.com/
#define MAG_DECLINIATION_DEGREES  -13.4 // for Hyde Park, NY

// ESCs calibration
// To calibrate all ESCs connected to the aircraft at the same time (useful to avoid unplugging/re-plugging each ESC)
// Warning: this creates a special version of code
//       You cannot fly with this special version. It is only to be used for calibrating ESCs
// #define ESC_CALIB_CANNOT_FLY  // uncomment to activate
 #define ESC_CALIB_LOW  MIN_MOTOR_OUTPUT
 #define ESC_CALIB_HIGH MAX_MOTOR_OUTPUT

// Choose the type of r/c reciever that will be used
//#define RX_TYPE RX_NORMAL
//#define RX_TYPE RX_DSM2_1024
#define RX_TYPE RX_DSM2_2048
//#define RX_DSM2_SERIAL_PORT 1

// uncomment to set the number of RX channels, otherwise it will default to what the control board/receiver can handle
//#define RXNUMCHANNELS 8

// un-comment if you don't want to include autotune code
//#define NO_AUTOTUNE
// To adjust how agressive the tuning is, adjust the AUTOTUNEMAXOSCILLATION value.  A larger
// value will result in more agressive tuning. A lower value will result in softer tuning.
// It will rock back and forth between -AUTOTUNE_TARGET_ANGLE and AUTOTUNE_TARGET_ANGLE degrees
#define AUTOTUNE_MAX_OSCILLATION 4.0
#define AUTOTUNE_TARGET_ANGLE 20.0 

// Gyro low pass filter.
// If your aircraft jumps around randomly, or it drifts from level over time when in level mode,
// then you are probably getting vibration feeding from your motors to your gyros.
// The first step is to isolate the control board from the frame of the aircraft (google Sorbothane).  If all else fails,
// increase the GYRO_LOW_PASS_FILTER from 0 through 10.  The lowest value that works correctly is the one you should use.
#define GYRO_LOW_PASS_FILTER 0

#define UNCRAHSABLE_MAX_ALTITUDE_OFFSET 30.0 // 30 meters above where uncrashability was enabled
#define UNCRAHSABLE_RADIUS 50.0 // 50 meter radius
