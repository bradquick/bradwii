/* 
Copyright 2013 Brad Quick

Some of this code is based on Multiwii code by Alexandre Dubus (www.multiwii.com)

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

#include "lib_fp.h"
#include "rx.h"
#include "defs.h"
#include "checkboxes.h"
#include "vectors.h"

#define ROLLINDEX 0
#define PITCHINDEX 1
#define YAWINDEX 2
#define THROTTLEINDEX 3
#define AUX1INDEX 4
#define AUX2INDEX 5
#define AUX3INDEX 6
#define AUX4INDEX 7

#define ALTITUDEINDEX 3 // pid gain index
#define NAVIGATIONINDEX 6 // pid gian index

#define XINDEX 0
#define YINDEX 1
#define ZINDEX 2

#define NUMPIDITEMS 10

#define NAVIGATIONMODEOFF 0
#define NAVIGATIONMODEPOSITIONHOLD 1
#define NAVIGATIONMODERETURNTOHOME 2


// put all of the global variables into one structure to make them easy to find
typedef struct
   {
   unsigned char usersettingsfromeeprom;            // set to 1 if user settings were read from eeprom
   fixedpointnum barorawaltitude;                  // Current altitude read from barometer, in meters (approximately)
   fixedpointnum debugvalue[4];                     // for display in the multiwii config program. Use for debugging.
   fixedpointnum timesliver;                        // The time in seconds (shifted TIMESLIVEREXTRASHIFT) since the last iteration of the main loop
   fixedpointnum gyrorate[3];                        // Corrected gyro rates in degrees per second
   fixedpointnum acc_g_vector[3];                  // Corrected accelerometer vector, in G's
   fixedpointnum altitude;                           // A filtered version of the baromemter's altitude
   fixedpointnum altitudevelocity;                  // The rate of change of the altitude
   fixedpointnum estimateddownvector[3];            // A unit vector (approximately) poining in the direction we think down is relative to the aircraft
   fixedpointnum estimatedwestvector[3];            // A unit vector (approximately) poining in the direction we think west is relative to the aircraft
   fixedpointnum currentestimatedeulerattitude[3]; // Euler Angles in degrees of how much we think the aircraft is Rolled, Pitched, and Yawed (from North)
   fixedpointnum rxvalues[RXNUMCHANNELS];            // The values of the RX inputs, ranging from -1.0 to 1.0
   fixedpointnum compassvector[3];                  // A unit vector (approximately) poining in the direction our 3d compass is pointing
   fixedpointnum heading_when_armed;                  // the heading we were pointing when arming was established
   fixedpointnum altitude_when_armed;                  // The altitude when arming established
   unsigned int motoroutputvalue[NUMMOTORS];         // Output values to send to our motors, from 1000 to 2000
   unsigned long activecheckboxitems;               // Bits for each checkbox item to show which are currently active
   unsigned long previousactivecheckboxitems;      // The previous state of these bits so we can tell when they turn on and off
   unsigned char armed;                              // A flag indicating that the aircraft is armed
   unsigned char gps_num_satelites;                  // How many satelites do we currently see?
   unsigned char gps_fix;                           // set to 1 if we have a gps fix
   fixedpointnum gps_home_latitude;                  // The latitude when the aircraft was armed
   fixedpointnum gps_home_longitude;               // The longitude when the aircraft was armed
   fixedpointnum gps_current_latitude;               // The current GPS latitude
   fixedpointnum gps_current_longitude;            // The current GPS longitude
   fixedpointnum gps_current_altitude;               // The current GPS altitude
   fixedpointnum gps_current_speed;                  // The current GPS speed
   fixedpointnum navigation_distance;               // The distance to to the navigation destination in meters (I think)
   fixedpointnum navigation_bearing;               // The bearing from the last waypoint to the next one
   unsigned char navigationmode;                     // See navigation.h
   unsigned char stable;                           // Set to 1 when our gravity vector is close to unit length
   unsigned long failsafetimer;                    // Timer for determining if we lose radio contact
   } globalstruct;

// put all of the user adjustable settings in one structure to make it easy to read and write to eeprom.
// We can add to the structure, but we shouldn't re-arrange the items to insure backward compatibility.
typedef struct
   {
   fixedpointnum maxyawrate;                        // maximum yaw rate (by pilot input) in degrees/sec
   fixedpointnum pid_pgain[NUMPIDITEMS];            // The various PID p gains
   fixedpointnum pid_igain[NUMPIDITEMS];            // The various PID i gains
   fixedpointnum pid_dgain[NUMPIDITEMS];            // The various PID d gains
   unsigned int checkboxconfiguration[NUMPOSSIBLECHECKBOXES]; // Bits that describe how the checkboxes are configured
   fixedpointnum gyrocalibration[3];               // Offsets used to calibrate the gyro
   fixedpointnum acccalibration[3];                  // Offsets used to calibrate the accelerometer
   fixedpointnum compasscalibrationmultiplier[3];   // Multipliers used to calibrate the compass
   int compasszerooffset[3];                        // Offsets used to calibrate the compass
   fixedpointnum maxpitchandrollrate;               // maximum pitch and roll rate (by pilot input) in degrees/sec
   } usersettingsstruct;

void defaultusersettings();
void calculatetimesliver();