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

void initrx();
void readrx();

#define THROTTLE_RX_TIMER FIRSTRXTIMER
#define ROLL_RX_TIMER FIRSTRXTIMER+1
#define PITCH_RX_TIMER FIRSTRXTIMER+2
#define YAW_RX_TIMER FIRSTRXTIMER+3
#define AUX1_RX_TIMER FIRSTRXTIMER+4
#define AUX2_RX_TIMER FIRSTRXTIMER+5
#if (RXNUMCHANNELS>6)
   #define AUX3_RX_TIMER FIRSTRXTIMER+6
   #define AUX4_RX_TIMER FIRSTRXTIMER+7
#endif

// convert from 1000-2000 range to fixedpointnum -1 to 1
#define FPMINMOTOROUTPUT (((fixedpointnum)MIN_MOTOR_OUTPUT-1500)<<7)
#define FPMAXMOTOROUTPUT (((fixedpointnum)MAX_MOTOR_OUTPUT-1500)<<7)
#define FPARMEDMINMOTOROUTPUT (((fixedpointnum)ARMED_MIN_MOTOR_OUTPUT-1500)<<7)
#define FPFAILSAFEMOTOROUTPUT (((fixedpointnum)FAILSAFE_MOTOR_OUTPUT-1500)<<7)
#define FPAUXMIDRANGELOW (((fixedpointnum)AUX_MID_RANGE_LOW-1500)<<7)
#define FPAUXMIDRANGEHIGH (((fixedpointnum)AUX_MID_RANGE_HIGH-1500)<<7)
#define FPTHROTTLELOW (((fixedpointnum)900-1500)<<7)
