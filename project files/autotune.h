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


#include "lib_fp.h"


#define AUTOTUNESTARTING 1
#define AUTOTUNESTOPPING 2
#define AUTOTUNETUNING 0

#define AUTOTUNEGOINGTOWARDTARGET 0
#define AUTOTUNEGOINGAWAYFROMTARGET 1

#define AUTOTUNESETTLINGTIME (FIXEDPOINTCONSTANT(.25)<<TIMESLIVEREXTRASHIFT) // 1/4 second

#define AUTOTUNEINCREASEMULTIPLIER FIXEDPOINTCONSTANT(1.05)
#define AUTOTUNEDECREASEMULTIPLIER FIXEDPOINTCONSTANT(.95)
#define AUTOTUNEMINIMUMIVALUE (1L<<3)

#define YAWGAINMULTIPLIER FIXEDPOINTCONSTANT(2.0) // yaw p gain is simply set to a multiple of the roll p gain

void autotune(fixedpointnum *angleerror,unsigned char startingorstopping);