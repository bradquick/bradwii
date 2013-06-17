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

#include "compass.h"
#include "defs.h"
#include "lib_i2c.h"
#include "lib_fp.h"
#include "lib_timers.h"
#include "main.h"

extern globalstruct global;
extern usersettingsstruct usersettings;

// note: when adding new compassas, these functions need to be included:
// void initcompass()  // initializes the compass
// void calibratecompass(); // starts a 30 second calibration procedure
// char readcompass()  // loads global.compassvector[] with the compass vector as fixedpointnum.  
//							  // returns 1 when returning a new reading, 0 if it's not time to read yet.
//                     // A unit vector good, but the length of the vector really isn't that important.  
//                     // We are more concerned with the direction of the vector.


#if (COMPASS_TYPE==NO_COMPASS)

void initcompass()
	{
	}
	
void calibratecompass()
	{
	}
	
char readcompass()
	{
	return(0);
	}

#else


#if (COMPASS_TYPE==HMC5843 || COMPASS_TYPE==HMC5883)

// ************************************************************************************************************
// I2C Compass HMC5843 & HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

unsigned long compasstimer;

void compassreadrawvalues(int *compassrawvalues)
	{
	unsigned char data[6];
	lib_i2c_readdata(MAG_ADDRESS,MAG_DATA_REGISTER,data,6);
	
#if (COMPASS_TYPE==HMC5843)
	COMPASS_ORIENTATION(compassrawvalues, ((data[0]<<8) | data[1]) ,
                     ((data[2]<<8) | data[3]) ,
                     ((data[4]<<8) | data[5]) );
#endif
#if (COMPASS_TYPE==HMC5883)  
    COMPASS_ORIENTATION(compassrawvalues, ((data[0]<<8) | data[1]) ,
                     ((data[4]<<8) | data[5]) ,
                     ((data[2]<<8) | data[3]) );
#endif
	// set the timer so we know when we can take our next reading
//	nextcompassreadtime=lib_timers_millisecondssincestart()+70;
	compasstimer=lib_timers_starttimer();
	}

void calibratecompass()
	{
	int minvalues[3]={0};
	int maxvalues[3]={0};
	int rawvalues[3];
	// find the zero offsets for the raw compass readings.  Assume that someone is flipping
	// the copter in all directions for the next 30 seconds
	
	// wait until a new reading is ready
	while (lib_timers_gettimermicroseconds(compasstimer)<70000L) {}
	
	compassreadrawvalues(rawvalues);
	
	// use the general timer to count off our 30 seconds
	unsigned long testtimer=lib_timers_starttimer();
	while (lib_timers_gettimermicroseconds(testtimer)<30000000L)
		{
		// wait until a new reading is ready
		while (lib_timers_gettimermicroseconds(compasstimer)<70000L) {}
		compassreadrawvalues(rawvalues);
		// the compass vectors that we gather should represent a sphere.  The problem is that the
		// center of the sphere may not be at 0,0,0 so we need to find out what they are
		for (int x=0;x<3;++x)
			{
			if (rawvalues[x]<minvalues[x]) minvalues[x]=rawvalues[x];
			if (rawvalues[x]>maxvalues[x]) maxvalues[x]=rawvalues[x];
			}
		}
		
	for (int x=0;x<3;++x)
		{
		usersettings.compasszerooffset[x]=(minvalues[x]+maxvalues[x])/2;
		usersettings.compasscalibrationmultiplier[x]=(1000L<<FIXEDPOINTSHIFT)/(maxvalues[x]-minvalues[x]);
		}
	}
	
void initcompass() 
	{ 
	lib_timers_delaymilli(100);

	// set gains for calibration
	
	// Other programs use the self test mode of the compass chip to try to calibrate it.  I don't think this is
	// correct.  As far as I can tell, self test mode is just for seeing if it's working.  It's not for calibration.
	lib_i2c_writereg(MAG_ADDRESS ,0x00 ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
	lib_i2c_writereg(MAG_ADDRESS ,0x01 ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
	lib_i2c_writereg(MAG_ADDRESS ,0x02 ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
	
//	nextcompassreadtime=lib_timers_millisecondssincestart()+70;
	}

char readcompass()
	{ // returns 1 if we actually read something, zero otherwise.  Sets global.compassnorthvector to a unit vector (approximately)
	if (lib_timers_gettimermicroseconds(compasstimer)>=70000L)
		{
		int compassrawvalues[3];
		compassreadrawvalues(compassrawvalues);

		// convert the raw values into a unit vector
		for (int x=0;x<3;++x)
			global.compassvector[x]=lib_fp_multiply(((fixedpointnum)(compassrawvalues[x]-usersettings.compasszerooffset[x]))<<7,usersettings.compasscalibrationmultiplier[x]);
		return(1);
		}
	else return(0);
		
	}

#endif
#endif