/*
 *  Created by Brad on 2/23/13.
 */

#include <avr/io.h>

// library headers
#include "lib_timers.h"
#include "lib_fp.h" 

// project file headers
#include "main.h"
#include "gyro.h" 
#include "accelerometer.h"
#include "baro.h"
#include "imu.h"
#include "compass.h"

extern globalstruct global;
extern usersettingsstruct usersettings;

attitudestruct estimatedattitude;
//fixedpointnum estimated_g_vector[3]={0,0,FIXEDPOINTONE}; // start pointing down
fixedpointnum estimated_compass_vector[3]={FIXEDPOINTONE,0,0}; // start pointing north

#define MAXACCMAGNITUDESQUARED 79299L // equivilent to magnitude of 1.1 don't use acc to update attitude if under too many G's
#define MINACCMAGNITUDESQUARED 53084L // equivilent to magnitude of .9

#define FIXEDPOINTPIOVER180 1144L // pi/180 for converting degrees to radians

// The following two rates can be defined in config.h to adjust imu performance.
// Ideally, we want to run mostly on the gyro since the gyro gives us instant feedback, but the gyro
// is integrated over time, which means that error continually accumulates and you will get drift.  The
// accelerometer is averaged in using a complimentary filter to gently remind us of which way gravity is
// pulling.  The acc value can be unreliable in the short term due to acceleration of the aircraft, but
// it works well over the long run.
// ONE_OVER_ACC_COMPLIMENTARY_FILTER_TIME_PERIOD adjusts how much the acc affects our positioning.
// GYRORATEMULTIPLIER is a multiplier for the gyro rates to compensate for errors in the gyro rate.

// To adjust these, start by cutting the GYRORATEMULTIPLIER in half.  This will make it think it's
// only rotating half as fast as it really is.
// Use the config program and look at the graphical representation of roll.  Roll the aircraft 90
// degrees quickly and you will see that it only rolls 45 degrees on the screen.  Hold it there and
// you will see it creep slowly up to 90 degrees.  The slow creep is caused by the acc complimentary
// filter.  By adjusting the ONE_OVER_ACC_COMPLIMENTARY_FILTER_TIME_PERIOD, you can make it creep
// more quickly or more slowly.  We want as slow a creep as necessary.
// To minimize how fast we need to creep, adjust the GYRORATEMULTIPLIER so that when you rotate quickly
// to 90 degrees, the graphic goes right to 90 degrees without requiring any creep.  This minimizes
// the need for a lot of acc input.

#ifndef ONE_OVER_ACC_COMPLIMENTARY_FILTER_TIME_PERIOD
#define ONE_OVER_ACC_COMPLIMENTARY_FILTER_TIME_PERIOD FIXEDPOINTONEOVERTWO
#endif

#ifndef GYRORATEMULTIPLIER
#define GYRORATEMULTIPLIER 76000L // 1.18 fudge factor for gyro rates. Theoretically, this value should be 65536L
#endif

//fixedpointnum ; // convert from degrees to radians and include fudge factor
fixedpointnum barotimeinterval=0;  // accumulated time between barometer reads
fixedpointnum lastbarorawaltitude; // remember our last reading so we can calculate altitude velocity

// read the acc and gyro a bunch of times and get an average of how far off they are.
// assumes the aircraft is sitting level and still.
void calibrategyroandaccelerometer()
	{
	for (int x=0;x<3;++x)
		{
		usersettings.gyrocalibration[x]=0;
		usersettings.acccalibration[x]=0;
		}
		
	// calibrate the gyro and acc
	for (int x=0;x<128;++x)
		{
		readgyro();
		readacc();
		global.acc_g_vector[ZINDEX]-=(1L<<FIXEDPOINTSHIFT); // veritcal vector should be at 1 G
		
		for (int x=0;x<3;++x)
			{
			usersettings.gyrocalibration[x]+=global.gyrorate[x];
			usersettings.acccalibration[x]+=global.acc_g_vector[x];
			}
		lib_timers_delaymilli(10);
		}

	for (int x=0;x<3;++x)
		{
		usersettings.gyrocalibration[x]=-usersettings.gyrocalibration[x]/128;
		usersettings.acccalibration[x]=-usersettings.acccalibration[x]/128;
		}
	}

void rotatevectorwithsmallangles(attitudestruct *theattitude,fixedpointnum rolldeltaangle,fixedpointnum pitchdeltaangle,fixedpointnum yawdeltaangle)
	{
	// rotate theattitude by the delta angles.
	// assumes that the delta angles are small angles in gegrees and that they are shifted left by TIMESLIVEREXTRASHIFT
	fixedpointnum v_tmp_x=theattitude->downvector[XINDEX];
	fixedpointnum v_tmp_y=theattitude->downvector[YINDEX];
	fixedpointnum v_tmp_z=theattitude->downvector[ZINDEX];

	fixedpointnum rolldeltaangleradians=lib_fp_multiply(rolldeltaangle,FIXEDPOINTPIOVER180);
	fixedpointnum pitchdeltaangleradians=lib_fp_multiply(pitchdeltaangle,FIXEDPOINTPIOVER180);
	fixedpointnum yawdeltaangleradians=lib_fp_multiply(yawdeltaangle,FIXEDPOINTPIOVER180);
	
   // remember that our delta angles are shifted left by TIMESLIVEREXTRASHIFT for resolution.  Take it out here
	theattitude->downvector[XINDEX] += (lib_fp_multiply(rolldeltaangleradians ,v_tmp_z) - lib_fp_multiply(yawdeltaangleradians,v_tmp_y))>>TIMESLIVEREXTRASHIFT;
	theattitude->downvector[YINDEX] += (lib_fp_multiply(pitchdeltaangleradians,v_tmp_z) + lib_fp_multiply(yawdeltaangleradians ,v_tmp_x))>>TIMESLIVEREXTRASHIFT; 
	theattitude->downvector[ZINDEX] -= (lib_fp_multiply(rolldeltaangleradians,v_tmp_x) + lib_fp_multiply(pitchdeltaangleradians ,v_tmp_y))>>TIMESLIVEREXTRASHIFT;

	estimatedattitude.degreesfromnorth+=(yawdeltaangle
													-lib_fp_multiply(pitchdeltaangle,lib_fp_sine(global.attitude[ROLLINDEX]))
													+lib_fp_multiply(rolldeltaangle,lib_fp_sine(global.attitude[PITCHINDEX]))
													)>>TIMESLIVEREXTRASHIFT;
	}
	
void attitudetoeulerangles(attitudestruct *theattitude,fixedpointnum *eulerangles)
	{
	eulerangles[ROLLINDEX]  =  lib_fp_atan2(theattitude->downvector[XINDEX] , theattitude->downvector[ZINDEX]) ;
	if (lib_fp_abs(eulerangles[ROLLINDEX])>FIXEDPOINT45 && lib_fp_abs(eulerangles[ROLLINDEX])<FIXEDPOINT135) 
		eulerangles[PITCHINDEX] = lib_fp_atan2(theattitude->downvector[YINDEX] , lib_fp_abs(theattitude->downvector[XINDEX]));
	else 
		eulerangles[PITCHINDEX] = lib_fp_atan2(theattitude->downvector[YINDEX] , theattitude->downvector[ZINDEX]);
	eulerangles[YAWINDEX]=theattitude->degreesfromnorth;
	}
	
void initimu()
	{
	// calibrate every time if we dont load any data from eeprom
	if (global.usersettingsfromeeprom==0) calibrategyroandaccelerometer();

	// create a conversion factor that will convert gyro data (in degrees per second) in to radians per second, calibrated
//	deltaangleconversionfactor=lib_fp_multiply(FIXEDPOINTPIOVER180,GYRORATEMULTIPLIER);

	estimatedattitude.degreesfromnorth=0;
	estimatedattitude.downvector[XINDEX]=0;
	estimatedattitude.downvector[YINDEX]=0;
	estimatedattitude.downvector[ZINDEX]=FIXEDPOINTONE;

	global.attitude[YAWINDEX]=0;
	
	lastbarorawaltitude=global.altitude=global.barorawaltitude;		

	global.altitudevelocity=0;
	}

void imucalculateestimatedattitude()
	{
	readgyro();
	readacc();

	// correct the gyro and acc readings to remove error
	fixedpointnum correctedacc_g_vector[3];
		
	for (int x=0;x<3;++x)
		{
		global.correctedgyrorate[x]=lib_fp_multiply(global.gyrorate[x]+usersettings.gyrocalibration[x],GYRORATEMULTIPLIER);
		correctedacc_g_vector[x]=global.acc_g_vector[x]+usersettings.acccalibration[x];
		}

	// calculate how many degrees we have rotated around each axis.  Keep in mind that timesliver is
	// shifted TIMESLIVEREXTRASHIFT bits to the left, so our delta angles will be as well.  This is
	// good because they are generally very small angles;
	
	// create a multiplier that will include timesliver and a conversion from degrees to radians
	// we need radians for small angle approximation
//	fixedpointnum multiplier=lib_fp_multiply(global.timesliver,deltaangleconversionfactor);

	fixedpointnum rolldeltaangle=lib_fp_multiply(global.correctedgyrorate[ROLLINDEX],global.timesliver);
	fixedpointnum pitchdeltaangle=lib_fp_multiply(global.correctedgyrorate[PITCHINDEX],global.timesliver);
	fixedpointnum yawdeltaangle=lib_fp_multiply(global.correctedgyrorate[YAWINDEX],global.timesliver);

	rotatevectorwithsmallangles(&estimatedattitude,rolldeltaangle,pitchdeltaangle,yawdeltaangle);
//	estimatedattitude.degreesfromnorth+=fpmult(<#long x#>, <#long y#>)
	
	// if the accelerometer's gravity vector is close to one G, use a complimentary filter
	// to gently adjust our estimated g vector so that it stays in line with the real one.
	// If the magnitude of the vector is not near one G, then it will be difficult to determine
	// which way is down, so we just skip it.
	fixedpointnum accmagnitudesquared=lib_fp_multiply(correctedacc_g_vector[XINDEX],correctedacc_g_vector[XINDEX])+lib_fp_multiply(correctedacc_g_vector[YINDEX],correctedacc_g_vector[YINDEX])+lib_fp_multiply(correctedacc_g_vector[ZINDEX],correctedacc_g_vector[ZINDEX]);

	if (accmagnitudesquared>MINACCMAGNITUDESQUARED && accmagnitudesquared<MAXACCMAGNITUDESQUARED)
		{
		lib_digitalio_setoutput(STABLE_INDICATOR_LED_OUTPUT, STABLE_INDICATOR_LED_OUTPUT_ON);
		for (int x=0;x<3;++x)
			{
			lib_fp_lowpassfilter(&estimatedattitude.downvector[x], correctedacc_g_vector[x],global.timesliver, ONE_OVER_ACC_COMPLIMENTARY_FILTER_TIME_PERIOD,TIMESLIVEREXTRASHIFT);
			}
		}
	else 
		lib_digitalio_setoutput(STABLE_INDICATOR_LED_OUTPUT, !STABLE_INDICATOR_LED_OUTPUT_ON);

			
	// convert our estimated gravity vector into roll and pitch angles
	// return the attitude (angles in degrees in fixedpointnum format) of the gravity vector supplied
	attitudetoeulerangles(&estimatedattitude,global.attitude);

	if (0)//global.activecheckboxitems & CHECKBOXMASKCOMPASS)
		{
		int gotnewcompassreading=readcompass();
		
		if (!(global.previousactivecheckboxitems & CHECKBOXMASKCOMPASS))
			{ // we just switched into compass mode
			// set the estimated compass vector to the actual compass vector immediately
			for (int x=0;x<3;++x)
				estimated_compass_vector[x]=global.compassvector[x];
			}
		else if (gotnewcompassreading)
			{
			// use the compass to correct the yaw in our estimated attitude.
			// the compass vector points somewhat north, but it also points down more than north where I live, so we can't
			// get the yaw directly from the compass vector.  Instead, we have to take a cross product of
			// the gravity vector and the compass vector, which should point west
			fixedpointnum v1=lib_fp_multiply(estimatedattitude.downvector[XINDEX],global.compassvector[ZINDEX])-lib_fp_multiply(estimatedattitude.downvector[ZINDEX],global.compassvector[XINDEX]);
			fixedpointnum v2=lib_fp_multiply(estimatedattitude.downvector[ZINDEX],global.compassvector[YINDEX])-lib_fp_multiply(estimatedattitude.downvector[YINDEX],global.compassvector[ZINDEX]);
	
			fixedpointnum compassangle  =  lib_fp_atan2(v1,v2)+((fixedpointnum)MAG_DECLINIATION_DEGREES)<<FIXEDPOINTSHIFT;

			// use the actual compass reading to slowly adjust our estimated yaw angle
			lib_fp_lowpassfilter(&estimatedattitude.degreesfromnorth, compassangle,global.timesliver, FIXEDPOINTONEOVERFOUR,TIMESLIVEREXTRASHIFT);
			}
		}
/*	else
		{ // we aren't using the comopass
		global.attitude[YAWINDEX]+=lib_fp_multiply(global.correctedgyrorate[YAWINDEX],global.timesliver)>>TIMESLIVEREXTRASHIFT;
		}*/

	barotimeinterval+=global.timesliver;
	
	if (readbaro())
		{ // we got a new baro reading
		lib_fp_lowpassfilter(&global.altitude, global.barorawaltitude,barotimeinterval>>TIMESLIVEREXTRASHIFT, FIXEDPOINTONEOVERONEHALF,0);

		// We want to put the baro velocity (change in baro over time) into a low pass filter but
		// we don't want to divide by the time interval to get velocity (divide is expensive) to then turn around and
		// multiply by the same time interval. So the following is the same as the lib_fp_lowpassfilter code
		// except we eliminate the multiply.
		fixedpointnum fraction=lib_fp_multiply(barotimeinterval>>TIMESLIVEREXTRASHIFT,FIXEDPOINTONEOVERONE);
	   global.altitudevelocity=(global.altitude-lastbarorawaltitude+lib_fp_multiply((FIXEDPOINTONE)-fraction,global.altitudevelocity));
		
		lastbarorawaltitude=global.altitude;
		barotimeinterval=0;
		}
		
	// use a dot product of the accelerometer vector and our estimated gravity vector to get
	// our altitude acceleration
/*   global.altitudeacc=lib_fp_multiply(correctedacc_g_vector[0],estimated_g_vector[0])
									 +lib_fp_multiply(correctedacc_g_vector[1],estimated_g_vector[1])
									 +lib_fp_multiply(correctedacc_g_vector[2],estimated_g_vector[2])
									 -(1L<<FIXEDPOINTSHIFT); // subtract off one g for actual gravity

velocity+=lib_fp_multiply(lib_fp_multiply(altitudeacc,321126L),global.timesliver)>>TIMESLIVEREXTRASHIFT;
lib_fp_lowpassfilter(&velocity, barovelocity,global.timesliver>>2, FIXEDPOINTONEOVERONEHALF,TIMESLIVEREXTRASHIFT-2);
//velocity=0;
//global.altitude+=lib_fp_multiply(velocity,global.timesliver)>>TIMESLIVEREXTRASHIFT;
//lib_fp_lowpassfilter(&global.altitude, global.barorawaltitude,global.timesliver>>2, FIXEDPOINTONEOVERFOUR,TIMESLIVEREXTRASHIFT-2);
*/
	

	}