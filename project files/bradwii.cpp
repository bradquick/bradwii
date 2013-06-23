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

/*

The code is for controlling multi-copters.  Many of the ideas in the code come from the Multi-Wii project
(see multiwii.com).  This project doesn't contain all of the features in Multi-Wii, but I believe it incorporates
a number of improvements.

In order to make the code run quickly on 8 bit processors, much of the math is done using fixed point numbers
instead of floating point.  Much pain was taken to write almost the entire code without performing any
division, which is slow. As a result, main loop cycles can take well under 2 milliseconds.

A second advantage is that I believe that this code is more logically layed out and better commented than 
some other multi-copter code.  It is designed to be easy to follow for the guy who wants to understand better how
the code works and maybe wants to modify it.

In general, I didn't include code that I haven't tested myself, therefore many aircraft configurations, control boards,
sensors, etc. aren't yet included in the code.  It should be fairly easy, however for interested coders to add the
components that they need.

If you find the code useful, I'd love to hear from you.  Email me at the address that's shown vertically below:

b        I made my
r			email address
a			vertical so
d			the spam bots
@			won't figure
j			it out.
a			- Thanks.
m
e
s
l
t
a
y
l
o
r
.
c
o
m

*/

#include "stdio.h"
#include <avr/io.h>
#include <avr/interrupt.h> 

// library headers
#include "lib_timers.h"
#include "lib_serial.h"
#include "lib_i2c.h"
#include "lib_digitalio.h"
#include "lib_pwm.h"
#include "lib_fp.h" 

// project file headers
#include "bradwii.h"
#include "rx.h"
#include "serial.h"
#include "output.h"
#include "gyro.h" 
#include "accelerometer.h"
#include "imu.h"
#include "baro.h"
#include "compass.h"
#include "eeprom.h"
#include "gps.h"
#include "navigation.h"
#include "pilotcontrol.h"
#include "autotune.h"

globalstruct global; // global variables
usersettingsstruct usersettings; // user editable variables

fixedpointnum altitudeholddesiredaltitude;
fixedpointnum integratedaltitudeerror; // for pid control

fixedpointnum integratedangleerror[3];

// limit pid windup
#define INTEGRATEDANGLEERRORLIMIT FIXEDPOINTCONSTANT(5000) 

// timesliver is a very small slice of time (.002 seconds or so).  This small value doesn't take much advantage
// of the resolution of fixedpointnum, so we shift timesliver an extra TIMESLIVEREXTRASHIFT bits.
unsigned long timeslivertimer=0;

// It all starts here:
int main(void)
	{
   // start with default user settings in case there's nothing in eeprom
	defaultusersettings();
   // try to load usersettings from eeprom
	readusersettingsfromeeprom();
	
   // set our LED as a digital output
   lib_digitalio_initpin(LED1_OUTPUT,DIGITALOUTPUT);

   //initialize the libraries that require initialization
	lib_timers_init();
	lib_i2c_init();
   
   // initialize all other modules
	initrx();
	initoutputs();
	serialinit();	
	initgyro();
	initacc();
	initbaro();
	initcompass();	
	initgps();
	initimu();
   
	// set the default i2c speed to 400 Mhz.  If a device needs to slow it down, it can, but it should set it back.
	lib_i2c_setclockspeed(I2C_400_MHZ);

	global.armed=0;
	global.navigationmode=NAVIGATIONMODEOFF;
   global.failsafetimer=lib_timers_starttimer();

	for(;;) 
		{
		// check to see what switches are activated
		checkcheckboxitems();
		
		// check for config program activity
		serialcheckforaction();	
		
		// load global.timesliver with the amount of time that has passed since we last went through this loop
		// convert from microseconds to fixedpointnum seconds shifted by TIMESLIVEREXTRASHIFT
		// 4295L is (FIXEDPOINTONE<<FIXEDPOINTSHIFT)*.000001
		global.timesliver=(lib_timers_gettimermicrosecondsandreset(&timeslivertimer)*4295L)>>(FIXEDPOINTSHIFT-TIMESLIVEREXTRASHIFT);
	
		// don't allow big jumps in time because of something slowing the update loop down (should never happen anyway)
		if (global.timesliver>(FIXEDPOINTONEFIFTIETH<<TIMESLIVEREXTRASHIFT)) global.timesliver=FIXEDPOINTONEFIFTIETH<<TIMESLIVEREXTRASHIFT;
		
		// run the imu to estimate the current attitude of the aircraft
		imucalculateestimatedattitude();

		// arm and disarm via rx aux switches
		if (global.rxvalues[THROTTLEINDEX]<FPARMEDMINMOTOROUTPUT)
			{ // see if we want to change armed modes
			if (!global.armed)
				{
				if (global.activecheckboxitems & CHECKBOXMASKARM)
					{
					global.armed=1;
					#if (GPS_TYPE!=NO_GPS)
						navigation_sethometocurrentlocation();
					#endif
					}
				}
			else if (!(global.activecheckboxitems & CHECKBOXMASKARM)) global.armed=0;
			}

		#if (GPS_TYPE!=NO_GPS)
		// turn on or off navigation when appropriate
		if (global.navigationmode==NAVIGATIONMODEOFF)
			{
			if (global.activecheckboxitems & CHECKBOXMASKRETURNTOHOME) // return to home switch turned on
				{
				navigation_set_destination(global.gps_home_latitude,global.gps_home_longitude);
				global.navigationmode=NAVIGATIONMODERETURNTOHOME;
				}
			else if (global.activecheckboxitems & CHECKBOXMASKPOSITIONHOLD) // position hold turned on
				{
				navigation_set_destination(global.gps_current_latitude,global.gps_current_longitude);
				global.navigationmode=NAVIGATIONMODEPOSITIONHOLD;
				}
			}
		else // we are currently navigating
			{ // turn off navigation if desired
			if ((global.navigationmode==NAVIGATIONMODERETURNTOHOME && !(global.activecheckboxitems & CHECKBOXMASKRETURNTOHOME))
				|| (global.navigationmode==NAVIGATIONMODEPOSITIONHOLD && !(global.activecheckboxitems & CHECKBOXMASKPOSITIONHOLD)))
				{
				global.navigationmode=NAVIGATIONMODEOFF;
            
            // we will be turning control back over to the pilot.
				resetpilotcontrol();
				}
			}
		#endif
		
		// read the receiver
		readrx();

		
		// turn on the LED when we are stable and the gps has 5 satelites or more
		#if (GPS_TYPE==NO_GPS)
			lib_digitalio_setoutput(LED1_OUTPUT, (!global.stable)==LED1_ON);
		#else
			lib_digitalio_setoutput(LED1_OUTPUT, (!(global.stable && global.gps_num_satelites>=5))==LED1_ON);
		#endif
		
		// get the angle error.  Angle error is the difference between our current attitude and our desired attitude.
      // It can be set by navigation, or by the pilot, etc.
		fixedpointnum angleerror[3];
		
#if (GPS_TYPE!=NO_GPS)
		// read the gps
		unsigned char gotnewgpsreading=readgps();

      // let the pilot control the aircraft.
      getangleerrorfrompilotinput(angleerror);
      
		// if we are navigating, use navigation to determine our desired attitude (tilt angles)
		if (global.navigationmode!=NAVIGATIONMODEOFF)
			{ // we are navigating
			navigation_setangleerror(gotnewgpsreading,angleerror);
			}
#endif

      if (global.rxvalues[THROTTLEINDEX]<FPARMEDMINMOTOROUTPUT)
			{
			// We are probably on the ground. Don't accumnulate error when we can't correct it
			resetpilotcontrol();
			
			// bleed off integrated error by averaging in a value of zero
			lib_fp_lowpassfilter(&integratedangleerror[ROLLINDEX],0L,global.timesliver>>TIMESLIVEREXTRASHIFT,FIXEDPOINTONEOVERONEFOURTH,0); 
			lib_fp_lowpassfilter(&integratedangleerror[PITCHINDEX],0L,global.timesliver>>TIMESLIVEREXTRASHIFT,FIXEDPOINTONEOVERONEFOURTH,0);
			lib_fp_lowpassfilter(&integratedangleerror[YAWINDEX],0L,global.timesliver>>TIMESLIVEREXTRASHIFT,FIXEDPOINTONEOVERONEFOURTH,0);
   		}

#ifndef NO_AUTOTUNE
		// let autotune adjust the angle error if the pilot has autotune turned on
		if (global.activecheckboxitems & CHECKBOXMASKAUTOTUNE)
			{
			if (!(global.previousactivecheckboxitems & CHECKBOXMASKAUTOTUNE))
				autotune(angleerror,AUTOTUNESTARTING); // tell autotune that we just started autotuning
			else
				autotune(angleerror,AUTOTUNETUNING); // tell autotune that we are in the middle of autotuning
			}
		else if (global.previousactivecheckboxitems & CHECKBOXMASKAUTOTUNE)
			autotune(angleerror,AUTOTUNESTOPPING); // tell autotune that we just stopped autotuning
#endif

		// get the pilot's throttle component
		// convert from fixedpoint -1 to 1 to fixedpoint 0 to 1
		fixedpointnum throttleoutput=(global.rxvalues[THROTTLEINDEX]>>1)+FIXEDPOINTCONSTANT(.5);

      // keep a flag to indicate whether we shoud apply altitude hold.  The pilot can turn it on or
      // uncrashability mode can turn it on.
      unsigned char altitudeholdactive=0;
      
      if (global.activecheckboxitems & CHECKBOXMASKALTHOLD)
         {
         altitudeholdactive=1;
         if (!(global.previousactivecheckboxitems & CHECKBOXMASKALTHOLD))
            { // we just turned on alt hold.  Remember our current alt. as our target
            altitudeholddesiredaltitude=global.altitude;
            integratedaltitudeerror=0;
            }
         }
         
      
      // uncrashability mode
      #define UNCRASHABLELOOKAHEADTIME FIXEDPOINTONE // look ahead one second to see if we are going to be at a bad altitude
      #define UNCRASHABLERECOVERYANGLE FIXEDPOINTCONSTANT(15) // don't let the pilot pitch or roll more than 20 degrees when altitude is too low.
      #define FPUNCRASHABLE_RADIUS FIXEDPOINTCONSTANT(UNCRAHSABLE_RADIUS) 
      #define FPUNCRAHSABLE_MAX_ALTITUDE_OFFSET FIXEDPOINTCONSTANT(UNCRAHSABLE_MAX_ALTITUDE_OFFSET)
#if (GPS_TYPE!=NO_GPS)
      // keep a flag that tells us whether uncrashability is doing gps navigation or not
      static unsigned char doinguncrashablenavigationflag;
#endif
      // we need a place to remember what the altitude was when uncrashability mode was turned on
      static fixedpointnum uncrasabilityminimumaltitude;
      static fixedpointnum uncrasabilitydesiredaltitude;
      static unsigned char doinguncrashablealtitudehold=0;
      
      if (global.activecheckboxitems & CHECKBOXMASKUNCRASHABLE) // uncrashable mode
         {
         // First, check our altitude
         // are we about to crash?
         if (!(global.previousactivecheckboxitems & CHECKBOXMASKUNCRASHABLE))
            { // we just turned on uncrashability.  Remember our current altitude as our new minimum altitude.
            uncrasabilityminimumaltitude=global.altitude;
            doinguncrashablenavigationflag=0;
#if (GPS_TYPE!=NO_GPS)
            // set this location as our new home
            navigation_sethometocurrentlocation();
#endif
            }
         
         // calculate our projected altitude based on how fast our altitude is changing
         fixedpointnum projectedaltitude=global.altitude+lib_fp_multiply(global.altitudevelocity,UNCRASHABLELOOKAHEADTIME);
         
         if (projectedaltitude>uncrasabilityminimumaltitude+FPUNCRAHSABLE_MAX_ALTITUDE_OFFSET)
            { // we are getting too high
            // Use Altitude Hold to bring us back to the maximum altitude.
            altitudeholddesiredaltitude=uncrasabilityminimumaltitude+FPUNCRAHSABLE_MAX_ALTITUDE_OFFSET;
            integratedaltitudeerror=0;
            altitudeholdactive=1;
            }            
         else if (projectedaltitude<uncrasabilityminimumaltitude)
            { // We are about to get below our minimum crashability altitude
            if (doinguncrashablealtitudehold==0)
               { // if we just entered uncrashability, set our desired altitude to the current altitude
               uncrasabilitydesiredaltitude=global.altitude;
               integratedaltitudeerror=0;
               doinguncrashablealtitudehold=1;
               }
            
            // don't apply throttle until we are almost level
            if (global.estimateddownvector[ZINDEX]>FIXEDPOINTCONSTANT(.4))
               {
               altitudeholddesiredaltitude=uncrasabilitydesiredaltitude;
               altitudeholdactive=1;
               }
            else throttleoutput=0; // we are trying to rotate to level, kill the throttle until we get there

            // make sure we are level!  Don't let the pilot command more than UNCRASHABLERECOVERYANGLE
            lib_fp_constrain(&angleerror[ROLLINDEX],-UNCRASHABLERECOVERYANGLE-global.currentestimatedeulerattitude[ROLLINDEX],UNCRASHABLERECOVERYANGLE-global.currentestimatedeulerattitude[ROLLINDEX]);
            lib_fp_constrain(&angleerror[PITCHINDEX],-UNCRASHABLERECOVERYANGLE-global.currentestimatedeulerattitude[PITCHINDEX],UNCRASHABLERECOVERYANGLE-global.currentestimatedeulerattitude[PITCHINDEX]);
            }
         else doinguncrashablealtitudehold=0;
         
#if (GPS_TYPE!=NO_GPS)
         // Next, check to see if our GPS says we are out of bounds
         // are we out of bounds?
         fixedpointnum bearingfromhome;
         fixedpointnum distancefromhome=navigation_getdistanceandbearing(global.gps_current_latitude,global.gps_current_longitude,global.gps_home_latitude,global.gps_home_longitude,&bearingfromhome);
         
         if (distancefromhome>FPUNCRASHABLE_RADIUS)
            { // we are outside the allowable area, navigate back toward home
            if (!doinguncrashablenavigationflag)
               { // we just started navigating, so we have to set the destination
               navigation_set_destination(global.gps_home_latitude,global.gps_home_longitude);
               doinguncrashablenavigationflag=1;
               }
               
            // Let the navigation figure out our roll and pitch attitudes
            navigation_setangleerror(gotnewgpsreading,angleerror);
            }
         else doinguncrashablenavigationflag=0;
#endif
         }
#if (GPS_TYPE!=NO_GPS)
      else doinguncrashablenavigationflag=0;
#endif

      // if we don't hear from the receiver for over a second, try to land safely
      if (lib_timers_gettimermicroseconds(global.failsafetimer)>1000000L)
         {
         throttleoutput=FPFAILSAFEMOTOROUTPUT;

         // make sure we are level!
         angleerror[ROLLINDEX]=-global.currentestimatedeulerattitude[ROLLINDEX];
         angleerror[PITCHINDEX]=-global.currentestimatedeulerattitude[PITCHINDEX];
         }

#if (BAROMETER_TYPE!=NO_BAROMETER)
      // check for altitude hold and adjust the throttle output accordingly
		if (altitudeholdactive)
			{
         integratedaltitudeerror+=lib_fp_multiply(altitudeholddesiredaltitude-global.altitude,global.timesliver);
         lib_fp_constrain(&integratedaltitudeerror,-INTEGRATEDANGLEERRORLIMIT,INTEGRATEDANGLEERRORLIMIT); // don't let the integrated error get too high
			
         // do pid for the altitude hold and add it to the throttle output
         throttleoutput+=lib_fp_multiply(altitudeholddesiredaltitude-global.altitude,usersettings.pid_pgain[ALTITUDEINDEX])
            -lib_fp_multiply(global.altitudevelocity,usersettings.pid_dgain[ALTITUDEINDEX])
            +lib_fp_multiply(integratedaltitudeerror,usersettings.pid_igain[ALTITUDEINDEX]);
            
         }
#endif
      if ((global.activecheckboxitems & CHECKBOXMASKAUTOTHROTTLE) || altitudeholdactive)
         {
         // Auto Throttle Adjust - Increases the throttle when the aircraft is tilted so that the vertical
         // component of thrust remains constant.
         // The AUTOTHROTTLEDEADAREA adjusts the value at which the throttle starts taking effect.  If this
         // value is too low, the aircraft will gain altitude when banked, if it's too low, it will lose
         // altitude when banked. Adjust to suit.
         #define AUTOTHROTTLEDEADAREA FIXEDPOINTCONSTANT(.25)

         if (global.estimateddownvector[ZINDEX]>FIXEDPOINTCONSTANT(.3))
            {
            // Divide the throttle by the throttleoutput by the z component of the down vector
            // This is probaly the slow way, but it's a way to do fixed point division
            fixedpointnum recriprocal=lib_fp_invsqrt(global.estimateddownvector[ZINDEX]);
            recriprocal=lib_fp_multiply(recriprocal,recriprocal);
         
            throttleoutput=lib_fp_multiply(throttleoutput-AUTOTHROTTLEDEADAREA,recriprocal)+AUTOTHROTTLEDEADAREA;
            }
         }

		// calculate output values.  Output values will range from 0 to 1.0

		// calculate pid outputs based on our angleerrors as inputs
		fixedpointnum pidoutput[3];
		
		for (int x=0;x<3;++x)
			{
			integratedangleerror[x]+=lib_fp_multiply(angleerror[x],global.timesliver);
			
			// don't let the integrated error get too high (windup)
			lib_fp_constrain(&integratedangleerror[x],-INTEGRATEDANGLEERRORLIMIT,INTEGRATEDANGLEERRORLIMIT);
			
			// do the attitude pid
			pidoutput[x]=lib_fp_multiply(angleerror[x],usersettings.pid_pgain[x])
				-lib_fp_multiply(global.gyrorate[x],usersettings.pid_dgain[x])
				+(lib_fp_multiply(integratedangleerror[x],usersettings.pid_igain[x])>>4);
			}

      lib_fp_constrain(&throttleoutput,0,FIXEDPOINTONE);

		// mix the outputs to create motor values
#if (AIRCRAFT_CONFIGURATION==QUADX)
		setmotoroutput(0,MOTOR_0_CHANNEL,throttleoutput-pidoutput[ROLLINDEX]+pidoutput[PITCHINDEX]-pidoutput[YAWINDEX]);
		setmotoroutput(1,MOTOR_1_CHANNEL,throttleoutput-pidoutput[ROLLINDEX]-pidoutput[PITCHINDEX]+pidoutput[YAWINDEX]);
		setmotoroutput(2,MOTOR_2_CHANNEL,throttleoutput+pidoutput[ROLLINDEX]+pidoutput[PITCHINDEX]+pidoutput[YAWINDEX]);
		setmotoroutput(3,MOTOR_3_CHANNEL,throttleoutput+pidoutput[ROLLINDEX]-pidoutput[PITCHINDEX]-pidoutput[YAWINDEX]);
#endif
		}
		
   return 0;   /* never reached */
	}


void defaultusersettings()
	{
	global.usersettingsfromeeprom=0; // this should get set to one if we read from eeprom
	
	// set default acro mode rotation rates
	usersettings.maxyawrate=125L<<FIXEDPOINTSHIFT; // degrees per second
	usersettings.maxpitchandrollrate=250L<<FIXEDPOINTSHIFT; // degrees per second

	// set default PID settings
	for (int x=0;x<3;++x)
		{
		usersettings.pid_pgain[x]=20L<<3; // 2.0 on configurator
		usersettings.pid_igain[x]=4L;     // .004 on configurator
		usersettings.pid_dgain[x]=8L<<2;     // 15 on configurator
		}

	usersettings.pid_pgain[YAWINDEX]=40L<<3; // 4 on configurator
	
	for (int x=3;x<NUMPIDITEMS;++x)
		{
		usersettings.pid_pgain[x]=0;
		usersettings.pid_igain[x]=0;
		usersettings.pid_dgain[x]=0;
		}
	
	usersettings.pid_pgain[ALTITUDEINDEX]=50L<<6; // 5.0 on configurator
	usersettings.pid_dgain[ALTITUDEINDEX]=6L<<6;     // 6 on configurator

	usersettings.pid_pgain[NAVIGATIONINDEX]=30L<<9; // 3.0 on configurator
	usersettings.pid_dgain[NAVIGATIONINDEX]=100L<<8;     // .100 on configurator
	
	// set default configuration checkbox settings.
	for (int x=0;x<NUMPOSSIBLECHECKBOXES;++x)
		{
		usersettings.checkboxconfiguration[x]=0;
		}
	usersettings.checkboxconfiguration[CHECKBOXARM]=CHECKBOXMASKAUX1HIGH;
	usersettings.checkboxconfiguration[CHECKBOXSEMIACRO]=CHECKBOXMASKAUX2HIGH;
	usersettings.checkboxconfiguration[CHECKBOXHIGHRATES]=CHECKBOXMASKAUX2HIGH;
	
	// reset the calibration settings
	for (int x=0;x<3;++x)
		{
		usersettings.compasszerooffset[x]=0;
		usersettings.compasscalibrationmultiplier[x]=1L<<FIXEDPOINTSHIFT;
		usersettings.gyrocalibration[x]=0;
		usersettings.acccalibration[x]=0;
		}
	}
