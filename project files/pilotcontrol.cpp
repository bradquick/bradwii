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

#include "pilotcontrol.h"
#include "bradwii.h"
#include "vectors.h"
#include "lib_timers.h"

extern globalstruct global;
extern usersettingsstruct usersettings;

// convert maximum tilt angles for level mode into fixed point
#define FP_LEVEL_MODE_MAX_TILT FIXEDPOINTCONSTANT(LEVEL_MODE_MAX_TILT)
#define FP_LEVEL_MODE_MAX_TILT_HIGH_ANGLE FIXEDPOINTCONSTANT(LEVEL_MODE_MAX_TILT_HIGH_ANGLE)
// convert high rate multiplier into fixed point
#define FP_HIGH_RATES_MULTILIER FIXEDPOINTCONSTANT(HIGH_RATES_MULTILIER)

// When the yaw stick is centered, allow compass hold.  This defines what centered is:
#define YAWCOMPASSRXDEADBAND FIXEDPOINTCONSTANT(.125) // 1/8 of the range

fixedpointnum filteredyawgyrorate=0;
fixedpointnum desiredcompassheading;
fixedpointnum highyawrate;
fixedpointnum highpitchandrollrate;

void resetpilotcontrol()
   { // called when switching from navigation control to pilot control or when idling on the ground.
   // keeps us from accumulating yaw error that we can't correct.
   desiredcompassheading=global.currentestimatedeulerattitude[YAWINDEX];
   
   // calculate our max rotation rates based on usersettings
   highyawrate=lib_fp_multiply(usersettings.maxyawrate, FP_HIGH_RATES_MULTILIER);
   highpitchandrollrate=lib_fp_multiply(usersettings.maxpitchandrollrate, FP_HIGH_RATES_MULTILIER);
   }

void getangleerrorfrompilotinput(fixedpointnum *angleerror)
   {
   // sets the ange errors for roll, pitch, and yaw based on where the pilot has the tx sticks.
   fixedpointnum rxrollvalue;
   fixedpointnum rxpitchvalue;
   
   // if in headfree mode, rotate the pilot's stick inputs by the angle that is the difference between where we are currently heading and where we were heading when we armed.
   if (global.activecheckboxitems & CHECKBOXMASKHEADFREE)
      {
      fixedpointnum angledifference = global.currentestimatedeulerattitude[YAWINDEX] - global.heading_when_armed;

      fixedpointnum cosangledifference = lib_fp_cosine(angledifference);
      fixedpointnum sinangledifference = lib_fp_sine(angledifference);
      rxpitchvalue = lib_fp_multiply(global.rxvalues[PITCHINDEX],cosangledifference) + lib_fp_multiply(global.rxvalues[ROLLINDEX],sinangledifference);
      rxrollvalue =  lib_fp_multiply(global.rxvalues[ROLLINDEX],cosangledifference) - lib_fp_multiply(global.rxvalues[PITCHINDEX],sinangledifference);
      }
   else
      {
      rxpitchvalue=global.rxvalues[PITCHINDEX];
      rxrollvalue=global.rxvalues[ROLLINDEX];
      }
   
   // first, calculate level mode values
   // how far is our estimated current attitude from our desired attitude?
   // desired angle is rxvalue (-1 to 1) times LEVEL_MODE_MAX_TILT
   // First, figure out which max angle we are using depending on aux switch settings.
   fixedpointnum levelmodemaxangle;
   if (global.activecheckboxitems & CHECKBOXMASKHIGHANGLE) levelmodemaxangle=FP_LEVEL_MODE_MAX_TILT_HIGH_ANGLE;
   else levelmodemaxangle=FP_LEVEL_MODE_MAX_TILT;

   // the angle error is how much our current angles differ from our desired angles.
   fixedpointnum levelmoderollangleerror=lib_fp_multiply(rxrollvalue,levelmodemaxangle)-global.currentestimatedeulerattitude[ROLLINDEX];
   fixedpointnum levelmodepitchangleerror=lib_fp_multiply(rxpitchvalue,levelmodemaxangle)-global.currentestimatedeulerattitude[PITCHINDEX];
   
   // In acro mode, we want the rotation rate to be proportional to the pilot's stick movement.  The desired rotation rate is
   // the stick movement * a multiplier.
   // Fill angleerror with acro values.  If we are currently rotating at rate X and
   // we want to be rotating at rate Y, then our angle should be off by (Y-X)*timesliver.  In theory, we should probably accumulate
   // this error over time, but our I term will do that anyway and we are talking about rates, which don't need to be perfect.
   // First figure out what max rotation rates we are using depending on our aux switch settings.
   fixedpointnum maxyawrate;
   fixedpointnum maxpitchandrollrate;
   if (global.activecheckboxitems & CHECKBOXMASKHIGHRATES)
      {
      maxyawrate=highyawrate;
      maxpitchandrollrate=highpitchandrollrate;
      }
   else
      {
      maxyawrate=usersettings.maxyawrate;
      maxpitchandrollrate=usersettings.maxpitchandrollrate;
      }
      
   angleerror[ROLLINDEX]=lib_fp_multiply(lib_fp_multiply(rxrollvalue,maxpitchandrollrate)-global.gyrorate[ROLLINDEX],global.timesliver);
   angleerror[PITCHINDEX]=lib_fp_multiply(lib_fp_multiply(rxpitchvalue,maxpitchandrollrate)-global.gyrorate[PITCHINDEX],global.timesliver);

   // put a low pass filter on the yaw gyro.  If we don't do this, things can get jittery.
   lib_fp_lowpassfilter(&filteredyawgyrorate, global.gyrorate[YAWINDEX], global.timesliver>>(TIMESLIVEREXTRASHIFT-3), FIXEDPOINTONEOVERONESIXTYITH, 3);

   // Calculate our yaw angle error
   angleerror[YAWINDEX]=lib_fp_multiply(lib_fp_multiply(global.rxvalues[YAWINDEX],maxyawrate)-filteredyawgyrorate,global.timesliver);

   // handle compass control
   if (global.activecheckboxitems & CHECKBOXMASKCOMPASS)
      { 
      if (!(global.previousactivecheckboxitems & CHECKBOXMASKCOMPASS))
         {// we just switched into compass mode
         // reset the angle error to zero so that we don't yaw because of the switch
         desiredcompassheading=global.currentestimatedeulerattitude[YAWINDEX];
         }
      
      // the compass holds only when right side up with throttle up (not sitting on the ground)
      // and the yaw stick is centered.  If it is centered, override the pilot's input
      if (global.estimateddownvector[ZINDEX]>0 && global.rxvalues[THROTTLEINDEX]>FPSTICKLOW && global.rxvalues[YAWINDEX]>-YAWCOMPASSRXDEADBAND && global.rxvalues[YAWINDEX]<YAWCOMPASSRXDEADBAND)
         {
         angleerror[YAWINDEX]=desiredcompassheading-global.currentestimatedeulerattitude[YAWINDEX];
         lib_fp_constrain180(&angleerror[YAWINDEX]);
         }
      else // the pilot is controlling yaw, so update the desired heading
         desiredcompassheading=global.currentestimatedeulerattitude[YAWINDEX];
      }
   
   // figure out how much to use acro mode and how much to use level mode.
   fixedpointnum acromodefraction; // these two values should total FIXEDPOINTONE.  They are the weights applied to acro and level mode control
   fixedpointnum levelmodefraction;

   if (global.activecheckboxitems & CHECKBOXMASKFULLACRO)
      { // acro mode
      acromodefraction=FIXEDPOINTONE;
      levelmodefraction=0;
      }
   else if (!(global.activecheckboxitems & CHECKBOXMASKSEMIACRO))
      { // level mode
      acromodefraction=0;
      levelmodefraction=FIXEDPOINTONE;
      }
   else
      { // semi acro mode
      // figure out how much the most moved stick is from centered
      fixedpointnum maxstickthrow;
   
      if (rxrollvalue<0)
         maxstickthrow=-rxrollvalue;
      else
         maxstickthrow=rxrollvalue;
         
      if (rxpitchvalue<0)
         {
         if (-rxpitchvalue>maxstickthrow) maxstickthrow=-rxpitchvalue;
         }
      else
         if (rxpitchvalue>maxstickthrow) maxstickthrow=rxpitchvalue;
         
   
      // if the aircraft is tipped more than 90 degrees, use full acro mode so we don't run into
      // euler issues when inverted.  This also allows us to pause while up-side-down if we want to.
      if (global.estimateddownvector[ZINDEX]<0)
         {
         acromodefraction=FIXEDPOINTONE;
         levelmodefraction=0;
         }
      else
         {
         acromodefraction=maxstickthrow;
      
         if (acromodefraction>FIXEDPOINTONE) acromodefraction=FIXEDPOINTONE;
         levelmodefraction=FIXEDPOINTONE-acromodefraction;
         }
      }

   // combine level and acro modes
   angleerror[ROLLINDEX]=lib_fp_multiply(angleerror[ROLLINDEX], acromodefraction)+lib_fp_multiply(levelmoderollangleerror, levelmodefraction);
   angleerror[PITCHINDEX]=lib_fp_multiply(angleerror[PITCHINDEX], acromodefraction)+lib_fp_multiply(levelmodepitchangleerror, levelmodefraction);   
   }
   
/*
Below is some old code that was using vectors.  The advantage is that it avoids singularities.  It also chooses the shortest
rotational path to the desired attitude.  The downside is that it's slow and it's difficult to mix flight modes.

#define FIXEDPOINTONEOVER500 (FIXEDPOINTONE/500L)
// convert from r/c input milliseconds (stick position) to a rotation rate (fixedpointnum degrees per second)
#define ROTATIONRATEMULTIPLIER ((2L<<FIXEDPOINTSHIFT)/500L) 
// when rotating by rate (such as yaw) don't let our desired angle get too far ahead of the actual angle
#define MAXANGLEERROR (15L<<FIXEDPOINTSHIFT)

fixedpointnum desiredwestvector[3]={FIXEDPOINTONE,0,0};
fixedpointnum lastyawerror=0;

void resetpilotcontrol()
   { // called when switching from navigation control to pilot control or when idling on the ground.
   // keeps us from accumulating yaw error that we can't correct.
   for (int x=0;x<3;++x)
      desiredwestvector[x]=global.estimatedwestvector[x];
   }

//#define MAXANGLEM
void getangleerrorfrompilotinput(fixedpointnum *angleerror)
   {
   // create a unit vector in the direction the r/c pilot would like down to point
   fixedpointnum desireddownvector[3];

   // get desired pitch and roll angles from the rx inputs
   fixedpointnum desiredrollangle=lib_fp_multiply(global.rxvalues[ROLLINDEX]-FPRXMIDPOINT,DESIREDANGLEMULTIPLIER);
   fixedpointnum desiredpitchangle=lib_fp_multiply(global.rxvalues[PITCHINDEX]-FPRXMIDPOINT,DESIREDANGLEMULTIPLIER);
   
   // rotate a unit vector around the pitch an roll axes
   fixedpointnum sineofrollangle=lib_fp_sine(desiredrollangle);
   fixedpointnum cosineofrollangle=lib_fp_cosine(desiredrollangle);
   fixedpointnum sineofpitchangle=lib_fp_sine(desiredpitchangle);
   fixedpointnum cosineofpitchangle=lib_fp_cosine(desiredpitchangle);
   
   desireddownvector[0]=lib_fp_multiply(sineofrollangle,cosineofpitchangle);
   desireddownvector[1]=lib_fp_multiply(sineofpitchangle,cosineofrollangle);
   desireddownvector[2]=lib_fp_multiply(cosineofpitchangle,cosineofrollangle);
   
   if (global.rxvalues[THROTTLEINDEX]<FPARMEDMINMOTOROUTPUT)
      { // we probably aren't off the ground.
      // don't accumulate error.  Use a low pass filter to bleed the error off slowly so we don't get abrupt motion if we are still in the air
      // don't drift while sitting on the ground
      for (int x=0;x<3;++x)
         desiredwestvector[x]=global.estimatedwestvector[x];
      }
   else
      {
      // rotate the desired west vector around the desired down vector according to the pilot's input
      // add the pilot's rotation rate to the angle to get our error
      fixedpointnum angle=lib_fp_multiply(lib_fp_multiply(global.rxvalues[YAWINDEX]-FPRXMIDPOINT,usersettings.maxyawrate*ROTATIONRATEMULTIPLIER),global.timesliver)>>TIMESLIVEREXTRASHIFT;

      // don't rotate if we are going to accumulate too much yaw error
      if ((lastyawerror<MAXANGLEERROR && angle>0) || (lastyawerror>-MAXANGLEERROR && angle<0))
         {
         // rotate the desiredwestvector by angle
         rotatevectorbyaxissmallangle(desiredwestvector,desireddownvector,angle);
         }
      }

   
   // keep desiredwestvector perpendicular to the desireddownvector
   // cross west with down to get north.  Cross down with north to get west, but now west will be perpendicular to down.
   fixedpointnum vector[3];

   vectorcrossproduct(desiredwestvector, desireddownvector,vector);
   if (vector[0]!=0 || vector[1]!=0 || vector[2]!=0)
      {
      vectorcrossproduct(desireddownvector,vector, desiredwestvector);
      normalizevector(desiredwestvector);
      }
   
   // find the axis of rotation and angle from our current down vector to the desired one
   fixedpointnum axisofrotation[3];
   vectorcrossproduct(global.estimateddownvector, desireddownvector, axisofrotation);

   // normalize the axis of rotation, but remember it's length
   fixedpointnum axislength=lib_fp_sqrt(normalizevector(axisofrotation));
   
   fixedpointnum rotatedwestvector[3];

   if (axislength>100L)
      {
      // get the angle of rotation between the two vectors
      fixedpointnum angle=lib_fp_atan2(axislength, vectordotproduct(global.estimateddownvector, desireddownvector));

      // calculate the roll and pitch errors
      // this is the same as a dot product of {0,1,0} and axisofrotation multiplied by angle
      angleerror[ROLLINDEX]=lib_fp_multiply(axisofrotation[1], angle);
   
      // this is the same as a dot product of {1,0,0} and axisofrotation multiplied by angle
      angleerror[PITCHINDEX]=-lib_fp_multiply(axisofrotation[0], angle);

      // rotate our current west vector by the axis and angle calculated above that's needed to get the down vectors aligned
      rotatevectorbyaxisangle(global.estimatedwestvector,axisofrotation,angle,rotatedwestvector);
      }
   else
      {
      angleerror[ROLLINDEX]=0;
      angleerror[PITCHINDEX]=0;
      for (int x=0;x<3;++x) rotatedwestvector[x]=global.estimatedwestvector[x];
      }
   
   // calculate how much more we need to rotate to get the westvectors aligned
   vectorcrossproduct(rotatedwestvector, desiredwestvector, axisofrotation);

   // normalize the axis of rotation, but remember it's length
   axislength=lib_fp_sqrt(normalizevector(axisofrotation));
   
   
   if (axislength>100L)
      {
      // get the angle of rotation between the two vectors
      fixedpointnum angle=lib_fp_atan2(axislength, vectordotproduct(rotatedwestvector, desiredwestvector));
      angle=lib_fp_multiply(vectordotproduct(axisofrotation, desireddownvector), angle);
   
      angleerror[YAWINDEX]=angle;
      }
   else angleerror[YAWINDEX]=0;
      
   lastyawerror=angleerror[YAWINDEX];
   }
   
#else // complex control is slower, but offers better control.

#define FIXEDPOINTONEOVER500 (FIXEDPOINTONE/500L)

fixedpointnum desiredwestvector[3]={FIXEDPOINTONE,0,0};
fixedpointnum desireddownvector[3]={0,0,FIXEDPOINTONE};
fixedpointnum lastyawerror=0;
fixedpointnum lastpitcherror=0;

void resetpilotcontrol()
   { // called when switching from navigation control to pilot control or when idling on the ground.
   // keeps us from accumulating yaw error that we can't correct.
   for (int x=0;x<3;++x)
      desiredwestvector[x]=global.estimatedwestvector[x];
   }

//#define MAXANGLEM
void getangleerrorfrompilotinput(fixedpointnum *angleerror)
   {
   // first, how much is our desired down vector rolled?
   fixedpointnum angle;
   fixedpointnum currentrollangle  =  lib_fp_atan2(desireddownvector[XINDEX] , desireddownvector[ZINDEX]) ;
   
   // how far is this from where the pilot wants to be (roll angle)
   fixedpointnum desiredangle=lib_fp_multiply(global.rxvalues[ROLLINDEX]-FPRXMIDPOINT,DESIREDANGLEMULTIPLIER);

   angle=desiredangle-currentrollangle;

   // rotate the disired downvector by this angle about the roll axis
   fixedpointnum sineofangle=lib_fp_sine(angle);
   fixedpointnum cosineofangle=lib_fp_cosine(angle);

   fixedpointnum vector[3];
   fixedpointnum vector2[3];
   
   vector[XINDEX]=lib_fp_multiply(desireddownvector[XINDEX], cosineofangle)+lib_fp_multiply(desireddownvector[ZINDEX], sineofangle);
   vector[YINDEX]=desireddownvector[YINDEX];
   vector[ZINDEX]=lib_fp_multiply(desireddownvector[ZINDEX], cosineofangle)-lib_fp_multiply(desireddownvector[XINDEX], sineofangle);

   vector2[XINDEX]=lib_fp_multiply(desiredwestvector[XINDEX], cosineofangle)+lib_fp_multiply(desiredwestvector[ZINDEX], sineofangle);
   vector2[YINDEX]=desiredwestvector[YINDEX];
   vector2[ZINDEX]=lib_fp_multiply(desiredwestvector[ZINDEX], cosineofangle)-lib_fp_multiply(desiredwestvector[XINDEX], sineofangle);

   // 2nd, how much is our desired down vector pitched?
   angle  =  lib_fp_atan2(desireddownvector[YINDEX] , desireddownvector[ZINDEX]) ;
   desiredangle = lib_fp_multiply(global.rxvalues[PITCHINDEX]-FPRXMIDPOINT,DESIREDANGLEMULTIPLIER);

   fixedpointnum desiredangle2=angle;
//   if ((lastpitcherror<MAXANGLEERROR && desiredangle2>0) || (lastpitcherror>-MAXANGLEERROR && desiredangle2<0))
      desiredangle2+=lib_fp_multiply(lib_fp_multiply(global.rxvalues[PITCHINDEX]-FPRXMIDPOINT,usersettings.maxyawrate*ROTATIONRATEMULTIPLIER),global.timesliver)>>TIMESLIVEREXTRASHIFT;
   
   // mix the desired angles based on how far we are rolled.
   fixedpointnum fraction=lib_fp_multiply(lib_fp_abs(currentrollangle),FIXEDPOINTONE/MAXDESIREDANGLE);
   desiredangle=lib_fp_multiply(fraction,desiredangle2)+lib_fp_multiply(FIXEDPOINTONE-fraction,desiredangle);
   
   angle=desiredangle-angle;

   sineofangle=lib_fp_sine(angle);
   cosineofangle=lib_fp_cosine(angle);
   
   // rotate the desired downvector by this angle about the pitch axis
   desireddownvector[XINDEX]=vector[XINDEX];
   desireddownvector[YINDEX]=lib_fp_multiply(vector[YINDEX], cosineofangle)+lib_fp_multiply(vector[ZINDEX], sineofangle);
   desireddownvector[ZINDEX]=lib_fp_multiply(vector[ZINDEX], cosineofangle)-lib_fp_multiply(vector[YINDEX], sineofangle);

   desiredwestvector[XINDEX]=vector2[XINDEX];
   desiredwestvector[YINDEX]=lib_fp_multiply(vector2[YINDEX], cosineofangle)+lib_fp_multiply(vector2[ZINDEX], sineofangle);
   desiredwestvector[ZINDEX]=lib_fp_multiply(vector2[ZINDEX], cosineofangle)-lib_fp_multiply(vector2[YINDEX], sineofangle);
   
   // don't need to do this every loop
   normalizevector(desireddownvector);
   
   // create a unit vector in the direction the r/c pilot would like down to point
//   fixedpointnum desireddownvector[3];

   // get desired pitch and roll angles from the rx inputs
   fixedpointnum desiredrollangle=lib_fp_multiply(global.rxvalues[ROLLINDEX]-FPRXMIDPOINT,DESIREDANGLEMULTIPLIER);
   fixedpointnum desiredpitchangle=lib_fp_multiply(global.rxvalues[PITCHINDEX]-FPRXMIDPOINT,DESIREDANGLEMULTIPLIER);
   
   // rotate a unit vector around the pitch and roll axes
   fixedpointnum sineofrollangle=lib_fp_sine(desiredrollangle);
   fixedpointnum cosineofrollangle=lib_fp_cosine(desiredrollangle);
   fixedpointnum sineofpitchangle=lib_fp_sine(desiredpitchangle);
   fixedpointnum cosineofpitchangle=lib_fp_cosine(desiredpitchangle);
   
   desireddownvector[0]=lib_fp_multiply(sineofrollangle,cosineofpitchangle);
   desireddownvector[1]=lib_fp_multiply(sineofpitchangle,cosineofrollangle);
   desireddownvector[2]=lib_fp_multiply(cosineofpitchangle,cosineofrollangle);
   
   if (global.rxvalues[THROTTLEINDEX]<FPARMEDMINMOTOROUTPUT)
      { // we probably aren't off the ground.
      // don't accumulate error.  Use a low pass filter to bleed the error off slowly so we don't get abrupt motion if we are still in the air
      // don't drift while sitting on the ground
      for (int x=0;x<3;++x)
         desiredwestvector[x]=global.estimatedwestvector[x];
      }
   else
      {
      // rotate the desired west vector around the desired down vector according to the pilot's input
      angle=lib_fp_multiply(lib_fp_multiply(global.rxvalues[YAWINDEX]-FPRXMIDPOINT,usersettings.maxyawrate*ROTATIONRATEMULTIPLIER),global.timesliver)>>TIMESLIVEREXTRASHIFT;

      // don't rotate if we are going to accumulate too much yaw error
      if ((lastyawerror<MAXANGLEERROR && angle>0) || (lastyawerror>-MAXANGLEERROR && angle<0))
         {
         // rotate the desiredwestvector by angle
         rotatevectorbyaxissmallangle(desiredwestvector,desireddownvector,angle);
         }
      }

   
   // keep desiredwestvector perpendicular to the desireddownvector
   // cross west with down to get north.  Cross down with north to get west, but now west will be perpendicular to down.

   vectorcrossproduct(desiredwestvector, desireddownvector,vector);
   if (vector[0]!=0 || vector[1]!=0 || vector[2]!=0)
      {
      vectorcrossproduct(desireddownvector,vector, desiredwestvector);
      normalizevector(desiredwestvector);
      }
   
   // find the axis of rotation and angle from our current down vector to the desired one
   fixedpointnum axisofrotation[3];
   vectorcrossproduct(global.estimateddownvector, desireddownvector, axisofrotation);

   // normalize the axis of rotation, but remember it's length
   fixedpointnum axislength=lib_fp_sqrt(normalizevector(axisofrotation));
   
   fixedpointnum rotatedwestvector[3];

   if (axislength>100L)
      {
      // get the angle of rotation between the two vectors
      fixedpointnum angle=lib_fp_atan2(axislength, vectordotproduct(global.estimateddownvector, desireddownvector));

      // calculate the roll and pitch errors
      // this is the same as a dot product of {0,1,0} and axisofrotation multiplied by angle
      angleerror[ROLLINDEX]=lib_fp_multiply(axisofrotation[1], angle);
   
      // this is the same as a dot product of {1,0,0} and axisofrotation multiplied by angle
      angleerror[PITCHINDEX]=-lib_fp_multiply(axisofrotation[0], angle);

      // rotate our current west vector by the axis and angle calculated above that's needed to get the down vectors aligned
      rotatevectorbyaxisangle(global.estimatedwestvector,axisofrotation,angle,rotatedwestvector);
      }
   else
      {
      angleerror[ROLLINDEX]=0;
      angleerror[PITCHINDEX]=0;
      for (int x=0;x<3;++x) rotatedwestvector[x]=global.estimatedwestvector[x];
      }
   
   // calculate how much more we need to rotate to get the westvectors aligned
   vectorcrossproduct(rotatedwestvector, desiredwestvector, axisofrotation);

   // normalize the axis of rotation, but remember it's length
   axislength=lib_fp_sqrt(normalizevector(axisofrotation));
   
   
   if (axislength>100L)
      {
      // get the angle of rotation between the two vectors
      fixedpointnum angle=lib_fp_atan2(axislength, vectordotproduct(rotatedwestvector, desiredwestvector));
      angle=lib_fp_multiply(vectordotproduct(axisofrotation, desireddownvector), angle);
   
      angleerror[YAWINDEX]=angle;
      }
   else angleerror[YAWINDEX]=0;
      
   lastyawerror=angleerror[YAWINDEX];
   lastpitcherror=angleerror[PITCHINDEX];
   }
   
*/

