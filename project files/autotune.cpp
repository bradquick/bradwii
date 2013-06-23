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

// The theory behind auto tuning is that we generate a step in the setpoint, then analyze how the system reacts.
// First, we set the I term to zero and work on only the P and D terms.
// The target angle alternates from negative to posive FPAUTOTUNETARGETANGLE degrees.  We set the target, then wait until
// we cross the zero angle, just to make sure our angular velocity is in the right direction, then we look for the peak
// which we recognize by observing the angular velocity switching direction.
// Our goal is to get zero overshoot.  By requiring zero overshoot, we can assume that any subsequent oscillations are
// caused by our D term, since at zero overshoot, the P term doesn't have any effect.  Once we see the first peak, we then
// start a timer and look for a second peak in the negative direction.  The difference between the first and second peak will
// tell us the effect the D term is having.  We will then try to keep this oscillation amplitude within a predeterrmined range.
//
// So, after we reach the target, then wait a little longer to record the peak oscillation, we have four choices:
// if (overshoot)
//      {
//      if (too much oscillation) decrease P
//      else increase D
//      }
//   else // undershoot
//      { 
//      if (too much oscillation) decrease D
//      else increase P
//      }


#include "config.h"
#include "bradwii.h"
#include "eeprom.h"
#include "autotune.h"

extern globalstruct global;
extern usersettingsstruct usersettings;

#ifndef NO_AUTOTUNE

#define FPAUTOTUNEMAXOSCILLATION FIXEDPOINTCONSTANT(AUTOTUNE_MAX_OSCILLATION)
#define FPAUTOTUNETARGETANGLE FIXEDPOINTCONSTANT(AUTOTUNE_TARGET_ANGLE)

unsigned char rising;
unsigned char sawfirstpeak;
int autotuneindex=ROLLINDEX;
fixedpointnum autotunetime;
fixedpointnum autotunepeak1;
fixedpointnum autotunepeak2;
fixedpointnum targetangle;
fixedpointnum targetangleatpeak;
fixedpointnum currentivalue;
fixedpointnum testvalue=0;

char cyclecount=1;

void autotune(fixedpointnum *angleerror,unsigned char startingorstopping)
   {
   if (!global.armed)
      {
      // we aren't armed.  Don't do anything, but if autotuning is started, save our settings to eeprom
      if (startingorstopping==AUTOTUNESTARTING)
         writeusersettingstoeeprom();

      return;
      }
      
   if (startingorstopping==AUTOTUNESTOPPING)
      {
      usersettings.pid_igain[autotuneindex]=currentivalue;
      
      usersettings.pid_igain[YAWINDEX]=usersettings.pid_igain[ROLLINDEX];
      usersettings.pid_dgain[YAWINDEX]=usersettings.pid_dgain[ROLLINDEX];
      usersettings.pid_pgain[YAWINDEX]=lib_fp_multiply(usersettings.pid_pgain[ROLLINDEX],YAWGAINMULTIPLIER);

      autotuneindex=!autotuneindex; // alternate between roll and pitch
      return;
      }

   if (startingorstopping==AUTOTUNESTARTING)
      {
      currentivalue=usersettings.pid_igain[autotuneindex];
      usersettings.pid_igain[autotuneindex]=0;
      cyclecount=1;
      sawfirstpeak=0;
      rising=0;
      targetangle=-FPAUTOTUNETARGETANGLE;
      }
   else
      {
      if (!sawfirstpeak)
         {
         if ((rising && global.currentestimatedeulerattitude[autotuneindex]>0 && global.gyrorate[autotuneindex]<0) // we have changed direction
            || (!rising && global.currentestimatedeulerattitude[autotuneindex]<0 && global.gyrorate[autotuneindex]>0))
            {
            if (cyclecount==0) // we are checking the I value
               {
               autotunepeak1=global.currentestimatedeulerattitude[autotuneindex]-targetangle;
               if (!rising) autotunepeak1=-autotunepeak1;

               if (autotunepeak1<(FPAUTOTUNEMAXOSCILLATION>>1))
                  currentivalue=lib_fp_multiply(currentivalue,AUTOTUNEINCREASEMULTIPLIER);
               else
                  {
                  currentivalue=lib_fp_multiply(currentivalue,AUTOTUNEDECREASEMULTIPLIER);
                  if (currentivalue<AUTOTUNEMINIMUMIVALUE) currentivalue=AUTOTUNEMINIMUMIVALUE;
                  }
               
               // go back to checking P and D
               cyclecount=1;
               rising=!rising;
               }
            else // we are checking P and D values
               {
               targetangleatpeak=targetangle;
               autotunepeak2=autotunepeak1=global.currentestimatedeulerattitude[autotuneindex];
         
               sawfirstpeak=1;
               autotunetime=0;
               usersettings.pid_igain[autotuneindex]=0;
               }
            }
         }
      else // we saw the first peak. looking for the second
         {
         if ((rising && global.currentestimatedeulerattitude[autotuneindex]<autotunepeak2)
            || (!rising && global.currentestimatedeulerattitude[autotuneindex]>autotunepeak2))
            autotunepeak2=global.currentestimatedeulerattitude[autotuneindex];
            
         autotunetime+=global.timesliver;
   
         if (autotunetime>AUTOTUNESETTLINGTIME)
            {
            if (!rising)
               {
               autotunepeak1=-autotunepeak1;
               autotunepeak2=-autotunepeak2;
               targetangleatpeak=-targetangleatpeak;
               }
         
            fixedpointnum oscillationamplitude=autotunepeak1-autotunepeak2;
         
            // analyze the data
            // Our goal is to have zero overshoot and to have AUTOTUNEMAXOSCILLATION amplitude
            if (autotunepeak1>targetangleatpeak) // overshoot
               {
               if (oscillationamplitude>FPAUTOTUNEMAXOSCILLATION) // we have too much oscillation, so we can't increase D, so decrease P
                  usersettings.pid_pgain[autotuneindex]=lib_fp_multiply(usersettings.pid_pgain[autotuneindex], AUTOTUNEDECREASEMULTIPLIER);
               else // we don't have too much oscillation, so we can increase D
                  usersettings.pid_dgain[autotuneindex]=lib_fp_multiply(usersettings.pid_dgain[autotuneindex], AUTOTUNEINCREASEMULTIPLIER);
               }
            else // undershoot
               {
               if (oscillationamplitude>FPAUTOTUNEMAXOSCILLATION) // we have too much oscillation, so we should lower D
                  usersettings.pid_dgain[autotuneindex]=lib_fp_multiply(usersettings.pid_dgain[autotuneindex], AUTOTUNEDECREASEMULTIPLIER);
               else // we don't have too much oscillation, so we increase P
                  usersettings.pid_pgain[autotuneindex]=lib_fp_multiply(usersettings.pid_pgain[autotuneindex], AUTOTUNEINCREASEMULTIPLIER);
               }
            
            // switch to the other direction and start a new cycle
            rising=!rising;
            sawfirstpeak=0;
         
            if (++cyclecount==3)
               { // switch to testing I value
               cyclecount=0;
            
               usersettings.pid_igain[autotuneindex]=currentivalue;
               }
            }
         }
      }
   
   if (rising) targetangle=global.rxvalues[autotuneindex]*20L+FPAUTOTUNETARGETANGLE;
   else targetangle=global.rxvalues[autotuneindex]*20L-FPAUTOTUNETARGETANGLE;
   
   // override the angle error for the axis we are tuning
   angleerror[autotuneindex]=targetangle-global.currentestimatedeulerattitude[autotuneindex];
   }

#endif