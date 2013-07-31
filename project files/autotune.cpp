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
int autotuneindex=ROLLINDEX;
fixedpointnum autotunetime;
fixedpointnum autotunepeak1;
fixedpointnum autotunepeak2;
fixedpointnum targetangle=0;
fixedpointnum targetangleatpeak;
fixedpointnum currentpvalueshifted;
fixedpointnum currentivalueshifted;
fixedpointnum currentdvalueshifted;

char cyclecount=1;

void autotune(fixedpointnum *angleerror,unsigned char startingorstopping)
   {
   if (!global.armed)
      {
      // we aren't armed.  Don't do anything, but if autotuning is started and we have collected
      // autotuning data, save our settings to eeprom
      if (startingorstopping==AUTOTUNESTARTING && targetangle!=0)
         writeusersettingstoeeprom();

      return;
      }
      
   if (startingorstopping==AUTOTUNESTOPPING)
      {
      usersettings.pid_igain[autotuneindex]=currentivalueshifted>>AUTOTUNESHIFT;
      
      // multiply by D multiplier.  The best D is usually a little higher than what the algroithm produces.
      usersettings.pid_dgain[autotuneindex]=lib_fp_multiply(currentdvalueshifted,FPAUTOTUNE_D_MULTIPLIER)>>AUTOTUNESHIFT;
      
      usersettings.pid_igain[YAWINDEX]=usersettings.pid_igain[ROLLINDEX];
      usersettings.pid_dgain[YAWINDEX]=usersettings.pid_dgain[ROLLINDEX];
      usersettings.pid_pgain[YAWINDEX]=lib_fp_multiply(usersettings.pid_pgain[ROLLINDEX],YAWGAINMULTIPLIER);

      autotuneindex=!autotuneindex; // alternate between roll and pitch
      return;
      }

   if (startingorstopping==AUTOTUNESTARTING)
      {
      currentpvalueshifted=usersettings.pid_pgain[autotuneindex]<<AUTOTUNESHIFT;
      currentivalueshifted=usersettings.pid_igain[autotuneindex]<<AUTOTUNESHIFT;
      // divide by D multiplier to get our working value.  We'll multiply by D multiplier when we are done.
      usersettings.pid_dgain[autotuneindex]=lib_fp_multiply(usersettings.pid_dgain[autotuneindex],FPONEOVERAUTOTUNE_D_MULTIPLIER);
      currentdvalueshifted=usersettings.pid_dgain[autotuneindex]<<AUTOTUNESHIFT;
      
      usersettings.pid_igain[autotuneindex]=0;
      cyclecount=1;
      autotunepeak1=autotunepeak2=0;
      rising=0;
      }
   else
      { // we are autotuning.  Analyze our current data.
      fixedpointnum currentangle;

      if (rising)
         {
         currentangle=global.currentestimatedeulerattitude[autotuneindex];
         }
      else
         {
         // convert the numbers so it looks as if we are working with a positive target
         targetangle=-targetangle;
         currentangle=-global.currentestimatedeulerattitude[autotuneindex];
         }
      
      if (autotunepeak2==0)
         { // we haven't seen the first peak yet
         // The peak will be when our angular velocity is negative.  To be sure we are in the right place,
         // we also check to make sure our angle position is greater than zero.
         if (currentangle>autotunepeak1)
            { // we are still going up
            autotunepeak1=currentangle;
            targetangleatpeak=targetangle;
            }
         else if (autotunepeak1>0) // we have changed direction.  We have seen the first peak.
            {
            if (cyclecount==0) // we are checking the I value
               {
               // when checking the I value, we would like to overshoot the target position by half of the max oscillation.
               if (currentangle-targetangle<(FPAUTOTUNEMAXOSCILLATION>>1))
                  currentivalueshifted=lib_fp_multiply(currentivalueshifted,AUTOTUNEINCREASEMULTIPLIER);
               else
                  {
                  currentivalueshifted=lib_fp_multiply(currentivalueshifted,AUTOTUNEDECREASEMULTIPLIER);
                  if (currentivalueshifted<AUTOTUNEMINIMUMIVALUE) currentivalueshifted=AUTOTUNEMINIMUMIVALUE;
                  }

               // go back to checking P and D
               cyclecount=1;
               rising=!rising;
               usersettings.pid_igain[autotuneindex]=0;
               autotunepeak1=autotunepeak2=0;
               }
            else // we are checking P and D values
               { // get set up to look for the 2nd peak
               autotunepeak2=currentangle;
               autotunetime=0;
               }
            }
         }
      else // we saw the first peak. looking for the second
         {
         autotunetime+=global.timesliver;
         
         if (currentangle<autotunepeak2)
            autotunepeak2=currentangle;
            
         fixedpointnum oscillationamplitude=autotunepeak1-autotunepeak2;
     
         // stop looking for the 2nd peak if we time out or if we change direction again after moving by more than half the maximum oscillation
         if (autotunetime>AUTOTUNESETTLINGTIME || (oscillationamplitude>(FPAUTOTUNEMAXOSCILLATION>>1) && currentangle>autotunepeak2))
            {
            // analyze the data
            // Our goal is to have zero overshoot and to have AUTOTUNEMAXOSCILLATION amplitude
            if (autotunepeak1>targetangleatpeak) // overshoot
               {
               // by removing the if and else, we tend to push toward a higher gain solution in the long run
   //            if (oscillationamplitude>FPAUTOTUNEMAXOSCILLATION) // we have too much oscillation, so we can't increase D, so decrease P
                  currentpvalueshifted=lib_fp_multiply(currentpvalueshifted, AUTOTUNEDECREASEMULTIPLIER);
  //             else // we don't have too much oscillation, so we can increase D
                  currentdvalueshifted=lib_fp_multiply(currentdvalueshifted, AUTOTUNEINCREASEMULTIPLIER);
               }
            else // undershoot
               {
               if (oscillationamplitude>FPAUTOTUNEMAXOSCILLATION) // we have too much oscillation, so we should lower D
                  currentdvalueshifted=lib_fp_multiply(currentdvalueshifted, AUTOTUNEDECREASEMULTIPLIER);
               else // we don't have too much oscillation, so we increase P
                  currentpvalueshifted=lib_fp_multiply(currentpvalueshifted, AUTOTUNEINCREASEMULTIPLIER);
               }
               
            usersettings.pid_pgain[autotuneindex]=currentpvalueshifted>>AUTOTUNESHIFT;
            usersettings.pid_dgain[autotuneindex]=currentdvalueshifted>>AUTOTUNESHIFT;
            
            // switch to the other direction and start a new cycle
            rising=!rising;
            autotunepeak1=autotunepeak2=0;
         
            if (++cyclecount==3)
               { // switch to testing I value
               cyclecount=0;
            
               usersettings.pid_igain[autotuneindex]=currentivalueshifted>>AUTOTUNESHIFT;
               }
            }
         }
      }
   
   if (rising) targetangle=/*global.rxvalues[autotuneindex]*20L+*/FPAUTOTUNETARGETANGLE;
   else targetangle=/*global.rxvalues[autotuneindex]*20L*/-FPAUTOTUNETARGETANGLE;
   
   // override the angle error for the axis we are tuning
   angleerror[autotuneindex]=targetangle-global.currentestimatedeulerattitude[autotuneindex];
   }

#endif