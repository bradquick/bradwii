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

#include "checkboxes.h"
#include "bradwii.h"
#include "rx.h"

extern globalstruct global;
extern usersettingsstruct usersettings;

char checkboxnames[]/* PROGMEM */= // names for dynamic generation of config GUI
   // this could be moved to program memory if we wanted to save a few bytes of space.
   "Arm;"
   "Thr. Helper;"
   "Alt. Hold;"
   "Mag. Hold;"
   "Pos. Hold;"
   "Ret. Home;"
   "Semi Acro;"
   "Full Acro;"
   "High Rates;"
   "High Angle;"
   "Auto Tune;"
   "Uncrashable;"
   "HF"
   ;

fixedpointnum slowrxvalues[RXNUMCHANNELS]={0};

// each checkbox item has a checkboxvalue.  The bits in this value represent low, medium, and high checkboxes
// for each of the aux switches, just as they show up in most config programs.
void checkcheckboxitems()
   {
   // use low pass filters to slow down the changes in radio signals so we don't respond to unintended glitches
   for (int x=0;x<RXNUMCHANNELS;++x)
      {
      lib_fp_lowpassfilter(&slowrxvalues[x],global.rxvalues[x],global.timesliver>>TIMESLIVEREXTRASHIFT,FIXEDPOINTONEOVERONESIXTEENTH,0);
      }
      
   global.previousactivecheckboxitems=global.activecheckboxitems;
   global.activecheckboxitems=0;
   
   unsigned int mask=0; // a mask of what aux states are true
#if (RXNUMCHANNELS>4)
   if (slowrxvalues[AUX1INDEX]<FPAUXMIDRANGELOW) // low
      mask |= (1<<0);
   else if (slowrxvalues[AUX1INDEX]>FPAUXMIDRANGEHIGH) // high
      mask |= (1<<2);
   else mask |=(1<<1); // mid
#endif

#if (RXNUMCHANNELS>5)
   if (slowrxvalues[AUX2INDEX]<FPAUXMIDRANGELOW) // low
      mask |= (1<<3);
   else if (slowrxvalues[AUX2INDEX]>FPAUXMIDRANGEHIGH) // high
      mask |= (1<<5);
   else mask |=(1<<4); //mid
#endif

#if (RXNUMCHANNELS>6)
   if (slowrxvalues[AUX3INDEX]<FPAUXMIDRANGELOW) // low
      mask |= (1<<6);
   else if (slowrxvalues[AUX3INDEX]>FPAUXMIDRANGEHIGH) // high
      mask |= (1<<8);
   else mask |=(1<<7); //mid
#endif

#if (RXNUMCHANNELS>7)
   if (slowrxvalues[AUX4INDEX]<FPAUXMIDRANGELOW) // low
      mask |= (1<<9);
   else if (slowrxvalues[AUX4INDEX]>FPAUXMIDRANGEHIGH) // high
      mask |= (1<<11);
   else mask |=(1<<10); //mid
#endif

   for (int x=0;x<NUMCHECKBOXES;++x)
      {
      if (usersettings.checkboxconfiguration[x] & mask) global.activecheckboxitems |= (1L<<x);
      }

#if (defined(STICK_ARM) | defined (STICK_DISARM))
   // figure out where the sticks are
   unsigned int stickmask=0;
   if (slowrxvalues[ROLLINDEX]<FPSTICKLOW) stickmask |= STICK_COMMAND_ROLL_LOW;
   else if (slowrxvalues[ROLLINDEX]>FPSTICKHIGH) stickmask |= STICK_COMMAND_ROLL_HIGH;

   if (slowrxvalues[PITCHINDEX]<FPSTICKLOW) stickmask |= STICK_COMMAND_PITCH_LOW;
   else if (slowrxvalues[PITCHINDEX]>FPSTICKHIGH) stickmask |= STICK_COMMAND_PITCH_HIGH;

   if (slowrxvalues[YAWINDEX]<FPSTICKLOW) stickmask |= STICK_COMMAND_YAW_LOW;
   else if (slowrxvalues[YAWINDEX]>FPSTICKHIGH) stickmask |= STICK_COMMAND_YAW_HIGH;


   // If the sticks are in the right positions, set the arm or disarm checkbox value
   // Start with the previous value in case the sticks aren't doing anything special
   global.activecheckboxitems=(global.activecheckboxitems & ~CHECKBOXMASKARM) | (global.previousactivecheckboxitems & CHECKBOXMASKARM);
   
   if ((stickmask & (STICK_ARM))==STICK_ARM) global.activecheckboxitems |= CHECKBOXMASKARM;
   
   else if ((stickmask & (STICK_DISARM))==STICK_DISARM) global.activecheckboxitems &= ~CHECKBOXMASKARM;
      
#endif
   }