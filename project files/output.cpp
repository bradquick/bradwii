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

#include <avr/io.h>
#include "output.h"
#include "lib_pwm.h"
#include "projectsettings.h"
#include "bradwii.h"
#include "lib_timers.h"

extern globalstruct global;

void initoutputs()
   {
#ifdef USEPWM1
   lib_pwm_init1(PWM1PHASECORRECTMODE,PWM1NORMALOUTPUTA | PWM1NORMALOUTPUTB,PWM1PRESCALER1,0x3FFF); // TOP to 16383
#endif
#ifdef USEPWM2
   lib_pwm_init2(PWM2FASTMODE,PWM2NORMALOUTPUTA | PWM2NORMALOUTPUTB,PWM2PRESCALER64);
#endif
#ifdef USEPWM3
   lib_pwm_init3(PWM3PHASECORRECTMODE,PWM3NORMALOUTPUTA | PWM3NORMALOUTPUTB | PWM3NORMALOUTPUTC,PWM3PRESCALER1,0x3FFF); // TOP to 16383
#endif
#ifdef USEPWM4
   lib_pwm_init4(PWM4PHASECORRECTMODE,PWM4NORMALOUTPUTA | PWM4NORMALOUTPUTB | PWM4NORMALOUTPUTC,PWM4PRESCALER1,0x3FFF); // TOP to 16383
#endif
#ifdef USEPWM411BIT
   lib_pwm_init4(PWM411BITPHASECORRECTMODE,/*PWM411BITNORMALOUTPUTA | PWM411BITNORMALOUTPUTB |*/ PWM411BITNORMALOUTPUTD,PWM411BITPRESCALER16,0x7FF); // TOP to 2047
#endif
#ifdef USEPWM5
   lib_pwm_init5(PWM5PHASECORRECTMODE,PWM5NORMALOUTPUTA | PWM5NORMALOUTPUTB | PWM5NORMALOUTPUTC,PWM5PRESCALER1,0x3FFF); // TOP to 16383
#endif

   lib_digitalio_initpin(MOTOR_0_PIN, DIGITALOUTPUT);
   lib_digitalio_initpin(MOTOR_1_PIN, DIGITALOUTPUT);
   lib_digitalio_initpin(MOTOR_2_PIN, DIGITALOUTPUT);
#if (NUMMOTORS>3)
   lib_digitalio_initpin(MOTOR_3_PIN, DIGITALOUTPUT);
#endif
#if (NUMMOTORS>4)
   lib_digitalio_initpin(MOTOR_4_PIN, DIGITALOUTPUT);
   lib_digitalio_initpin(MOTOR_5_PIN, DIGITALOUTPUT);
#endif
   
   setallmotoroutputs(MIN_MOTOR_OUTPUT);
   
 /********  special version of MultiWii to calibrate all attached ESCs ************/
#if defined(ESC_CALIB_CANNOT_FLY)
    setallmotoroutputs(ESC_CALIB_HIGH);
    lib_timers_delaymilliseconds(3000);
    setallmotoroutputs(ESC_CALIB_LOW);

    while (1)
      {
      lib_timers_delaymilliseconds(500);
      lib_digitalio_setoutput(LED1_OUTPUT, 0);
      lib_timers_delaymilliseconds(500);
      lib_digitalio_setoutput(LED1_OUTPUT, 1);
      }
#endif
   }

void setmotoroutput(unsigned char motornum, unsigned char motorchannel,fixedpointnum fpvalue)
   { // set the output of a motor
   // convert from fixedpoint 0 to 1 into int 1000 to 2000
   int value=1000+((fpvalue*1000L)>>FIXEDPOINTSHIFT);
   
   if (!global.armed) value=MIN_MOTOR_OUTPUT;
   else if (value<ARMED_MIN_MOTOR_OUTPUT) value=ARMED_MIN_MOTOR_OUTPUT;
   if (value>MAX_MOTOR_OUTPUT) value=MAX_MOTOR_OUTPUT;
   global.motoroutputvalue[motornum]=value;
   setoutput(motorchannel,value);
   }
   
void setallmotoroutputs(int value)
   {
   setoutput(MOTOR_0_CHANNEL,value);
   setoutput(MOTOR_1_CHANNEL,value);
   setoutput(MOTOR_2_CHANNEL,value);
#if (NUMMOTORS>3)
   setoutput(MOTOR_3_CHANNEL,value);
#endif
#if (NUMMOTORS>4)
   setoutput(MOTOR_4_CHANNEL,value);
   setoutput(MOTOR_5_CHANNEL,value);
#endif
#if (NUMMOTORS>6)
   setoutput(MOTOR_6_CHANNEL,value);
   setoutput(MOTOR_7_CHANNEL,value);
#endif
   }
   
void setoutput(unsigned char outputchannel, unsigned int value)
   {
   unsigned char timernum=outputchannel & 0xF0;
   unsigned char timerchannel=outputchannel & 0x0F;
   
   unsigned int pwmvalue;
#ifndef EXT_MOTOR_RANGE 
   pwmvalue=value<<3;
#else
   pwmvalue=((value<<4) - 16000) + 128;
#endif

#ifdef USEPWM1
   if (timernum==OUTPUT_TIMER1)
      {
      if (timerchannel==OUTPUT_CHANNELA) lib_pwm_setduty1A(pwmvalue);
      else if (timerchannel==OUTPUT_CHANNELB) lib_pwm_setduty1B(pwmvalue);
      return;
      }
#endif
#ifdef USEPWM2
   if (timernum==OUTPUT_TIMER2)
      { // timer 2 only has an 8 bit range
#ifndef EXT_MOTOR_RANGE 
      pwmvalue=value>>3;
#else
      pwmvalue=((value>>2) - 250) + 2;
#endif
      if (timerchannel==OUTPUT_CHANNELA) lib_pwm_setduty2A(pwmvalue);
      else if (timerchannel==OUTPUT_CHANNELB) lib_pwm_setduty2B(pwmvalue);
      return;
      }
#endif
#ifdef USEPWM3
   if (timernum==OUTPUT_TIMER3)
      {
      if (timerchannel==OUTPUT_CHANNELA) lib_pwm_setduty3A(pwmvalue);
      else if (timerchannel==OUTPUT_CHANNELB) lib_pwm_setduty3B(pwmvalue);
      else if (timerchannel==OUTPUT_CHANNELC) lib_pwm_setduty3C(pwmvalue);
      return;
      }
#endif
#ifdef USEPWM4
   if (timernum==OUTPUT_TIMER4)
      {
      if (timerchannel==OUTPUT_CHANNELA) lib_pwm_setduty4A(pwmvalue);
      else if (timerchannel==OUTPUT_CHANNELB) lib_pwm_setduty4B(pwmvalue);
      else if (timerchannel==OUTPUT_CHANNELC) lib_pwm_setduty4C(pwmvalue);
      return;
      }
#endif
#ifdef USEPWM411BIT
#ifndef EXT_MOTOR_RANGE 
   pwmvalue=value;
#else
   pwmvalue=((value-1000)<<1)+16;
#endif
   if (timernum==OUTPUT_TIMER4)
      {
      if (timerchannel==OUTPUT_CHANNELA) lib_pwm_setduty4A(pwmvalue);
      else if (timerchannel==OUTPUT_CHANNELB) lib_pwm_setduty4B(pwmvalue);
      else if (timerchannel==OUTPUT_CHANNELD) lib_pwm_setduty4D(pwmvalue);
      return;
      }
#endif

#ifdef USEPWM5
   if (timernum==OUTPUT_TIMER5)
      {
      if (timerchannel==OUTPUT_CHANNELA) lib_pwm_setduty5A(pwmvalue);
      else if (timerchannel==OUTPUT_CHANNELB) lib_pwm_setduty5B(pwmvalue);
      else if (timerchannel==OUTPUT_CHANNELC) lib_pwm_setduty5C(pwmvalue);
      return;
      }
#endif
   }