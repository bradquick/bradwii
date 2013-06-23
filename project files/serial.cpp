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

 
#include <stdlib.h>
#include <avr/io.h>

#include <string.h>
#include "lib_serial.h"
#include "lib_timers.h"
#include "lib_fp.h"

#include "bradwii.h"
#include "serial.h"
#include "defs.h"
#include "checkboxes.h"
#include "compass.h"
#include "eeprom.h"
#include "imu.h"
#include "gps.h"

#define MSP_VERSION 0
#define  VERSION  10


extern globalstruct global;
extern usersettingsstruct usersettings;

extern const char checkboxnames[];

void serialinit()
   {
#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT0)
   lib_serial_initport(0,SERIAL_0_BAUD);
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT1)
   lib_serial_initport(1,SERIAL_1_BAUD);
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT2)
   lib_serial_initport(2,SERIAL_2_BAUD);
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT3)
   lib_serial_initport(3,SERIAL_3_BAUD);
   lib_serial_sendstring(3,"test\n\r");
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORTUSB)
   lib_serial_initport(USBPORTNUMBER,0);
#endif
   }

#if (MULTIWII_CONFIG_SERIAL_PORTS!=NOSERIALPORT)
#define SERIALSTATEIDLE 0
#define SERIALSTATEGOTDOLLARSIGN 1
#define SERIALSTATEGOTM 2
#define SERIALSTATEGOTLESSTHANSIGN 3
#define SERIALSTATEGOTDATASIZE 4
#define SERIALSTATEGOTCOMMAND 5
#define SERIALSTAGEGOTPAYLOAD 6

// datagram format is $M<[data size][command][data...][checksum]
// response format is $M>[data size][command][data...][checksum]
//                 or $M![data size][command][data...][checksum] on error
unsigned char serialreceivestate[5]={0};
unsigned char serialcommand[5];
unsigned char serialdatasize[5];
unsigned char serialchecksum[5];

void sendandchecksumcharacter(char portnumber,unsigned char c)
   {
   lib_serial_sendchar(portnumber,c);
   serialchecksum[portnumber]^=c;
   }
   
void sendandchecksumdata(char portnumber,unsigned char *data,char length)
   {
   for (int x=0;x<length;++x)
      sendandchecksumcharacter(portnumber,data[x]);
   }
   
void sendandchecksumint(char portnumber,unsigned int value)
   {
   sendandchecksumdata(portnumber,(unsigned char *)&value,2);
   }
   
void sendandchecksumlong(char portnumber,unsigned long value)
   {
   sendandchecksumdata(portnumber,(unsigned char *)&value,4);
   }
   
void sendgoodheader(char portnumber,unsigned char size)
   {
   lib_serial_sendchar(portnumber,'$');
   lib_serial_sendchar(portnumber,'M');
   lib_serial_sendchar(portnumber,'>');
   lib_serial_sendchar(portnumber,size);
   serialchecksum[portnumber]=size;
   sendandchecksumcharacter(portnumber,serialcommand[portnumber]);
   }
   
void senderrorheader(char portnumber)
   {
   lib_serial_sendchar(portnumber,'$');
   lib_serial_sendchar(portnumber,'M');
   lib_serial_sendchar(portnumber,'!');
   lib_serial_sendchar(portnumber,0);
   serialchecksum[portnumber]=0;
   sendandchecksumcharacter(portnumber,serialcommand[portnumber]);
   }
   
void evaluatecommand(char portnumber,unsigned char *data)
   {
   unsigned char command=serialcommand[portnumber];
   if (command==MSP_IDENT)
      { // send rx data
      sendgoodheader(portnumber,7);
      sendandchecksumcharacter(portnumber,VERSION);
      sendandchecksumcharacter(portnumber,AIRCRAFT_CONFIGURATION);
      sendandchecksumcharacter(portnumber,MSP_VERSION);
      for (int x=0;x<4;++x) sendandchecksumcharacter(portnumber,0); // 32 bit "capability"
      }
   else if (command==MSP_RC)
      { // send rx data
      sendgoodheader(portnumber,16);
      for (int x=0;x<8;++x)
         {
         int value=0;
         if (x<RXNUMCHANNELS) value=((global.rxvalues[x]*500L)>>FIXEDPOINTSHIFT)+1500;

         sendandchecksumdata(portnumber,(unsigned char *)&value,2);
         }
      }
   else if (command==MSP_ATTITUDE)
      { // send attitude data
      sendgoodheader(portnumber,6);
      // convert our estimated gravity vector into roll and pitch angles
      int value;
      value=(global.currentestimatedeulerattitude[0]*10)>>FIXEDPOINTSHIFT;
      sendandchecksumdata(portnumber,(unsigned char *)&value,2);
      value=(global.currentestimatedeulerattitude[1]*10)>>FIXEDPOINTSHIFT;
      sendandchecksumdata(portnumber,(unsigned char *)&value,2);
      value=(global.currentestimatedeulerattitude[2])>>FIXEDPOINTSHIFT;
      sendandchecksumdata(portnumber,(unsigned char *)&value,2);
      }
   else if (command==MSP_ALTITUDE)
      { // send attitude data
      sendgoodheader(portnumber,4);
      fixedpointnum fp=(global.altitude*25)>>(FIXEDPOINTSHIFT-2);
      sendandchecksumdata(portnumber,(unsigned char *)&fp,4);
      }
   else if (command==MSP_MAG_CALIBRATION)
      { // send attitude data
      if (!global.armed) calibratecompass();
      sendgoodheader(portnumber,0);
      }
   else if (command==MSP_ACC_CALIBRATION)
      { // send attitude data
      if (!global.armed) calibrategyroandaccelerometer();
      sendgoodheader(portnumber,0);
      }

   else if (command==MSP_RAW_IMU)
      { // send attitude data
      sendgoodheader(portnumber,18);
      for (int x=0;x<3;++x)
         { // convert from g's to what multiwii uses
         int value=global.acc_g_vector[x]>>8;
         sendandchecksumdata(portnumber,(unsigned char *)&value,2);
         }
      for (int x=0;x<3;++x)
         { // convert from degrees per second to /8000
         int value=(global.gyrorate[x])>>14; // this is aproximate
         sendandchecksumdata(portnumber,(unsigned char *)&value,2);
         }
      for (int x=0;x<3;++x)
         { // convert from
         int value=(global.compassvector[x])>>8; // no particular units
//         int value=global.compassrawvalue[x]; // mag values go here
         sendandchecksumdata(portnumber,(unsigned char *)&value,2);
         }
      }
   else if (command==MSP_STATUS)
      { // send attitude data
      sendgoodheader(portnumber,10);
      sendandchecksumint(portnumber,(global.timesliver*15)>>8); // convert from fixedpointnum to microseconds
      sendandchecksumint(portnumber,0); // i2c error count
      sendandchecksumint(portnumber,0); // baro mag, gps, sonar
      sendandchecksumdata(portnumber,(unsigned char *)&global.activecheckboxitems,4); // options1
      }
   else if (command==MSP_MOTOR)
      { // send motor output data
      sendgoodheader(portnumber,16);
      for (int x=0;x<8;++x)
         {
         if (x < NUMMOTORS)
            sendandchecksumint(portnumber,global.motoroutputvalue[x]); // current motor value
         else
            sendandchecksumint(portnumber,0);
         }
      }
   else if (command==MSP_PID)
      { // send pid data
      sendgoodheader(portnumber,3*NUMPIDITEMS);
      for (int x=0;x<NUMPIDITEMS;++x)
         {
         if (x==ALTITUDEINDEX)
            sendandchecksumcharacter(portnumber,usersettings.pid_pgain[x]>>7);
         else if (x==NAVIGATIONINDEX)
            sendandchecksumcharacter(portnumber,usersettings.pid_pgain[x]>>11);
         else
            sendandchecksumcharacter(portnumber,usersettings.pid_pgain[x]>>3);
         sendandchecksumcharacter(portnumber,usersettings.pid_igain[x]);
         if (x==NAVIGATIONINDEX)
            sendandchecksumcharacter(portnumber,usersettings.pid_dgain[x]>>8);
         else if (x==ALTITUDEINDEX)
            sendandchecksumcharacter(portnumber,usersettings.pid_dgain[x]>>9);
         else
            sendandchecksumcharacter(portnumber,usersettings.pid_dgain[x]>>2);
         }
      }
   else if (command==MSP_SET_PID)
      {
      for (int x=0;x<NUMPIDITEMS;++x)
         {
         if (x==ALTITUDEINDEX)
            usersettings.pid_pgain[x]=((fixedpointnum)(*data++))<<7;
         else if (x==NAVIGATIONINDEX)
            usersettings.pid_pgain[x]=((fixedpointnum)(*data++))<<11;
         else
            usersettings.pid_pgain[x]=((fixedpointnum)(*data++))<<3;
         usersettings.pid_igain[x]=((fixedpointnum)(*data++));
         if (x==NAVIGATIONINDEX)
            usersettings.pid_dgain[x]=((fixedpointnum)(*data++))<<8;
         else if (x==ALTITUDEINDEX)
            usersettings.pid_dgain[x]=((fixedpointnum)(*data++))<<9;
         else
            usersettings.pid_dgain[x]=((fixedpointnum)(*data++))<<2;

         }
// while testing, make roll pid equal to pitch pid so I only have to change one thing.
//usersettings.pid_pgain[ROLLINDEX]=usersettings.pid_pgain[PITCHINDEX];
//usersettings.pid_igain[ROLLINDEX]=usersettings.pid_igain[PITCHINDEX];
//usersettings.pid_dgain[ROLLINDEX]=usersettings.pid_dgain[PITCHINDEX];
      sendgoodheader(portnumber,0);
      }
   else if (command==MSP_DEBUG)
      { // send attitude data
      sendgoodheader(portnumber,8);
      for (int x=0;x<4;++x)
         { // convert from g's to what multiwii uses
         int value=global.debugvalue[x];
         sendandchecksumdata(portnumber,(unsigned char *)&value,2);
         }
      }
   else if (command==MSP_BOXNAMES)
      { // send names of checkboxes
      char length=strlen(checkboxnames);
      sendgoodheader(portnumber,length);
      sendandchecksumdata(portnumber,(unsigned char *)checkboxnames,length);
      }
   else if (command==MSP_SET_BOX)
      { // receive check box settings
      unsigned char *ptr=(unsigned char *)usersettings.checkboxconfiguration;
      for (int x=0;x<NUMCHECKBOXES*2;++x)
         {
         *ptr++=*data++;
         }
      }
   else if (command==MSP_BOX)
      { // send check box settings
      sendgoodheader(portnumber,NUMCHECKBOXES*2);
      sendandchecksumdata(portnumber,(unsigned char *)usersettings.checkboxconfiguration,NUMCHECKBOXES*2);
      }
   else if (command==MSP_RESET_CONF)
      { // reset user settings
      sendgoodheader(portnumber,0);
      defaultusersettings();
      }
   else if (command==MSP_EEPROM_WRITE)
      { // reset user settings
      sendgoodheader(portnumber,0);
      if (!global.armed) writeusersettingstoeeprom();
      }
   else if (command==MSP_RAW_GPS)
      { // reset user settings
      sendgoodheader(portnumber,14);
      sendandchecksumcharacter(portnumber,0); // gps fix
      sendandchecksumcharacter(portnumber, global.gps_num_satelites);
      sendandchecksumlong(portnumber,lib_fp_multiply(global.gps_current_latitude,156250L)); //156250L is 10,000,000L>>LATLONGEXTRASHIFT); 
      sendandchecksumlong(portnumber,lib_fp_multiply(global.gps_current_longitude,156250L)); 
      sendandchecksumint(portnumber,global.gps_current_altitude>>FIXEDPOINTSHIFT); // gps altitude
      sendandchecksumint(portnumber,(global.gps_current_speed*100)>>FIXEDPOINTSHIFT); // gps speed
      }
   else if (command==MSP_COMP_GPS)
      { // reset user settings
      sendgoodheader(portnumber,5);
      sendandchecksumint(portnumber,(global.navigation_distance)>>FIXEDPOINTSHIFT); 
      sendandchecksumint(portnumber,(global.navigation_bearing)>>FIXEDPOINTSHIFT); 
      sendandchecksumcharacter(portnumber,0); // gps update
      }
   else if (command==MSP_RC_TUNING)
      { // user settings
      sendgoodheader(portnumber,7);
      sendandchecksumcharacter(portnumber,0); // rcRate
      sendandchecksumcharacter(portnumber,0); // rcExpo
      sendandchecksumcharacter(portnumber,usersettings.maxpitchandrollrate>>(FIXEDPOINTSHIFT+3)); // rollPitchRate
      sendandchecksumcharacter(portnumber,usersettings.maxyawrate>>(FIXEDPOINTSHIFT+2)); // yawRate
      sendandchecksumcharacter(portnumber,0); // dynThrPID
      sendandchecksumcharacter(portnumber,0); // thrMid8
      sendandchecksumcharacter(portnumber,0); // thrExpo8
      }
   else if (command==MSP_SET_RC_TUNING)
      { // user settings
      data++; //rcRate
      data++; //rcExpo
      usersettings.maxpitchandrollrate=((fixedpointnum)(*data++))<<(FIXEDPOINTSHIFT+3); // rollPitchRate
      usersettings.maxyawrate=((fixedpointnum)(*data++))<<(FIXEDPOINTSHIFT+2); // yawRate
      data++; // dynThrPID
      data++; // thrMid8
      data++; // thrExpo8
      sendgoodheader(portnumber,0);
      }

   else // we don't know this command
      {
      senderrorheader(portnumber);
      }
   lib_serial_sendchar(portnumber,serialchecksum[portnumber]);
   }

void serialcheckportforaction(char portnumber)
   {
   int numcharsavailable;
   while (numcharsavailable=lib_serial_numcharsavailable(portnumber))
      {
      if (serialreceivestate[portnumber]==SERIALSTATEGOTCOMMAND)
         {
         // this is the only state where we have to read more than one byte, so do this first, even though it's not first in the sequence of events
         if (numcharsavailable>serialdatasize[portnumber]) // we need to wait for data plus the checksum
            {
            unsigned char data[65];
            lib_serial_getdata(portnumber, data, serialdatasize[portnumber]+1);
            for (int x=0;x<serialdatasize[portnumber];++x)
               serialchecksum[portnumber]^=data[x];
            if (serialchecksum[portnumber]==data[serialdatasize[portnumber]])
               {
               evaluatecommand(portnumber,data);
               }
            serialreceivestate[portnumber]=SERIALSTATEIDLE;
            }
         }
      else
         {
         char c=lib_serial_getchar(portnumber);
         
         if (serialreceivestate[portnumber]==SERIALSTATEIDLE)
            {
            if (c=='$') serialreceivestate[portnumber]=SERIALSTATEGOTDOLLARSIGN;
            }
         else if (serialreceivestate[portnumber]==SERIALSTATEGOTDOLLARSIGN)
            {
            if (c=='M') serialreceivestate[portnumber]=SERIALSTATEGOTM;
            else serialreceivestate[portnumber]=SERIALSTATEIDLE;
            }
         else if (serialreceivestate[portnumber]==SERIALSTATEGOTM)
            {
            if (c=='<') serialreceivestate[portnumber]=SERIALSTATEGOTLESSTHANSIGN;
            else serialreceivestate[portnumber]=SERIALSTATEIDLE;
            }
         else if (serialreceivestate[portnumber]==SERIALSTATEGOTLESSTHANSIGN)
            {
            serialdatasize[portnumber]=c;
            serialchecksum[portnumber]=c;
            serialreceivestate[portnumber]=SERIALSTATEGOTDATASIZE;
            }
         else if (serialreceivestate[portnumber]==SERIALSTATEGOTDATASIZE)
            {
            serialcommand[portnumber]=c;
            serialchecksum[portnumber]^=c;
            serialreceivestate[portnumber]=SERIALSTATEGOTCOMMAND;
            }
         }
      }
   }

//#define SERIALTEXTDEBUG
#ifdef SERIALTEXTDEBUG
void serialprintnumber(char portnumber,long num,int digits,int decimals,char usebuffer)
   // prints a int number, right justified, using digits # of digits, puting a
   // decimal decimals places from the end, and using blank
   // to fill all blank spaces
   {
   char stg[12];
   char *ptr;
   int x;

   ptr=stg+11;
   
   *ptr='\0';
   if (num<0)
      {
      num=-num;
      *(--ptr)='-';
      }
   else
      *(--ptr)=' ';
      
   for (x=1;x<=digits;++x)
      {
      if (num==0)
         *(--ptr)=' ';
      else
         {
         *(--ptr)=48+num-(num/10)*10;
         num/=10;
         }
      if (x==decimals) *(--ptr)='.';
      }
   lib_serial_sendstring(portnumber,ptr);
   }

void serialprintfixedpoint(char portnumber,fixedpointnum fp)
   {
   serialprintnumber(portnumber,lib_fp_multiply(fp,1000),7,3,1);
   lib_serial_sendstring(portnumber,"\n\r");
   }
   
void serialcheckportforactiontest(char portnumber)
   {
   int numcharsavailable=lib_serial_numcharsavailable(portnumber);
   if (numcharsavailable)
      {
      char c=lib_serial_getchar(portnumber);
      lib_serial_sendstring(portnumber,"got char\n\r");
      
      if (c=='r')
         { // receiver values
         for (int x=0;x<6;++x)
            {
            serialprintfixedpoint(portnumber,global.rxvalues[x]);      
            }
         }
      else if (c=='g')
         { // gyro values
         for (int x=0;x<3;++x)
            {
            serialprintfixedpoint(portnumber,global.gyrorate[x]);
            }
         }
      else if (c=='a')
         { // acc g values
         for (int x=0;x<3;++x)
            {
            serialprintfixedpoint(portnumber,global.acc_g_vector[x]);
            }
         }
      else if (c=='t')
         { // atttude angle values
         for (int x=0;x<3;++x)
            {
            serialprintfixedpoint(portnumber,global.currentestimatedeulerattitude[x]);
            }
         }
      else if (c=='e')
         { // atttude angle values
         serialprintfixedpoint(portnumber,global.estimateddownvector[0]);
         serialprintfixedpoint(portnumber,global.estimateddownvector[1]);
         serialprintfixedpoint(portnumber,global.estimateddownvector[2]);
         serialprintfixedpoint(portnumber,global.estimatedwestvector[0]);
         serialprintfixedpoint(portnumber,global.estimatedwestvector[1]);
         serialprintfixedpoint(portnumber,global.estimatedwestvector[2]);
         }
      else if (c=='d')
         { // debug values
         for (int x=0;x<3;++x)
         serialprintfixedpoint(portnumber,global.debugvalue[x]);
         }
      else if (c=='l')
         { // altitude 
         serialprintfixedpoint(portnumber,global.altitude);
         }
      else if (c=='p')
         { // atttude angle values
         serialprintfixedpoint(portnumber,usersettings.pid_pgain[0]);
         serialprintfixedpoint(portnumber,usersettings.pid_igain[0]);
         serialprintfixedpoint(portnumber,usersettings.pid_dgain[0]);
         }
      lib_serial_sendstring(portnumber,"\n\r");
      }
   }
#endif
#endif

void serialcheckforaction()
   { // to be called by the main program every cycle so that we can check to see if we need to respond to incoming characters
#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT0)
   serialcheckportforaction(0);
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT1)
   serialcheckportforaction(1);
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT2)
   serialcheckportforaction(2);
//   serialcheckportforactiontest(2);
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT3)
#ifdef SERIALTEXTDEBUG
   serialcheckportforactiontest(3);
#else
   serialcheckportforaction(3);
#endif
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORTUSB)
   serialcheckportforaction(USBPORTNUMBER);
#endif
   
   }