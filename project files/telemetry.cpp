//
//  frskytelemetry.cpp
//  Project
//
//  Created by Brad on 1/11/15.
//
//

#include "telemetry.h"
#include "lib_serial.h"
#include "lib_timers.h"
#include "lib_fp.h"
#include "bradwii.h"
#include "gps.h"

extern globalstruct global;
extern usersettingsstruct usersettings;

// ****************************************************************
// FrSky telemetry
// based on code last worked on by:
// Hikari  www.stpchikari.com
// disq
// QuadBow
//
// Note: the FrSky receiver has inverted serial signals, so a hardware inverter is required
// ****************************************************************

unsigned long telemetrytimer;
unsigned char telemetrycount;

void inittelemetry()
   {
#ifdef TELEMETRY_SERIAL_PORT
   lib_serial_initport(TELEMETRY_SERIAL_PORT,9600);
   telemetrytimer=lib_timers_starttimer();
   telemetrycount=0;
#endif
   }

#ifdef TELEMETRY_SERIAL_PORT
void inline writefrsky8(unsigned char Data)
   {
   lib_serial_sendchar(TELEMETRY_SERIAL_PORT,Data);
   }

void inline writefrskystuffedbyte(unsigned char Data) //byte stuffing
   {
   if (Data == 0x5E)
      {
      writefrsky8(0x5D);
      writefrsky8(0x3E);
      }
   else if (Data == 0x5D)
      {
      writefrsky8(0x5D);
      writefrsky8(0x3D);
      }
   else
      {
      writefrsky8(Data);
      }
   }

//void inline writefrsky16(unsigned short Data)
//   {
//   writefrskystuffedbyte(Data & 0xff);
//   writefrskystuffedbyte(Data >> 8);
//   }

void inline sendDataHead(unsigned char Data_id)
   {
   writefrsky8(Protocol_Header);
   writefrsky8(Data_id);
   }

void inline sendDataTail(void)
   {
   writefrsky8(Protocol_Tail);
   }

void sendshortvalue(unsigned char id,short value)
   {
   sendDataHead(id);
   writefrskystuffedbyte(value & 0xff);
   writefrskystuffedbyte(value >> 8);
   }

void sendfixedpointnum(unsigned char bpId, unsigned char apId, fixedpointnum value,int resolution)
   {
   short bpVal;
   unsigned short apVal;
   
   bpVal = value>>FIXEDPOINTSHIFT; // value before the decimal point ("bp" is "before point")
   apVal = ((value & 0xFFFF) * resolution)>>FIXEDPOINTSHIFT; // value after the decimal point

   sendshortvalue(bpId,bpVal);
   sendshortvalue(apId,apVal);
   }

void sendlatitudeorlongitude(unsigned char bpId, unsigned char apId, fixedpointnum value)
   {
   // convert from decimal degrees to ddmm.mmmmm format
   fixedpointnum degrees=(value>>LATLONGEXTRASHIFT) & 0xFFFF0000;
   fixedpointnum minutes=((value - (degrees<<LATLONGEXTRASHIFT))*60L)>>LATLONGEXTRASHIFT;
   
   value=degrees*100+minutes;
//global.debugvalue[1]=degrees>>FIXEDPOINTSHIFT;
//global.debugvalue[2]=minutes>>FIXEDPOINTSHIFT;

   sendfixedpointnum(bpId,apId,value,10000);
   }


#endif


void checktelemetryforaction()
   {
#ifdef TELEMETRY_SERIAL_PORT
   if (lib_timers_gettimermicroseconds(telemetrytimer)>=250000L)
      {
      // Data sent every 250ms
      sendshortvalue(ID_Ang_X,global.currentestimatedeulerattitude[ROLLINDEX]>>FIXEDPOINTSHIFT);
      sendshortvalue(ID_Ang_Y,global.currentestimatedeulerattitude[PITCHINDEX]>>FIXEDPOINTSHIFT);
      
      // Data sent every 1s
      switch (++telemetrycount)
         {
         case 1:
            // send altitude
            sendfixedpointnum(ID_Altitude_bp,ID_Altitude_ap,global.altitude,100);
            
            // send the current direction we are facing in course, even though it's not our course
            sendfixedpointnum(ID_Course_bp,ID_Course_ap,global.currentestimatedeulerattitude[YAWINDEX],100); // zero after decimal point?
            
            // send the gps speed. Convert from meters per second to km/h
            sendfixedpointnum(ID_GPS_speed_bp,ID_GPS_speed_ap,lib_fp_multiply(global.gps_current_speed,FIXEDPOINTCONSTANT(3.6)),0); // zero after decimal point?
            break;
         case 2:
//LATLONGEXTRASHIFT
            // send gps position
            sendlatitudeorlongitude(ID_Longitude_bp,ID_Longitude_ap,lib_fp_abs(global.gps_current_longitude));
            sendshortvalue(ID_E_W,global.gps_current_longitude < 0 ? 'W' : 'E');
            sendlatitudeorlongitude(ID_Latitude_bp,ID_Latitude_ap,lib_fp_abs(global.gps_current_latitude));
            sendshortvalue(ID_N_S,global.gps_current_latitude < 0 ? 'S' : 'N');

            // send gps altitude
            sendfixedpointnum(ID_GPS_Altitude_bp, ID_GPS_Altitude_ap, global.gps_current_altitude,100); // zero after decimal point?
//            send_Voltage_ampere();
            break;
         case 3:
            // send acc info
            sendshortvalue(ID_Acc_X,global.acc_g_vector[XINDEX]>>6);
            sendshortvalue(ID_Acc_Y,global.acc_g_vector[YINDEX]>>6);
            sendshortvalue(ID_Acc_Z,global.acc_g_vector[ZINDEX]>>6);
            sendshortvalue(ID_Gyro_X,global.gyrorate[XINDEX]>>8); // not sure of the scaling or units
            sendshortvalue(ID_Gyro_X,global.gyrorate[YINDEX]>>8);
            sendshortvalue(ID_Gyro_X,global.gyrorate[ZINDEX]>>8);
            break;
         case 4:
            // send the number of satelites as RPM
            sendshortvalue(ID_RPM,global.gps_num_satelites);
            
            // send the distance in Temperature 2
            sendshortvalue(ID_Temperature2,global.navigation_distance>>FIXEDPOINTSHIFT);

            telemetrycount = 0; // start over
            break;
         }
      sendDataTail();

      // restart the timer
      telemetrytimer=lib_timers_starttimer();
      }
#endif
   }


