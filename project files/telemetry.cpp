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

#if (TELEMETRYMODE==NOTELEMTRY)
void inittelemetry()
   {
   }

void checktelemetryforaction()
   {
   }
#endif

#if (TELEMETRYMODE==TELEMETRYFRSKYHUB)

unsigned long telemetrytimer;
unsigned char telemetrycount;

void inittelemetry()
   {
   lib_serial_initport(TELEMETRY_SERIAL_PORT,9600);
   telemetrytimer=lib_timers_starttimer();
   telemetrycount=0;
   }

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
   // convert from 10,000,000 degrees to ddmm.mmmmm format
   // degrees=value/10,000,000;
   // = value*10^-7
   // = value*(10^7<<23)>>23;
   long degrees=lib_fp_multiply(value,FIXEDPOINTCONSTANT(0.8388608))>>23;
   long remainder=value-degrees*10000000L;
   // convert remainder to 10,000 minutes
   // = remainder*60/1000;
   // = remainder *.06
   // = remainder * (.06<<11)>>11
   remainder=lib_fp_multiply(remainder,FIXEDPOINTCONSTANT(122.88))>>11;
   // minutes=remainder/10,000;
   // = remainder*(10^-4<<22)>>22;
   long minutes=lib_fp_multiply(remainder,FIXEDPOINTCONSTANT(419.4304))>>22;
   remainder=remainder-minutes*10000L;
   
   sendshortvalue(bpId,degrees*100+minutes);
   sendshortvalue(apId,remainder);
   }

void checktelemetryforaction()
   {
   if (lib_timers_gettimermicroseconds(telemetrytimer)>=250000L)
      {
      // Data sent every 250ms
      // send altitude velocity in cm/sec
      sendshortvalue(ID_Vario,(global.altitudevelocity*100L)>>FIXEDPOINTSHIFT);
      
      // Data sent every 1s
      switch (++telemetrycount)
         {
         case 1:
            // send altitude
            sendfixedpointnum(ID_Altitude_bp,ID_Altitude_ap,global.altitude,100);
            
            // send the current direction we are facing in course, even though it's not our course
            // convert from -180, 180 to 0, 360
            if (global.currentestimatedeulerattitude[YAWINDEX]<0)
               sendfixedpointnum(ID_Course_bp,ID_Course_ap,global.currentestimatedeulerattitude[YAWINDEX]+FIXEDPOINTCONSTANT(360.0),100);
            else
               sendfixedpointnum(ID_Course_bp,ID_Course_ap,global.currentestimatedeulerattitude[YAWINDEX],100);
            // send the gps speed. Convert from meters per second to knots
            sendfixedpointnum(ID_GPS_speed_bp,ID_GPS_speed_ap,lib_fp_multiply(global.gps_current_speed,FIXEDPOINTCONSTANT(1.94384)),0); // zero after decimal point?
            break;
         case 2:
            // send gps position
            sendlatitudeorlongitude(ID_Longitude_bp,ID_Longitude_ap,lib_fp_abs(global.gps_current_longitude));
            sendshortvalue(ID_E_W,global.gps_current_longitude < 0 ? 'W' : 'E');
            sendlatitudeorlongitude(ID_Latitude_bp,ID_Latitude_ap,lib_fp_abs(global.gps_current_latitude));
            sendshortvalue(ID_N_S,global.gps_current_latitude < 0 ? 'S' : 'N');

            // send gps altitude
            sendfixedpointnum(ID_GPS_Altitude_bp, ID_GPS_Altitude_ap, global.gps_current_altitude,100); // zero after decimal point?
            break;
         case 3:
            // send acc info
            sendshortvalue(ID_Acc_X,global.acc_g_vector[XINDEX]>>6);
            sendshortvalue(ID_Acc_Y,global.acc_g_vector[YINDEX]>>6);
            sendshortvalue(ID_Acc_Z,global.acc_g_vector[ZINDEX]>>6);
            break;
         case 4:
            // send the number of satelites as RPM
            sendshortvalue(ID_RPM,global.gps_num_satelites);
            
            // send the navigation distance in Temperature1
            sendshortvalue(ID_Temperature1,global.navigation_distance>>FIXEDPOINTSHIFT);

            // send the navigation bearing in Temperature 2
            sendshortvalue(ID_Temperature2,global.navigation_bearing>>FIXEDPOINTSHIFT);

            telemetrycount = 0; // start over
            break;
         }
      sendDataTail();

      // restart the timer
      telemetrytimer=lib_timers_starttimer();
      }
   }
#endif


#if (TELEMETRYMODE==TELEMETRYFRSKYSMARTPORT)

//const unsigned int frSkyDataIdTable[] = {
//    FSSP_DATAID_SPEED     ,
//    FSSP_DATAID_VFAS      ,
//    FSSP_DATAID_CURRENT   ,
//    //FSSP_DATAID_RPM       ,
//    FSSP_DATAID_ALTITUDE  ,
//    FSSP_DATAID_FUEL      ,
//    //FSSP_DATAID_ADC1      ,
//    //FSSP_DATAID_ADC2      ,
//    FSSP_DATAID_LATLONG   ,
//    FSSP_DATAID_LATLONG   , // twice
//    //FSSP_DATAID_CAP_USED  ,
//    FSSP_DATAID_VARIO     ,
//    //FSSP_DATAID_CELLS     ,
//    //FSSP_DATAID_CELLS_LAST,
//    FSSP_DATAID_HEADING   ,
//    FSSP_DATAID_ACCX      ,
//    FSSP_DATAID_ACCY      ,
//    FSSP_DATAID_ACCZ      ,
//    FSSP_DATAID_T1        ,
//    FSSP_DATAID_T2        ,
//    FSSP_DATAID_GPS_ALT   ,
//    0
//};

#define SMARTPORT_BAUD 57600

unsigned char telemetrycount;
unsigned char smartportstate;

#define SMARTPORTIDLE 0
#define SMARTPORTGOTSTART 1

void inittelemetry()
   { // set up the serial port for smart port communication
   lib_serial_initport(TELEMETRY_SERIAL_PORT,57600);
   telemetrycount=0;
   smartportstate=SMARTPORTIDLE;
   }

//   from: https://code.google.com/p/telemetry-convert/wiki/FrSkySPortProtocol
//
//   Logical procotol
//      Receiver (e.g. FrSky X8R) polls sensors by sending:
//         0x7E (8bit start-stop)
//         8_bit_sensor_id
//         If a sensor is present it answers by sending:
//         0x10 (8bit data frame header)
//         value_type (16 bit, e.g. voltage / speed)
//         value (32 bit, may be signed or unsigned depending on value type)
//         checksum (8 bit)
//   If no sensor is present no answer gets sent and the receiver goes on to the next sensor-id
//   Sensor Ids
//
//   The X8R receiver does not poll all possible sensor ids (0x00-0xFF) but only a few selected ones. There is no public documentation on used FrSky sensor ids available so you will have to find and use your own.
//
//   Ids known to work are: 0x00, 0xA1
//
//   If two sensors use the same id the bus will not work since both sensors will try to send at the same time. This will result in telemtry connection breaking up.
//
//   8XR (November 2013 firmware X8R_131105.frk) polling is observed to request data for 28 different sensor IDs in the following sequence:
//
//   7E A1 7E 22 7E 83 7E E4 7E 45 7E C6 7E 67 7E 48 7E E9 7E 6A 7E CB 7E AC 7E 0D 7E 8E 7E 2F 7E D0 7E 71 7E F2 7E 53 7E 34 7E 95 7E 16 7E B7 7E 98 7E 39 7E BA 7E 1B 7E 00
//
// The receiver doesn't seem to look at the response to make sure it is of the same type of data requested, so it
// turns out that hte sensor can resond to any request from the receiver with any type of data.

void sendnextsmartportdata();

void checktelemetryforaction()
   { // wait to receive a START_STOP byte followed by a request byte
   if (lib_serial_numcharsavailable(TELEMETRY_SERIAL_PORT))
      {
      char c=lib_serial_getchar(TELEMETRY_SERIAL_PORT);

      if (smartportstate==SMARTPORTIDLE)
         {
         if (c==FSSP_START_STOP)
            {
            smartportstate=SMARTPORTGOTSTART;
            }
         }
      else if (smartportstate==SMARTPORTGOTSTART)
         { // it appears that we don't have to respond to the right request with the right data, so we just
         // resond to these requests with whatever we want to respond with.
         if ((c == FSSP_SENSOR_ID1) ||
            (c == FSSP_SENSOR_ID2) ||
            (c == FSSP_SENSOR_ID3) ||
            (c == FSSP_SENSOR_ID4))
            sendnextsmartportdata();
            
         smartportstate=SMARTPORTIDLE;
         }
      }
   }


static void sendsmartportbyte(unsigned char c, unsigned int *crcp)
   { // send a byte of smart port data.  If crcp is not zero, keep track of the crc
   // smart port escape sequence
   if (c == 0x7D || c == 0x7E)
      {
      lib_serial_sendchar(TELEMETRY_SERIAL_PORT, 0x7D);
      c ^= 0x20;
      }

   lib_serial_sendchar(TELEMETRY_SERIAL_PORT, c);

   if (crcp == 0) return;

   // update the checksum
   unsigned int crc = *crcp;
   crc += c;
   crc += crc >> 8;
   crc &= 0x00FF;
   *crcp = crc;
   }

static void sendsmartportpaket(unsigned int id, unsigned long val)
   {  // send the id and value as a smart port packet
   unsigned int crc = 0;
   sendsmartportbyte(FSSP_DATA_FRAME, &crc);
   unsigned char *u8p = (unsigned char*)&id;
   sendsmartportbyte(u8p[0], &crc);
   sendsmartportbyte(u8p[1], &crc);
   u8p = (unsigned char*)&val;
   sendsmartportbyte(u8p[0], &crc);
   sendsmartportbyte(u8p[1], &crc);
   sendsmartportbyte(u8p[2], &crc);
   sendsmartportbyte(u8p[3], &crc);
   sendsmartportbyte(0xFF - (unsigned char)crc, 0);
   }

void sendsmartportlatlongvalue(long value,unsigned char thisislongitude)
   {
   // the same ID is sent twice, one for longitude, one for latitude
   // the MSB of the sent unsigned long tells which is which

   // convert from 10,000,000 degrees to
   // units=10000 minutes
   // =(value*60*10^-7*10^4)
   // =(value*.06)
   // =(value>>3 * .06<<7)>>4
   // =(ourvalue>>3 * 1.1444091796875)>>3;
   
   value=lib_fp_multiply(value>>3,FIXEDPOINTCONSTANT(7.68))>>4;
   unsigned long unsignedvalue;
   long signedvalue;

   unsignedvalue = signedvalue = value;
   if (signedvalue < 0)
      {
      unsignedvalue = -signedvalue;
      unsignedvalue |= 0x40000000;
      }
   if (thisislongitude) unsignedvalue |= 0x80000000;
   sendsmartportpaket(FSSP_DATAID_LATLONG, unsignedvalue);
   }

// send the next packet in our list
void sendnextsmartportdata()
   {
   switch(++telemetrycount)
      {
      case 1:
         // send the gps speed. Convert from meters per second to km/h
         sendsmartportpaket(FSSP_DATAID_SPEED, lib_fp_multiply(global.gps_current_speed,FIXEDPOINTCONSTANT(3.6))>>FIXEDPOINTSHIFT); //  in KM/H?
         break;
      case 2:
         // send the number of satelites as RPM.  Multiplying by 60 makes it compatible with the Hub format.
         sendsmartportpaket(FSSP_DATAID_RPM, global.gps_num_satelites*60);
         break;
      case 3: // altitude
         sendsmartportpaket(FSSP_DATAID_ALTITUDE, (global.altitude*100)>>FIXEDPOINTSHIFT); // unknown given unit, requested 100 = 1 meter
         break;
      case 4: // logitude
         sendsmartportlatlongvalue(global.gps_current_longitude,1);
         break;
      case 5: // latitude
         sendsmartportlatlongvalue(global.gps_current_latitude,0);
         break;
      case 6: // vario
          sendsmartportpaket(FSSP_DATAID_VARIO, (global.altitudevelocity*100L)>>FIXEDPOINTSHIFT); // requested in 100 = 1m/s
          break;
      case 7: // the current heading
          if (global.currentestimatedeulerattitude[YAWINDEX]<0)
            sendsmartportpaket(FSSP_DATAID_HEADING, ((global.currentestimatedeulerattitude[YAWINDEX]+FIXEDPOINTCONSTANT(360.0))*50L)>>(FIXEDPOINTSHIFT-1));
          else
            sendsmartportpaket(FSSP_DATAID_HEADING, (global.currentestimatedeulerattitude[YAWINDEX]*50L)>>(FIXEDPOINTSHIFT-1));
          break;
      case 8: // accelerometer data.  Not sure the units are correct
          sendsmartportpaket(FSSP_DATAID_ACCX, global.acc_g_vector[XINDEX]>>6);
          break;
      case 9:
          sendsmartportpaket(FSSP_DATAID_ACCY, global.acc_g_vector[YINDEX]>>6);
          break;
      case 10:
          sendsmartportpaket(FSSP_DATAID_ACCZ, global.acc_g_vector[ZINDEX]>>6);
          break;
      case 11: // gps altitude
         sendsmartportpaket(FSSP_DATAID_GPS_ALT, (global.gps_current_altitude*100)>>FIXEDPOINTSHIFT); // requested 100 = 1 meter
         break;
      case 12: // gps speed
         sendsmartportpaket(FSSP_DATAID_GPS_SPEED, lib_fp_multiply(global.gps_current_speed,FIXEDPOINTCONSTANT(1.94384)));
         break;
      case 13: // send the navigation distance in Temperature1
         sendsmartportpaket(FSSP_DATAID_T1, global.navigation_distance>>FIXEDPOINTSHIFT);
         break;
      case 14: // send the navigation bearing in Temperature2
         sendsmartportpaket(FSSP_DATAID_T2, global.navigation_bearing>>FIXEDPOINTSHIFT);
         break;


      default:
          telemetrycount=0; // start over
          break;
      }
   }

#endif
