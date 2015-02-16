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

#include "gps.h"
#include "defs.h"
#include "lib_serial.h"
#include "bradwii.h"
#include "lib_i2c.h"
#include "lib_timers.h"

// when adding GPS's, the following functions must be included:
// initgps()  // does any initialization of the gps
// readgps()  // sets global.gps_current_latitude in fixedpointnum degrees shifted left by LATLONGEXTRASHIFT
//                    global.gps_current_longitude in fixedpointnum degrees shifted left by LATLONGEXTRASHIFT
//                    global.gps_num_satelites,global.gps_current_altitude in fixedpointnum meters
//                      returns 1 if a new fix is acquired, 0 otherwise.

extern globalstruct global;

#if (GPS_TYPE==NO_GPS)
void initgps()
   {
   }
   
char readgps()
   { // returns 1 if new data was read
   return(0);
   }
   
#endif

#if (GPS_TYPE==SERIAL_GPS)

void initgps()
   {
   lib_serial_initport(GPS_SERIAL_PORT,GPS_BAUD);
   global.gps_num_satelites=0;
   }

#define GPSDATASIZE 15
#define FRAME_GGA 1
#define FRAME_RMC 2

char gpsdata[GPSDATASIZE];
unsigned char gpsparameternumber;
unsigned char gpsdataindex=0;
unsigned char gpsframetype;
unsigned char gpschecksum;
unsigned char gpsfinalchecksum;
unsigned char gpsfix;
fixedpointnum gpslatitudedegrees;
fixedpointnum gpslongitudedegrees;

unsigned char hextochar(unsigned char n) 
   {      // convert '0'..'9','A'..'F' to 0..15
   if (n>='A') return(n-'A'+10);
   else return(n-'0');
   }

//fixedpointnum gpsstringtoangle(char *string)
//   { // takes a gps string and converts it to a fixedpointnum angle
//   // "4807.123" means 48 degrees, 7.123 minutes, south
//   // how many digits are there before the decimal point?
//   int index=0;
//   while (string[index]>='0' && string[index]<='9') ++index;
//   
//   // convert the minutes part
//   fixedpointnum minutes=lib_fp_stringtofixedpointnum(&string[index-2]);
//
//   string[index-2]='\0';
//   fixedpointnum degrees=lib_fp_stringtofixedpointnum(string);
//   return((degrees<<LATLONGEXTRASHIFT)+lib_fp_multiply(minutes,69905L));  // 69905L is (1/60) * 2^16 * 2^6
//   }

fixedpointnum gpsstringtoangle(char *string)
   { // takes a gps string and converts it to a fixedpointnum angle
   // "4807.123" means 48 degrees, 7.123 minutes, south
   // how many digits are there before the decimal point?
   int index=0;
   while (string[index]>='0' && string[index]<='9') ++index;

   // convert the minutes part.  Use two digits before the decimal and 5 digits after
   // with the decimal point, this gives 10 characters to look at.
   // The largest raw minute value we should get will be 6,000,000 (7 total digits)
   fixedpointnum minutes=0;
   char *ptr=&string[index-2];
   
//if (*(ptr-1)=='3') global.debugvalue[2]=lib_fp_stringtolong(&string[index+1]);
//else global.debugvalue[3]=lib_fp_stringtolong(&string[index+1]);
   for (int count=0;count<8;++count)
      {
      if (*ptr=='.') ++ptr; // ignore the decimal point
      else
         {
         minutes*=10;
         if (*ptr) minutes+=(*ptr++) - '0';
         }
      }
   string[index-2]='\0';
   fixedpointnum degrees=lib_fp_stringtolong(string);

   return((degrees<<(FIXEDPOINTSHIFT+LATLONGEXTRASHIFT))+(lib_fp_multiply(minutes,11728124L)>>8));  // 11728124L is (2^16 * 2^6 * 2^16 * 2^8)/(60 * 10^5)
   }

char readgps()
   {
   while (lib_serial_numcharsavailable(GPS_SERIAL_PORT))
      {
      char c=lib_serial_getchar(GPS_SERIAL_PORT);

      if (c=='$') // start of a new message
         {
         gpsparameternumber=0;
         gpsdataindex=0;
         gpschecksum=0;
         gpsframetype=0;
         }
      else
         {
         if (c==',' || c=='*')
            { // we just finished loading a parameter, interpret it
            gpsdata[gpsdataindex]='\0';

            if (gpsparameternumber == 0) 
               { //frame identification
               if (gpsdata[0] == 'G' && gpsdata[1] == 'P' && gpsdata[2] == 'G' && gpsdata[3] == 'G' && gpsdata[4] == 'A') gpsframetype=FRAME_GGA;
               else if (gpsdata[0] == 'G' && gpsdata[1] == 'P' && gpsdata[2] == 'R' && gpsdata[3] == 'M' && gpsdata[4] == 'C') gpsframetype = FRAME_RMC;
               } 
            else if (gpsframetype == FRAME_GGA) 
               {
               if (gpsparameternumber == 2) 
                  {
                  gpslatitudedegrees = gpsstringtoangle(gpsdata);
                  }
               else if (gpsparameternumber == 3)
                  {
                  if ( gpsdata[0] == 'S') gpslatitudedegrees=-gpslatitudedegrees;
                  }
               else if (gpsparameternumber == 4)
                  {
                  gpslongitudedegrees = gpsstringtoangle(gpsdata);
                  }
               else if (gpsparameternumber == 5)
                  {
                  if ( gpsdata[0] == 'W') gpslongitudedegrees=-gpslongitudedegrees;
                  }
               else if (gpsparameternumber == 6)
                  {
                  gpsfix = gpsdata[0]-'0';
                  }
               else if (gpsparameternumber == 7)
                  {
                  global.gps_num_satelites = lib_fp_stringtolong(gpsdata);
                  }
               else if (gpsparameternumber == 9)
                  {
                  global.gps_current_altitude = lib_fp_stringtofixedpointnum(gpsdata);
                  }
               } 
            else if (gpsframetype == FRAME_RMC) 
               {
               if (gpsparameternumber == 7)
                  {
                  // 1 knot = 0.514444444 m / s
//                  global.gps_current_speed=lib_fp_multiply(lib_fp_stringtofixedpointnum(gpsdata),33715L); // 33715L is .514444444 * 2^16
                  global.gps_current_speed=lib_fp_multiply(lib_fp_stringtofixedpointnum(gpsdata),FIXEDPOINTCONSTANT(0.514444444));
                  }
               else if (gpsparameternumber == 8) 
                  {
//                  GPS_ground_course = grab_fields(gpsdata,1); 
                  }                 //ground course deg*10 
               }
            
            ++gpsparameternumber;
            gpsdataindex = 0; // get ready for the next parameter
            if (c == '*') gpsfinalchecksum=gpschecksum;
            }
         else if (c == '\r' || c == '\n') 
            { // end of the message
            unsigned char checksum = hextochar(gpsdata[0]);
            checksum <<= 4;
            checksum += hextochar(gpsdata[1]);

            if (checksum != gpsfinalchecksum || gpsframetype!=FRAME_GGA || !gpsfix) return(0);
            gpsframetype=0; // so we don't check again

            global.gps_current_latitude=gpslatitudedegrees;
            global.gps_current_longitude=gpslongitudedegrees;
            
            return(1); // we got a good frame
            }
         else if (gpsdataindex<GPSDATASIZE)
            {
            gpsdata[gpsdataindex++]=c;
            }
         gpschecksum^=c;
         }
      }
   return(0); // no complete data yet
   }

#elif (GPS_TYPE==I2C_GPS)

/**************************************************************************************/
/***************                       I2C GPS                     ********************/
/**************************************************************************************/
#define I2C_GPS_ADDRESS                         0x20 //7 bits       
///////////////////////////////////////////////////////////////////////////////////////////////////
// I2C GPS NAV registers
///////////////////////////////////////////////////////////////////////////////////////////////////
//
#define I2C_GPS_STATUS_00                            00 //(Read only)
        #define I2C_GPS_STATUS_NEW_DATA       0x01      // New data is available (after every GGA frame)
        #define I2C_GPS_STATUS_2DFIX          0x02      // 2dfix achieved
        #define I2C_GPS_STATUS_3DFIX          0x04      // 3dfix achieved
        #define I2C_GPS_STATUS_WP_REACHED     0x08      // Active waypoint has been reached (not cleared until new waypoint is set)
        #define I2C_GPS_STATUS_NUMSATS        0xF0      // Number of sats in view

#define I2C_GPS_COMMAND                              01 // (write only)
        #define I2C_GPS_COMMAND_POSHOLD       0x01      // Start position hold at the current gps positon
        #define I2C_GPS_COMMAND_START_NAV     0x02      // get the WP from the command and start navigating toward it
        #define I2C_GPS_COMMAND_SET_WP        0x03      // copy current position to given WP      
        #define I2C_GPS_COMMAND_UPDATE_PIDS   0x04      // update PI and PID controllers from the PID registers, this must be called after a pid register is changed
        #define I2C_GPS_COMMAND_NAV_OVERRIDE  0x05      // do not nav since we tring to controll the copter manually (not implemented yet)
        #define I2C_GPS_COMMAND_STOP_NAV      0x06      // Stop navigation (zeroes out nav_lat and nav_lon
        #define I2C_GPS_COMMAND__7            0x07
        #define I2C_GPS_COMMAND__8            0x08      
        #define I2C_GPS_COMMAND__9            0x09
        #define I2C_GPS_COMMAND__a            0x0a
        #define I2C_GPS_COMMAND__b            0x0b
        #define I2C_GPS_COMMAND__c            0x0c
        #define I2C_GPS_COMMAND__d            0x0d
        #define I2C_GPS_COMMAND__e            0x0e
        #define I2C_GPS_COMMAND__f            0x0f

        #define I2C_GPS_COMMAND_WP_MASK       0xF0       // Waypoint number

#define I2C_GPS_WP_REG                              02   // Waypoint register (Read only)
        #define I2C_GPS_WP_REG_ACTIVE_MASK    0x0F       // Active Waypoint lower 4 bits
        #define I2C_GPS_WP_REG_PERVIOUS_MASK  0xF0       // pervious Waypoint upper 4 bits
        
#define I2C_GPS_REG_VERSION                         03   // Version of the I2C_NAV SW unsigned char
#define I2C_GPS_REG_RES2                            04   // reserved for future use (unsigned char)
#define I2C_GPS_REG_RES3                            05   // reserved for future use (unsigned char)
#define I2C_GPS_REG_RES4                            06   // reserved for future use (unsigned char)


#define I2C_GPS_LOCATION                            07   // current location 8 byte (lat, lon) long
#define I2C_GPS_NAV_LAT                             15   // Desired banking towards north/south int
#define I2C_GPS_NAV_LON                             17   // Desired banking toward east/west    int
#define I2C_GPS_WP_DISTANCE                         19   // Distance to current WP in cm uint32
#define I2C_GPS_WP_TARGET_BEARING                   23   // bearing towards current wp 1deg = 1000 int
#define I2C_GPS_NAV_BEARING                         25   // crosstrack corrected bearing towards current wp 1deg = 1000 int
#define I2C_GPS_HOME_TO_COPTER_BEARING              27   // bearing from home to copter 1deg = 1000 int
#define I2C_GPS_DISTANCE_TO_HOME                    29   // distance to home in m int
        
#define I2C_GPS_GROUND_SPEED                        31   // GPS ground speed in m/s*100 (unsigned int)      (Read Only)
#define I2C_GPS_ALTITUDE                            33   // GPS altitude in meters (unsigned int)           (Read Only)
#define I2C_GPS_GROUND_COURSE                       35   // GPS ground course (unsigned int)
#define I2C_GPS_RES1                                37   // reserved for future use (unsigned int)
#define I2C_GPS_TIME                                39   // UTC Time from GPS in hhmmss.sss * 100 (unsigned long)(unneccesary precision) (Read Only)

unsigned long gpstimer;

void initgps()
   {
   gpstimer=lib_timers_starttimer();
   }

char readgps()
   { // returns 1 if new data was read
   if (lib_timers_gettimermicroseconds(gpstimer)<50000) return(0); // 20 hz

   gpstimer=lib_timers_starttimer();
   
   unsigned char shiftedaddress=I2C_GPS_ADDRESS<<1;
   lib_i2c_start_wait(shiftedaddress+I2C_WRITE);
   if (lib_i2c_write(I2C_GPS_STATUS_00))
      return(0); 

   lib_i2c_rep_start(shiftedaddress+I2C_READ);

   unsigned char gps_status= lib_i2c_readnak();
 
   global.gps_num_satelites = (gps_status & 0xf0) >> 4;
   if (gps_status & I2C_GPS_STATUS_3DFIX) 
      { //Check is we have a good 3d fix (numsats>5)
      long longvalue;
      unsigned char *ptr=(unsigned char *)&longvalue;
      
      lib_i2c_rep_start(shiftedaddress+I2C_WRITE);
      lib_i2c_write(I2C_GPS_LOCATION); 
      lib_i2c_rep_start(shiftedaddress+I2C_READ);

      *ptr++ = lib_i2c_readack();
      *ptr++ = lib_i2c_readack();
      *ptr++ = lib_i2c_readack();
      *ptr = lib_i2c_readack();

      global.gps_current_latitude=lib_fp_multiply(longvalue,27488L); // convert from 10,000,000 m to fixedpointnum
      
      ptr=(unsigned char *)&longvalue;
      
      *ptr++ = lib_i2c_readack();
      *ptr++ = lib_i2c_readack();
      *ptr++ = lib_i2c_readack();
      *ptr = lib_i2c_readnak();
      
      global.gps_current_longitude=lib_fp_multiply(longvalue,27488L); // convert from 10,000,000 m to fixedpointnum

      int intvalue;
      ptr= (unsigned char *)&intvalue;
      
      lib_i2c_rep_start(shiftedaddress+I2C_WRITE);
      lib_i2c_write(I2C_GPS_GROUND_SPEED); 
      lib_i2c_rep_start(shiftedaddress+I2C_READ);

      *ptr++ = lib_i2c_readack();
      *ptr++ = lib_i2c_readack();
      
      global.gps_current_speed=intvalue*665L; // convert fro cm/s to fixedpointnum to m/s
      
      ptr= (unsigned char *)&intvalue;
      *ptr++ = lib_i2c_readack();
      *ptr = lib_i2c_readnak();

      global.gps_current_altitude=((fixedpointnum)intvalue)<<FIXEDPOINTSHIFT;

      lib_i2c_stop();
      return(1);
      }
   lib_i2c_stop();

   return(0);
   }

#endif

#if (GPS_TYPE==UBLOX_GPS)

#include "lib_prg_mem.h"

int gpsdataindex;

// Packet checksum accumulators
unsigned char checksum_a;
unsigned char checksum_b;

unsigned char gotfix;

gps_data_struct gps_data;

PROGRAMCHAR UBLOX_INIT[]  =
   {
   0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19,                            //disable all default NMEA messages
   0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15,
   0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11,
   0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F,
   0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13,
   0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17,
   0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x02,0x01,0x0E,0x47,                            //set POSLLH MSG rate
   0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x03,0x01,0x0F,0x49,                            //set STATUS MSG rate
   0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x06,0x01,0x12,0x4F,                            //set SOL MSG rate
   0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x12,0x01,0x1E,0x67,                            //set VELNED MSG rate
#if (GPS_SBAS_TYPE==SBAS_EGNOS)
   0xB5,0x62,0x06,0x16,0x08,0x00,0x03,0x07,0x03,0x00,0x51,0x08,0x00,0x00,0x8A,0x41,   //set SBAS to EGNOS
#elif (GPS_SBAS_TYPE==SBAS_WAAS)
   0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x04, 0xE0, 0x04, 0x00, 0x19, 0x9D,
#elif (GPS_SBAS_TYPE==SBAS_MSAS)
   0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x00, 0x02, 0x02, 0x00, 0x35, 0xEF,
#elif (GPS_SBAS_TYPE==SBAS_GAGAN)
   0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x80, 0x01, 0x00, 0x00, 0xB2, 0xE8,
#else // SBAS_AUTO
   0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xE5,
#endif

   0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A //set rate to 5Hz
   };

void sendgpscharslowly(char c)
   { // add a character to the output buffer, then wait to make sure it goes before we send another one
   // this way, we can get away with a very small serial output buffer
   lib_serial_sendchar(GPS_SERIAL_PORT, c);
   while (!lib_serial_outputbufferisempty(GPS_SERIAL_PORT))
      {
      }
   // the following line may be needed at higher bauds
//   lib_timers_delaymilliseconds(200);
   }

void sendgpsstring(PROGRAMCHAR * str)
   { // send a string from program memory, slowly
   char b;
   while (str && (b = lib_prg_mem_getchar(str++)))
      sendgpscharslowly(b);
   }

void initgps()
   {
   global.gps_num_satelites=0;
   global.gps_fix=0;
   gotfix=0;
   gpsdataindex=PREAMBLE1INDEX; // we are waiting for the preamble 1 character
   
   // give the gps tme to boot up
   lib_timers_delaymilliseconds(1000);
   
   // set the gps to the desired baud rate.  Try all different bauds when sending the "set baud rate" message.
   unsigned long init_speed[5] = {9600,19200,38400,57600,115200};
   for(unsigned char i=0;i<5;i++)
      {
      lib_serial_initport(GPS_SERIAL_PORT,init_speed[i]);          // switch UART speed for sending SET BAUDRATE command (NMEA mode)
      #if (GPS_BAUD==19200)
         sendgpsstring(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));     // 19200 baud - minimal speed for 5Hz update rate
      #endif  
      #if (GPS_BAUD==38400)
         sendgpsstring(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));     // 38400 baud
      #endif  
      #if (GPS_BAUD==57600)
         sendgpsstring(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));     // 57600 baud
      #endif  
      #if (GPS_BAUD==115200)
         sendgpsstring(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));    // 115200 baud
      #endif  
      lib_timers_delaymilliseconds(200);
      }
      
   lib_serial_initport(GPS_SERIAL_PORT,GPS_BAUD);
   
   // send the configuration data in UBX protocol
   for (unsigned char i=0; i<sizeof(UBLOX_INIT); i++)
      {
      sendgpscharslowly(lib_prg_mem_getchar(UBLOX_INIT+i));
      }
   }

char handlepacket()
   {
   switch (gps_data.msg_id)
      {
      case MSG_POSLLH:
         if (gotfix)
            { // .4194304 is 10^-7 *2^16 *2^6 to convert from degrees *10^-7 to fixedpointnum degrees<<LATLONGEXTRASHIFT
            global.gps_current_longitude = lib_fp_multiply(gps_data.posllh.longitude, FIXEDPOINTCONSTANT(.4194304));
            global.gps_current_latitude = lib_fp_multiply(gps_data.posllh.latitude, FIXEDPOINTCONSTANT(.4194304));
            global.gps_current_altitude = lib_fp_multiply(gps_data.posllh.altitude_msl,FIXEDPOINTCONSTANT(65.536)); // convert from mm to fixedpointnum meters.  65.536 is .001 * 2^16
            }
         global.gps_fix = gotfix;
         return (1); // new location received
         break;
      case MSG_SOL:
         gotfix= 0;
         if ((gps_data.solution.fix_status & NAV_STATUS_FIX_VALID) && (gps_data.solution.fix_type == FIX_3D || gps_data.solution.fix_type == FIX_2D)) gotfix = 1;
         global.gps_num_satelites = gps_data.solution.satellites;
         break;
      case MSG_VELNED:
         global.gps_current_speed = lib_fp_multiply(gps_data.velned.speed_2d,FIXEDPOINTCONSTANT(655.36));  // convert from cm/s to fixedpoingnum m/s.  655.35 is .01 * 2^16
   //      global.gps_current_course = lib_fp_multiply(gps_data.velned.heading_2d,FIXEDPOINTCONSTANT(.065536));  // convert from deg * 100000 to fixedpointnum degrees.   is 10^-6 * 2^16.
         break;
      }
   return(0);
   }

char readgps()
   { // check the gps serial port for incoming characters
   // returns 1 if a new position is acquired
   while (lib_serial_numcharsavailable(GPS_SERIAL_PORT))
      {
      unsigned char c=lib_serial_getchar(GPS_SERIAL_PORT);

      if (gpsdataindex==PREAMBLE1INDEX)
         {
         if (c==PREAMBLE1)
            { // we recieved the first byte correctly
            ++gpsdataindex;
            }
         }
      else if (gpsdataindex==PREAMBLE2INDEX)
         {
         if (c==PREAMBLE2)
            { // we recieved the second byte correctly
            checksum_a=checksum_b=0;
            
            ++gpsdataindex;
            }
         else gpsdataindex=PREAMBLE1;
         }
      else if (gpsdataindex==CHECKSUMAINDEX)
         {
         if (c==checksum_a) ++gpsdataindex;
         else gpsdataindex=PREAMBLE1INDEX; // start over
         }
      else if (gpsdataindex==CHECKSUMBINDEX)
         {
         gpsdataindex=PREAMBLE1INDEX; // start over
         if (c==checksum_b) return(handlepacket());
         }
      else // we must be into the data part of the packet
         {
         unsigned char *gps_data_ptr=(unsigned char *)&gps_data;
         
         gps_data_ptr[gpsdataindex++]=c;
         
         checksum_a+=c;
         checksum_b+=checksum_a;
         
         if (gpsdataindex==GOTLENGTHINDEX)
            { // we just finished receiving the length
            if (gps_data.length>sizeof(gps_data_struct)-4)
               { // this packet is going to be too long to fit into our data structure.  Start over.
               gpsdataindex=PREAMBLE1INDEX;
               }
            }
         else if (gpsdataindex>=gps_data.length+4)
            { // we have received the whole packet.  Get ready for the checksum
            gpsdataindex=CHECKSUMAINDEX;
            }
         }
      }
   return(0); // didn't receive new location
   }

#endif
      