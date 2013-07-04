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
fixedpointnum gpsdegrees;

unsigned char hextochar(unsigned char n) 
   {      // convert '0'..'9','A'..'F' to 0..15
   if (n>='A') return(n-'A'+10);
   else return(n-'0');
   }

fixedpointnum gpsstringtoangle(char *string)
   { // takes a gps string and converts it to a fixedpointnum angle
   // "4807.123" means 48 degrees, 7.123 minutes, south
   // how many digits are there before the decimal point?
   int index=0;
   while (string[index]>='0' && string[index]<='9') ++index;
   
   // convert the minutes part
   fixedpointnum minutes=lib_fp_stringtofixedpointnum(&string[index-2]);

   string[index-2]='\0';
   fixedpointnum degrees=lib_fp_stringtofixedpointnum(string);
   return((degrees<<LATLONGEXTRASHIFT)+lib_fp_multiply(minutes,69905L));  // 69905L is (1/60) * 2^16 * 2^6
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
                  gpsdegrees = gpsstringtoangle(gpsdata);
                  }
               else if (gpsparameternumber == 3 && gpsdegrees!=0)
                  {
                  if ( gpsdata[0] == 'S') global.gps_current_latitude=-gpsdegrees;
                  else global.gps_current_latitude=gpsdegrees;
                  }
               else if (gpsparameternumber == 4)
                  {
                  gpsdegrees = gpsstringtoangle(gpsdata);
                  }
               else if (gpsparameternumber == 5 && gpsdegrees!=0)
                  {
                  if ( gpsdata[0] == 'W') global.gps_current_longitude=-gpsdegrees;
                  else global.gps_current_longitude=gpsdegrees;
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
                  global.gps_current_speed=lib_fp_multiply(lib_fp_stringtofixedpointnum(gpsdata),33715L); // 33715L is .514444444 * 2^16
                  }
               else if (gpsparameternumber == 8) 
                  {
//                  GPS_ground_course = grab_fields(gpsdata,1); 
                  }                 //ground course deg*10 
               }
            
            ++gpsparameternumber;
            gpsdataindex = 0; // get read for the next parameter
            if (c == '*') gpsfinalchecksum=gpschecksum;
            }
         else if (c == '\r' || c == '\n') 
            { // end of the message
            unsigned char checksum = hextochar(gpsdata[0]);
            checksum <<= 4;
            checksum += hextochar(gpsdata[1]);

            if (checksum != gpsfinalchecksum || gpsframetype!=FRAME_GGA || !gpsfix) return(0);
            gpsframetype=0; // so we don't check again

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
         
#endif

