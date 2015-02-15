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

#include "lib_fp.h"

#define LATLONGEXTRASHIFT 6  // keep latitudes and longitudes shifted extra for accuracy

void initgps();
char readgps();

fixedpointnum gpsgetdistanceandbearing(fixedpointnum lat1,fixedpointnum lon1,fixedpointnum lat2,fixedpointnum lon2,fixedpointnum *bearing);

#if (GPS_TYPE==UBLOX_GPS)

typedef struct
   {
   unsigned char preamble1;
   unsigned char preamble2;
   unsigned char msg_class;
   unsigned char msg_id;
   unsigned int length;
   } ubx_header_struct;

typedef struct
   {
   unsigned long time;  // GPS msToW
   long longitude;
   long latitude;
   long altitude_ellipsoid;
   long altitude_msl;
   unsigned long horizontal_accuracy;
   unsigned long vertical_accuracy;
   }ubx_nav_posllh_struct;

typedef struct
   {
   unsigned long time;
   long time_nsec;
   int week;
   unsigned char fix_type;
   unsigned char fix_status;
   long ecef_x;
   long ecef_y;
   long ecef_z;
   unsigned long position_accuracy_3d;
   long ecef_x_velocity;
   long ecef_y_velocity;
   long ecef_z_velocity;
   unsigned long speed_accuracy;
   unsigned int position_DOP;
   unsigned char res;
   unsigned char satellites;
   unsigned long res2;
   } ubx_nav_solution_struct;

typedef struct
   {
   unsigned long time;  // GPS msToW
   long ned_north;
   long ned_east;
   long ned_down;
   unsigned long speed_3d;
   unsigned long speed_2d;
   long heading_2d;
   unsigned long speed_accuracy;
   unsigned long heading_accuracy;
   } ubx_nav_velned_struct;

typedef struct
   {
  // Receive buffer
   unsigned char msg_class;
   unsigned char msg_id;
   unsigned int length;
  
   union
      {
      ubx_nav_posllh_struct posllh;
      ubx_nav_solution_struct solution;
      ubx_nav_velned_struct velned;
      };
   } gps_data_struct;
  

enum ubs_protocol_bytes
   {
   PREAMBLE1 = 0xb5,
   PREAMBLE2 = 0x62,
   CLASS_NAV = 0x01,
   CLASS_ACK = 0x05,
   CLASS_CFG = 0x06,
   MSG_ACK_NACK = 0x00,
   MSG_ACK_ACK = 0x01,
   MSG_POSLLH = 0x2,
   MSG_STATUS = 0x3,
   MSG_SOL = 0x6,
   MSG_VELNED = 0x12,
   MSG_CFG_PRT = 0x00,
   MSG_CFG_RATE = 0x08,
   MSG_CFG_SET_RATE = 0x01,
   MSG_CFG_NAV_SETTINGS = 0x24
   };

enum ubs_nav_fix_type
   {
   FIX_NONE = 0,
   FIX_DEAD_RECKONING = 1,
   FIX_2D = 2,
   FIX_3D = 3,
   FIX_GPS_DEAD_RECKONING = 4,
   FIX_TIME = 5
   };

enum ubx_nav_status_bits
   {
   NAV_STATUS_FIX_VALID = 1
   };

#define PREAMBLE1INDEX -2
#define PREAMBLE2INDEX -1
#define GOTLENGTHINDEX 4
#define CHECKSUMAINDEX -10
#define CHECKSUMBINDEX -9

#endif
