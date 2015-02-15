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

#include "defs.h"

// this file defines the values needed by my libraries


// Serial Ports

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT0)
#define USESERIALPORT0
#define SERIAL0OUTPUTBUFFERSIZE 150
#define SERIAL0INPUTBUFFERSIZE 64
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT1)
#define USESERIALPORT1
#define SERIAL1OUTPUTBUFFERSIZE 150
#define SERIAL1INPUTBUFFERSIZE 64
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT2)
#define USESERIALPORT2
#define SERIAL2OUTPUTBUFFERSIZE 150
#define SERIAL2INPUTBUFFERSIZE 64
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT3)
#define USESERIALPORT3
#define SERIAL3OUTPUTBUFFERSIZE 150
#define SERIAL3INPUTBUFFERSIZE 64
#endif

#if (RX_TYPE==RX_DSM2_1024 || RX_TYPE==RX_DSM2_2048 || RX_TYPE==RX_SBUS)

#if (RX_SERIAL_PORT==0)
	#define USESERIALPORT0
	#define SERIAL0OUTPUTBUFFERSIZE 1
	#define SERIAL0INPUTBUFFERSIZE 1
#endif

#if (RX_SERIAL_PORT==1)
	#define USESERIALPORT1
	#define SERIAL1OUTPUTBUFFERSIZE 1
	#define SERIAL1INPUTBUFFERSIZE 1
#endif

#if (RX_SERIAL_PORT==2)
	#define USESERIALPORT2
	#define SERIAL2OUTPUTBUFFERSIZE 1
	#define SERIAL2INPUTBUFFERSIZE 1
#endif

#if (RX_SERIAL_PORT==3)
	#define USESERIALPORT3
	#define SERIAL3OUTPUTBUFFERSIZE 1
	#define SERIAL3INPUTBUFFERSIZE 1
#endif

#endif

#if (TELEMETRYMODE==TELEMETRYFRSKYSMARTPORT || TELEMETRYMODE==TELEMETRYFRSKYSERIAL)

   #if (TELEMETRY_SERIAL_PORT==0)
      #define USESERIALPORT0
      #define SERIAL0OUTPUTBUFFERSIZE 100
      #if (TELEMETRYMODE==TELEMETRYFRSKYSMARTPORT)
         #define SERIAL0INPUTBUFFERSIZE 10
      #else
         #define SERIAL0INPUTBUFFERSIZE 1
      #endif
   #endif

   #if (TELEMETRY_SERIAL_PORT==1)
      #define USESERIALPORT1
      #define SERIAL1OUTPUTBUFFERSIZE 100
      #if (TELEMETRYMODE==TELEMETRYFRSKYSMARTPORT)
         #define SERIAL1INPUTBUFFERSIZE 10
      #else
         #define SERIAL1INPUTBUFFERSIZE 1
      #endif
   #endif

   #if (TELEMETRY_SERIAL_PORT==2)
      #define USESERIALPORT2
      #define SERIAL2OUTPUTBUFFERSIZE 100
      #if (TELEMETRYMODE==TELEMETRYFRSKYSMARTPORT)
         #define SERIAL2INPUTBUFFERSIZE 10
      #else
         #define SERIAL2INPUTBUFFERSIZE 1
      #endif
   #endif

   #if (TELEMETRY_SERIAL_PORT==3)
      #define USESERIALPORT3
      #define SERIAL3OUTPUTBUFFERSIZE 100
      #if (TELEMETRYMODE==TELEMETRYFRSKYSMARTPORT)
         #define SERIAL3INPUTBUFFERSIZE 10
      #else
         #define SERIAL3INPUTBUFFERSIZE 1
      #endif
   #endif

#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORTUSB)
	#define USESERIALPORTUSB
#endif

#if (GPS_TYPE==SERIAL_GPS || GPS_TYPE==UBLOX_GPS)

#if (GPS_SERIAL_PORT==0)
	#define USESERIALPORT0
	#define SERIAL0OUTPUTBUFFERSIZE 2
	#define SERIAL0INPUTBUFFERSIZE 100
#endif

#if (GPS_SERIAL_PORT==1)
	#define USESERIALPORT1
	#define SERIAL1OUTPUTBUFFERSIZE 2
	#define SERIAL1INPUTBUFFERSIZE 100
#endif

#if (GPS_SERIAL_PORT==2)
	#define USESERIALPORT2
	#define SERIAL2OUTPUTBUFFERSIZE 2
	#define SERIAL2INPUTBUFFERSIZE 100
#endif

#if (GPS_SERIAL_PORT==3)
#define USESERIALPORT3
#define SERIAL3OUTPUTBUFFERSIZE 2
#define SERIAL3INPUTBUFFERSIZE 100
#endif

#endif

#if (MICROCONTROLLER_TYPE==MEGA2560)
	#define USEPWM3
	#define USEPWM4
	#define USEDIGITALPORTB
	#define USEDIGITALPORTC
	#define USEDIGITALPORTK

	#define USEDIGITALPORTE // for PWM
	#define USEDIGITALPORTH // for PWM
	
	// timer 1 is free, use it for better resolution and less overhead
	#define USETIMER1FORGENERALTIMER

#elif (MICROCONTROLLER_TYPE==MEGA328P)
	#define USEPWM1
	#define USEPWM2
	#define USEDIGITALPORTB
	#define USEDIGITALPORTD
   #if (RXNUMCHANNELS>6)
      #define USEDIGITALPORTC
   #endif

#elif (MICROCONTROLLER_TYPE==MEGA32U4)
	#define USEPWM1
	#define USEPWM3
	#define USEPWM411BIT
	#define USEDIGITALPORTB
	#define USEDIGITALPORTC
	#define USEDIGITALPORTD
	#define USEDIGITALPORTE
#endif

