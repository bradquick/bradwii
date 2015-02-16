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

 
#include "bradwii.h"
#include "rx.h"
#include "defs.h"
#include "lib_timers.h"
#include "lib_digitalio.h"
#include "lib_serial.h"
#include <avr/io.h>

// when adding new receivers, the following functions must be included:
// initrx()   // initializes the r/c receiver
// readrx()   // loads global.rxvalues with r/c values as fixedpointnum's from -1 to 1 (0 is the center).

extern globalstruct global;

#if (RX_TYPE==RX_NORMAL)

unsigned long rxtimer[RXNUMCHANNELS];
volatile unsigned int rxrawvalues[RXNUMCHANNELS];

#define processcallback(INDEX)                                             \
   {                                                                              \
   if (newstate)                                                                  \
      {                                                                           \
      unsigned int value=lib_timers_gettimermicroseconds(rxtimer[INDEX]);                  \
      if (value>900 && value<2050) rxrawvalues[INDEX]=value;            \
      }                                                                           \
   else rxtimer[INDEX]=lib_timers_starttimer();                                             \
   }

void throttlecallback(unsigned char interruptnumber,unsigned char newstate) // call back will get called any time the pin changes
   {
   processcallback(THROTTLEINDEX);
   
   // reset the failsafe timer
   global.failsafetimer=lib_timers_starttimer();
   }

void rollcallback(unsigned char interruptnumber,unsigned char newstate) // call back will get called any time the pin changes
   {
   processcallback(ROLLINDEX);
   }

void pitchcallback(unsigned char interruptnumber,unsigned char newstate) // call back will get called any time the pin changes
   {
   processcallback(PITCHINDEX);
   }

void yawcallback(unsigned char interruptnumber,unsigned char newstate) // call back will get called any time the pin changes
   {
   processcallback(YAWINDEX);
   }

void aux1callback(unsigned char interruptnumber,unsigned char newstate) // call back will get called any time the pin changes
   {
   processcallback(AUX1INDEX);
   }

void aux2callback(unsigned char interruptnumber,unsigned char newstate) // call back will get called any time the pin changes
   {
   processcallback(AUX2INDEX);
   }

#if (RXNUMCHANNELS>6)
void aux3callback(unsigned char interruptnumber,unsigned char newstate) // call back will get called any time the pin changes
   {
   processcallback(AUX3INDEX);
   }

void aux4callback(unsigned char interruptnumber,unsigned char newstate) // call back will get called any time the pin changes
   {
   processcallback(AUX4INDEX);
   }
#endif

void initrx()
   {
   for (int x=0;x<RXNUMCHANNELS;++x)
      global.rxvalues[x]=0; // middle position
   global.rxvalues[THROTTLEINDEX]=FPTHROTTLELOW; // default throttle to low position
      
   lib_digitalio_initpin(THROTTLE_RX_INPUT,DIGITALINPUT);
   lib_digitalio_setinterruptcallback(THROTTLE_RX_INPUT,throttlecallback);    

   lib_digitalio_initpin(ROLL_RX_INPUT,DIGITALINPUT);
   lib_digitalio_setinterruptcallback(ROLL_RX_INPUT,rollcallback); 

   lib_digitalio_initpin(PITCH_RX_INPUT,DIGITALINPUT);
   lib_digitalio_setinterruptcallback(PITCH_RX_INPUT,pitchcallback); 

   lib_digitalio_initpin(YAW_RX_INPUT,DIGITALINPUT);
   lib_digitalio_setinterruptcallback(YAW_RX_INPUT,yawcallback); 

#if (RXNUMCHANNELS>4)
   lib_digitalio_initpin(AUX1_RX_INPUT,DIGITALINPUT);
   lib_digitalio_setinterruptcallback(AUX1_RX_INPUT,aux1callback); 
#endif

#if (RXNUMCHANNELS>5)
   lib_digitalio_initpin(AUX2_RX_INPUT,DIGITALINPUT);
   lib_digitalio_setinterruptcallback(AUX2_RX_INPUT,aux2callback); 
#endif

#if (RXNUMCHANNELS>6)
   lib_digitalio_initpin(AUX3_RX_INPUT,DIGITALINPUT);
   lib_digitalio_setinterruptcallback(AUX3_RX_INPUT,aux3callback); 
#endif

#if (RXNUMCHANNELS>7)
   lib_digitalio_initpin(AUX4_RX_INPUT,DIGITALINPUT);
   lib_digitalio_setinterruptcallback(AUX4_RX_INPUT,aux4callback); 
#endif
   }

void readrx()
   {
   for (int x=0;x<RXNUMCHANNELS;++x)
      {
      // convert from 1000-2000 range to -1 to 1 fixedpointnum range and low pass filter to remove glitches
      lib_fp_lowpassfilter(&global.rxvalues[x], ((fixedpointnum)rxrawvalues[x]-1500)*131L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
      }
   }

#elif (RX_TYPE==RX_DSM2_1024 || RX_TYPE==RX_DSM2_2048)

#if (RX_TYPE==RX_DSM2_1024)
   #define DSM2_CHAN_SHIFT  2       // Assumes 10 bit frames, that is 1024 mode.
   #define DSM2_CHAN_MASK   0x03    // Assumes 10 bit frames, that is 1024 mode.
#else
   #define DSM2_CHAN_SHIFT  3       // Assumes 11 bit frames, that is 2048 mode.
   #define DSM2_CHAN_MASK   0x07    // Assumes 11 bit frames, that is 2048 mode.
#endif

#define DSM2_BAUD 115200L

#define DSM2STATE_NEWFRAMESARTED 0
#define DSM2STATE_WAITINGFORFIRSTCHAR 1
#define DSM2STATE_WAITINGFORSECONDCHAR 2

volatile unsigned int rxrawvalues[RXNUMCHANNELS];
unsigned long dsm2timer;
unsigned char dsm2state;
unsigned char dsm2firstchar;
unsigned char dsm2channelindex[]={RX_CHANNEL_ORDER};

// this callback will get called whenever we receive a character on our dsm2 serial port
void dsm2serialcallback(unsigned char c)
   {
   // there is a 11ms delay between packets.  We need to use this delay to determine when we are
   // getting new data.
   unsigned long microsecondssincelastchar=lib_timers_gettimermicrosecondsandreset(&dsm2timer);
   if (microsecondssincelastchar>5000)
      { // this is a new packet.  Skip this first byte
      dsm2state=DSM2STATE_NEWFRAMESARTED;
      }
   else if (dsm2state==DSM2STATE_NEWFRAMESARTED)
      { // skip the 2nd byte too
      dsm2state=DSM2STATE_WAITINGFORFIRSTCHAR;
      }
   else if (dsm2state==DSM2STATE_WAITINGFORFIRSTCHAR)
      {
      dsm2firstchar=c;
      dsm2state=DSM2STATE_WAITINGFORSECONDCHAR;
      }
   else
      {
      unsigned char channel = 0x0F & (dsm2firstchar >> DSM2_CHAN_SHIFT);
      
      if (channel < RXNUMCHANNELS)
         {
         channel=dsm2channelindex[channel];
         rxrawvalues[channel] = ((unsigned int)(dsm2firstchar & DSM2_CHAN_MASK) << 8) + c;
         }
      dsm2state=DSM2STATE_WAITINGFORFIRSTCHAR;

      // reset the failsafe timer
      global.failsafetimer=lib_timers_starttimer();
      }
   }
   
void initrx()
   {
   dsm2timer=lib_timers_starttimer();
   
   lib_serial_initport(RX_SERIAL_PORT,DSM2_BAUD);
   lib_serial_setrxcallback(RX_SERIAL_PORT,dsm2serialcallback);
   }   

void readrx()
   {
   for (int x=0;x<RXNUMCHANNELS;++x)
      {
#if (RX_TYPE==RX_DSM2_1024)
      // convert from 0-1024 range to -1 to 1 fixedpointnum range and low pass filter to remove glitches
      lib_fp_lowpassfilter(&global.rxvalues[x], ((fixedpointnum)rxrawvalues[x]-512)<<7, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
#else
      // convert from 0-2048 range to -1 to 1 fixedpointnum range and low pass filter to remove glitches
      lib_fp_lowpassfilter(&global.rxvalues[x], ((fixedpointnum)rxrawvalues[x]-1024)<<6, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
#endif
      }
   }


#elif (RX_TYPE==RX_CPPM)

unsigned char channelindex[]={RX_CHANNEL_ORDER};
volatile unsigned int rxrawvalues[RXNUMCHANNELS];

void serialsumcallback(unsigned char interruptnumber, unsigned char newstate) 
   { // gke
   static unsigned long starttime = 0;
   static unsigned char chan = 0;
   unsigned int width;
  
   if (newstate)
      {
      width = lib_timers_gettimermicrosecondsandreset(&starttime);
      
      if ( width > 3000)
         {
         global.failsafetimer=lib_timers_starttimer();  // reset the failsafe timer
         chan = 0;  
         }
      else if (chan < RXNUMCHANNELS)
         {
         if (width>900 && width<2050)
            rxrawvalues[chan] = width;
//         else
//            {
//            global.debugvalue[1] = width;
//            global.debugvalue[0] = ++glitches;
//            }
          
         chan++;
         }
      } 
   }

void readrx()
   {
   unsigned char chan;
   
   for (chan=0; chan<RXNUMCHANNELS;++chan) // convert from 1000-2000 range to -1 to 1 fixedpointnum range and low pass filter to remove glitches
      lib_fp_lowpassfilter(&global.rxvalues[channelindex[chan]], ((fixedpointnum)rxrawvalues[chan]-1500)*131L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
   }

void initrx()
   {
   for (int x=0;x<RXNUMCHANNELS;++x)
      global.rxvalues[x]=0; // middle position
   global.rxvalues[THROTTLEINDEX]=FPTHROTTLELOW; // default throttle to low position
      
   lib_digitalio_initpin(THROTTLE_RX_INPUT,DIGITALINPUT);
   lib_digitalio_setinterruptcallback(THROTTLE_RX_INPUT, serialsumcallback);   
   }


#elif (RX_TYPE==RX_SBUS)

#define SBUS_FRAME_BEGIN_BYTE 0x0F
#define SBUS_FRAME_END_BYTE 0x00

#define SBUS_FLAG_RESERVED_1        (1 << 0)
#define SBUS_FLAG_RESERVED_2        (1 << 1)
#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

// this is the s bus data format:
//struct sbusFrame_s {
//    uint8_t syncByte;
//    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
//    unsigned int chan0 : 11;
//    unsigned int chan1 : 11;
//    unsigned int chan2 : 11;
//    unsigned int chan3 : 11;
//    unsigned int chan4 : 11;
//    unsigned int chan5 : 11;
//    unsigned int chan6 : 11;
//    unsigned int chan7 : 11;
//    unsigned int chan8 : 11;
//    unsigned int chan9 : 11;
//    unsigned int chan10 : 11;
//    unsigned int chan11 : 11;
//    unsigned int chan12 : 11;
//    unsigned int chan13 : 11;
//    unsigned int chan14 : 11;
//    unsigned int chan15 : 11;
//    uint8_t flags;
//    uint8_t endByte;
//} __attribute__ ((__packed__));

unsigned char sbuschannelorder[]={RX_CHANNEL_ORDER};
unsigned long sbustimer;
unsigned char numframebytesreceived; // should cycle from zero through 24.  We won't save the 25th byte (end)
unsigned char rawframedata[24];
unsigned char goodframedata[24];
unsigned char gotnewpacketflag;

// this callback will get called whenever we receive a character on our sbus serial port
void sbusserialcallback(unsigned char c)
   {
   // if we haven't received any characters for 2500 microseconds, we may be out of sync. Start over.
   unsigned long microsecondssincelastchar=lib_timers_gettimermicrosecondsandreset(&sbustimer);
   if (microsecondssincelastchar>2500)
      numframebytesreceived=0;
      
   if (numframebytesreceived==0) // we are waiting for the sync byte
      {
      if (c!=SBUS_FRAME_BEGIN_BYTE) return;
      }
   
   else if (numframebytesreceived==24) // this byte should be the end byte
      {
      if (c==SBUS_FRAME_END_BYTE && gotnewpacketflag!=2)
         {
         // save this packet
         gotnewpacketflag=0; // keep main thread from processing the packet as we copy it
         for (int x=0;x<24;++x)
            goodframedata[x]=rawframedata[x];

         gotnewpacketflag=1;
         }

      numframebytesreceived=0; // start over
      return;
      }

   rawframedata[numframebytesreceived++]=c;
   }


void initrx()
   { // initialize the serial port for receiving sbus
   numframebytesreceived=0;
   gotnewpacketflag=0;
   
   sbustimer=lib_timers_starttimer();

   lib_serial_initport(RX_SERIAL_PORT,100000); // don't seem to need to set the parity to E and stop bits to 2
   lib_serial_setrxcallback(RX_SERIAL_PORT,sbusserialcallback);
   }   

void readrx()
   {
   // only update the values if we have new, verified data
   if (gotnewpacketflag)
      {
      gotnewpacketflag=2; // we are processing it.  Don't let the interrupt change it.
      
      // read the good frame data and set the values
      // convert from 0-2048 range to -1 to 1 fixedpointnum range
      global.rxvalues[sbuschannelorder[0]]  = (((fixedpointnum)(goodframedata[1] | goodframedata[2]<< 8) & 0x07FF)-1010)<<6;
      global.rxvalues[sbuschannelorder[1]]  = (((fixedpointnum)(goodframedata[2]>>3 | goodframedata[3]<<5) & 0x07FF)-1010)<<6;
      global.rxvalues[sbuschannelorder[2]]  = (((fixedpointnum)(goodframedata[3]>>6|goodframedata[4]<<2 | goodframedata[5]<<10) & 0x07FF)-1010)<<6;
      global.rxvalues[sbuschannelorder[3]]  = (((fixedpointnum)(goodframedata[5]>>1|goodframedata[6]<<7) & 0x07FF)-1010)<<6;
#if (RXNUMCHANNELS>4)
      global.rxvalues[sbuschannelorder[4]]  = (((fixedpointnum)(goodframedata[6]>>4|goodframedata[7]<<4) & 0x07FF)-1010)<<6;;
      global.rxvalues[sbuschannelorder[5]]  = (((fixedpointnum)(goodframedata[7]>>7|goodframedata[8]<<1|goodframedata[9]<<9) & 0x07FF)-1010)<<6;
#endif
#if (RXNUMCHANNELS>6)
      global.rxvalues[sbuschannelorder[6]]  = (((fixedpointnum)(goodframedata[9]>>2|goodframedata[10]<<6) & 0x07FF)-1010)<<6;
      global.rxvalues[sbuschannelorder[7]]  = (((fixedpointnum)(goodframedata[10]>>5|goodframedata[11]<<3) & 0x07FF)-1010)<<6;
#endif
#if (RXNUMCHANNELS==16)
      global.rxvalues[sbuschannelorder[8]]  = (((fixedpointnum)(goodframedata[12]|goodframedata[13]<< 8) & 0x07FF)-1010)<<6;
      global.rxvalues[sbuschannelorder[9]]  = (((fixedpointnum)(goodframedata[13]>>3|goodframedata[14]<<5) & 0x07FF)-1010)<<6;
      global.rxvalues[sbuschannelorder[10]] = (((fixedpointnum)(goodframedata[14]>>6|goodframedata[15]<<2|goodframedata[16]<<10) & 0x07FF)-1010)<<6;
      global.rxvalues[sbuschannelorder[11]] = (((fixedpointnum)(goodframedata[16]>>1|goodframedata[17]<<7) & 0x07FF)-1010)<<6;
      global.rxvalues[sbuschannelorder[12]] = (((fixedpointnum)(goodframedata[17]>>4|goodframedata[18]<<4) & 0x07FF)-1010)<<6;
      global.rxvalues[sbuschannelorder[13]] = (((fixedpointnum)(goodframedata[18]>>7|goodframedata[19]<<1|goodframedata[20]<<9) & 0x07FF)-1010)<<6;
      global.rxvalues[sbuschannelorder[14]] = (((fixedpointnum)(goodframedata[20]>>2|goodframedata[21]<<6) & 0x07FF)-1010)<<6;
      global.rxvalues[sbuschannelorder[15]] = (((fixedpointnum)(goodframedata[21]>>5|goodframedata[22]<<3) & 0x07FF)-1010)<<6;
#endif

      if (!(goodframedata[23] & SBUS_FLAG_FAILSAFE_ACTIVE))
         global.failsafetimer=lib_timers_starttimer();  // reset the failsafe timer
         
      gotnewpacketflag=0; // ready for a new one
      }
   }

#endif