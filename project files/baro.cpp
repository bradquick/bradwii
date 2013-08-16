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

#include "baro.h"
#include "lib_fp.h"
#include "lib_i2c.h"
#include "bradwii.h"
#include "lib_timers.h"
#include "math.h"

extern globalstruct global;

// note: when adding new compassas, these functions need to be included:
// void initbaro()   // initializes the barometer and doesn't return until a good reading has been set
// char readbaro()   // sets global.barorawaltitude in fixedpointnum meters  
//                     // returns 1 when returning a new reading, 0 if it's not time to read yet.
                  


#if (BAROMETER_TYPE==NO_BAROMETER)
void initbaro()
   {
   global.barorawaltitude=0;
   }
   
char readbaro()
   {
   return(0);
   }
   
#elif (BAROMETER_TYPE==BMP085)

#define BMP085_ADDRESS 0x77
#define BMP085_MODE 3 //we can get more unique samples and get better precision using average

unsigned long barotimer;

static struct 
   {
   // sensor registers from the BOSCH BMP085 datasheet
   int  ac1, ac2, ac3;
   unsigned int ac4, ac5, ac6;
   int  b1, b2, mb, mc, md;
   } bmp085_ctx;  

unsigned int bmp085_ut=0;

void bmp085_readcalibration()
   {
  //read calibration data in one go
   unsigned char s_bytes = (unsigned char*)&bmp085_ctx.md - (unsigned char*)&bmp085_ctx.ac1 + sizeof(bmp085_ctx.ac1);
   lib_i2c_readdata(BMP085_ADDRESS, 0xAA, (unsigned char *)&bmp085_ctx.ac1, s_bytes);
   // now fix endianness
   int *p;
   for (p = &bmp085_ctx.ac1; p <= &bmp085_ctx.md; p++) 
      {
      unsigned char *ptr=(unsigned char *)p;
      unsigned char c=*ptr;
      *ptr=*(ptr+1);
      *(ptr+1)=c;
      }
   }

void bmp085_starttemperaturereading() 
   {
   bmp085_ut=0; // by setting to zero, we will remember that we are waiting for temperature next time through the loop
   lib_i2c_writereg(BMP085_ADDRESS,0xf4,0x2e);
   barotimer=lib_timers_starttimer();
   }

void bmp085_startpressurereading() 
   {
   // start the pressure reading
   lib_i2c_writereg(BMP085_ADDRESS,0xf4,0x34+(BMP085_MODE<<6)); // control register value for oversampling setting 3
   barotimer=lib_timers_starttimer();
   }

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void bmp085_gettemperature()
   {
   unsigned char data[2];
   // read temperature
   lib_i2c_readdata(BMP085_ADDRESS, 0xF6, data, 2);
   bmp085_ut = (((unsigned int)data[0] <<8) | ((unsigned int)data[1]));
   }
   
void bmp085_getpressureandcalculatealtitude()
   { // this is the code provided by the manufacturer.  Unfortunately, it uses division, which is slow.
   unsigned long up;
   unsigned char data[3];
   // read pressure
   lib_i2c_readdata(BMP085_ADDRESS, 0xF6, data, 3);
   unsigned char *ptr=(unsigned char *)&up;
   *ptr++=data[2];
   *ptr++=data[1];
   *ptr++=data[0];
   *ptr=0;

   long  x1, x2, x3, b3, b5, b6, p, tmp;
   unsigned long b4, b7;
   long pressure;

   // Temperature calculations
   x1 = ((long)bmp085_ut - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;
   x2 = ((long)bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
   b5 = x1 + x2;
   // Pressure calculations
   b6 = b5 - 4000;
   x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12)) >> 11; 
   x2 = bmp085_ctx.ac2 * b6 >> 11;
   x3 = x1 + x2;
   tmp = bmp085_ctx.ac1;
   tmp = (tmp*4 + x3) << BMP085_MODE;
   b3 = (tmp+2)/4;
   x1 = bmp085_ctx.ac3 * b6 >> 13;
   x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12)) >> 16;
   x3 = ((x1 + x2) + 2) >> 2;
   b4 = (bmp085_ctx.ac4 * (unsigned long)(x3 + 32768)) >> 15;
   b7 = ((unsigned long) (up >> (8-BMP085_MODE)) - b3) * (50000 >> BMP085_MODE);
   p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
   x1 = (p >> 8) * (p >> 8);
   x1 = (x1 * 3038) >> 16;
   x2 = (-7357 * p) >> 16;
   pressure = p + ((x1 + x2 + 3791) >> 4);
  
   //   global.barorawaltitude=(1.0f - pow(pressure/101325.0f, 0.190295f)) * 4433000.0f; // this is the real formula (in centimeters)
   // estimate meters as a linear formula so we don't have to do all of the expensive math
   // meters=10351-.1024*p << close enough
   // 439804651L is .1024 << 16 << 16 (an extra 16 to convert pressure to a fixedpointnum)
   global.barorawaltitude=(10351L<<FIXEDPOINTSHIFT)-lib_fp_multiply(pressure,439804651L);
   }

void initbaro()
   {
   global.altitude=0;
   bmp085_readcalibration();
   bmp085_starttemperaturereading();
   
   while (readbaro()==0) {} // make sure we start off with a good reading
   }
   
char readbaro()
   { // returns 1 if we actually have a new reading
   if (bmp085_ut==0) // we are waiting for temperature
      { // wait 5 milliseconds
      if (lib_timers_gettimermicroseconds(barotimer)>5000L)
         {
         bmp085_gettemperature();
         bmp085_startpressurereading();
         }
      }
   else // we are waiting for pressure
      { // wait 26 milliseconds
      if (lib_timers_gettimermicroseconds(barotimer)>26000L)
         {
         bmp085_getpressureandcalculatealtitude();
         bmp085_starttemperaturereading();
         return(1);
         }
      }
   return(0);
   }

#elif BAROMETER_TYPE==MS5611

#define MS561101BA_ADDRESS 0x77 //CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)

// ************************************************************************************************************
// I2C Barometer MS561101BA
// ************************************************************************************************************
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12

// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096

long baroTemperature;

static struct
   {
   // sensor registers from the MS561101BA datasheet
   unsigned int c[7];
   union {unsigned long val; unsigned char raw[4]; } ut; //uncompensated T
   union {unsigned long val; unsigned char raw[4]; } up; //uncompensated P
   unsigned char  state;
   unsigned long barotimer;
   } ms561101ba_ctx;

void i2c_MS561101BA_reset()
   {
   lib_i2c_writereg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);
   }

void i2c_MS561101BA_readCalibration()
   {
   union {unsigned int val; unsigned char raw[2]; } data;
   for(unsigned char i=0;i<6;i++)
      {
      lib_i2c_rep_start(MS561101BA_ADDRESS<<1);
      lib_i2c_write(0xA2+2*i);
      lib_timers_delaymilliseconds(10);
      lib_i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);//I2C read direction => 1
      lib_timers_delaymilliseconds(10);
      data.raw[1] = lib_i2c_readack();  // read a 16 bit register
      data.raw[0] = lib_i2c_readnak();
      ms561101ba_ctx.c[i+1] = data.val;
      }
   }

// read uncompensated temperature value: send command first
void i2c_MS561101BA_UT_Start()
   {
   lib_i2c_rep_start(MS561101BA_ADDRESS<<1);      // I2C write direction
   lib_i2c_write(MS561101BA_TEMPERATURE + OSR);  // register selection
   lib_i2c_stop();
   }

// read uncompensated pressure value: send command first
void i2c_MS561101BA_UP_Start ()
   {
   lib_i2c_rep_start(MS561101BA_ADDRESS<<1);      // I2C write direction
   lib_i2c_write(MS561101BA_PRESSURE + OSR);     // register selection
   lib_i2c_stop();
   }

// read uncompensated pressure value: read result bytes
void i2c_MS561101BA_UP_Read ()
   {
   lib_i2c_rep_start(MS561101BA_ADDRESS<<1);
   lib_i2c_write(0);
   lib_i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);
   ms561101ba_ctx.up.raw[2] = lib_i2c_readack();
   ms561101ba_ctx.up.raw[1] = lib_i2c_readack();
   ms561101ba_ctx.up.raw[0] = lib_i2c_readnak();
   }

// read uncompensated temperature value: read result bytes
void i2c_MS561101BA_UT_Read()
   {
   lib_i2c_rep_start(MS561101BA_ADDRESS<<1);
   lib_i2c_write(0);
   lib_i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);
   ms561101ba_ctx.ut.raw[2] = lib_i2c_readack();
   ms561101ba_ctx.ut.raw[1] = lib_i2c_readack();
   ms561101ba_ctx.ut.raw[0] = lib_i2c_readnak();
   }

void i2c_MS561101BA_Calculate()
   {
   long off2,sens2,delt;

   long long dT       = (long)ms561101ba_ctx.ut.val - ((long)ms561101ba_ctx.c[5] << 8);
   baroTemperature  = 2000 + ((dT * ms561101ba_ctx.c[6])>>23);
   long long off      = ((unsigned long)ms561101ba_ctx.c[2] <<16) + ((dT * ms561101ba_ctx.c[4]) >> 7);
   long long sens     = ((unsigned long)ms561101ba_ctx.c[1] <<15) + ((dT * ms561101ba_ctx.c[3]) >> 8);

   if (baroTemperature < 2000)
      { // temperature lower than 20st.C
      delt = baroTemperature-2000;
      delt  = 5*delt*delt;
      off2  = delt>>1;
      sens2 = delt>>2;
      if (baroTemperature < -1500)
         { // temperature lower than -15st.C
         delt  = baroTemperature+1500;
         delt  = delt*delt;
         off2  += 7 * delt;
         sens2 += (11 * delt)>>1;
         }
      off  -= off2; 
      sens -= sens2;
      }

   long pressure     = (( (ms561101ba_ctx.up.val * sens ) >> 21) - off) >> 15;

   //   global.barorawaltitude=(1.0f - pow(pressure/101325.0f, 0.190295f)) * 4433000.0f; // this is the real formula (in centimeters)
   // estimate meters as a linear formula so we don't have to do all of the expensive math
   // meters=10351-.1024*p << close enough
   // 439804651L is .1024 << 16 << 16 (an extra 16 to convert pressure to a fixedpointnum)
   global.barorawaltitude=(10351L<<FIXEDPOINTSHIFT)-lib_fp_multiply(pressure,439804651L);
   }

void initbaro()
   {
   lib_timers_delaymilliseconds(10);
   i2c_MS561101BA_reset();
   lib_timers_delaymilliseconds(100);
   i2c_MS561101BA_readCalibration();
   lib_timers_delaymilliseconds(10);
   i2c_MS561101BA_UT_Start(); 
   ms561101ba_ctx.barotimer = lib_timers_starttimer();
   }

char readbaro()
   {                            
   //return 0: no new data available, no computation ;  1: new value available
   // first UT conversion is started in init procedure      
   if (lib_timers_gettimermicroseconds(ms561101ba_ctx.barotimer) < 10000)
      return 0;

   // reset the timer for the next reading
   ms561101ba_ctx.barotimer = lib_timers_starttimer();  // UT and UP conversion take 8.5ms so we do next reading after 10ms

   // either the temperature or pressure reading has finished.  Which one?
   if (ms561101ba_ctx.state == 0)
      {
      i2c_MS561101BA_UT_Read(); 
      i2c_MS561101BA_UP_Start(); 
      ms561101ba_ctx.state = 1;
      return 0;
      }
   else
      {
      i2c_MS561101BA_UP_Read();
      i2c_MS561101BA_UT_Start(); 
      i2c_MS561101BA_Calculate();
      ms561101ba_ctx.state = 0; 
      return 1;
      }
   }

#endif
