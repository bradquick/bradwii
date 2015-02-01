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

#include "compass.h"
#include "defs.h"
#include "lib_i2c.h"
#include "lib_fp.h"
#include "lib_timers.h"
#include "bradwii.h"

extern globalstruct global;
extern usersettingsstruct usersettings;

// note: when adding new compassas, these functions need to be included:
// void initcompass()  // initializes the compass
// char readcompass()  // loads global.compassvector[] with the compass vector as fixedpointnum.  
//                       // returns 1 when returning a new reading, 0 if it's not time to read yet.
//                     // A unit vector good, but the length of the vector really isn't that important.  
//                     // We are more concerned with the direction of the vector.

fixedpointnum compassfilteredrawvalues[3]={0};

void compassfilterrawvalues(int *rawvalues)
   {
   for (int x=0;x<3;++x)
      {
      lib_fp_lowpassfilter(&compassfilteredrawvalues[x],rawvalues[x],FIXEDPOINTCONSTANT(.07),FIXEDPOINTCONSTANT(1.0/.125),0);
//global.debugvalue[x]=compassfilteredrawvalues[x];
      }
   }



#if (COMPASS_TYPE==NO_COMPASS)

void initcompass()
   {
   }
   
void calibratecompass()
   {
   }
   
char readcompass()
   {
   return(0);
   }

#else

#if (COMPASS_TYPE==HMC5843 || COMPASS_TYPE==HMC5883)

// ************************************************************************************************************
// I2C Compass HMC5843 & HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

unsigned long compasstimer;

void compassreadrawvalues(int *compassrawvalues)
   {
   unsigned char data[6];
   lib_i2c_readdata(MAG_ADDRESS,MAG_DATA_REGISTER,data,6);
   
#if (COMPASS_TYPE==HMC5843)
   COMPASS_ORIENTATION(compassrawvalues, ((data[0]<<8) | data[1]) ,
                     ((data[2]<<8) | data[3]) ,
                     ((data[4]<<8) | data[5]) );
#endif
#if (COMPASS_TYPE==HMC5883)  
    COMPASS_ORIENTATION(compassrawvalues, ((data[0]<<8) | data[1]) ,
                     ((data[4]<<8) | data[5]) ,
                     ((data[2]<<8) | data[3]) );
#endif

   // apply a low pass filter to the raw values.  They won't be raw anymore.
   
   
   // set the timer so we know when we can take our next reading
   compasstimer=lib_timers_starttimer();
   }

void initcompass() 
   { 
   lib_timers_delaymilliseconds(100);

   // set gains for calibration
   
   // Other programs use the self test mode of the compass chip to try to calibrate it.  I don't think this is
   // correct.  As far as I can tell, self test mode is just for seeing if it's working.  It's not for calibration.
   lib_i2c_writereg(MAG_ADDRESS ,0x00 ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
   lib_i2c_writereg(MAG_ADDRESS ,0x01 ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
   lib_i2c_writereg(MAG_ADDRESS ,0x02 ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
   }

char readcompass()
   { // returns 1 if we actually read something, zero otherwise.  Sets global.compassnorthvector to a unit vector (approximately)
   if (lib_timers_gettimermicroseconds(compasstimer)>=70000L)
      {
      int compassrawvalues[3];
      compassreadrawvalues(compassrawvalues);

      compassfilterrawvalues(compassrawvalues);
      // convert the raw values into a unit vector
      for (int x=0;x<3;++x)
         {
         global.compassvector[x]=lib_fp_multiply(((fixedpointnum)(compassfilteredrawvalues[x]-usersettings.compasszerooffset[x]))<<7,usersettings.compasscalibrationmultiplier[x]);
         }
      return(1);
      }
   else return(0);
   }

#endif

// MAG3110 code added thanks to Michael Buffington

#if (COMPASS_TYPE==MAG3110)

#define MAG_ADDRESS       0x0E
#define MAG_DATA_REGISTER 0x01
#define MAG_CTRL_REG1     0x10
#define MAG_CTRL_REG2     0x11

unsigned long compasstimer;

void compassreadrawvalues(int *compassrawvalues)
   {
   unsigned char data[6];
   lib_i2c_readdata(MAG_ADDRESS,MAG_DATA_REGISTER,data,6);

   /*
      You may need to invert values here depending on the orientation
      of your magnetometer.
   */
   COMPASS_ORIENTATION(compassrawvalues,
       ((data[0]<<8) | data[1]),
      -((data[2]<<8) | data[3]),
       ((data[4]<<8) | data[5]));

   // set the timer so we know when we can take our next reading
   compasstimer = lib_timers_starttimer();
   }

void mag_command(unsigned char value, int reg_num)
   {
   int reg;
   if (reg_num == 1) reg = MAG_CTRL_REG1;
   if (reg_num == 2) reg = MAG_CTRL_REG2;

   lib_timers_delaymilliseconds(100);
   lib_i2c_writereg(MAG_ADDRESS, reg, value);
   }

void initcompass()
   {
   /*
      Values for control register 1
      0x80 = DR:20Hz ; OS ratio:64 ; ADC rate: 1280Hz (from Multiwii 2.2)
      0x42 = DR:20Hz ; OS ratio:16 ; ADC rate:  320Hz (a guess)
   */

   mag_command(0x00, 1); // put in standby mode (not sure if this is useful)
   mag_command(0x80, 1); // set output, sampling, and ADC rates
   mag_command(0x11, 2); // set mag to automatically reset when it sees mag spikes
   mag_command(0x01, 1); // put mag in active mode
   }

char readcompass()
   {
   fixedpointnum a;
   fixedpointnum b;
   
   // returns 1 if we actually read something, zero otherwise.  Sets global.compassnorthvector to a unit vector (approximately)
   if (lib_timers_gettimermicroseconds(compasstimer) >= 70000L)
      {
      int compassrawvalues[3];
      compassreadrawvalues(compassrawvalues);
      compassfilterrawvalues(compassrawvalues);

      // convert the raw values into a unit vector
      for (int axis=0;axis<3;++axis)
         {
         a = (compassfilteredrawvalues[axis] - usersettings.compasszerooffset[axis]) << 7;
         b = usersettings.compasscalibrationmultiplier[axis];

         global.compassvector[axis] = lib_fp_multiply(a,b);
         }
      return(1);
      }

   return(0);
   }

#endif

#if COMPASS_TYPE==HMC5883_VIA_MPU6050

#define MPU6050_ADDRESS 0x68
#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

unsigned long compasstimer;

void initcompass()
   {
   lib_timers_delaymilliseconds(100);
   
   // Put the gyro/acc in i2c bypass mode so we can talk directly to the compas to set it up
   lib_i2c_writereg(MPU6050_ADDRESS, 0x37, 0x02);           //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
   
   // Initialize the compass
   // Other programs use the self test mode of the compass chip to try to calibrate it.  I don't think this is
   // correct.  As far as I can tell, self test mode is just for seeing if it's working.  It's not for calibration.
   lib_i2c_writereg(MAG_ADDRESS ,0x00 ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
   lib_i2c_writereg(MAG_ADDRESS ,0x01 ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
   lib_i2c_writereg(MAG_ADDRESS ,0x02 ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
  
   //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
   lib_i2c_writereg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
   lib_i2c_writereg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
   lib_i2c_writereg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
   lib_i2c_writereg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
   lib_i2c_writereg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
   lib_i2c_writereg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
   
   compasstimer=lib_timers_starttimer();
   }

void compassreadrawvalues(int *compassrawvalues)
   {
   unsigned char data[6];
   lib_i2c_readdata(MPU6050_ADDRESS,0x49,data,6);
   
#if (COMPASS_TYPE==HMC5843_VIA_MPU6050)
   COMPASS_ORIENTATION(compassrawvalues, ((data[0]<<8) | data[1]) ,
                     ((data[2]<<8) | data[3]) ,
                     ((data[4]<<8) | data[5]) );
#endif
#if (COMPASS_TYPE==HMC5883_VIA_MPU6050)  
    COMPASS_ORIENTATION(compassrawvalues, ((data[0]<<8) | data[1]) ,
                     ((data[4]<<8) | data[5]) ,
                     ((data[2]<<8) | data[3]) );
#endif
   // set the timer so we know when we can take our next reading
   compasstimer=lib_timers_starttimer();
   }

char readcompass()
   { // returns 1 if we actually read something, zero otherwise.  Sets global.compassvector to a unit vector (approximately)
   if (lib_timers_gettimermicroseconds(compasstimer)>=70000L)
      {
      int compassrawvalues[3];
      compassreadrawvalues(compassrawvalues);
      compassfilterrawvalues(compassrawvalues);

      // convert the raw values into a unit vector
      for (int x=0;x<3;++x)
         global.compassvector[x]=lib_fp_multiply(((fixedpointnum)(compassfilteredrawvalues[x]-usersettings.compasszerooffset[x]))<<7,usersettings.compasscalibrationmultiplier[x]);

      return(1);
      }
   else return(0);
      
   }

#endif

void calibratecompass()
   {
   int minvalues[3];
   int maxvalues[3];
   int rawvalues[3];
   
   for (int x=0;x<3;++x)
      {
      minvalues[x]=9999;
      maxvalues[x]=-9999;
      }
      
   // find the zero offsets for the raw compass readings.  Assume that someone is flipping
   // the copter in all directions for the next 30 seconds
   
   // wait until a new reading is ready
   while (lib_timers_gettimermicroseconds(compasstimer)<70000L) {}
   
   compassreadrawvalues(rawvalues);
   
   // use the general timer to count off our 30 seconds
   unsigned long testtimer=lib_timers_starttimer();
   while (lib_timers_gettimermicroseconds(testtimer)<30000000L)
      {
      // wait until a new reading is ready
      while (lib_timers_gettimermicroseconds(compasstimer)<70000L) {}
      compassreadrawvalues(rawvalues);
      compassfilterrawvalues(rawvalues);
      // the compass vectors that we gather should represent a sphere.  The problem is that the
      // center of the sphere may not be at 0,0,0 so we need to find out what they are
      for (int x=0;x<3;++x)
         {
         if (compassfilteredrawvalues[x]<minvalues[x]) minvalues[x]=compassfilteredrawvalues[x];
         if (compassfilteredrawvalues[x]>maxvalues[x]) maxvalues[x]=compassfilteredrawvalues[x];
         }
      }
      
   for (int x=0;x<3;++x)
      {
      usersettings.compasszerooffset[x]=(minvalues[x]+maxvalues[x])/2;
      usersettings.compasscalibrationmultiplier[x]=(1000L<<FIXEDPOINTSHIFT)/(maxvalues[x]-minvalues[x]);
      }
   }
   

#endif