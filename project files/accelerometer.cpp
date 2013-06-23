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

#include "bradwii.h"
#include "defs.h"
#include "rx.h"
#include "lib_i2c.h"
#include "accelerometer.h"
#include "lib_fp.h"
#include "lib_timers.h"

// when adding accelerometers, you need to include the following functions:
// void initacc() // initializes the accelerometer
// void readacc() // loads global.acc_g_vector with acc values in fixedpointnum g's

extern globalstruct global;

#if (ACCELEROMETER_TYPE==BMA180)

#if !defined(BMA180_ADDRESS) 
  #define BMA180_ADDRESS 0x40
#endif

// ************************************************************************************************************
// I2C Accelerometer BMA180
// ************************************************************************************************************
// I2C adress: 0x80 (8bit)    0x40 (7bit) (SDO connection to VCC) 
// I2C adress: 0x82 (8bit)    0x41 (7bit) (SDO connection to VDDIO)
// Resolution: 14bit
//
// Control registers:
//
// 0x20    bw_tcs:      |                                           bw<3:0> |                        tcs<3:0> |
//                      |                                             150Hz |                        xxxxxxxx |
// 0x30    tco_z:       |                                                tco_z<5:0>    |     mode_config<1:0> |
//                      |                                                xxxxxxxxxx    |                   00 |
// 0x35    offset_lsb1: |          offset_x<3:0>              |                   range<2:0>       | smp_skip |
//                      |          xxxxxxxxxxxxx              |                    8G:   101       | xxxxxxxx |
// ************************************************************************************************************

void initacc() 
   {
   lib_timers_delaymilli(10);
   //default range 2G: 1G = 4096 unit.
   lib_i2c_writereg(BMA180_ADDRESS,0x0D,1<<4); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
   lib_timers_delaymilli(5);
   unsigned char control = lib_i2c_readreg(BMA180_ADDRESS, 0x20);
   control = control & 0x0F;        // save tcs register
   control = control | (0x01 << 4); // register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 20Hz
   lib_i2c_writereg(BMA180_ADDRESS, 0x20, control);
   lib_timers_delaymilli(5);
   control = lib_i2c_readreg(BMA180_ADDRESS, 0x30);
   control = control & 0xFC;        // save tco_z register
   control = control | 0x00;        // set mode_config to 0
   lib_i2c_writereg(BMA180_ADDRESS, 0x30, control);
   lib_timers_delaymilli(5); 
   control = lib_i2c_readreg(BMA180_ADDRESS, 0x35);
   control = control & 0xF1;        // save offset_x and smp_skip register
   control = control | (0x05 << 1); // set range to 8G
   lib_i2c_writereg(BMA180_ADDRESS, 0x35, control);
   lib_timers_delaymilli(5); 
   }

void readacc() 
   {
   unsigned char data[6];
   lib_i2c_readdata(BMA180_ADDRESS,0x02,(unsigned char *)&data,6);

   // convert readings to fixedpointnum (in g's)
  //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /4 => 12 bit resolution
  ACC_ORIENTATION(global.acc_g_vector, (((data[1]<<8) | data[0])>>2) * 64L,
                                       (((data[3]<<8) | data[2])>>2) * 64L,
                                       (((data[5]<<8) | data[4])>>2) * 64L);
   }
   
#elif (ACCELEROMETER_TYPE==MPU6050)
#define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board

void initacc() 
   {
   lib_i2c_writereg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
   }
   
void readacc() 
   {
   unsigned char data[6];

   lib_i2c_readdata(MPU6050_ADDRESS,0x3B,(unsigned char *)&data,6);

   // convert readings to fixedpointnum (in g's)
   //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /4 => 12 bit resolution
   ACC_ORIENTATION(global.acc_g_vector, (((data[0]<<8) | data[1])>>2) * 64L,
                                       (((data[2]<<8) | data[3])>>2) * 64L,
                                       (((data[4]<<8) | data[5])>>2) * 64L);
   }
#endif
