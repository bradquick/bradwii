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

#define OUTPUT_TIMER1 0x10
#define OUTPUT_TIMER2 0x20
#define OUTPUT_TIMER3 0x30
#define OUTPUT_TIMER4 0x40
#define OUTPUT_TIMER5 0x50
#define OUTPUT_CHANNELA 0x01
#define OUTPUT_CHANNELB 0x02
#define OUTPUT_CHANNELC 0X03
#define OUTPUT_CHANNELD 0X04

void initoutputs();
//void setoutputs(unsigned int *values,char numvalues);
void setoutput(unsigned char outputchannel, unsigned int value);
void setmotoroutput(unsigned char motornum, unsigned char motorchannel,fixedpointnum fpvalue);
void setallmotoroutputs(int value);