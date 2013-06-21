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

#include <avr/eeprom.h>
#include "eeprom.h"
#include "bradwii.h"

extern usersettingsstruct usersettings;
extern globalstruct global;

#define MAGICNUMBER 12345

void writeusersettingstoeeprom()
	{
	int magicnumber=MAGICNUMBER;
	int size=sizeof(usersettingsstruct);
	
	eeprom_write_block((const void*)&magicnumber, (void*)0, sizeof(magicnumber));
	eeprom_write_block((const void*)&size, (void*)2, sizeof(size));
	eeprom_write_block((const void*)&usersettings, (void*)4, size);
	}
	
void readusersettingsfromeeprom()
	{
	int magicnumber=0;
	int size=0;
	eeprom_read_block((void*)&magicnumber, (void*)0, sizeof(magicnumber));

	if (magicnumber!=MAGICNUMBER) return;

	eeprom_read_block((void*)&size, (void*)2, sizeof(size));
	if (size>sizeof(usersettingsstruct)) size=sizeof(usersettingsstruct);
	
	eeprom_read_block((void*)&usersettings, (void*)4, size);
	
	global.usersettingsfromeeprom=1; // set a flag so the rest of the program know it's working with calibtated settings
	}