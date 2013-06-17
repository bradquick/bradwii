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

#define NUMPOSSIBLECHECKBOXES 20 // allocate room for more checkboxes in eeprom memory in case we add them later

#define CHECKBOXARM 0
#define CHECKBOXAUTOTHROTTLE 1
#define CHECKBOXALTHOLD 2
#define CHECKBOXCOMPASS 3
#define CHECKBOXPOSITIONHOLD 4
#define CHECKBOXRETURNTOHOME 5
#define CHECKBOXSEMIACRO 6
#define CHECKBOXFULLACRO 7
#define CHECKBOXHIGHRATES 8
#define CHECKBOXHIGHANGLE 9
#define CHECKBOXAUTOTUNE 10
#define CHECKBOXUNCRASHABLE 11
#define NUMCHECKBOXES 12

#define CHECKBOXMASKARM (1<<CHECKBOXARM)
#define CHECKBOXMASKAUTOTHROTTLE (1<<CHECKBOXAUTOTHROTTLE)
#define CHECKBOXMASKALTHOLD (1<<CHECKBOXALTHOLD)
#define CHECKBOXMASKCOMPASS (1<<CHECKBOXCOMPASS)
#define CHECKBOXMASKPOSITIONHOLD (1<<CHECKBOXPOSITIONHOLD)
#define CHECKBOXMASKRETURNTOHOME (1<<CHECKBOXRETURNTOHOME)
#define CHECKBOXMASKSEMIACRO (1<<CHECKBOXSEMIACRO)
#define CHECKBOXMASKFULLACRO (1<<CHECKBOXFULLACRO)
#define CHECKBOXMASKHIGHRATES (1<<CHECKBOXHIGHRATES)
#define CHECKBOXMASKHIGHANGLE (1<<CHECKBOXHIGHANGLE)
#define CHECKBOXMASKAUTOTUNE (1<<CHECKBOXAUTOTUNE)
#define CHECKBOXMASKUNCRASHABLE (1<<CHECKBOXUNCRASHABLE)


#define CHECKBOXMASKAUX1LOW (1<<0)
#define CHECKBOXMASKAUX1MID (1<<1)
#define CHECKBOXMASKAUX1HIGH (1<<2)
#define CHECKBOXMASKAUX2LOW (1<<3)
#define CHECKBOXMASKAUX2MID (1<<4)
#define CHECKBOXMASKAUX2HIGH (1<<5)
#define CHECKBOXMASKAUX3LOW (1<<6)
#define CHECKBOXMASKAUX3MID (1<<7)
#define CHECKBOXMASKAUX3HIGH (1<<8)
#define CHECKBOXMASKAUX4LOW (1<<9)
#define CHECKBOXMASKAUX4MID (1<<10)
#define CHECKBOXMASKAUX4HIGH (1<<11)

void checkcheckboxitems();