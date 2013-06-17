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

void navigation_sethometocurrentlocation();
void navigation_set_destination(fixedpointnum latitude,fixedpointnum longitude);
void navigation_setangleerror(unsigned char gotnewgpsreading,fixedpointnum *angleerror);
fixedpointnum navigation_getdistanceandbearing(fixedpointnum lat1,fixedpointnum lon1,fixedpointnum lat2,fixedpointnum lon2,fixedpointnum *bearing);
