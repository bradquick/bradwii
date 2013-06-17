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

#pragma once

#include "lib_fp.h"

// an attitude is defined as first a rotation from north, then a 3d vector that points down (gravity) from the rotated reference.
typedef struct
	{
	fixedpointnum westvector[3];
	fixedpointnum downvector[3];
	} attitudestruct;

void vectorcrossproduct(fixedpointnum *v1,fixedpointnum *v2,fixedpointnum *v3);
fixedpointnum normalizevector(fixedpointnum *v);
fixedpointnum vectordotproduct(fixedpointnum *v1,fixedpointnum *v2);
void vectordifferencetoeulerangles(fixedpointnum *v1,fixedpointnum *v2,fixedpointnum *euler);
void attitudetoeulerangles(attitudestruct *theattitude,fixedpointnum *eulerangles);
void rotatevectorwithsmallangles(fixedpointnum *v,fixedpointnum rolldeltaangle,fixedpointnum pitchdeltaangle,fixedpointnum yawdeltaangle);
void rotatevectorbyaxisangle(fixedpointnum *v1,fixedpointnum *axisvector,fixedpointnum angle,fixedpointnum *v2);
void rotatevectorbyaxissmallangle(fixedpointnum *v1,fixedpointnum *axisvector,fixedpointnum angle);
