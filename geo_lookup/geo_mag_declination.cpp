/****************************************************************************
 *
 *   Copyright (c) 2014 MAV GEO Library (MAVGEO). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name MAVGEO nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
* @file geo_mag_declination.cpp
*
* Calculation / lookup table for Earth's magnetic field declination (deg), inclination (deg) and strength (mTesla).
* Data generated by https://www.ngdc.noaa.gov/geomag-web/#igrfgrid IGRF calculator on 20 Feb 2019
*
* XXX Lookup table currently too coarse in resolution (only full degrees)
* and lat/lon res - needs extension medium term.
*
*/

#include "geo_mag_declination.h"

#include <mathlib/mathlib.h>

#include <stdint.h>

using math::constrain;

/** set this always to the sampling in degrees for the table below */
static constexpr float SAMPLING_RES = 10.0f;
static constexpr float SAMPLING_MIN_LAT	= -90.0f;
static constexpr float SAMPLING_MAX_LAT	= 90.0f;
static constexpr float SAMPLING_MIN_LON	= -180.0f;
static constexpr float SAMPLING_MAX_LON	= 180.0f;
enum Table {declination, inclination, strength};

// Wingtra: extended declination data in degrees
static constexpr const int16_t declination_table[19][37] = \
{
    { 149,139,129,119,109,99,89,79,69,59,49,39,29,19,9,-1,-11,-21,-31,-41,-51,-61,-71,-81,-91,-101,-111,-121,-131,-141,-151,-161,-171,179,169,159,149 },
    { 130,117,106,96,87,78,69,61,53,46,38,30,23,15,8,0,-7,-15,-23,-31,-40,-49,-58,-67,-76,-86,-96,-107,-119,-131,-144,-158,-172,172,157,143,130       },
    { 86,78,71,66,61,56,51,46,41,35,29,23,16,10,4,-1,-7,-14,-20,-28,-36,-44,-52,-60,-68,-76,-84,-93,-102,-113,-127,-149,177,139,112,96,86             },
    { 48,46,45,43,42,41,39,37,33,28,23,16,10,4,-1,-6,-10,-15,-20,-27,-34,-42,-49,-56,-62,-68,-72,-74,-75,-73,-61,-21,26,43,47,48,48                   },
    { 31,31,31,30,30,30,30,29,27,24,18,11,3,-4,-9,-13,-15,-18,-21,-27,-34,-41,-47,-52,-56,-57,-56,-52,-44,-30,-14,2,14,22,27,30,31                    },
    { 22,23,23,23,22,22,22,23,22,19,13,5,-4,-12,-17,-20,-22,-22,-23,-26,-31,-36,-42,-45,-46,-44,-39,-31,-21,-11,-3,4,10,15,19,21,22                   },
    { 17,17,18,18,17,17,17,17,16,13,8,-1,-10,-18,-22,-25,-26,-25,-22,-20,-21,-25,-30,-32,-31,-28,-23,-16,-9,-3,0,4,7,11,14,16,17                      },
    { 13,13,14,14,14,13,13,12,11,9,3,-5,-14,-20,-24,-25,-24,-21,-16,-11,-9,-11,-14,-17,-18,-16,-12,-8,-3,0,1,3,6,8,11,12,13                           },
    { 11,11,11,11,11,10,10,10,8,5,0,-8,-16,-21,-23,-22,-19,-15,-10,-5,-2,-2,-4,-7,-9,-8,-6,-4,-1,1,1,2,4,7,9,10,11                                    },
    { 10,9,9,9,9,9,9,8,7,3,-3,-10,-16,-20,-20,-18,-13,-9,-5,-2,1,2,0,-2,-4,-4,-3,-2,0,0,0,1,3,5,7,9,10                                                },
    { 9,9,9,9,9,9,9,8,5,1,-4,-11,-16,-18,-17,-14,-10,-5,-2,0,2,3,2,0,-1,-2,-2,-1,0,-1,-1,-1,1,3,6,8,9                                                 },
    { 8,9,9,10,10,10,10,8,5,0,-6,-12,-15,-16,-15,-11,-7,-3,-1,1,3,4,3,2,1,0,0,0,-1,-2,-3,-4,-2,0,3,6,8                                                },
    { 6,9,10,11,12,12,11,9,5,-1,-8,-13,-15,-15,-13,-10,-6,-3,0,2,3,4,4,4,3,2,1,0,-1,-3,-5,-6,-6,-3,0,4,6                                              },
    { 5,8,11,13,14,15,13,10,5,-2,-9,-14,-16,-16,-13,-10,-6,-3,0,3,4,6,6,6,6,5,4,2,-1,-5,-8,-9,-9,-7,-3,1,5                                            },
    { 3,7,11,14,16,17,16,12,5,-4,-12,-17,-19,-18,-15,-12,-8,-4,0,3,5,8,9,10,10,10,7,4,-1,-6,-11,-12,-12,-9,-5,-1,3                                    },
    { 3,7,12,16,18,19,18,13,4,-8,-18,-23,-25,-23,-20,-16,-11,-6,-1,3,7,11,14,16,18,17,14,8,0,-8,-13,-15,-14,-11,-7,-2,3                               },
    { 2,7,12,17,20,21,19,12,-3,-19,-30,-34,-34,-31,-26,-21,-15,-9,-2,4,10,15,20,25,27,28,25,18,6,-7,-15,-17,-16,-13,-9,-3,2                           },
    { 1,6,11,14,16,14,4,-17,-38,-49,-51,-49,-44,-39,-32,-25,-17,-10,-2,6,13,21,28,34,40,45,47,45,36,18,-2,-12,-15,-13,-10,-5,1                        },
    { -179,-169,-159,-149,-139,-129,-119,-109,-99,-89,-79,-69,-59,-49,-39,-29,-19,-9,1,11,21,31,41,51,60,70,80,90,100,110,121,131,141,151,161,171,-179},
};

// Wingtra: extended inclination data in degrees
static constexpr const int8_t inclination_table[19][37] = \
{
    { -72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72,-72},
    { -78,-78,-77,-76,-75,-73,-72,-71,-70,-69,-68,-67,-67,-66,-66,-65,-65,-65,-66,-66,-66,-67,-68,-69,-70,-71,-73,-74,-75,-76,-77,-78,-79,-79,-79,-79,-78},
    { -81,-79,-77,-75,-74,-72,-70,-67,-65,-64,-62,-61,-60,-60,-60,-60,-60,-60,-60,-61,-61,-62,-64,-66,-68,-70,-73,-75,-78,-81,-83,-85,-86,-86,-84,-83,-81},
    { -78,-76,-74,-72,-70,-68,-65,-63,-60,-57,-55,-54,-54,-55,-56,-57,-58,-59,-59,-59,-59,-60,-61,-63,-66,-69,-73,-76,-80,-83,-86,-87,-86,-84,-82,-80,-78},
    { -72,-70,-68,-66,-64,-62,-60,-57,-54,-51,-49,-48,-49,-51,-55,-58,-60,-61,-61,-60,-60,-60,-61,-63,-66,-69,-73,-76,-78,-80,-81,-80,-79,-77,-76,-74,-72},
    { -64,-62,-60,-59,-57,-55,-53,-50,-47,-44,-41,-41,-43,-48,-53,-58,-62,-65,-66,-65,-63,-62,-61,-63,-65,-68,-71,-73,-73,-74,-73,-72,-71,-70,-68,-66,-64},
    { -55,-53,-51,-49,-46,-44,-42,-40,-37,-33,-31,-31,-35,-41,-49,-55,-60,-65,-67,-68,-66,-63,-61,-61,-62,-64,-65,-66,-66,-65,-64,-63,-62,-61,-59,-57,-55},
    { -42,-40,-37,-35,-33,-30,-28,-25,-22,-18,-15,-16,-22,-31,-40,-49,-55,-60,-63,-63,-61,-58,-55,-53,-53,-54,-55,-55,-54,-53,-51,-51,-50,-49,-47,-45,-42},
    { -25,-22,-20,-17,-15,-12,-10,-7,-3,1,3,1,-6,-16,-28,-38,-44,-48,-50,-50,-48,-44,-40,-38,-38,-38,-39,-39,-38,-36,-35,-35,-35,-34,-31,-28,-25         },
    { -5,-2,1,3,5,8,10,13,16,20,21,19,12,1,-10,-20,-27,-30,-30,-29,-27,-23,-19,-17,-16,-17,-18,-18,-17,-16,-15,-16,-16,-15,-13,-9,-5                     },
    { 15,18,21,23,24,27,29,31,34,36,36,34,28,19,10,2,-4,-5,-5,-4,-2,2,5,7,8,7,7,7,7,7,7,6,5,5,7,11,15                                                    },
    { 31,34,36,38,40,41,43,46,48,49,49,46,42,35,29,24,20,19,20,21,23,25,28,30,30,30,29,29,29,29,28,27,25,25,26,28,31                                     },
    { 43,46,47,49,51,53,55,57,58,59,58,56,53,49,45,42,40,39,40,41,43,44,46,47,47,47,47,47,47,47,46,44,42,41,40,42,43                                     },
    { 53,54,56,57,59,61,64,66,67,68,67,65,62,59,57,55,54,54,55,56,57,58,59,59,60,60,60,61,61,60,59,57,55,53,52,52,53                                     },
    { 62,63,64,65,67,69,71,73,75,75,74,72,70,68,67,66,65,65,65,66,66,67,68,68,69,70,70,71,71,70,69,67,65,63,62,62,62                                     },
    { 71,71,72,73,75,77,78,80,81,81,80,79,77,76,74,73,73,73,73,73,73,74,74,75,76,77,78,78,79,78,77,75,74,72,71,71,71                                     },
    { 79,79,80,81,82,83,84,85,86,86,85,83,82,81,80,79,79,78,78,78,79,79,79,80,81,82,83,84,84,84,83,82,81,80,79,79,79                                     },
    { 86,86,86,87,87,88,88,89,88,88,87,86,86,85,84,84,83,83,83,83,83,84,84,84,85,86,87,87,88,88,88,88,87,87,86,86,86                                     },
    { 88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88                                     },
};

// Wingtra: extended strength data in centi-Tesla
static constexpr const int8_t strength_table[19][37] = \
{
    { 55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55},
    { 61,60,59,58,57,56,55,54,53,52,50,49,48,48,47,46,46,46,46,47,48,48,50,51,52,54,55,57,58,59,60,61,61,62,61,61,61},
    { 63,62,60,59,57,55,53,51,49,47,45,43,41,40,39,38,37,37,38,38,40,41,44,46,49,52,55,58,61,63,64,65,66,66,65,64,63},
    { 62,60,58,56,54,52,49,46,43,40,38,35,34,32,31,30,30,30,30,31,33,35,38,42,46,51,55,59,62,64,66,67,67,66,65,64,62},
    { 59,56,54,52,49,47,44,41,38,35,32,29,28,27,26,26,25,25,25,26,28,30,34,39,44,49,54,58,61,64,65,66,65,64,63,61,59},
    { 54,52,49,47,45,42,40,37,34,30,27,25,24,24,24,24,24,24,24,24,25,28,32,37,42,48,52,56,59,61,62,62,62,60,59,56,54},
    { 49,46,44,42,40,37,35,33,30,28,25,23,22,23,23,24,25,25,26,26,26,28,31,36,41,46,51,54,56,57,57,57,56,55,53,51,49},
    { 43,41,39,37,35,33,32,30,28,26,24,23,23,23,24,25,26,28,29,29,29,30,32,36,40,45,48,51,52,52,51,51,50,49,47,45,43},
    { 38,36,35,33,32,31,30,29,28,27,26,25,24,24,25,26,28,30,31,32,32,32,33,35,39,42,44,46,47,46,45,45,44,43,41,40,38},
    { 34,33,32,32,31,31,31,30,30,30,29,28,27,27,27,28,29,31,32,33,33,33,34,35,37,39,41,42,43,42,41,40,39,38,36,35,34},
    { 33,33,32,32,33,33,34,34,35,35,34,33,31,30,30,30,31,32,33,34,35,35,36,37,39,40,41,42,42,41,40,39,37,36,34,33,33},
    { 34,34,34,35,36,37,39,40,41,41,40,39,37,35,34,34,35,35,36,37,38,39,40,41,42,43,44,45,45,45,43,41,39,37,35,34,34},
    { 37,37,38,39,41,42,44,46,47,47,46,45,43,41,40,39,39,40,41,42,42,43,45,46,47,48,49,50,51,50,49,46,43,41,39,38,37},
    { 42,42,43,44,46,48,50,51,52,53,52,50,49,47,45,45,44,44,45,46,46,47,49,50,51,53,54,55,56,56,54,52,49,46,44,43,42},
    { 48,48,49,50,52,53,55,56,57,57,56,55,53,51,50,49,48,48,48,49,49,50,52,53,55,57,58,59,60,60,58,56,54,52,50,49,48},
    { 54,54,54,55,56,57,58,58,59,58,58,57,56,54,53,52,51,51,51,51,52,53,54,55,57,58,60,61,62,61,61,59,58,56,55,54,54},
    { 57,57,57,57,58,58,58,58,58,58,57,57,56,55,55,54,53,53,53,53,54,54,55,56,57,59,60,60,61,61,61,60,59,59,58,58,57},
    { 58,58,58,57,57,57,57,57,57,57,56,56,56,56,55,55,55,55,55,55,55,56,56,56,57,57,58,58,59,59,59,59,58,58,58,58,58},
    { 57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57},
};

static unsigned
get_lookup_table_index(float *val, float min, float max)
{
	/* for the rare case of hitting the bounds exactly
	 * the rounding logic wouldn't fit, so enforce it.
	 */

	/* limit to table bounds - required for maxima even when table spans full globe range */
	/* limit to (table bounds - 1) because bilinear interpolation requires checking (index + 1) */
	*val = constrain(*val, min, max - SAMPLING_RES);

	return static_cast<unsigned>((-(min) + *val) / SAMPLING_RES);
}

// WINGTRA: Adaptation for extended declination table
static void
get_data_min_max(Table table_type, const unsigned& min_lat_index, const unsigned& min_lon_index, const float& lon_scale,
				float& data_min, float& data_max)
{
	float data_sw, data_se, data_ne, data_nw;

	switch(table_type) {
		case declination:
			data_sw = declination_table[min_lat_index][min_lon_index];
			data_se = declination_table[min_lat_index][min_lon_index + 1];
			data_ne = declination_table[min_lat_index + 1][min_lon_index + 1];
			data_nw = declination_table[min_lat_index + 1][min_lon_index];
			break;

		case inclination:
			data_sw = inclination_table[min_lat_index][min_lon_index];
			data_se = inclination_table[min_lat_index][min_lon_index + 1];
			data_ne = inclination_table[min_lat_index + 1][min_lon_index + 1];
			data_nw = inclination_table[min_lat_index + 1][min_lon_index];
			break;

		case strength:
			data_sw = strength_table[min_lat_index][min_lon_index];
			data_se = strength_table[min_lat_index][min_lon_index + 1];
			data_ne = strength_table[min_lat_index + 1][min_lon_index + 1];
			data_nw = strength_table[min_lat_index + 1][min_lon_index];
			break;
			
		default:
			// should never enter this case
			data_min = 0;
			data_max = 0;
			return;
	}

	data_min = lon_scale * (data_se - data_sw) + data_sw;
	data_max = lon_scale * (data_ne - data_nw) + data_nw;
}

static float
get_table_data(float lat, float lon, /* WINGTRA: const int8_t table[13][37]*/ Table table_type)
{
	/*
	 * If the values exceed valid ranges, return zero as default
	 * as we have no way of knowing what the closest real value
	 * would be.
	 */
	if (lat < -90.0f || lat > 90.0f ||
	    lon < -180.0f || lon > 180.0f) {
		return 0.0f;
	}

	/* round down to nearest sampling resolution */
	float min_lat = int(lat / SAMPLING_RES) * SAMPLING_RES;
	float min_lon = int(lon / SAMPLING_RES) * SAMPLING_RES;

	/* find index of nearest low sampling point */
	unsigned min_lat_index = get_lookup_table_index(&min_lat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
	unsigned min_lon_index = get_lookup_table_index(&min_lon, SAMPLING_MIN_LON, SAMPLING_MAX_LON);

	/* WINGTRA: this is calculated in another function
	const float data_sw = table[min_lat_index][min_lon_index];
	const float data_se = table[min_lat_index][min_lon_index + 1];
	const float data_ne = table[min_lat_index + 1][min_lon_index + 1];
	const float data_nw = table[min_lat_index + 1][min_lon_index];
	*/

	/* Wingtra: adapt bilinear interpolation on the four grid corners */
	float data_min, data_max;
	const float lat_scale = constrain((lat - min_lat) / SAMPLING_RES, 0.0f, 1.0f);
	const float lon_scale = constrain((lon - min_lon) / SAMPLING_RES, 0.0f, 1.0f);
	get_data_min_max(table_type, min_lat_index, min_lon_index, lon_scale, data_min, data_max);
	/* WINGTRA END */

	return lat_scale * (data_max - data_min) + data_min;
}

float get_mag_declination(float lat, float lon)
{
	return get_table_data(lat, lon, declination); // WINGTRA
}

float get_mag_inclination(float lat, float lon)
{
	return get_table_data(lat, lon, inclination); // WINGTRA
}

float get_mag_strength(float lat, float lon)
{
	return get_table_data(lat, lon, strength); // WINGTRA
}
