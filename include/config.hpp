/*
   pidvals.hpp : PID values for a specific vehicle

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>


namespace hf {

//RC config
enum {
    DEMAND_ROLL = 0,
    DEMAND_PITCH,
    DEMAND_YAW,
    DEMAND_THROTTLE,
    DEMAND_AUX1,
    DEMAND_AUX2,
    DEMAND_AUX3,
    DEMAND_AUX4
};


// Define number of RC channels, and min/max PWM
#define CONFIG_RC_CHANS 8
#define CONFIG_PWM_MIN  990
#define CONFIG_PWM_MAX  2010

// For logical combinations of stick positions (low, center, high)
#define ROL_LO (1 << (2 * DEMAND_ROLL))
#define ROL_CE (3 << (2 * DEMAND_ROLL))
#define ROL_HI (2 << (2 * DEMAND_ROLL))
#define PIT_LO (1 << (2 * DEMAND_PITCH))
#define PIT_CE (3 << (2 * DEMAND_PITCH))
#define PIT_HI (2 << (2 * DEMAND_PITCH))
#define YAW_LO (1 << (2 * DEMAND_YAW))
#define YAW_CE (3 << (2 * DEMAND_YAW))
#define YAW_HI (2 << (2 * DEMAND_YAW))
#define THR_LO (1 << (2 * DEMAND_THROTTLE))
#define THR_CE (3 << (2 * DEMAND_THROTTLE))
#define THR_HI (2 << (2 * DEMAND_THROTTLE))

#define CONFIG_RC_EXPO_8                            65
#define CONFIG_RC_RATE_8                            90
#define CONFIG_THR_MID_8                            50
#define CONFIG_THR_EXPO_8                           0
#define CONFIG_MINCHECK                             1100
#define CONFIG_MAXCHECK                             1900

#define PITCH_LOOKUP_LENGTH    7
#define THROTTLE_LOOKUP_LENGTH 12


//PID config
// Level (accelerometer)
static const uint8_t CONFIG_LEVEL_P          = 10;
static const uint8_t CONFIG_LEVEL_I          = 1;

// Rate (gyro): P must be positive
static const uint8_t CONFIG_RATE_PITCHROLL_P = 5;
static const uint8_t CONFIG_RATE_PITCHROLL_I = 4;
static const uint8_t CONFIG_RATE_PITCHROLL_D = 3;

// Yaw: P must be positive
static const uint8_t CONFIG_YAW_P            = 40;
static const uint8_t CONFIG_YAW_I            = 20;

// For altitude hover
#define CONFIG_HOVER_ALT_P  120
#define CONFIG_HOVER_ALT_I  45
#define CONFIG_HOVER_ALT_D  1



// Config =====================================================

#define CONFIG_MAGNETIC_DECLINATION                 0

#define CONFIG_CALIBRATING_ACC_MSEC                 1400

#define CONFIG_YAW_CONTROL_DIRECTION                1    // 1 or -1 
#define CONFIG_RC_LOOPTIME_MSEC                     20
#define CONFIG_CALIBRATE_ACCTIME_MSEC               500
#define CONFIG_SMALL_ANGLE                          250  // tenths of a degree
#define CONFIG_ALTITUDE_UPDATE_MSEC                 25   // based on accelerometer low-pass filter

}