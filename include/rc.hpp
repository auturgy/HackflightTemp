/*
   rc.hpp : RC receiver class header

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/mw.h

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "board.hpp"
#include "config.hpp"


namespace hf {

class RC {
private:
    int16_t dataAverage[CONFIG_RC_CHANS][4];
    uint8_t commandDelay;                               // cycles since most recent movement
    int32_t averageIndex;
    int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];     // lookup table for expo & RC rate PITCH+ROLL
    int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE
    int16_t midrc;
    bool    useSerial;
    Board*  board;

public:
    void init(Board * _board);

    int16_t data[CONFIG_RC_CHANS]; // raw PWM values for MSP
    int16_t command[4];            // stick PWM values for mixer, MSP
    uint8_t sticks;                // stick positions for command combos

    void update(void);

    bool changed(void);

    void computeExpo(void);

    uint8_t auxState(void);

    bool throttleIsDown(void);
};
} //namespace


#ifdef __arm__
extern "C" {
#endif

//TODO: define interface for ARM?

#ifdef __arm__
}
#endif
