/*
   mixer.hpp : Mixer class header

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
#include "stabilize.hpp"


namespace hf {

class Mixer {
public:
    int16_t  motorsDisarmed[4];

    void init(class RC * _rc, class Stabilize * _stabilize, Board * _board);

    void update(bool armed);

private:
    RC        * rc;
    Stabilize * stabilize;
    Board * board;

    // Custom mixer data per motor
    typedef struct motorMixer_t {
        float throttle;
        float roll;
        float pitch;
        float yaw;
    } motorMixer_t;

    static constexpr motorMixer_t mixerQuadX[] = {
        { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
        { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
        { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
        { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    };

};

}


#ifdef __arm__
extern "C" {
#endif
    //TODO put interface for arm
#ifdef __arm__
} // extern "C"
#endif
