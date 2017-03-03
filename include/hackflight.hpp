/*
   hackflight.hpp : general header

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

#include <cstdlib>
#include "board.hpp"
#include "imu.hpp"
#include "rc.hpp"
#include "mixer.hpp"
#include "msp.hpp"


namespace hf {

class Hackflight {

private:
    class TimedTask {
    private:
        uint32_t usec;
        uint32_t period;

    public:
        void init(uint32_t _period) {

            this->period = _period;
            this->usec = 0;
        }

        bool checkAndUpdate(uint32_t currentTime) {

            bool result = (int32_t)(currentTime - this->usec) >= 0;

            if (result)
                this->update(currentTime);

            return result;
        }

        void update(uint32_t currentTime) {

            this->usec = currentTime + this->period;
        }

        bool check(uint32_t currentTime) {

            return (int32_t)(currentTime - this->usec) >= 0;
        }
    };

public:
    void setup(Board *board_val);
    void loop(void);


private:
    TimedTask imuTask;
    TimedTask rcTask;
    TimedTask accelCalibrationTask;
    TimedTask altitudeEstimationTask;

    uint32_t imuLooptimeUsec;
    uint16_t calibratingGyroCycles;
    uint16_t calibratingAccCycles;
    uint16_t calibratingG;
    bool     haveSmallAngle;
    bool     armed;

    //objects we use
    IMU        imu;
    RC         rc;
    Mixer      mixer;
    MSP        msp;
    Stabilize  stab;
    Board      *board;
};

//global definitions
void debug(const char * fmt, ...);
}