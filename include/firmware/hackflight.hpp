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
#include "ImuBoardBase.hpp"
#include "mixer.hpp"
#include "msp.hpp"
#include "common.hpp"

namespace hf {

class Hackflight {
public:
    void init(ImuBoardBase* _board);
    void update(void);


private:
    bool     armed;

    //objects we use
    Mixer mixer;
    MSP msp;
    Stabilize stab;
    ImuBoardBase *board;

    int16_t motors[4];
}; //class


/********** CPP *******************/

void Hackflight::init(ImuBoardBase* _board)
{
    board = _board;
    board->init();

    stab.init(board->getRc(), board->getImu());
    mixer.init(board->getRc(), &stab); 
    msp.init(board->getImu(), &mixer, board->getRc(), board);

    // do any extra initializations (baro, sonar, etc.)
    //TODO: re-enable this
    //board->extrasInit(&msp);

    // ensure not armed
    armed = false;
}

void Hackflight::update(void)
{
    bool isMoving;
    board->update(armed, isMoving);
    if (!isMoving)
        stab.resetIntegral();

    // handle serial communications
    msp.update(armed);

    // update stability PID controller 
    stab.update();

    // update mixer
    mixer.update(armed, motors);

    for (int i = 0; i < 4; ++i)
        board->writeMotor(i, motors[i]);

} // loop()


} //namespace