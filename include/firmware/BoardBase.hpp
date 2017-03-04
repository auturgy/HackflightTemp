/*
board.hpp : class header for board-specific routines

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

#include <cstdint>
#include "SerialBase.hpp"

namespace hf {

//abstraction for minimal board
class BoardBase {
public: //interface
    virtual uint16_t readPWM(uint8_t chan) = 0;
    virtual void writeMotor(uint8_t index, uint16_t value) = 0;
    virtual bool rcUseSerial(void) = 0;
    virtual uint16_t rcReadSerial(uint8_t chan) = 0;
    virtual bool rcSerialReady(void) = 0;

    virtual SerialBase* getSerial() = 0;

    virtual void checkReboot(bool pendReboot) = 0;
    virtual void reboot(void) = 0;
}; // class BoardBase



} //namespace