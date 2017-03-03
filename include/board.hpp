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

namespace hf {

//this class abstracts various boards
class Board {
public:
    // Core functionality
    virtual void     delayMilliseconds(uint32_t msec) = 0;
    virtual void     dump(char * msg) = 0;
    virtual uint32_t getMicros() = 0;
    virtual void     imuInit(uint16_t & acc1G, float & gyroScale) = 0;
    virtual void     imuRead(int16_t accADC[3], int16_t gyroADC[3]) = 0;
    virtual void     init(uint32_t & imuLooptimeUsec, uint32_t & calibratingGyroMsec) = 0;
    virtual void     ledGreenOff(void) = 0;
    virtual void     ledGreenOn(void) = 0;
    virtual void     ledRedOff(void) = 0;
    virtual void     ledRedOn(void) = 0;
    
    virtual uint16_t readPWM(uint8_t chan) = 0;
    virtual void     writeMotor(uint8_t index, uint16_t value) = 0;


    //optional serial port
    virtual bool     rcUseSerial(void)
    {
        return false;
    }
    virtual uint16_t rcReadSerial(uint8_t chan)
    {
        return 0;
    }
    virtual bool     rcSerialReady(void)
    {
        return false;
    }
    virtual uint8_t  serialAvailableBytes(void)
    {
        return 0;
    }
    virtual uint8_t  serialReadByte(void)
    {
        return 0;
    }
    virtual void     serialWriteByte(uint8_t c)
    {
    }


    //board addons functionality
    virtual void     extrasCheckSwitch(void) 
    {}
    virtual uint8_t  extrasGetTaskCount(void)
    {
        return 0;
    }
    virtual bool     extrasHandleMSP(uint8_t command)
    {
        return true;
    }

    //TODO: this is causing circular reference
    //virtual void     extrasInit(MSP * _msp)
    //{}

    virtual void     extrasPerformTask(uint8_t taskIndex)
    {}

    //messaging during simulations
    virtual void     showArmedStatus(bool armed)
    {}
    virtual void     showAuxStatus(uint8_t status)
    {}

    //STM32
    virtual void     checkReboot(bool pendReboot)
    {}
    virtual void     reboot(void)
    {}
}; // class Board


} //namespace