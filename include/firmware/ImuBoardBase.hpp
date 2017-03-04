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
#include "BoardBase.hpp"
#include "imu.hpp"
#include "rc.hpp"
#include "TimedTask.hpp"

namespace hf {

//Abstract minimal board with RC and IMU capability
class ImuBoardBase : public BoardBase {
public: //interface
    // Core functionality
    virtual void init();
    virtual void update(bool& armed, bool& isMoving);


    virtual const Config& getConfig() = 0;
    virtual void delayMilliseconds(uint32_t msec) = 0;
    virtual void dump(char * msg) = 0;
    virtual void imuRead(IMU::ADC& adc) = 0;
    //Returns the number of microseconds since the Arduino board began running the current program
    //In Arduino, this will go overflow, resolution is 4-8 microseconds depending on flavor
    virtual uint64_t getMicros() = 0;


    /************************* Redeclare/provide defaults for BoardBase ***************************/
    virtual uint16_t readPWM(uint8_t chan) override = 0;
    virtual void writeMotor(uint8_t index, uint16_t value) override = 0;

    //optional serial port
    virtual bool rcUseSerial(void) override
    {
        return false;
    }
    virtual uint16_t rcReadSerial(uint8_t chan) override
    {
        return 0;
    }
    virtual bool rcSerialReady(void) override
    {
        return false;
    }
    virtual uint8_t serialAvailableBytes(void) override
    {
        return 0;
    }
    virtual uint8_t serialReadByte(void) override
    {
        return 0;
    }
    virtual void serialWriteByte(uint8_t c) override
    {
    }

    //STM32
    virtual void checkReboot(bool pendReboot) override
    {}
    virtual void reboot(void) override
    {}
    /*********************************************************************************************/


    //optional board addons functionality
    virtual void setLed(int8_t id, bool is_on, float max_brightness = 255)
    {}
    virtual void extrasCheckSwitch(void) 
    {}
    virtual uint8_t  extrasGetTaskCount(void)
    {
        return 0;
    }
    virtual bool     extrasHandleMSP(uint8_t command)
    {
        return true;
    }

    //TODO: this is causing circular reference, need to think about design
    //virtual void extrasInit(MSP * _msp)
    //{}

    virtual void extrasPerformTask(uint8_t taskIndex)
    {}

    //messaging during simulations
    virtual void showArmedStatus(bool armed)
    {}
    virtual void showAuxStatus(uint8_t status)
    {}

public:
    IMU* getImu()
    {
        return &imu;
    }
    RC* getRc()
    {
        return &rc;
    }

protected:
    virtual void flashLeds(uint16_t onOffCount)
    {
        setLed(0, false);
        setLed(1, false);
        for (uint8_t i = 0; i < onOffCount; i++) {
            setLed(0, i % 2);
            setLed(1, i % 2);
            delayMilliseconds(50);
        }
    }

    virtual void updateCalibrationState(bool& armed, bool& isMoving);
    virtual void updateImu(bool armed);

private:
    IMU        imu;
    RC         rc;

    TimedTask imuTask;
    TimedTask rcTask;
    TimedTask accelCalibrationTask;
    TimedTask altitudeEstimationTask;

    uint16_t smallAngle;

    uint32_t disarmTime;
    IMU::ADC imu_adc;
}; // class ImuBoardBase


//default implementations for IMU
void ImuBoardBase::init()
{
    const Config& config = getConfig();

    delayMilliseconds(config.initDelayMs);

    //flash the LEDs to indicate startup
    flashLeds(config.ledFlashCountOnStartup);

    //initialize timed tasks
    imuTask.init(config.imu.imuLoopMicro);
    rcTask.init(config.rc.rcLoopMilli * 1000);
    accelCalibrationTask.init(config.imu.accelCalibrationPeriodMilli * 1000);
    altitudeEstimationTask.init(config.imu.attitudeUpdatePeriodMilli * 1000);

    imu.init(config.imu);
    rc.init();

    smallAngle = config.imu.smallAngle;
    disarmTime = 0;
}

void ImuBoardBase::updateCalibrationState(bool& armed, bool& isMoving)
{
    uint64_t currentTimeMicro = this->getMicros();
    isMoving = true;

    if (rcTask.checkAndUpdate(currentTimeMicro) || this->rcSerialReady()) {

        // update RC channels
        rc.update(this);

        // useful for simulator
        if (armed)
            this->showAuxStatus(rc.auxState());

        //TODO: need to improve isMoving, currently we just return value to effect PID reset for integral component
        if (rc.throttleIsDown()) 
            isMoving = false;

        if (rc.changed()) {
            if (armed) {
                // Disarm on throttle down + yaw
                if (rc.sticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
                    if (armed) {
                        armed = false;
                        isMoving = false;
                        this->showArmedStatus(armed);
                        // Reset disarm time so that it works next time we arm the board->
                        if (disarmTime != 0)
                            disarmTime = 0;
                    }
                }
            } 
            else { //not armed
                   // gyro calibration
                if (rc.sticks == THR_LO + YAW_LO + PIT_LO + ROL_CE)
                    imu.resetCalibration(true, false);

                // Arm via throttle-low / yaw-right
                if (rc.sticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)
                    if (imu.isGyroCalibrated() && imu.isAccelCalibrated()) 
                        if (!rc.auxState()) // aux switch must be in zero position
                            if (!armed) {
                                armed = true;
                                this->showArmedStatus(armed);
                            }

                // accel calibration
                if (rc.sticks == THR_HI + YAW_LO + PIT_LO + ROL_CE)
                    imu.resetCalibration(false, true);
            } // not armed
        } // rc.changed()

          // Detect aux switch changes for hover, altitude-hold, etc.
        this->extrasCheckSwitch();
    } 
    else { //don't have RC update

        static int taskOrder;   // never call all functions in the same loop, to avoid high delay spikes

        this->extrasPerformTask(taskOrder);

        if (++taskOrder >= this->extrasGetTaskCount()) // using >= supports zero or more tasks
            taskOrder = 0;
    }

    // use LEDs to indicate calibration status
    if (!imu.isGyroCalibrated() || !imu.isAccelCalibrated())  {
        this->setLed(0, true);
    }
    else {
        if (imu.isAccelCalibrated())
            this->setLed(0, false);
        if (armed)
            this->setLed(1, true);
        else
            this->setLed(1, false);
    }
}


void ImuBoardBase::updateImu(bool armed)
{
    uint64_t currentTimeMicro = this->getMicros();

    if (imuTask.checkAndUpdate(currentTimeMicro)) {
        this->imuRead(imu_adc);
        imu.update(currentTimeMicro, armed, imu_adc);


        DebugUtils::debug("%d %d %d\n", imu.angle[0], imu.angle[1], imu.angle[2]);

        // measure loop rate just afer reading the sensors
        currentTimeMicro = this->getMicros();

        // compute exponential RC commands
        rc.computeExpo();
    } // IMU update
}

void ImuBoardBase::update(bool& armed, bool& isMoving)
{
    this->updateCalibrationState(armed, isMoving);
    this->updateImu(armed);
}



} //namespace