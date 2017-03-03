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
#include "crossplatform.h"


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
}; //class


/********** CPP *******************/

void Hackflight::setup(Board *board_val)
{
    board = board_val;

    uint32_t calibratingGyroMsec;

    // Get particulars for board
    board->init(imuLooptimeUsec, calibratingGyroMsec);

    // sleep for 100ms
    board->delayMilliseconds(100);

    // flash the LEDs to indicate startup
    board->ledRedOff();
    board->ledGreenOff();
    for (uint8_t i = 0; i < 10; i++) {
        board->ledRedOn();
        board->ledGreenOn();
        board->delayMilliseconds(50);
        board->ledRedOff();
        board->ledGreenOff();
        board->delayMilliseconds(50);
    }

    // compute cycles for calibration based on board's time constant
    calibratingGyroCycles = (uint16_t)(1000. * calibratingGyroMsec / imuLooptimeUsec);
    calibratingAccCycles  = (uint16_t)(1000. * CONFIG_CALIBRATING_ACC_MSEC  / imuLooptimeUsec);

    // initializing timing tasks
    imuTask.init(imuLooptimeUsec);
    rcTask.init(CONFIG_RC_LOOPTIME_MSEC * 1000);
    accelCalibrationTask.init(CONFIG_CALIBRATE_ACCTIME_MSEC * 1000);
    altitudeEstimationTask.init(CONFIG_ALTITUDE_UPDATE_MSEC * 1000);

    // initialize our external objects with objects they need
    rc.init(board);
    stab.init(&rc, &imu);
    imu.init(calibratingGyroCycles, calibratingAccCycles, board);
    mixer.init(&rc, &stab, board); 
    msp.init(&imu, &mixer, &rc, board);

    // do any extra initializations (baro, sonar, etc.)
    //TODO: re-enable this
    //board->extrasInit(&msp);

    // always do gyro calibration at startup
    calibratingG = calibratingGyroCycles;

    // assume shallow angle (no accelerometer calibration needed)
    haveSmallAngle = true;

    // ensure not armed
    armed = false;

} // setup

void Hackflight::loop(void)
{
    static bool     accCalibrated;
    static uint16_t calibratingA;
    static uint32_t currentTime;
    static uint32_t disarmTime;

    bool rcSerialReady = board->rcSerialReady();

    if (rcTask.checkAndUpdate(currentTime) || rcSerialReady) {

        // update RC channels
        rc.update();

        rcSerialReady = false;

        // useful for simulator
        if (armed)
            board->showAuxStatus(rc.auxState());

        // when landed, reset integral component of PID
        if (rc.throttleIsDown()) 
            stab.resetIntegral();

        if (rc.changed()) {

            if (armed) {      // actions during armed

                              // Disarm on throttle down + yaw
                if (rc.sticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
                    if (armed) {
                        armed = false;
                        board->showArmedStatus(armed);
                        // Reset disarm time so that it works next time we arm the board->
                        if (disarmTime != 0)
                            disarmTime = 0;
                    }
                }
            } else {         // actions during not armed

                             // gyro calibration
                if (rc.sticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) 
                    calibratingG = calibratingGyroCycles;

                // Arm via throttle-low / yaw-right
                if (rc.sticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)
                    if (calibratingG == 0 && accCalibrated) 
                        if (!rc.auxState()) // aux switch must be in zero position
                            if (!armed) {
                                armed = true;
                                board->showArmedStatus(armed);
                            }

                // accel calibration
                if (rc.sticks == THR_HI + YAW_LO + PIT_LO + ROL_CE)
                    calibratingA = calibratingAccCycles;

            } // not armed

        } // rc.changed()

          // Detect aux switch changes for hover, altitude-hold, etc.
        board->extrasCheckSwitch();

    } else {                    // not in rc loop

        static int taskOrder;   // never call all functions in the same loop, to avoid high delay spikes

        board->extrasPerformTask(taskOrder);

        taskOrder++;

        if (taskOrder >= board->extrasGetTaskCount()) // using >= supports zero or more tasks
            taskOrder = 0;
    }

    currentTime = board->getMicros();

    if (imuTask.checkAndUpdate(currentTime)) {

        imu.update(currentTime, armed, calibratingA, calibratingG);

        haveSmallAngle = abs(imu.angle[0]) < CONFIG_SMALL_ANGLE && abs(imu.angle[1]) < CONFIG_SMALL_ANGLE;

        DebugUtils::debug("%d %d %d\n", imu.angle[0], imu.angle[1], imu.angle[2]);

        // measure loop rate just afer reading the sensors
        currentTime = board->getMicros();

        // compute exponential RC commands
        rc.computeExpo();

        // use LEDs to indicate calibration status
        if (calibratingA > 0 || calibratingG > 0) {
            board->ledGreenOn();
        }
        else {
            if (accCalibrated)
                board->ledGreenOff();
            if (armed)
                board->ledRedOn();
            else
                board->ledRedOff();
        }

        // periodically update accelerometer calibration status
        static bool on;
        if (accelCalibrationTask.check(currentTime)) {
            if (!haveSmallAngle) {
                accCalibrated = false; 
                if (on) {
                    board->ledGreenOff();
                    on = false;
                }
                else {
                    board->ledGreenOn();
                    on = true;
                }
                accelCalibrationTask.update(currentTime);
            } else {
                accCalibrated = true;
            }
        }

        // handle serial communications
        msp.update(armed);

        // update stability PID controller 
        stab.update();

        // update mixer
        mixer.update(armed);

    } // IMU update


} // loop()


} //namespace