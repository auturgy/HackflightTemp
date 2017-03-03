// Board implementation ======================================================
#include <cstdio>
#include <cstdint>
#include "board.hpp"
#include "config.hpp"
#include "crossplatform.h"


namespace hf {

class SimBoard : public Board {

public:
    virtual void init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec) override
    {
        looptimeMicroseconds = 10000;
        calibratingGyroMsec = 100;  // long enough to see but not to annoy

        //TODO: provide implementation below
        //leds[0].init(greenLedHandle, 0, 1, 0);
        //leds[1].init(redLedHandle, 1, 0, 0);
    }

    virtual void imuInit(uint16_t & acc1G, float & gyroScale) override
    {
        // Mimic MPU6050
        acc1G = 4096;
        gyroScale = 16.4f;
    }

    virtual void imuRead(int16_t accADC[3], int16_t gyroADC[3]) override
    {
        // Convert from radians to tenths of a degree

        for (int k=0; k<3; ++k) {
            accADC[k]  = (int16_t)(400000 * accel[k]);
        }

        gyroADC[1] = -(int16_t)(1000 * gyro[0]);
        gyroADC[0] = -(int16_t)(1000 * gyro[1]);
        gyroADC[2] = -(int16_t)(1000 * gyro[2]);
    }

    virtual void ledGreenOff(void) override
    {
        leds[0].set(false);
    }

    virtual void ledGreenOn(void) override
    {
        leds[0].set(true);
    }

    virtual void ledRedOff(void) override
    {
        leds[1].set(false);
    }

    virtual void ledRedOn(void) override
    {
        leds[1].set(true);
    }


    virtual uint32_t getMicros() override
    {
        return micros; 
    }

    virtual bool rcUseSerial(void) override
    {
        return false;
    }

    virtual uint16_t readPWM(uint8_t chan) override
    {
        // Special handling for throttle
        float demand = (chan == 3) ? throttleDemand : demands[chan];

        // Special handling for pitch, roll on PS3, XBOX360
        if (chan < 2) {
           if (controller == PS3)
            demand /= 2;
           if (controller == XBOX360)
            demand /= 1.5;
        }

        // Joystick demands are in [-1,+1]
        int pwm =  (int)(CONFIG_PWM_MIN + (demand + 1) / 2 * (CONFIG_PWM_MAX - CONFIG_PWM_MIN));

        return pwm;
    }

    virtual void dump(char * msg) override
    {
        printf("%s\n", msg);
    }


    virtual void writeMotor(uint8_t index, uint16_t value) override
    {
        thrusts[index] = (value - 1000) / 1000.0f;
    }

    virtual void showArmedStatus(bool armed) override
    {
        //TODO: provide implementtaion
        //if (armed) 
        //    startToast("                    ARMED", 1, 0, 0);
    }

    virtual void showAuxStatus(uint8_t status) override
    {
        if (status != auxStatus) {
            char message[100];
            switch (status) {
                case 1:
                    sprintf(message, "ENTERING ALT-HOLD");
                    break;
                case 2:
                    sprintf(message, "ENTERING GUIDED MODE");
                    break;
                default:
                    sprintf(message, "ENTERING NORMAL MODE");
            }
            //TODO: provide implementtaion
            //startToast(message, 1,1,0);
        }

        auxStatus = status;
    }
    
    virtual void delayMilliseconds(uint32_t msec) override
    {
        //TODO: thread.sleep?
    }


private:
    // LED support
    class LED {
    private:
        int handle;
        float color[3];
        bool on;
    public:
        void init(int _handle, float r, float g, float b)
        {
            this->handle = _handle;
            this->color[0] = r;
            this->color[1] = g;
            this->color[2] = b;
            this->on = false;
        }

        void set(bool status)
        {
            this->on = status;
            float black[3] = {0,0,0};
            //TODO: provide implementation below
            //simSetShapeColor(this->handle, NULL, 0, this->on ? this->color : black);
        }
    };

    LED leds[2];
    uint32_t micros;
    // Launch support
    bool ready;

    // needed for spring-mounted throttle stick
    float throttleDemand;
    const float SPRINGY_THROTTLE_INC = .01f;

    // IMU support
    float accel[3];
    float gyro[3];

    // Barometer support
    int baroPressure;

    // Motor support
    float thrusts[4];

    // 100 Hz timestep, used for simulating microsend timer
    float timestep;

    int particleCount;

    // Handles from scene
    int motorList[4];
    int motorJointList[4];
    int quadcopterHandle;
    int accelHandle;
    int greenLedHandle;
    int redLedHandle;

    // Support for reporting status of aux switch (alt-hold, etc.)
    uint8_t auxStatus;
    // Stick demands from controller
    float demands[5];

    // Controller type
    // We currently support these controllers
    enum controller_t { KEYBOARD, DSM, TARANIS, SPEKTRUM, EXTREME3D, PS3 , XBOX360 };
    controller_t controller;
};

} //namespace