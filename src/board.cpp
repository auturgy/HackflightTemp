// Board implementation ======================================================

#include <board.hpp>
#include <hackflight.hpp>
#include <cstdint>

namespace hf {

// LED support

class LED {

private:

    int handle;
    float color[3];
    bool on;

public:

    LED(void) { }

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

static LED leds[2];
static uint32_t micros;
// Launch support
static bool ready;

// needed for spring-mounted throttle stick
static float throttleDemand;
static const float SPRINGY_THROTTLE_INC = .01f;

// IMU support
static float accel[3];
static float gyro[3];

// Barometer support
static int baroPressure;

// Motor support
static float thrusts[4];

// 100 Hz timestep, used for simulating microsend timer
static float timestep;

static int particleCount;

// Handles from scene
static int motorList[4];
static int motorJointList[4];
static int quadcopterHandle;
static int accelHandle;
static int greenLedHandle;
static int redLedHandle;

// Support for reporting status of aux switch (alt-hold, etc.)
static uint8_t auxStatus;
// Stick demands from controller
static float demands[5];

// Controller type
// We currently support these controllers
enum controller_t { KEYBOARD, DSM, TARANIS, SPEKTRUM, EXTREME3D, PS3 , XBOX360 };
static controller_t controller;


void Board::init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec)
{
    looptimeMicroseconds = 10000;
    calibratingGyroMsec = 100;  // long enough to see but not to annoy

    //TODO: provide implementation below
    //leds[0].init(greenLedHandle, 0, 1, 0);
    //leds[1].init(redLedHandle, 1, 0, 0);
}

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    // Mimic MPU6050
    acc1G = 4096;
    gyroScale = 16.4f;
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    // Convert from radians to tenths of a degree

    for (int k=0; k<3; ++k) {
        accADC[k]  = (int16_t)(400000 * accel[k]);
    }

    gyroADC[1] = -(int16_t)(1000 * gyro[0]);
    gyroADC[0] = -(int16_t)(1000 * gyro[1]);
    gyroADC[2] = -(int16_t)(1000 * gyro[2]);
}

void Board::ledGreenOff(void)
{
    leds[0].set(false);
}

void Board::ledGreenOn(void)
{
    leds[0].set(true);
}

void Board::ledRedOff(void)
{
    leds[1].set(false);
}

void Board::ledRedOn(void)
{
    leds[1].set(true);
}


uint32_t Board::getMicros()
{
    return micros; 
}

bool Board::rcUseSerial(void)
{
    return false;
}

uint16_t Board::readPWM(uint8_t chan)
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

void Board::dump(char * msg)
{
    printf("%s\n", msg);
}


void Board::writeMotor(uint8_t index, uint16_t value)
{
    thrusts[index] = (value - 1000) / 1000.0f;
}

void Board::showArmedStatus(bool armed)
{
    //TODO: provide implementtaion
    //if (armed) 
    //    startToast("                    ARMED", 1, 0, 0);
}

void Board::showAuxStatus(uint8_t status)
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

// Unused ==========================================================================================

void Board::extrasCheckSwitch(void)
{
}

uint8_t  Board::extrasGetTaskCount(void){
    return 0;
}

bool Board::extrasHandleMSP(uint8_t command)
{
    return true;
}

void Board::extrasInit(class MSP * _msp)
{
    (void)_msp;
}

void Board::extrasPerformTask(uint8_t taskIndex)
{
    (void)taskIndex;
}


bool Board::rcSerialReady(void)
{
    return false;
}

uint16_t Board::rcReadSerial(uint8_t chan)
{
    (void)chan;
    return 0;
}

void Board::checkReboot(bool pendReboot)
{
    (void)pendReboot;
}

void Board::reboot(void)
{
}

void Board::delayMilliseconds(uint32_t msec)
{
}

uint8_t Board::serialAvailableBytes(void)
{
    return 0;
}

uint8_t Board::serialReadByte(void)
{
    return 0;
}

void Board::serialWriteByte(uint8_t c)
{
}


}