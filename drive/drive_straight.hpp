#ifndef DRIVE_STRAIGHT_GUARD
#define DRIVE_STRAIGHT_GUARD
#include "encoder.h"
#include "PID_v1.h"
#include "gyroscope.hpp"
#include "arduinocomm.h"
#include <memory>
#include "utility.hpp"

class Drive_straight{
    private:
        Encoder *encoderRight = nullptr;
        Encoder *encoderLeft = nullptr;
        gyroscope *gyro = nullptr;
        Ping *ping = nullptr;
        unsigned int speed;
        unsigned long distance;
        std::function<void()> callback = nullptr;
        bool reverse = false;
        bool use_ramping = true;
        bool ignore_stop = false;
        void setup_and_start_drive_straight(unsigned int speed);
        void drive_straight();
        void do_drive();

    public:
        Drive_straight(Encoder *encoderRight, Encoder *encoderLeft, gyroscope *gyro,Ping *ping,unsigned int speed, unsigned long distance, std::function<void()> callback = nullptr, bool reverse = false, bool use_ramping = true,bool ignore_stop = false, PID_config pidconfig_encoder, PID_config pidconfig_gyro);
};
#endif//Guard
