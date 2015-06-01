#ifndef DRIVE_ROTATE_GUARD
#define DRIVE_ROTATE_GUARD
#include "encoder.h"
#include "PID_v1.h"
#include "gyroscope.hpp"
#include "arduinocomm.h"
#include "drive.h"

class Drive_rotate{
    public:
        Drive_rotate(Encoder *encoderRight, Encoder *encoderLeft, gyroscope *gyro, unsigned int speed, float degrees, Rotation_Direction direction = LEFT, function<void()> callback = nullptr, PID_config pidconfig_encoder, PID_config pidconfig_gyro);;
    private:
        Encoder *encoderRight = nullptr;
        Encoder *encoderLeft = nullptr;
        gyroscope *gyro = nullptr;
        unsigned int speed = 0;
        unsigned float angle = 0;
        Rotation_direction direction = LEFT;
        std::function<void()> callback = nullptr;
        void Drive_rotate::do_rotate();
};
#endif//Guard
