#ifndef DRIVE_ROTATE_GUARD
#define DRIVE_ROTATE_GUARD
#include "encoder.h"
#include "PID_v1.h"
#include "gyroscope.hpp"
#include "arduinocomm.h"
#include "drive.h"
#include <functional>

class Drive_rotate{
    public:
        Drive_rotate(Arduinocomm *serial, uint32_t *encoderRight, uint32_t *encoderLeft, gyroscope *gyro, unsigned int speed, float degrees, PID_config pidconfig_encoder, PID_config pidconfig_gyro, std::function<void()> callback = nullptr, Rotation_Direction direction = LEFT);
        bool update();
        std::function<void()> callback = nullptr;
    private:
        Arduinocomm *serial;
        uint32_t *encoderRight;
        uint32_t *encoderLeft;
        gyroscope *gyro;
        unsigned int speed = 0;
        float angle = 0;
        Rotation_Direction direction = LEFT;
        PID *rotationPID;
        PID *encoderPID;
        PID *encoderPID_2;
        float rotationPID_output = 0, rotationPID_input = 0, rotationPID_setpoint = 0;
        float encoder_pid_Output = 0, encoder_pid_Input = 0, encoder_pid_SetPoint = 0;
        float encoder_pid_Output_2 = 0, encoder_pid_Input_2 = 0, encoder_pid_SetPoint_2 = 0;
        uint8_t leftSpeed = 90, rightSpeed = 90;

        void do_rotate(int,int);
};
#endif//Guard
