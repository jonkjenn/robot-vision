#ifndef DRIVE_STRAIGHT_GUARD
#define DRIVE_STRAIGHT_GUARD
#include "encoder.h"
#include "PID_v1.h"
#include "gyroscope.hpp"
#include "arduinocomm.h"
#include <memory>
#include "utility.hpp"
#include "ping.hpp"
#include <iostream>

class Drive_straight{
    private:
        Arduinocomm *serial = nullptr;
        uint32_t *encoderRight;
        uint32_t *encoderLeft;
        int32_t *encoderRightSpeed;
        int32_t *encoderLeftSpeed;
        gyroscope *gyro = nullptr;
        Ping *ping = nullptr;
        unsigned int speed;
        uint8_t leftSpeed = 90, rightSpeed = 90;
        uint32_t distance;
        bool reverse = false;
        bool use_ramping = true;
        bool ignore_stop = false;
        void setup_and_start_drive_straight(unsigned int speed);
        void drive_straight();
        void do_drive();

        PID *rotationPID;
        PID *encoderPID;
        float rotationPID_output = 0, rotationPID_input = 0, rotationPID_setpoint = 0;
        float encoder_pid_Output = 0, encoder_pid_Input = 0, encoder_pid_SetPoint = 0;

        uint32_t get_distance();

        void modify_power_by_speed(int target_speed, int *left_mod, int *right_mod);

        void do_drive(int mod_left, int mod_right);

    public:
        Drive_straight(Arduinocomm *serial, uint32_t *encoderLeft, uint32_t *encoderRight,int32_t *encoderLeftSpeed, int32_t *encoderRightSpeed, gyroscope *gyro,Ping *ping,unsigned int speed, uint32_t distance, PID_config pidconfig_encoder, PID_config pidconfig_gyro, std::function<void()> callback = nullptr, bool reverse = false , bool use_ramping = true);
        bool update();
        std::function<void()> callback = nullptr;
};
#endif//Guard
