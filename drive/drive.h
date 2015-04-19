#ifndef BACHELOR_DRIVE
#define BACHELOR_DRIVE

#include "encoder.h"
#include "PID_v1.h"
#include "gyroscope.hpp"
#include <memory>
#include "arduinocomm.h"
#include <thread>
#include "ping.hpp"
#include <iostream>
#include "utility.hpp"

enum Rotation_Direction{ LEFT,RIGHT};
class Drive{
    private:
        unsigned int stopPower = 90;//stand still
        unsigned long _startTime = 0;
        unsigned long _duration = 0;
        unsigned long _distance = 0;
        Encoder encoderRight;
        Encoder encoderLeft;
        uint8_t maxLeftSpeed = 90;
        uint8_t maxRightSpeed = 90;
        uint8_t leftSpeed= 90;
        uint8_t rightSpeed = 90;

        float speedMod = 1.0;
        const float rotation_mod = (1.0 - 90.0/110.0);

        uint8_t currentLeftSpeed= 90;
        uint8_t currentRightSpeed = 90;

        uint8_t prevLeftSpeed = 90;
        uint8_t prevRightSpeed = 90;

        double encoder_pid_SetPoint,encoder_pid_Input,encoder_pid_Output;
        double encoder_consKp=0.00025, encoder_consKi=0.0, encoder_consKd=0.0;
        double enc_rot_kp=3.0, enc_rot_ki=5.0, enc_rot_kd=0.0;
        std::unique_ptr<PID> encoderPID = nullptr;

        Rotation_Direction rot_dir = LEFT;
        double rotationPID_setpoint = 0.0,rotationPID_input = 0.0,rotationPID_output = 0.0;
        double rotationPIDKp = 50.0, rotationPIDKi = 1.0, rotationPIDKd = 0.0;
        std::unique_ptr<PID> rotationPID = nullptr;

        enum State{ DRIVING_MANUAL, DRIVING_DURATION, DRIVING_DISTANCE, ROTATING, STOPPED };
        State state = STOPPED;

        gyroscope *gyro = NULL;
        Ping *ping = NULL;

        std::shared_ptr<Arduinocomm> serial = nullptr;

        std::function<void()> driveCompletedCallback = nullptr;

        void encoder_thread();

        bool check_bounds();

        void do_drive();
        void do_rotate();
        void update_encoder(Encoder &encoderR, Encoder &encoderL, gyroscope &gyro, Ping &ping);
    public:
        ~Drive();
        Drive(unsigned char encoder_left_a, unsigned char encoder_left_b, unsigned char encoder_right_a, unsigned char encoder_right_b, const std::shared_ptr<Arduinocomm> serial);
        void driveDuration(unsigned int speed, unsigned long duration, std::function<void()> callback);
        void driveDistance(unsigned int speed, unsigned long distance, std::function<void()> callback);
        void rotate(unsigned int speed, float degrees,Rotation_Direction direction, std::function<void()> callback);
        void drive(unsigned int power1, unsigned int power2);
        void update();
        void stop();
        uint32_t getDistance();
};
#endif
