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
        uint64_t _distance = 0;
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

        float encoder_pid_SetPoint = 0,encoder_pid_Input = 0,encoder_pid_Output = 0;
        float encoder_consKp=1, encoder_consKi=0, encoder_consKd=0;
        float enc_rot_kp=3, enc_rot_ki=5, enc_rot_kd=0;
        std::unique_ptr<PID> encoderPID = nullptr;

        Rotation_Direction rot_dir = LEFT;
        float rotationPID_setpoint = 0,rotationPID_input = 0,rotationPID_output = 0;
        float rotationPIDKp = 0.25, rotationPIDKi = 0, rotationPIDKd = 0;
        std::unique_ptr<PID> rotationPID = nullptr;

        enum State{ DRIVING_MANUAL, DRIVING_STRAIGHT , DRIVING_DURATION, DRIVING_DISTANCE, ROTATING,WAITING_FOR_STOP,  STOPPED };
        State state = STOPPED;

        gyroscope *gyro = NULL;
        Ping *ping = NULL;

        bool enable_ping_stop = true;

        Arduinocomm *serial = nullptr;

        std::function<void()> driveCompletedCallback = nullptr;

        void encoder_thread();

        bool check_bounds();

        void do_drive();
        void do_rotate();
        void update_encoder(Encoder &encoderR, Encoder &encoderL, gyroscope &gyro, Ping &ping);
        bool wait_stop = false;
        uint64_t stop_timer = 0;
        bool _reverse = false;
        bool _use_ramping = false;
    public:
        ~Drive();
        Drive(unsigned char encoder_left_a, unsigned char encoder_left_b, unsigned char encoder_right_a, unsigned char encoder_right_b, Arduinocomm *serial);
        void driveDuration(unsigned int speed, unsigned long duration, std::function<void()> callback);
        void driveDistance(unsigned int speed, unsigned long distance, std::function<void()> callback, bool reverse = false, bool use_ramping = true,bool ignore_stop = false);
        void rotate(unsigned int speed, float degrees,Rotation_Direction direction, std::function<void()> callback);
        bool drive(unsigned int power1, unsigned int power2);
        void set_distance_sensor_stop(bool value);
        bool driveManual();
        void update();
        void stop();
        void confirm_stop();
        uint32_t getDistance();
        void setup_driveStraight(unsigned int speed);
        void driveStraight();
        void ramp(float target_speed);
};
#endif
