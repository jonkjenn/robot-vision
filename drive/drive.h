#ifndef BACHELOR_DRIVE
#define BACHELOR_DRIVE

#include "drive_straight.hpp"
#include "PID_v1.h"
#include "gyroscope.hpp"
#include <memory>
#include "arduinocomm.h"
#include <thread>
#include "ping.hpp"
#include <iostream>
#include "utility.hpp"
#include <fstream>
#include <atomic>
enum Rotation_Direction{ LEFT,RIGHT};
class Drive{
    private:
        std::thread encoder_thread;
        std::atomic<bool> quit;
        std::unique_ptr<Drive_straight> drive_straight = nullptr;
        Encoder encoderRight;
        Encoder encoderLeft;
        gyroscope *gyro = NULL;
        Ping *ping = NULL;
        Arduinocomm *serial = nullptr;
    public:
        ~Drive();
        bool enable_drive = true;
        Drive(unsigned char encoder_left_a, unsigned char encoder_left_b, unsigned char encoder_right_a, unsigned char encoder_right_b, Arduinocomm *serial);
        void driveDuration(unsigned int speed, unsigned long duration, std::function<void()> callback = nullptr);
        void driveDistance(unsigned int speed, unsigned long distance, std::function<void()> callback = nullptr, bool reverse = false, bool use_ramping = true,bool ignore_stop = false);
        void rotate(unsigned int speed, float degrees,Rotation_Direction direction, std::function<void()> callback = nullptr);
        bool drive(unsigned int power1, unsigned int power2);
        void set_distance_sensor_stop(bool value);
        bool driveManual();
        void update();
        void stop(std::function<void()> callback = nullptr);
        void confirm_stop();
        uint32_t getDistance();
        void setup_and_start_drive_straight(unsigned int speed);
        void drive_straight();
        void modify_power_by_speed(int target_speed);
        void modify_power_by_speed_rotate(int target_speed);
        void modify_power_by_distance();
        void stop_driver();
        void abort();
};
#endif
