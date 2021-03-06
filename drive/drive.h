#ifndef BACHELOR_DRIVE
#define BACHELOR_DRIVE

#include "drive_rotate.hpp"
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
#include "encoder.h"
#include <map>

class Drive_straight;
class Drive_rotate;

class Drive{
    private:
        std::thread encoder_thread;
        std::atomic<bool> quit;
        std::unique_ptr<Drive_straight> straight = nullptr;
        std::unique_ptr<Drive_rotate> rotator = nullptr;

        Encoder encoderRight;
        Encoder encoderLeft;
        gyroscope *gyro = NULL;
        Ping *ping = NULL;
        Arduinocomm *serial = nullptr;

        std::map<std::string, PID_config> pids;
        uint32_t _current_left_encoder_distance = 0, _current_right_encoder_distance = 0;
        int32_t _current_left_encoder_speed = 0, _current_right_encoder_speed = 0;

        enum State{ DRIVING_MANUAL, DRIVING_STRAIGHT , DRIVING_DURATION, DRIVING_DISTANCE, ROTATING,WAITING_FOR_STOP,  STOPPED };
        State state = STOPPED;

        bool enable_ping_stop = false;
        bool wait_stop = false;
        uint64_t stop_timer = 0;

        void update_encoder(Encoder *encoderR, Encoder *encoderL);
        std::function<void()> driveCompletedCallback = nullptr;
        std::function<void()> stopConfirmedCallback = nullptr;

        unsigned int stopPower = 90;//stand still
    public:
        ~Drive();
        bool enable_drive = true;
        Drive(unsigned char encoder_left_a, unsigned char encoder_left_b, unsigned char encoder_right_a, unsigned char encoder_right_b, Arduinocomm *arduino_serial, std::map<std::string,PID_config> &pids);
        void driveDistance(unsigned int speed, uint32_t distance, std::function<void()> callback = nullptr, bool reverse = false, bool use_ramping = true);
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

        static void stop(Arduinocomm *serial)
        {
            if(serial)
            {
                serial->drive(90,90);
            }
        }

};
#endif
