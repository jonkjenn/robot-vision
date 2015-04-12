#ifndef BACHELOR_DRIVE
#define BACHELOR_DRIVE

#include "easylogging++.h"
#include "encoder.h"
#include "PID_v1.h"
#include "gyroscope.hpp"
#include <memory>
#include "arduinocomm.h"

using DriveCompletedCallback = void(*)();

class Drive{
    private:
        unsigned int stopPower = 90;//stand still
        unsigned long _startTime = 0;
        unsigned long _duration = 0;
        unsigned long _distance = 0;
        Encoder encoderRight;
        Encoder encoderLeft;
        uint8_t leftSpeed= 90;
        uint8_t rightSpeed = 90;

        double encoder_pid_SetPoint,encoder_pid_Input,encoder_pid_Output;
        double encoder_consKp=1.0, encoder_consKi=0.0, encoder_consKd=0.0, encoder_pid_weight = 1.0;
        std::unique_ptr<PID> encoderPID;
        enum State{ DRIVING_MANUAL, DRIVING_DURATION, DRIVING_DISTANCE, ROTATING, STOPPED };

        State state = STOPPED;

        std::function<void> driveCompletedCallback;

        std::unique_ptr<gyroscope> gyro;

        std::shared_ptr<Arduinocomm> serial;

    public:
        void setup(unsigned char encoder_left_a, unsigned char encoder_left_b, unsigned char encoder_right_a, unsigned char encoder_right_b, const std::shared_ptr<Arduinocomm>& serial);
        void driveDuration(unsigned int speed, unsigned long duration);
        void driveDistance(unsigned int speed, unsigned long distance);
        void rotate(unsigned int speed, uint16_t degrees);
        void drive(unsigned int power1, unsigned int power2, unsigned long duration);
        bool update();
        void stop();
        uint32_t getDistance();
};
#endif
