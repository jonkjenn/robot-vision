#ifndef BACHELOR_DRIVE
#define BACHELOR_DRIVE

#include "easylogging++.h"
#include "encoder.h"
#include "PID_v1.h"
#include "gyroscope.hpp"
#include <memory>
#include "arduinocomm.h"

enum Rotation_Direction{ LEFT,RIGHT};
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

        float speedMod = 1.0;
        const float rotation_mod = (1.0 - 95.0/110.0);

        uint8_t currentLeftSpeed= 90;
        uint8_t currentRightSpeed = 90;

        double encoder_pid_SetPoint,encoder_pid_Input,encoder_pid_Output;
        double encoder_consKp=3.0, encoder_consKi=5.0, encoder_consKd=0.0;
        double enc_rot_kp=3.0, enc_rot_ki=5.0, enc_rot_kd=0.0;
        std::unique_ptr<PID> encoderPID;
        enum State{ DRIVING_MANUAL, DRIVING_DURATION, DRIVING_DISTANCE, ROTATING, STOPPED };
        State state = STOPPED;

        Rotation_Direction rot_dir = LEFT;


        std::unique_ptr<gyroscope> gyro;

        std::shared_ptr<Arduinocomm> serial;

        std::function<void()> driveCompletedCallback;

        void encoder_thread();

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
