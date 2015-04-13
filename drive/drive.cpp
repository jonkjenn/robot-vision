#include "drive.h"

using namespace std;

Drive::Drive(unsigned char encoder_left_a, unsigned char encoder_left_b, unsigned char encoder_right_a, unsigned char encoder_right_b, const shared_ptr<Arduinocomm> arduino_serial)
{
    serial = arduino_serial;

    gyro = unique_ptr<gyroscope>(new gyroscope("/dev/ttyTHS1",115200));

    encoderRight.setup(encoder_right_a,encoder_right_b);
    encoderLeft.setup(encoder_left_a,encoder_left_b);

    encoderPID = unique_ptr<PID>(new PID(&encoder_pid_Input, &encoder_pid_Output, &encoder_pid_SetPoint,encoder_consKp,encoder_consKi,encoder_consKd,DIRECT,-10,10,&encoder_pid_weight));
    encoder_pid_SetPoint = 0.0;
    encoderPID->SetMode(AUTOMATIC);
}

//Speed: 0-180, 90 = stop, 180 = max speed forward.
//Duration in milliseconds
void Drive::driveDuration(unsigned int speed, unsigned long duration, function<void()> callback)
{
    if(state != STOPPED){return;}

    LOG(DEBUG) << "Starting driveduration";

    driveCompletedCallback = callback;

    state = DRIVING_DURATION;
    encoderRight.reset();
    encoderLeft.reset();

    //Serial.println("Driving with speed: " + String(speed));
    _duration = duration * 1000;
    _startTime = micros();

    LOG(DEBUG) << "Sending serial drive signal";

    serial->drive(speed,speed);

    leftSpeed = speed;
    rightSpeed = speed;
}

void Drive::driveDistance(unsigned int speed, unsigned long distance, function<void()> callback)
{
    if(state != STOPPED){return;}

    driveCompletedCallback = callback;

    encoderRight.reset();
    encoderLeft.reset();

    state = DRIVING_DISTANCE;
    _distance = distance*1000;

    serial->drive(speed,speed);

    leftSpeed = speed;
    rightSpeed = speed;
}

void Drive::drive(unsigned int power1, unsigned int power2)
{
    if(state == DRIVING_MANUAL || state == STOPPED){
        state = DRIVING_MANUAL;
        serial->drive(power1,power2);
    }
}

void Drive::rotateLeft(unsigned int speed, float degrees, function<void()> callback)
{
    if(state != STOPPED){return;}

    driveCompletedCallback = callback;

    state = ROTATING;
    gyro->start(degrees);
    serial->drive(180-speed, speed);
}

void Drive::rotateRight(unsigned int speed, float degrees, function<void()> callback)
{
    if(state != STOPPED){return;}

    driveCompletedCallback = callback;

    state = ROTATING;
    gyro->start(degrees);
    serial->drive(speed, 180-speed);
}

//Speed from 90-180, automatically calculates the reverse speed
void Drive::rotate(unsigned int speed, float degrees, function<void()> callback)
{
    if(state != STOPPED){return;}

    driveCompletedCallback = callback;

    state = ROTATING;
    gyro->start(degrees);
    serial->drive(speed, 180-speed);
}

void Drive::update()
{
    encoderLeft.update();
    encoderRight.update();

    if(state == DRIVING_DURATION)
    {
        if(_duration > 0 && ((micros()-_startTime) > _duration))
        {
            //Serial.println("Stopping");
            stop();
            _duration = 0;

            if(driveCompletedCallback){driveCompletedCallback();}
        }
    }else if(state == DRIVING_DISTANCE)
    {
        if(encoderLeft.getDistance() >= _distance)
        {
            LOG(DEBUG) << "Drive stopping";
            LOG(DEBUG) << "Left distance";
            LOG(DEBUG) << encoderLeft.getDistance();
            LOG(DEBUG) << "Right distance";
            LOG(DEBUG) << encoderRight.getDistance();
            stop();
            _distance = 0;

            if(driveCompletedCallback){driveCompletedCallback();}
        }
        else
        {
            encoder_pid_Input = encoderRight.getSpeed() - encoderLeft.getSpeed();
            encoderPID->Compute();
            //ST2.write(rightSpeed + encoder_pid_Output);
            //
            //
            //O
            /*Serial.println("rightdistance");
            Serial.println(encoderLeft.getDistance());
            Serial.println("leftdistance");
            Serial.println(encoderRight.getDistance());
            */
/*            Serial.println("rightSpeed: ");
            Serial.println(rightSpeed);
            Serial.println("leftSpeed: ");
            Serial.println(rightSpeed + encoder_pid_Output);*/
        }
    }
    else if(state == ROTATING)
    {
        gyro->update();
        if(abs(gyro->total_rotation) >= abs(gyro->goal_rotation))
        {
            if(driveCompletedCallback){driveCompletedCallback();}
            stop();
        }
    }
}

void Drive::stop()
{
    LOG(DEBUG) << "STOP";
    encoderRight.reset();
    encoderLeft.reset();

    serial->drive(stopPower,stopPower);
    state = STOPPED;
}

uint32_t Drive::getDistance()
{
    return (encoderRight.getDistance() + encoderLeft.getDistance());
}

Drive::~Drive()
{
    stop();
}
