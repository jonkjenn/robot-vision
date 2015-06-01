#include "drive_rotate.hpp"

using namespace std;

Drive_rotate::Drive_rotate(const Encoder &encoderRight, const Encoder &encoderLeft, const gyroscope &gyro, unsigned int speed, float degrees, Rotation_Direction direction, function<void()> callback, PID_config pidconfig_encoder, PID_config pidconfig_gyro)
{
    this->encoderLeft = encoderLeft;
    this->encoderRight = encoderRight;
    this->gyro = gyro;
    this->speed = speed;
    this->callback = callback;
    this->angle = degrees;
    this->direction = direction;
    this->ignore_stop = ignore_stop;

    if(state != STOPPED){return;}

    LOG(DEBUG) << "Starting rotating\n";

    speed = 110;

    int max_extra_speed = 30;

    reset_pid_values();

    rotationPID = unique_ptr<PID>(new PID(&rotationPID_input, &rotationPID_output, &rotationPID_setpoint, 0.15, 0, 0, DIRECT, -max_extra_speed,max_extra_speed));
    rotationPID_setpoint = 0.0;
    rotationPID->SetMode(AUTOMATIC);

    rot_dir = direction;

    encoderPID = unique_ptr<PID>(new PID(&encoder_pid_Input, &encoder_pid_Output, &encoder_pid_SetPoint,0.00175,0.0,0.0,DIRECT,-30,30));
    encoder_pid_SetPoint = 0.0;
    encoderPID->SetMode(AUTOMATIC);

    encoderPID_2 = unique_ptr<PID>(new PID(&encoder_pid_Input_2, &encoder_pid_Output_2, &encoder_pid_SetPoint_2,0.00175,0,0.0,DIRECT,-30,30));
    encoder_pid_SetPoint_2 = 0.0;
    encoderPID_2->SetMode(AUTOMATIC);

    driveCompletedCallback = callback;

    state = ROTATING;
    gyro->start(degrees);

    maxLeftSpeed = speed + max_extra_speed;
    maxRightSpeed = speed + max_extra_speed;

    prevRightSpeed = 90;
    prevLeftSpeed = 90;

    leftSpeed = speed;
    rightSpeed = speed;

    currentLeftSpeed = leftSpeed;
    currentRightSpeed = rightSpeed;
    do_rotate();
}

void Drive_rotate::do_rotate()
{
    if(currentRightSpeed == prevRightSpeed && currentLeftSpeed == prevLeftSpeed){
        //cout << "SAME" << endl;
        return;}

    prevRightSpeed = currentRightSpeed;
    prevLeftSpeed = currentLeftSpeed;

    if(!check_bounds()){return;}

    //LOG(DEBUG) << "left: " << (int)currentLeftSpeed <<  " right: " <<  (int)currentRightSpeed << endl;

    if(rot_dir == LEFT)
    {
        serial.drive(180 - currentLeftSpeed, currentRightSpeed);
        cout << "Rotating: " << (int)(180 - currentLeftSpeed) << " , " << (int)currentRightSpeed << endl;
    }
    else if(rot_dir == RIGHT)
    {
        serial.drive(currentLeftSpeed, 180 - currentRightSpeed);
    }
}

