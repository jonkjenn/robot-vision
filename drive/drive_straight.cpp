#include "drive_straight.hpp"

using namespace std;

Drive_straight::Drive_straight(gyroscope *gyro,Ping *ping,unsigned int speed, unsigned long distance, std::function<void()> callback = nullptr, bool reverse = false, bool use_ramping = true,bool ignore_stop = false, PID_config pidconfig_encoder, PID_config pidconfig_gyro)
{
    this->encoderLeft = encoderLeft;
    this->encoderRight = encoderRight;
    this->gyro = gyro;
    this->ping = ping;
    this->speed = speed;
    this->distance = distance;
    this->callback = callback;
    this->reverse = reverse;
    this->use_ramping = use_ramping;
    this->ignore_stop = ignore_stop;

    encoderRight.reset();
    encoderLeft.reset();

    state = DRIVING_DISTANCE;
    //Static distance reduction
    _distance = distance*1000;

    setup_and_start_drive_straight(speed);
}

void Drive_straight::setup_and_start_drive_straight(unsigned int speed)
{

    float direction = _reverse?REVERSE:DIRECT;

    //1m
    //p 400, 100
    //p 500 , k 50, (0.5h, 1.25v)
    //p 800, k 0, (1.5h, 1.75v)
    //p 300, k 0, (0.25h, 0v), (-0.25h, 0.25v)
    //p 200, k 0, (0h, -0.5v), (-0.25h 0.25v), (-0.25h, 0.25v)
    //p 200, k 0, (-0.25v, -0.5h)
    //
    //2m
    //p 200, k 0, (2.5v,1.75h)
    //p 400, k 0, (2.0v,1.75h)
    //p 400, k 0, (1.0v,0.75h)
    //p 400, k 0, (1.75v,1.0h)
    //p 400, k 0, (1.75v,1.25h)

    rotationPID = unique_ptr<PID>(new PID(&rotationPID_input, &rotationPID_output, &rotationPID_setpoint, 500, 0, 0, DIRECT, 90.0-speed,0));
    rotationPID_setpoint = 0;
    rotationPID->SetMode(AUTOMATIC);

    gyro->start(0);

    int maximum_extra_power = 20;
    encoder_speedPID = unique_ptr<PID>(new PID(&encoder_speed_pid_Input, &encoder_speed_pid_Output, &encoder_speed_pid_SetPoint,0.2,0,0,DIRECT,-20,maximum_extra_power));
    encoder_speed_pid_SetPoint = 0;
    encoder_speedPID->SetMode(AUTOMATIC);

    /*
    encoderPID = unique_ptr<PID>(new PID(&encoder_pid_Input, &encoder_pid_Output, &encoder_pid_SetPoint,0.01,0.0,0,DIRECT,-20,20));
    encoder_pid_SetPoint = 0.0;
    encoderPID->SetMode(AUTOMATIC);

    encoderPID_2 = unique_ptr<PID>(new PID(&encoder_pid_Input_2, &encoder_pid_Output_2, &encoder_pid_SetPoint_2,0.01,0.0,0,DIRECT,-20,20));
    encoder_pid_SetPoint_2 = 0.0;
    encoderPID_2->SetMode(AUTOMATIC);*/

    maxLeftSpeed = speed + maximum_extra_power;
    maxRightSpeed = speed + maximum_extra_power;

    prevRightSpeed = 90;
    prevLeftSpeed = 90;

    leftSpeed = speed;
    rightSpeed = speed;
    currentLeftSpeed = leftSpeed;
    currentRightSpeed = rightSpeed;
}

void Drive_straight::drive_straight()
{

    //Keeping straight with gyro
    rotationPID_input = abs(gyro->get_total_rotation());
    rotationPID->Compute();

    float rot = gyro->get_total_rotation();

    if(_reverse){rot*=-1;}

    if(rot > 0.01)//If rotated towards right, then turn towards left
    {
        //LOG(DEBUG) << "Turning right" << endl;
        currentLeftSpeed = currentLeftSpeed + (int)rotationPID_output;
        currentRightSpeed = currentRightSpeed;
    }
    else if(rot < 0.01) //if rotated towards left, then turn towards right
    {
        //LOG(DEBUG) << "Turning left" << endl;
        currentRightSpeed = currentRightSpeed + (int)rotationPID_output;
        currentLeftSpeed = currentLeftSpeed;
    }

    //modify_power_by_distance();

    //if(currentLeftSpeed < 90){currentLeftSpeed = 90;}
    //if(currentRightSpeed < 90){currentRightSpeed = 90;}

    LOG(DEBUG) << "Total rotation: " << rot << endl;
    LOG(DEBUG) << "Rotation pid input: " << rotationPID_input << endl;
    LOG(DEBUG) << "Rotation pid output: " << rotationPID_output << endl;
    //LOG(DEBUG) << "Encoder pid input: " << encoder_pid_Input << endl;
    //LOG(DEBUG) << "Encoder pid output: " << encoder_pid_Output << endl;
    LOG(DEBUG) << "Left speed: " << (int)currentLeftSpeed << " Right speed: " << (int)currentRightSpeed << endl;

    if(currentLeftSpeed < 90){currentLeftSpeed = 90;}
    if(currentRightSpeed < 90){currentRightSpeed = 90;}

    do_drive();
}

void Drive_straight::do_drive()
{
    if(currentRightSpeed == prevRightSpeed && currentLeftSpeed == prevLeftSpeed){return;}
    prevRightSpeed = currentRightSpeed;
    prevLeftSpeed = currentLeftSpeed;

    if(!check_bounds()){return;}

    if(reverse)
    {
        serial.drive(180 - currentLeftSpeed,180-currentRightSpeed);
    }
    else
    {
        serial.drive(currentLeftSpeed, currentRightSpeed);
    }
}
