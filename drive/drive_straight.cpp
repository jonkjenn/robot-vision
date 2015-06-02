#include "drive_straight.hpp"

using namespace std;

Drive_straight::Drive_straight(Arduinocomm *serial, uint32_t *encoderLeft, uint32_t *encoderRight,int32_t *encoderLeftSpeed, int32_t *encoderRightSpeed, gyroscope *gyro,Ping *ping,unsigned int speed, uint32_t distance, PID_config pidconfig_encoder, PID_config pidconfig_gyro, std::function<void()> callback , bool reverse , bool use_ramping)
{
    this->encoderLeft = encoderLeft;
    this->encoderRight = encoderRight;
    this->encoderLeftSpeed = encoderLeftSpeed;
    this->encoderRightSpeed = encoderRightSpeed;
    this->gyro = gyro;
    this->ping = ping;
    this->speed = speed;
    this->distance = distance;
    this->callback = callback;
    this->reverse = reverse;
    this->use_ramping = use_ramping;
    this->serial = serial;
    this->leftSpeed = speed;
    this->rightSpeed = speed;

    rotationPID_setpoint = pidconfig_gyro.setpoint;
    rotationPID = new PID(&rotationPID_input, &rotationPID_output, &rotationPID_setpoint, pidconfig_gyro.kp, pidconfig_gyro.ki, pidconfig_gyro.kd, DIRECT, pidconfig_gyro.maximum,pidconfig_gyro.minimum);
    rotationPID->SetMode(AUTOMATIC);

    encoder_pid_SetPoint = pidconfig_encoder.setpoint;
    encoderPID = new PID(&encoder_pid_Input, &encoder_pid_Output, &encoder_pid_SetPoint,pidconfig_encoder.kp,pidconfig_encoder.ki,pidconfig_encoder.kd,DIRECT,pidconfig_encoder.minimum,pidconfig_encoder.maximum);
    encoderPID->SetMode(AUTOMATIC);

    gyro->start(0);
}

uint32_t Drive_straight::get_distance()
{
    return (*encoderLeft + *encoderRight)/2;
}

void Drive_straight::modify_power_by_speed(int target_speed, int *left_mod, int *right_mod)
{
    float speed = (*encoderLeftSpeed+*encoderRightSpeed)/2.0 ;
    encoder_pid_Input = (speed - target_speed);//mm/s
    cout << "target speed: " << target_speed << " speed " << speed << endl;
    cout << "encoder speed pid input: " << encoder_pid_Input << endl;
    encoderPID->Compute();
    cout << "encoder speed pid output: " << encoder_pid_Output << endl;
    *left_mod += encoder_pid_Output;
    *right_mod += encoder_pid_Output;
}

bool Drive_straight::update()
{
    cout << "Distance: " << get_distance() << endl;
    cout << "_distance: " << distance << endl;
    cout << "Left distance" << endl;
    cout << *encoderLeft << endl;
    cout << "Right distance" << endl;
    cout << *encoderRight << endl;

    if(get_distance() >= distance)
    {
        cout << "Drive stopping from distance " << micros() << endl;
        distance = 0;

        return true;
    }
    else
    {
        float target_speed = 150;

        int left_mod = 0, right_mod = 0;

        int ramp_distance = 50000;//(_reverse?250000:50000);
        if(use_ramping && abs(get_distance()) < ramp_distance)//5 cm
        {
            cout << "RAMP UP" << endl;
            //target_speed = 100 + encoder_distance/50000.0 * 200;
            modify_power_by_speed(target_speed, &left_mod, &right_mod);
        }
        else if(use_ramping && distance - get_distance() <= 100000) //10 cm
        {
            cout << "RAMP DOWN" << endl;
            modify_power_by_speed(150, &left_mod, &right_mod);
        }
        else
        {
            cout << "Regular" << endl;
            //This should not be hardcoded, should vary with set engine speed
            modify_power_by_speed(200, &left_mod, &right_mod);
        }

        //Keeping straight with gyro
        rotationPID_input = abs(gyro->get_total_rotation());
        rotationPID->Compute();

        float rot = gyro->get_total_rotation();

        if(reverse){rot*=-1;}

        if(rot > 0.01)//If rotated towards right, then turn towards left
        {
            //cout << "Turning right" << endl;
            left_mod += (int)rotationPID_output;
        }
        else if(rot < 0.01) //if rotated towards left, then turn towards right
        {
            //cout << "Turning left" << endl;
            right_mod += (int)rotationPID_output;
        }

        //modify_power_by_distance();

        //if(currentLeftSpeed < 90){currentLeftSpeed = 90;}
        //if(currentRightSpeed < 90){currentRightSpeed = 90;}

        cout << "Total rotation: " << rot << endl;
        cout << "Rotation pid input: " << rotationPID_input << endl;
        cout << "Rotation pid output: " << rotationPID_output << endl;
        //cout << "Encoder pid input: " << encoder_pid_Input << endl;
        //cout << "Encoder pid output: " << encoder_pid_Output << endl;
        cout << "Left speed: " << (int)leftSpeed + left_mod << " Right speed: " << (int)rightSpeed + right_mod << endl;

        do_drive(left_mod,right_mod);
    }

    return false;

}
void Drive_straight::do_drive(int left_mod, int right_mod)
{
    if(reverse)
    {
        serial->drive(180 - (leftSpeed + left_mod),180-(rightSpeed + right_mod));
    }
    else
    {
        serial->drive(leftSpeed + left_mod, rightSpeed + right_mod);
    }
}
