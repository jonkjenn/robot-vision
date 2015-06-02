#include "drive_rotate.hpp"

using namespace std;

Drive_rotate::Drive_rotate(Arduinocomm *serial, uint32_t *encoderRight, uint32_t *encoderLeft, gyroscope *gyro, unsigned int speed, float degrees, PID_config pidconfig_encoder, PID_config pidconfig_gyro, std::function<void()> callback , Rotation_Direction direction)
{
    this->encoderLeft = encoderLeft;
    this->encoderRight = encoderRight;
    this->gyro = gyro;
    this->speed = speed;
    this->callback = callback;
    this->angle = degrees;
    this->direction = direction;
    this->serial = serial;

    cout << "Starting rotating\n";

    rotationPID_setpoint = pidconfig_gyro.setpoint;
    rotationPID = new PID(&rotationPID_input, &rotationPID_output, &rotationPID_setpoint, pidconfig_gyro.kp, pidconfig_gyro.ki, pidconfig_gyro.kd, DIRECT, pidconfig_gyro.maximum,pidconfig_gyro.minimum);
    rotationPID->SetMode(AUTOMATIC);

    encoder_pid_SetPoint = pidconfig_encoder.setpoint;
    encoderPID = new PID(&encoder_pid_Input, &encoder_pid_Output, &encoder_pid_SetPoint,pidconfig_encoder.kp,pidconfig_encoder.ki,pidconfig_encoder.kd,DIRECT,pidconfig_encoder.minimum,pidconfig_encoder.maximum);
    encoderPID->SetMode(AUTOMATIC);

    encoder_pid_SetPoint_2 = pidconfig_encoder.setpoint;
    encoderPID_2 = new PID(&encoder_pid_Input_2, &encoder_pid_Output_2, &encoder_pid_SetPoint_2,pidconfig_encoder.kp,pidconfig_encoder.ki,pidconfig_encoder.kd,DIRECT,pidconfig_encoder.minimum,pidconfig_encoder.maximum);
    encoderPID_2->SetMode(AUTOMATIC);

    gyro->start(degrees);

    /*leftSpeed = speed;
      rightSpeed = speed;

      currentLeftSpeed = leftSpeed;
      currentRightSpeed = rightSpeed;*/
}

bool Drive_rotate::update()
{
    float target_speed = 0.8;

    int mod_left = 0;
    int mod_right = 0;

    encoder_pid_Input = (float)*encoderLeft - (float)*encoderRight;//mm/s
    encoderPID->Compute();
    cout << "PID1 " << encoder_pid_Output <<  " left distance: "  << (encoderLeft) << " input left: " << encoder_pid_Input << endl;//mm/sendl;
    mod_left += encoder_pid_Output;
    encoder_pid_Input_2 = (float)*encoderRight - (float)*encoderLeft;//mm/s
    encoderPID_2->Compute();
    cout << "PID2 " << encoder_pid_Output_2 <<  " right distance: "  << (encoderRight)  << "input right: " << encoder_pid_Input_2 << endl;//mm/sendl;
    mod_right +=  encoder_pid_Output_2;

    cout << "Distance rotation: " << gyro->get_distance_rotation() << endl;
    cout << "Current rotation: " << gyro->get_current_rotation() << endl;

    cout << "current left: " << (int)leftSpeed << endl;
    cout << "current right: " << (int)rightSpeed << endl;

    if(gyro->get_distance_rotation() < 15.0)
    {
        target_speed = 0.01;
    }
    else if(gyro->get_distance_rotation() < 30.0)
    {
        target_speed = 0.05;
    }
    else
    {
        target_speed = 0.5;
    }

    rotationPID_input = (abs(gyro->get_current_rotation())  - target_speed)* 10.0;

    cout << "rotationpidinput: " << rotationPID_input << endl;
    rotationPID->Compute();

    mod_left += abs(rotationPID_output);
    mod_right += abs(rotationPID_output);

    cout << "current left: " << (int)leftSpeed << endl;
    cout << "current right: " << (int)rightSpeed << endl;

    cout << "pidout: " << rotationPID_output << endl;

    cout << "current left: " << (int)leftSpeed << endl;
    cout << "current right: " << (int)rightSpeed << endl;


    //if(abs(abs(gyro.total_rotation) - abs(gyro.goal_rotation)) < 0.0194)
    if(gyro->get_distance_rotation() < 0.1)
    {
        cout << "Rotation complete\n";
        return true;
    }
    do_rotate(mod_left, mod_right);

    return false;
}

void Drive_rotate::do_rotate(int mod_left, int mod_right)
{
    if(direction == LEFT)
    {
        serial->drive(180 - (leftSpeed + mod_left), (rightSpeed + mod_right));
        cout << "Rotating: " << (int)(180 - (leftSpeed + mod_left)) << " , " << (int)(rightSpeed + mod_right) << endl;
    }
    else if(direction == RIGHT)
    {
        serial->drive(leftSpeed + mod_left, 180 - (rightSpeed + mod_right));
    }
}

