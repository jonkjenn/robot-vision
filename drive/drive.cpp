#include "drive.h"
#include "inttypes.h"

using namespace std;

Drive::Drive(unsigned char encoder_left_a, unsigned char encoder_left_b, unsigned char encoder_right_a, unsigned char encoder_right_b, Arduinocomm *arduino_serial)
{
    serial = arduino_serial;

    gyro = new gyroscope("/dev/ttyTHS1",921600);

    ping = new Ping(165);

    encoderRight.setup(encoder_right_a,encoder_right_b);
    encoderLeft.setup(encoder_left_a,encoder_left_b);

    serial->drive(90,90);
}

void Drive::update_encoder(Encoder &encoderR, Encoder &encoderL, gyroscope &gyro, Ping &ping)
        //printf("encoderL %" PRIu64 "\n", micros()-start);
        // start = micros();
{
    while(true)
    {
        //uint64_t start = micros();
        encoderRight.update();
        encoderLeft.update();
        //printf("encoderL %" PRIu64 "\n", micros()-start);
        // start = micros();
        gyro.update();
        //printf("gyro %" PRIu64 "\n", micros()-start);
        //start = micros();
        ping.update();
        //printf("ping %" PRIu64 "\n", micros()-start);
        //start = micros();
    }
}

//Speed: 0-180, 90 = stop, 180 = max speed forward.
//Duration in milliseconds
void Drive::driveDuration(unsigned int speed, unsigned long duration, function<void()> callback)
{
    if(state != STOPPED){return;}

    LOG(DEBUG) << "Starting driveduration\n";

    driveCompletedCallback = callback;

    state = DRIVING_DURATION;
    encoderRight.reset();
    encoderLeft.reset();

    //Serial.println("Driving with speed: " + String(speed));
    _duration = duration * 1000;
    _startTime = micros();

    LOG(DEBUG) << "Sending serial drive signal\n";

    serial->drive(speed,speed);

    leftSpeed = speed;
    rightSpeed = speed;
}

void Drive::setup_and_start_drive_straight(unsigned int speed)
{
    maxLeftSpeed = speed;
    maxRightSpeed = speed;

    float direction = _reverse?REVERSE:DIRECT;

    rotationPID = unique_ptr<PID>(new PID(&rotationPID_input, &rotationPID_output, &rotationPID_setpoint, 1, 0, 0, DIRECT, 90.0-speed,5));
    rotationPID_setpoint = 0;
    rotationPID->SetMode(AUTOMATIC);

    gyro->start(0);

    encoderPID = unique_ptr<PID>(new PID(&encoder_pid_Input, &encoder_pid_Output, &encoder_pid_SetPoint,1,0,0,DIRECT,-10,speed-100.0));
    encoder_pid_SetPoint = 0;
    encoderPID->SetMode(AUTOMATIC);

    prevRightSpeed = 90;
    prevLeftSpeed = 90;

    leftSpeed = 100;
    rightSpeed = 100;
    currentLeftSpeed = leftSpeed;
    currentRightSpeed = rightSpeed;

    serial->drive(speed,speed);
}

void Drive::ramp(float target_speed)
{
    if(encoderLeft.getDistance() < 100000)//50 cm
    {
        encoder_pid_Input = (int)((encoderLeft.getSpeed() - target_speed)*100.0);//mm/s
        encoderPID->Compute();
        currentLeftSpeed = leftSpeed + encoder_pid_Output;
        currentRightSpeed =  rightSpeed + encoder_pid_Output;
    }
    else
    {
        currentLeftSpeed = maxLeftSpeed;
        currentRightSpeed = maxRightSpeed;
    }
}

void Drive::drive_straight()
{
    //This should not be hardcoded, should vary with set engine speed
    float target_speed = 0.80;

    //Trying to keep the speed constant
    encoder_pid_Input = (int)((encoderLeft.getSpeed() - target_speed)*100.0);//mm/s
    encoderPID->Compute();
    currentLeftSpeed = leftSpeed + encoder_pid_Output;
    currentRightSpeed =  rightSpeed + encoder_pid_Output;

    //Keeping straight with gyro
    rotationPID_input = (int)(abs(gyro->get_total_rotation()) * 100.0);
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

    //if(currentLeftSpeed < 90){currentLeftSpeed = 90;}
    //if(currentRightSpeed < 90){currentRightSpeed = 90;}

    LOG(DEBUG) << "Total rotation: " << rot << endl;
    LOG(DEBUG) << "Rotation pid input: " << rotationPID_input << endl;
    LOG(DEBUG) << "Rotation pid output: " << rotationPID_output << endl;
    LOG(DEBUG) << "Encoder pid input: " << encoder_pid_Input << endl;
    LOG(DEBUG) << "Encoder pid output: " << encoder_pid_Output << endl;
    LOG(DEBUG) << "Left speed: " << (int)currentLeftSpeed << " Right speed: " << (int)currentRightSpeed << endl;

    do_drive();
}

//Distance mm
void Drive::driveDistance(unsigned int speed, unsigned long distance, function<void()> callback, bool reverse, bool use_ramping,bool ignore_stop)
{
    if(!ignore_stop && state != STOPPED){return;}

    _reverse = reverse;
    _use_ramping = use_ramping;

    driveCompletedCallback = callback;

    encoderRight.reset();
    encoderLeft.reset();

    state = DRIVING_DISTANCE;
    _distance = distance*1000;

    setup_and_start_drive_straight(speed);
}

bool Drive::driveManual()
{
    if(state == STOPPED)
    {
        state = DRIVING_MANUAL;
        return true;
    }
    return false;
}

bool Drive::drive(unsigned int left, unsigned int right)
{
    if(state == DRIVING_MANUAL){
        if(left == prevLeftSpeed && right == prevRightSpeed)
        {
            return true;
        }

        LOG(DEBUG) << "Driving " << (int)left << " " << (int)right << endl;

        prevLeftSpeed = left;
        prevRightSpeed = right;
        serial->drive(left,right);
        return true;
    }
    return false;
}

//Speed from 90-180, automatically calculates the reverse speed
void Drive::rotate(unsigned int speed, float degrees, Rotation_Direction direction, function<void()> callback)
{
    if(state != STOPPED){return;}

    LOG(DEBUG) << "Starting rotating\n";

    uint8_t max_extra_speed = 20;

    rotationPID = unique_ptr<PID>(new PID(&rotationPID_input, &rotationPID_output, &rotationPID_setpoint, rotationPIDKp, rotationPIDKi, rotationPIDKd, DIRECT, -20,max_extra_speed));
    rotationPID_setpoint = 0.0;
    rotationPID->SetMode(AUTOMATIC);

    rot_dir = direction;

    encoderPID = unique_ptr<PID>(new PID(&encoder_pid_Input, &encoder_pid_Output, &encoder_pid_SetPoint,encoder_consKp,encoder_consKi,encoder_consKd,DIRECT,-10,0));
    encoder_pid_SetPoint = 0.0;
    encoderPID->SetMode(AUTOMATIC);

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

uint64_t ptime = 0;
float prevdist = 0.0;
void Drive::update()
{
    if(state == WAITING_FOR_STOP)
    {
        stop();
        return;
    }

    ptime = nanos();
    encoderRight.update();
    encoderLeft.update();

    ptime = nanos();
    gyro->update();

    ptime = nanos();
    ping->update();

    float dist = ping->get_distance();

    if(enable_ping_stop && state != STOPPED  && state != ROTATING && dist > 0 && dist < 0.5)
    {
        LOG(DEBUG) << "Ping distance: " << ping->get_distance() << endl;
        //LOG(DEBUG) << "Ping stop" << endl;
        prevdist = dist;
        stop();
        return;
    }
    if(state == DRIVING_DURATION)
    {
        if(_duration > 0 && ((micros()-_startTime) > _duration))
        {
            //Serial.println("Stopping");
            stop();
            _duration = 0;
            return;
        }
    }else if(state == DRIVING_DISTANCE)
    {
        LOG(DEBUG) << "Distance: " << getDistance() << endl;
        LOG(DEBUG) << "_distance: " << _distance << endl;
        if(getDistance() >= _distance)
        {
            LOG(DEBUG) << "Drive stopping" << endl;
            LOG(DEBUG) << "Left distance" << endl;
            LOG(DEBUG) << encoderLeft.getDistance();
            LOG(DEBUG) << "Right distance" << endl;
            LOG(DEBUG) << encoderRight.getDistance();
            stop();
            _distance = 0;

            return;
        }
        else
        {
            float target_speed = 0.15;
            if(_use_ramping && encoderLeft.getDistance() < 100000)//50 cm
            {
                ramp(0.15);
            }

            drive_straight();
        }
    }
    else if(state == ROTATING)
    {
        float target_speed = 0.8;

        /*LOG(DEBUG) << "Distance rotation: " << gyro->get_distance_rotation() << endl;
          LOG(DEBUG) << "Current rotation: " << gyro->get_current_rotation() << endl;*/

        if(gyro->get_distance_rotation() < 5.0)
        {
            target_speed = 0.5;
        }
        else if(gyro->get_distance_rotation() < 20.0)
        {
            target_speed = 0.5;
        }

        rotationPID_input = (abs(gyro->get_current_rotation())  - target_speed)* 100;
        //LOG(DEBUG) << "pidinput: " << rotationPID_input << endl;
        rotationPID->Compute();

        currentLeftSpeed = leftSpeed + rotationPID_output;
        currentRightSpeed = rightSpeed + rotationPID_output;

        //LOG(DEBUG) << "pidout: " << rotationPID_output << endl;

        //LOG(DEBUG) << "current left: " << (int)currentLeftSpeed << endl;
        //LOG(DEBUG) << "current right: " << (int)currentRightSpeed << endl;
        //LOG(DEBUG) << "target speed: " << target_speed << endl;
        //LOG(DEBUG) << "current speed: " << gyro->get_current_rotation() << endl;

        do_rotate();

        /*
           float rspeed = encoderRight.getSpeed();
           float lspeed = encoderLeft.getSpeed();

        //LOG(DEBUG) << "Left speed: " << lspeed;
        //LOG(DEBUG) << "Right speed: " << rspeed;

        unsigned int tempspeedLeft = 90;
        unsigned int tempspeedRight = 90;

        if(rspeed < lspeed && rspeed > 0.0001)//Left wheel running faster then right wheel
        {
        encoder_pid_Input = lspeed - rspeed;
        encoderPID->Compute();

        //LOG(DEBUG) << "Pid output: " << (int)encoder_pid_Output;

        tempspeedLeft = leftSpeed + (int)encoder_pid_Output;
        tempspeedRight = rightSpeed;

        }else//Right wheel running faster then left wheel
        {
        encoder_pid_Input = lspeed - rspeed;
        encoderPID->Compute();

        tempspeedRight = rightSpeed + (int)encoder_pid_Output;
        tempspeedLeft = leftSpeed;
        }*/

        //if(abs(abs(gyro->total_rotation) - abs(gyro->goal_rotation)) < 0.0194)
        if(gyro->get_distance_rotation() < 2.0)
        {
            LOG(DEBUG) << "Rotation complete\n";
            stop();
        }
    }
}


void Drive::do_drive()
{
    if(currentRightSpeed == prevRightSpeed && currentLeftSpeed == prevLeftSpeed){return;}
    prevRightSpeed = currentRightSpeed;
    prevLeftSpeed = currentLeftSpeed;

    if(!check_bounds()){return;}

    /*LOG(DEBUG) << "Left: " << (int)currentLeftSpeed << endl;
      LOG(DEBUG) << "Right: " << (int)currentRightSpeed << endl;*/

    if(_reverse)
    {
        serial->drive(180 - currentLeftSpeed,180-currentRightSpeed);
    }
    else
    {
        serial->drive(currentLeftSpeed, currentRightSpeed);
    }
}

void Drive::do_rotate()
{
    if(currentRightSpeed == prevRightSpeed && currentLeftSpeed == prevLeftSpeed){
        //cout << "SAME" << endl;
        return;}

    cout << "Rotating" << endl;

    prevRightSpeed = currentRightSpeed;
    prevLeftSpeed = currentLeftSpeed;

    if(!check_bounds()){return;}

    if(rot_dir == LEFT)
    {
        serial->drive(180 - currentLeftSpeed, currentRightSpeed);
    }
    else if(rot_dir == RIGHT)
    {
        serial->drive(currentLeftSpeed, 180 - currentRightSpeed);
    }
}

bool Drive::check_bounds()
{
    //If speeds are out of bounds
    if(currentLeftSpeed < 90 || currentRightSpeed < 90 || currentLeftSpeed > maxLeftSpeed || currentRightSpeed > maxRightSpeed)
    {
        LOG(DEBUG) << "Check bounds stopped\n";
        LOG(DEBUG) << "Left speed: " << (int) leftSpeed <<endl;
        LOG(DEBUG) << "Right speed: " << (int) rightSpeed << endl;
        LOG(DEBUG) << "Current Left speed: " << (int) currentLeftSpeed << endl;
        LOG(DEBUG) << "Current right speed: " << (int) currentRightSpeed << endl;
        currentLeftSpeed = 90;
        currentRightSpeed = 90;
        leftSpeed = 90;
        rightSpeed = 90;
        stop();
        return false;
    }
    return true;
}

void Drive::stop(function<void()> callback)
{
    if(callback != nullptr){driveCompletedCallback = callback;}

    if(state != WAITING_FOR_STOP || micros() - stop_timer > 5000)
    {
        state = WAITING_FOR_STOP;
        LOG(DEBUG) << "STOP" << endl;
        wait_stop = true;
        stop_timer = micros();
        _reverse = false;

        serial->drive(stopPower,stopPower);
    }
}

void Drive::confirm_stop()
{
    if(state == WAITING_FOR_STOP)
    {
        LOG(DEBUG) << "Confirmed stop" << endl;
        encoderRight.reset();
        encoderLeft.reset();
        wait_stop = false;
        state = STOPPED;
        if(driveCompletedCallback){driveCompletedCallback();}
    }
}

uint32_t Drive::getDistance()
{
    //LOG(DEBUG) << "encR: " << encoderRight.getDistance() << " encL: " << encoderLeft.getDistance() << endl;
    return (float)(encoderRight.getDistance() + encoderLeft.getDistance())/2.0;
}

void Drive::set_distance_sensor_stop(bool value)
{
    enable_ping_stop = value;
}

Drive::~Drive()
{
    //stop();
}
