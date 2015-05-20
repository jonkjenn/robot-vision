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
    encoder_thread = thread(&Drive::update_encoder,this, &encoderRight,&encoderLeft);

    //csv.open("drive.csv");

    serial->drive(90,90);
    quit.store(false);
}

void Drive::update_encoder(Encoder *encoderR, Encoder *encoderL)
{
    while(true)
    {
        if(quit.load()){cout << "encoder stopping" << endl;return;}
        //uint64_t start = micros();
        encoderR->update();
        encoderL->update();
        //printf("encoderL %" PRIu64 "\n", micros()-start);
        // start = micros();
        //printf("gyro %" PRIu64 "\n", micros()-start);
        //start = micros();
        //ping.update();
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

    rotationPID = unique_ptr<PID>(new PID(&rotationPID_input, &rotationPID_output, &rotationPID_setpoint, 400, 0, 100, DIRECT, 90.0-speed,0));
    rotationPID_setpoint = 0;
    rotationPID->SetMode(AUTOMATIC);

    gyro->start(0);

    encoder_speedPID = unique_ptr<PID>(new PID(&encoder_speed_pid_Input, &encoder_speed_pid_Output, &encoder_speed_pid_SetPoint,0.3,0,0,DIRECT,-20,speed-100.0));
    encoder_speed_pid_SetPoint = 0;
    encoder_speedPID->SetMode(AUTOMATIC);

    /*
    encoderPID = unique_ptr<PID>(new PID(&encoder_pid_Input, &encoder_pid_Output, &encoder_pid_SetPoint,0.01,0.0,0,DIRECT,-20,20));
    encoder_pid_SetPoint = 0.0;
    encoderPID->SetMode(AUTOMATIC);

    encoderPID_2 = unique_ptr<PID>(new PID(&encoder_pid_Input_2, &encoder_pid_Output_2, &encoder_pid_SetPoint_2,0.01,0.0,0,DIRECT,-20,20));
    encoder_pid_SetPoint_2 = 0.0;
    encoderPID_2->SetMode(AUTOMATIC);*/

    prevRightSpeed = 90;
    prevLeftSpeed = 90;

    leftSpeed = 100;
    rightSpeed = 100;
    currentLeftSpeed = leftSpeed;
    currentRightSpeed = rightSpeed;

    serial->drive(speed,speed);
}

void Drive::modify_power_by_speed_rotate(int target_speed)//mm /s
{
    encoder_pid_Input = (int)_current_left_encoder_distance - (int)_current_right_encoder_distance;//mm/s
    encoderPID->Compute();
    LOG(DEBUG) << "PID1 " << encoder_pid_Output <<  " left distance: "  << (_current_left_encoder_distance) << " input left: " << encoder_pid_Input << endl;//mm/sendl;
    currentLeftSpeed = leftSpeed + encoder_pid_Output;
    encoder_pid_Input_2 = (int)_current_right_encoder_distance - (int)_current_left_encoder_distance;//mm/s
    encoderPID_2->Compute();
    LOG(DEBUG) << "PID2 " << encoder_pid_Output_2 <<  " right distance: "  << (_current_right_encoder_distance)  << "input right: " << encoder_pid_Input_2 << endl;//mm/sendl;
    currentRightSpeed =  rightSpeed + encoder_pid_Output_2;
    if(currentLeftSpeed < 90) { currentLeftSpeed = 90;}
    if(currentRightSpeed < 90) { currentRightSpeed = 90;}
}

void Drive::modify_power_by_speed(int target_speed)//mm /s
{
    encoder_speed_pid_Input = ((encoderLeft.getSpeed()+encoderRight.getSpeed())/2.0 - target_speed);//mm/s
    cout << "encoder speed pid input: " << encoder_speed_pid_Input << endl;
    encoder_speedPID->Compute();
    cout << "encoder speed pid output: " << encoder_speed_pid_Output << endl;
    currentLeftSpeed = leftSpeed + encoder_speed_pid_Output;
    currentRightSpeed =  rightSpeed + encoder_speed_pid_Output;
}

void Drive::modify_power_by_distance()
{
    encoder_pid_Input = (int)_current_left_encoder_distance - (int)_current_right_encoder_distance;//mm/s
    encoderPID->Compute();
    LOG(DEBUG) << "PID1 " << encoder_pid_Output <<  " left distance: "  << (_current_left_encoder_distance) << " input left: " << encoder_pid_Input << endl;//mm/sendl;
    currentLeftSpeed = currentLeftSpeed + encoder_pid_Output;
    encoder_pid_Input_2 = (int)_current_right_encoder_distance - (int)_current_left_encoder_distance;//mm/s
    encoderPID_2->Compute();
    LOG(DEBUG) << "PID2 " << encoder_pid_Output_2 <<  " right distance: "  << (_current_right_encoder_distance)  << "input right: " << encoder_pid_Input_2 << endl;//mm/sendl;
    currentRightSpeed =  currentRightSpeed + encoder_pid_Output_2;
}

void Drive::drive_straight()
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

    //LOG(DEBUG) << "Total rotation: " << rot << endl;
    //LOG(DEBUG) << "Rotation pid input: " << rotationPID_input << endl;
    //LOG(DEBUG) << "Rotation pid output: " << rotationPID_output << endl;
    LOG(DEBUG) << "Encoder pid input: " << encoder_pid_Input << endl;
    LOG(DEBUG) << "Encoder pid output: " << encoder_pid_Output << endl;
    LOG(DEBUG) << "Left speed: " << (int)currentLeftSpeed << " Right speed: " << (int)currentRightSpeed << endl;

    if(currentLeftSpeed < 90){currentLeftSpeed = 90;}
    if(currentRightSpeed < 90){currentRightSpeed = 90;}

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
    //Static distance reduction
    _distance = (distance-15)*1000;

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
        prevRightSpeed = (right<180-4?right+4:right);
        serial->drive(left,prevRightSpeed);
        return true;
    }
    return false;
}

//Speed from 90-180, automatically calculates the reverse speed
void Drive::rotate(unsigned int speed, float degrees, Rotation_Direction direction, function<void()> callback)
{
    if(state != STOPPED){return;}

    LOG(DEBUG) << "Starting rotating\n";

    speed = 110;

    uint8_t max_extra_speed = 30;

    rotationPID = unique_ptr<PID>(new PID(&rotationPID_input, &rotationPID_output, &rotationPID_setpoint, 0.25, rotationPIDKi, rotationPIDKd, DIRECT, -max_extra_speed,max_extra_speed));
    rotationPID_setpoint = 0.0;
    rotationPID->SetMode(AUTOMATIC);

    rot_dir = direction;

    encoderPID = unique_ptr<PID>(new PID(&encoder_pid_Input, &encoder_pid_Output, &encoder_pid_SetPoint,0.00375,0.0,0.0,DIRECT,-30,30));
    encoder_pid_SetPoint = 0.0;
    encoderPID->SetMode(AUTOMATIC);

    encoderPID_2 = unique_ptr<PID>(new PID(&encoder_pid_Input_2, &encoder_pid_Output_2, &encoder_pid_SetPoint_2,0.00375,0,0.0,DIRECT,-30,30));
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

uint64_t ptime = 0;
float prevdist = 0.0;
void Drive::update()
{
    if(state == WAITING_FOR_STOP)
    {
        stop();
        return;
    }

    _current_left_encoder_distance = encoderLeft.getDistance();
    _current_right_encoder_distance = encoderRight.getDistance();

    //ptime = nanos();
//    encoderRight.update();
//  encoderLeft.update();

    //ptime = nanos();
    gyro->update();

    //ptime = nanos();
    ping->update();


    //LOG(DEBUG) << "Speed: " << encoderLeft.getSpeed() << endl;


    float dist = ping->get_distance();
    //  ping_distance               
    uint64_t encoder_distance = getDistance();

    //LOG(DEBUG) << "Ping distance: " << dist << endl;

    if(enable_ping_stop && state != STOPPED  && state != ROTATING && dist > 0 && dist < 0.5)
    {
        LOG(DEBUG) << "Ping distance: " << ping->get_distance() << endl;
        //LOG(DEBUG) << "Ping stop" << endl;
        prevdist = dist;
        stop();
        return;
    }

    if(micros() - ptime < 250){return;}
    ptime = micros();

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
        LOG(DEBUG) << "Distance: " << encoder_distance << endl;
        LOG(DEBUG) << "_distance: " << _distance << endl;
            LOG(DEBUG) << "Left distance" << endl;
            LOG(DEBUG) << _current_left_encoder_distance << endl;
            LOG(DEBUG) << "Right distance" << endl;
            LOG(DEBUG) << _current_right_encoder_distance << endl;
        if(encoder_distance >= _distance)
        {
            LOG(DEBUG) << "Drive stopping" << endl;
            LOG(DEBUG) << "Left distance" << endl;
            LOG(DEBUG) << _current_left_encoder_distance << endl;
            LOG(DEBUG) << "Right distance" << endl;
            LOG(DEBUG) << _current_right_encoder_distance << endl;
            stop();
            _distance = 0;

            return;
        }
        else
        {
            float target_speed = 150;
            int ramp_distance = (_reverse?250000:50000);
            if(_use_ramping && abs(encoder_distance) < ramp_distance)//5 cm
            {
                LOG(DEBUG) << "RAMP UP" << endl;
                target_speed = 100 + encoder_distance/50000.0 * 200;
                modify_power_by_speed(target_speed);
            }
            else if(_distance - encoder_distance <= 100000) //10 cm
            {
                LOG(DEBUG) << "RAMP DOWN" << endl;
                modify_power_by_speed(150);
            }
            else
            {
                LOG(DEBUG) << "Regular" << endl;
                //This should not be hardcoded, should vary with set engine speed
                modify_power_by_speed(1500);
            }
            drive_straight();
        }
    }
    else if(state == ROTATING)
    {
        float target_speed = 0.8;

        LOG(DEBUG) << "Distance rotation: " << gyro->get_distance_rotation() << endl;
        LOG(DEBUG) << "Current rotation: " << gyro->get_current_rotation() << endl;

        LOG(DEBUG) << "current left: " << (int)leftSpeed << endl;
        LOG(DEBUG) << "current right: " << (int)rightSpeed << endl;

        modify_power_by_speed_rotate(40);

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

        rotationPID_input = (abs(gyro->get_current_rotation())  - target_speed)* 100;
        //LOG(DEBUG) << "pidinput: " << rotationPID_input << endl;
        rotationPID->Compute();

        //currentLeftSpeed = leftSpeed + rotationPID_output;
        //currentRightSpeed = rightSpeed + rotationPID_output;

        LOG(DEBUG) << "current left: " << (int)currentLeftSpeed << endl;
        LOG(DEBUG) << "current right: " << (int)currentRightSpeed << endl;

        currentLeftSpeed = currentLeftSpeed + rotationPID_output;
        currentRightSpeed = currentRightSpeed + rotationPID_output;

        if(currentLeftSpeed < 90) {currentLeftSpeed = 90;}
        if(currentRightSpeed < 90) {currentRightSpeed = 90;}

        LOG(DEBUG) << "pidout: " << rotationPID_output << endl;

        LOG(DEBUG) << "current left: " << (int)currentLeftSpeed << endl;
        LOG(DEBUG) << "current right: " << (int)currentRightSpeed << endl;
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
        if(gyro->get_distance_rotation() < 0.1)
        {
            LOG(DEBUG) << "Rotation complete\n";
            stop();
        }
    }
        // 1: time ,2: ultrasound distance         3                                    4                                   5                                       6                                           7                               8                               9                           10                      11                              12                              13              
    //csv << micros() << ";" << dist << ";" << gyro->get_current_rotation() << ";" << gyro->get_total_rotation() << ";" << gyro->get_distance_rotation() << ";" << _current_left_encoder_distance << ";" << _current_right_encoder_distance << ";" << rotationPID_input << ";" << rotationPID_output << ";" << encoder_pid_Input << ";" << encoder_pid_Output << ";" << (int)currentLeftSpeed << ";" << (int)currentRightSpeed << endl;
}


void Drive::do_drive()
{
    if(currentRightSpeed == prevRightSpeed && currentLeftSpeed == prevLeftSpeed){return;}
    prevRightSpeed = currentRightSpeed;
    prevLeftSpeed = currentLeftSpeed;

    if(!check_bounds()){return;}

        //LOG(DEBUG) << "Left: " << (int)currentLeftSpeed << endl;
      //LOG(DEBUG) << "Right: " << (int)currentRightSpeed << endl;

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

    prevRightSpeed = currentRightSpeed;
    prevLeftSpeed = currentLeftSpeed;

    if(!check_bounds()){return;}

    //LOG(DEBUG) << "left: " << (int)currentLeftSpeed <<  " right: " <<  (int)currentRightSpeed << endl;

    if(rot_dir == LEFT)
    {
        //serial->drive(180 - currentLeftSpeed-6, currentRightSpeed);
        serial->drive(180 - currentLeftSpeed, currentRightSpeed);
        cout << "Rotating: " << (int)(180 - currentLeftSpeed) << " , " << (int)currentRightSpeed << endl;
    }
    else if(rot_dir == RIGHT)
    {
        serial->drive(currentLeftSpeed, 180 - currentRightSpeed);
    }
}

bool Drive::check_bounds()
{
    //If speeds are out of bounds
    if(currentLeftSpeed < 90 || currentRightSpeed < 90)
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
    }
    else if(currentLeftSpeed > maxLeftSpeed || currentRightSpeed > maxRightSpeed)
    {
        currentLeftSpeed = maxLeftSpeed;
        currentRightSpeed = maxRightSpeed;
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
    return (float)(_current_right_encoder_distance + _current_left_encoder_distance)/2.0;
}

void Drive::set_distance_sensor_stop(bool value)
{
    enable_ping_stop = value;
}

void Drive::stop_driver()
{
    quit.store(true);
    encoder_thread.join();
    cout << " encoder thread joined" << endl;
}

Drive::~Drive()
{
    //stop();
}
