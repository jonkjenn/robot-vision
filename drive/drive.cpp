#include "drive.h"

using namespace std;

Drive::Drive(unsigned char encoder_left_a, unsigned char encoder_left_b, unsigned char encoder_right_a, unsigned char encoder_right_b, Arduinocomm *arduino_serial, map<string,PID_config> &pids)
{
    this->pids = pids;

    serial = arduino_serial;

    gyro = new gyroscope("/dev/ttyTHS1",921600);

    ping = new Ping(165);

    encoderRight.setup(encoder_right_a,encoder_right_b);
    encoderLeft.setup(encoder_left_a,encoder_left_b);
    encoder_thread = thread(&Drive::update_encoder,this, &encoderRight,&encoderLeft);

    //csv.open("drive.csv");

    if(enable_drive)
    {
        serial->drive(90,90);
    }
    quit.store(false);
}

void Drive::update_encoder(Encoder *encoderR, Encoder *encoderL)
{
    while(true)
    {
        if(quit.load()){cout << "encoder stopping" << endl;return;}
        encoderR->update();
        encoderL->update();
    }
}


/*void Drive::modify_power_by_distance()
{
    encoder_pid_Input = (int)_current_left_encoder_distance - (int)_current_right_encoder_distance;//mm/s
    encoderPID->Compute();
    cout << "PID1 " << encoder_pid_Output <<  " left distance: "  << (_current_left_encoder_distance) << " input left: " << encoder_pid_Input << endl;//mm/sendl;
    currentLeftSpeed = currentLeftSpeed + encoder_pid_Output;
    encoder_pid_Input_2 = (int)_current_right_encoder_distance - (int)_current_left_encoder_distance;//mm/s
    encoderPID_2->Compute();
    cout << "PID2 " << encoder_pid_Output_2 <<  " right distance: "  << (_current_right_encoder_distance)  << "input right: " << encoder_pid_Input_2 << endl;//mm/sendl;
    currentRightSpeed =  currentRightSpeed + encoder_pid_Output_2;
}*/

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

        cout << "Driving " << (int)left << " " << (int)right << endl;

        if(enable_drive)
        {
            //left = left<180-4?left+4:left;
            serial->drive(left,right);
        }
        return true;
    }
    return false;
}


uint64_t ptime = 0;
float prevdist = 0.0;
void Drive::update()
{
    /*for(int i=0;i<500;i++)
    {
        serial->sendcustombyte(240);
    }*/

    if(state == WAITING_FOR_STOP)
    {
        cout << "stop because waiting for stop" << endl;
        stop();
        return;
    }

    _current_left_encoder_distance = encoderLeft.getDistance();
    _current_right_encoder_distance = encoderRight.getDistance();
    _current_left_encoder_speed = encoderLeft.getSpeed();
    _current_right_encoder_speed = encoderRight.getSpeed();

    gyro->update();

    ping->update();

    float dist = ping->get_distance();
    if(enable_ping_stop && state != STOPPED  && state != ROTATING && dist > 0 && dist < 0.5)
    {
        cout << "Ping distance: " << ping->get_distance() << endl;
        prevdist = dist;
        stop();
        return;
    }

    if(state == DRIVING_DISTANCE)
    {
        if(straight->update())
        {
            driveCompletedCallback = straight->callback;
            stop();
            straight.release();
        }
    }
    else if(state == ROTATING)
    {
        if(rotator->update())
        {
            driveCompletedCallback = rotator->callback;
            stop();
            rotator.release();
        }
    }
        // 1: time ,2: ultrasound distance         3                                    4                                   5                                       6                                           7                               8                               9                           10                      11                              12                              13              
    //csv << micros() << ";" << dist << ";" << gyro->get_current_rotation() << ";" << gyro->get_total_rotation() << ";" << gyro->get_distance_rotation() << ";" << _current_left_encoder_distance << ";" << _current_right_encoder_distance << ";" << rotationPID_input << ";" << rotationPID_output << ";" << encoder_pid_Input << ";" << encoder_pid_Output << ";" << (int)currentLeftSpeed << ";" << (int)currentRightSpeed << endl;
}

int docount = 0;

void Drive::abort()
{
}

void Drive::stop(function<void()> callback)
{
    if(callback != nullptr){driveCompletedCallback = callback;}

    if(state != WAITING_FOR_STOP || (state == WAITING_FOR_STOP && micros() - stop_timer > 100000))
    {
        state = WAITING_FOR_STOP;
        cout << "state: " << state << endl;
        cout << "STOP" << endl;
        wait_stop = true;
        stop_timer = micros();

        if(enable_drive)
        {
            serial->drive(stopPower,stopPower);
        }
    }
}

void Drive::confirm_stop()
{
    if(state == WAITING_FOR_STOP)
    {
        wait_stop = false;

        cout << "Confirmed stop" << micros() << endl;
        encoderRight.reset();
        encoderLeft.reset();
        state = STOPPED;
        if(driveCompletedCallback){driveCompletedCallback();}
    }
}

uint32_t Drive::getDistance()
{
    //cout << "encR: " << encoderRight.getDistance() << " encL: " << encoderLeft.getDistance() << endl;
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

void Drive::driveDistance(unsigned int speed, uint32_t distance, std::function<void()> callback, bool reverse, bool use_ramping)
{
    straight = unique_ptr<Drive_straight>(new Drive_straight(serial,&_current_left_encoder_distance,&_current_right_encoder_distance,& _current_left_encoder_speed,& _current_right_encoder_speed, gyro,ping,speed,distance,pids["straight_encoder"], pids["straight_gyro"],callback,reverse,use_ramping));
}

//Speed from 90-180, automatically calculates the reverse speed
void Drive::rotate(unsigned int speed, float degrees, Rotation_Direction direction, function<void()> callback)
{
    rotator = unique_ptr<Drive_rotate>(new Drive_rotate(serial, &_current_right_encoder_distance,&_current_left_encoder_distance,gyro,speed,degrees, pids["rotation_encoder"],pids["rotation_gyro"], callback, direction));;
}
