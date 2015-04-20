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

    //thread encoder_thread (&Drive::update_encoder,this, ref(encoderRight), ref(encoderLeft), ref(*gyro), ref(*ping));
    //encoder_thread.detach();

    /*thread encoder_thread(&Drive::encoder_thread,this, ref(frame));
    capture_thread.detach();*/
}

void Drive::update_encoder(Encoder &encoderR, Encoder &encoderL, gyroscope &gyro, Ping &ping)
{
    while(true)
    {
        //uint64_t start = micros();
        encoderRight.update();
        //printf("encoderR %" PRIu64 "\n", micros()-start);
        //start = micros();
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

//Distance mm
void Drive::driveDistance(unsigned int speed, unsigned long distance, function<void()> callback)
{
    if(state != STOPPED){return;}

    driveCompletedCallback = callback;


    encoderRight.reset();
    encoderLeft.reset();

    state = DRIVING_DISTANCE;
    _distance = distance*1000;

    maxLeftSpeed = speed;
    maxRightSpeed = speed;

    printf("step1\n");
    rotationPID = unique_ptr<PID>(new PID(&rotationPID_input, &rotationPID_output, &rotationPID_setpoint, 1, rotationPIDKi, rotationPIDKd, DIRECT, 90-speed,0));
    rotationPID_setpoint = 0;
    rotationPID->SetMode(AUTOMATIC);

    printf("step2\n");
    gyro->start(0);

    encoderPID = unique_ptr<PID>(new PID(&encoder_pid_Input, &encoder_pid_Output, &encoder_pid_SetPoint,1,0,0,DIRECT,-10,speed-100));

    encoder_pid_SetPoint = 0;
    encoderPID->SetMode(AUTOMATIC);

    serial->drive(speed,speed);

    prevRightSpeed = 90;
    prevLeftSpeed = 90;

    leftSpeed = 100;
    rightSpeed = 100;
    currentLeftSpeed = leftSpeed;
    currentRightSpeed = rightSpeed;
}

void Drive::drive(unsigned int power1, unsigned int power2)
{
    if(state == DRIVING_MANUAL || state == STOPPED){
        state = DRIVING_MANUAL;
        serial->drive(power1,power2);
    }
}

//Speed from 90-180, automatically calculates the reverse speed
void Drive::rotate(unsigned int speed, float degrees, Rotation_Direction direction, function<void()> callback)
{
    if(state != STOPPED){return;}

    LOG(DEBUG) << "Starting rotating\n";

    rotationPID = unique_ptr<PID>(new PID(&rotationPID_input, &rotationPID_output, &rotationPID_setpoint, rotationPIDKp, rotationPIDKi, rotationPIDKd, DIRECT, -20,0));
    rotationPID_setpoint = 0.0;
    rotationPID->SetMode(AUTOMATIC);

    rot_dir = direction;

    encoderPID = unique_ptr<PID>(new PID(&encoder_pid_Input, &encoder_pid_Output, &encoder_pid_SetPoint,encoder_consKp,encoder_consKi,encoder_consKd,DIRECT,-10,0));
    encoder_pid_SetPoint = 0.0;
    encoderPID->SetMode(AUTOMATIC);

    driveCompletedCallback = callback;

    state = ROTATING;
    gyro->start(degrees);

    maxLeftSpeed = speed;
    maxRightSpeed = speed;

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
    ptime = nanos();
    encoderRight.update();
    encoderLeft.update();
    cout << "Encoders: " << nanos() - ptime << endl;


    ptime = nanos();
    gyro->update();
    cout << "Gyro: " << nanos() - ptime << endl;

    ptime = nanos();
    ping->update();
    cout << "Ping: " << nanos() - ptime << endl;
    //start = micros();
    ///
    //LOG(DEBUG) << "Duration: " << micros() - ptime << endl;
    //ptime = micros();
    float dist = ping->get_distance();
    if(false && state != STOPPED && dist > 0 && dist < 0.5)
    //if(dist != prevdist)
    {
        //LOG(DEBUG) << "Ping distance: " << ping->get_distance() << endl;
        LOG(DEBUG) << "Ping stop" << endl;
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
            if(driveCompletedCallback){driveCompletedCallback();}
            return;
        }
    }else if(state == DRIVING_DISTANCE)
    {
        if(encoderLeft.getDistance() >= _distance)
        {
            LOG(DEBUG) << "Drive stopping" << endl;
            LOG(DEBUG) << "Left distance" << endl;
            LOG(DEBUG) << encoderLeft.getDistance();
            LOG(DEBUG) << "Right distance" << endl;
            LOG(DEBUG) << encoderRight.getDistance();
            stop();
            _distance = 0;

            if(driveCompletedCallback){driveCompletedCallback();}
            return;
        }
        else
        {
            //encoder_pid_Input = encoderRight.getSpeed() - encoderLeft.getSpeed();
            /*LOG(DEBUG) << "pid input: " << encoder_pid_Input;
            LOG(DEBUG) << "Left distance";
            LOG(DEBUG) << encoderLeft.getDistance();
            LOG(DEBUG) << "Right distance";
            LOG(DEBUG) << encoderRight.getDistance();
            LOG(DEBUG) << "Encoder pid: " << encoder_pid_Output;*/

            //LOG(DEBUG) << "Speed: " << (encoderRight.getSpeed() + encoderLeft.getSpeed())/2.0;

            //Keeping straight with encoder
            
            /*
            if(encoderLeft.getDistance() < encoderRight.getDistance())
            {
                encoder_pid_Input = (double)encoderRight.getDistance() - (double)encoderLeft.getDistance();
                encoderPID->Compute();
                if(currentRightSpeed != rightSpeed + (int)encoder_pid_Output)
                {
                    //LOG(DEBUG) << "Distance: " << encoderLeft.getDistance();
                    currentRightSpeed = rightSpeed + (int)encoder_pid_Output;
                    currentLeftSpeed = leftSpeed;

                    //LOG(DEBUG) << "Speeds: " << (int)rightSpeed << " , " << (int)currentLeftSpeed << " ," << (int)currentRightSpeed;
                }
            }
            else
            {
                encoder_pid_Input = (double)encoderLeft.getDistance() - (double)encoderRight.getDistance();
                encoderPID->Compute();
                if(currentLeftSpeed != leftSpeed + (int)encoder_pid_Output)
                {
                    //LOG(DEBUG) << "Distance: " << encoderLeft.getDistance();
                    currentLeftSpeed = leftSpeed + (int)encoder_pid_Output;
                    currentRightSpeed = rightSpeed;


                    LOG(DEBUG) << "Speeds: " << (int)rightSpeed << " , " << (int)currentLeftSpeed << " ," << (int)currentRightSpeed;
                }
            }*/

            //Keeping straight with gyro
            rotationPID_input = (int)(abs(gyro->get_total_rotation()) * 100.0);
            //LOG(DEBUG) << "input: " << (int)(abs(gyro->get_total_rotation()) * 100.0) << endl;
            rotationPID->Compute();
            //LOG(DEBUG) << "output: " << encoder_pid_Output << endl;

            //LOG(DEBUG) << "rotation output: " << gyro->get_total_rotation() << endl;
            //LOG(DEBUG) << "gyrypid output: " << rotationPID_output << endl;

            float target_speed = 0.15;
            if(encoderLeft.getDistance() < 100000)//50 cm
            {
                encoder_pid_Input = (int)((encoderLeft.getSpeed() - target_speed)*100.0);//mm/s
                encoderPID->Compute();
                LOG(DEBUG) << "input: " <<(int)((encoderLeft.getSpeed() - target_speed)*1000.0);//mm/s (int)encoder_pid_Output << endl;
                LOG(DEBUG) << "speed: " <<  encoderLeft.getSpeed() << endl;
                LOG(DEBUG) << "output: " << (int)encoder_pid_Output << endl;
                //
                //LOG(DEBUG) << "Distance : " << encoderLeft.getDistance();
                //LOG(DEBUG) << "EncoderPID out " << encoder_pid_Output;
                //
                currentLeftSpeed = leftSpeed + encoder_pid_Output;
                currentRightSpeed =  rightSpeed + encoder_pid_Output;

                //LOG(DEBUG) << "Current left speed: " << (int)currentLeftSpeed;
            }
            else
            {
                LOG(DEBUG) << "Driving faster " << endl;
                currentLeftSpeed = maxLeftSpeed;
                currentRightSpeed = maxRightSpeed;
            }

            if(gyro->get_total_rotation() > 0.05)//If rotated towards right, then turn towards left
            {
                currentLeftSpeed = currentLeftSpeed + (int)rotationPID_output;
                currentRightSpeed = currentRightSpeed;
            }
            else if(gyro->get_total_rotation() < 0.05) //if rotated towards left, then turn towards right
            {
                currentRightSpeed = currentRightSpeed + (int)rotationPID_output;
                currentLeftSpeed = currentLeftSpeed;
            }

            if(currentLeftSpeed < 90){currentLeftSpeed = 90;}
            if(currentRightSpeed < 90){currentRightSpeed = 90;}

            //LOG(DEBUG) << "Current left speed: " << (int)currentLeftSpeed << endl;

            ptime = nanos();
            do_drive();
            cout << "Do_driver: " << nanos() - ptime << endl;

            //ST2.write(rightSpeed + encoder_pid_Output);
            //
            //
            //O
            /*Serial.println("rightdistance");
            Serial.println(encoderLeft.getDistance());
            Serial.println("leftdistance");
            Serial.println(encoderRight.getDistance());*/
/*            Serial.println("rightSpeed: ");
            Serial.println(rightSpeed);
            Serial.println("leftSpeed: ");
            Serial.println(rightSpeed + encoder_pid_Output);*/
        }
    }
    else if(state == ROTATING)
    {
        float target_speed = 1.0;

        LOG(DEBUG) << "Distance rotation: " << gyro->get_distance_rotation() << endl;
        LOG(DEBUG) << "Current rotation: " << gyro->get_current_rotation() << endl;

        if(gyro->get_distance_rotation() < 5.0)
        {
            target_speed = 0.5;
        }
        else if(gyro->get_distance_rotation() < 20.0)
        {
            target_speed = 0.5;
        }

        rotationPID_input = abs(gyro->get_current_rotation()) - target_speed;
        rotationPID->Compute();

        currentLeftSpeed = leftSpeed + rotationPID_output;
        currentRightSpeed = rightSpeed + rotationPID_output;

        //LOG(DEBUG) << "current left: " << (int)currentLeftSpeed << endl;
        //LOG(DEBUG) << "current right: " << (int)currentRightSpeed << endl;

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
        }

        //If new speed is different the old speed
        if((currentLeftSpeed != tempspeedLeft && currentRightSpeed == tempspeedRight) 
                || (currentRightSpeed != tempspeedRight && currentLeftSpeed == tempspeedLeft))
        {
            currentLeftSpeed = tempspeedLeft;
            currentRightSpeed = tempspeedRight;

            if(gyro->distance_rotation > 5.0)
            {
                do_rotate();
            }
        }

        if(gyro->distance_rotation < 5.0)
        {
            //Modifier that will reduce rotation speed (110) from 110 down to 95 depending on how close the rotation is to completed
            //speedMod = 1.0 - (1.0 - gyro->distance_rotation/5.0) * rotation_mod;

            //currentLeftSpeed = leftSpeed * speedMod * speedMod;
            //currentRightSpeed = rightSpeed * speedMod * speedMod;

            if(currentLeftSpeed < 90){currentLeftSpeed = 90;}
            if(currentRightSpeed < 90){currentRightSpeed = 90;}

            LOG(DEBUG) << "SpeedMod " << speedMod;
            LOG(DEBUG) << "Distance " << gyro->distance_rotation << " , left-speed: " << (int)currentLeftSpeed << " right-speed: "  << (int)currentRightSpeed;

            do_rotate();
        }
        else
        {
            speedMod = 1.0;
        }
        */

        //if(abs(abs(gyro->total_rotation) - abs(gyro->goal_rotation)) < 0.0194)
        if(gyro->get_distance_rotation() < 2.0)
        {
            LOG(DEBUG) << "Rotation complete\n";
            if(driveCompletedCallback){driveCompletedCallback();}
            gyro = nullptr;
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

    LOG(DEBUG) << "Left: " << (int)currentLeftSpeed << endl;
    LOG(DEBUG) << "Right: " << (int)currentRightSpeed << endl;

    serial->drive(currentLeftSpeed, currentRightSpeed);
}

void Drive::do_rotate()
{
    if(currentRightSpeed == prevRightSpeed && currentLeftSpeed == prevLeftSpeed){return;}

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
    //stop();
}
