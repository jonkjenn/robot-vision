#include "PID_v1.h"
#include <stdio.h>
#include "drive.h"
#include <fstream>
#include "utility.hpp"
#include <memory>
#include <iostream>

/*
 * Limits the power so we avoid driving to fast and try to avoid stoppping while turning
 */
static uint8_t check_power(const uint8_t min_power, const uint8_t max_power, uint8_t power)
{
    uint8_t power_out = power;

    //if(power_out<90 && power_out >70){power_out = 70;}
    if(power_out < min_power) { std::cout << "less" << std::endl; power_out = min_power; }
    else if(power_out > max_power) { std::cout << "more" << std::endl; power_out = max_power; }

    return power_out;
}

static uint8_t scale_power(const uint8_t min_power,const uint8_t max_power, uint8_t power)
{
    //For scaling the motor power with how much we want to turn
    float power_scaling = power/256.0;

    return max_power - round(power_scaling*(max_power-min_power));
}

static bool check_line_found(unsigned int position)
{
    return position < 7000 && position > 0;
}

static bool check_if_stopcount_stop(bool on_line, unsigned int &stopcount, unsigned int maximum_stopcount)
{
    if(!on_line)
    {
        stopcount++;
        if(stopcount>maximum_stopcount)
        {
            return true;
        }

        return false;
    }
    else
    {
        stopcount = 0;
        return false;
    }
}

static int getaverage(int16_t (&array)[5])
{
    int avg = 0;
    for(int i=0;i<5;i++)
    {
        avg += array[i];
    }
    return avg/5.0;
}

static int16_t GetMedian(int16_t *daArray, int size) {
    // Allocate an array of the same size and sort it.
    int16_t* dpSorted = new int16_t[20];
    for (int i = 0; i < size; ++i) {
        dpSorted[i] = daArray[i];
    }
    for (int i = size - 1; i > 0; --i) {
        for (int j = 0; j < i; ++j) {
            if (dpSorted[j] > dpSorted[j+1]) {
                int16_t dTemp = dpSorted[j];
                dpSorted[j] = dpSorted[j+1];
                dpSorted[j+1] = dTemp;
            }
        }
    }

    // Middle or average of middle values in the sorted array.
    int16_t dMedian = 0;
    if ((size % 2) == 0) {
        dMedian = ((dpSorted[size/2] + dpSorted[(size/2) - 1])/2.0);
    } else {
        dMedian = dpSorted[size/2];
    }
    delete [] dpSorted;
    return dMedian;
}

//Template for mocking
template <class DriveClass>
class LineFollower{

    private:
        // This example is designed for use with eight QTR-1RC sensors or the eight sensors of a
        // QTR-8RC module.  These reflectance sensors should be connected to digital inputs 3 to 10.
        // The QTR-8RC's emitter control pin (LEDON) can optionally be connected to digital pin 2, 
        // or you can leave it disconnected and change the EMITTER_PIN #define below from 2 to 
        // QTR_NO_EMITTER_PIN.

        // The setup phase of this example calibrates the sensor for ten seconds and turns on
        // the LED built in to the Arduino on pin 13 while calibration is going on.
        // During this phase, you should expose each reflectance sensor to the lightest and 
        // darkest readings they will encounter.
        // For example, if you are making a line follower, you should slide the sensors across the
        // line during the calibration phase so that each sensor can get a reading of how dark the
        // line is and how light the ground is.  Improper calibration will result in poor readings.
        // If you want to skip the calibration phase, you can get the raw sensor readings
        // (pulse times from 0 to 2500 us) by calling qtrrc.read(sensorValues) instead of
        // qtrrc.readLine(sensorValues).

        // The main loop of the example reads the calibrated sensor values and uses them to
        // estimate the position of a line.  You can test this by taping a piece of 3/4" black
        // electrical tape to a piece of white paper and sliding the sensor across it.  It
        // prints the sensor values to the serial monitor as numbers from 0 (maximum reflectance) 
        // to 1000 (minimum reflectance) followed by the estimated location of the line as a number
        // from 0 to 5000.  1000 means the line is directly under sensor 1, 2000 means directly
        // under sensor 2, etc.  0 means the line is directly under sensor 0 or was last seen by
        // sensor 0 before being lost.  5000 means the line is directly under sensor 5 or was
        // last seen by sensor 5 before being lost.

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low

        // sensors 0 through 7 are connected to digital pins 3 through 10, respectively
        //        QTRSensors *qtrrc;
        uint16_t sensorValues[NUM_SENSORS];
        //Servo ST1,ST2;//ST1 left motor, ST2 right motor

        //unsigned int preCalibratedMin[] = {800, 588, 488, 532, 536, 536, 580, 812};
        uint16_t preCalibratedMin[8] = {750, 550, 500, 500, 450, 400, 400, 450};
        uint16_t preCalibratedMax[8] = {2500,2500,2500,2500,2500,2500,2500,2500};

        float out_pid_SetPoint,out_pid_Input,out_pid_Output;
        float out_consKp=1.0, out_consKi=0.0, out_consKd=0.0, out_pid_weight = 1.0;
        float inn_pid_SetPoint,inn_pid_Input,inn_pid_Output;
        float inn_consKp=2.0, inn_consKi=0.0, inn_consKd=0.0, inn_pid_weight = 1.0;

        std::unique_ptr<PID> outerPID = nullptr;
        std::unique_ptr<PID> innerPID = nullptr;

        uint8_t max_power = 130;
        uint8_t min_power = 70;
        const uint8_t stopPower = 90;//stand still
        uint8_t power_range = max_power - min_power;

        unsigned int prevLeftPower = 90;
        unsigned int prevRightPower = 90;

        const unsigned int MAXIMUM_STOPCOUNT = 20;

        int dtest = 0;
        int dtest_mod = 1;

        std::ofstream csv;

        //Turn within the limits of max_power and min_power
        //Direction from -256 to 255, negativ right, positiv left
        void do_turn(int direction)
        {
            if(!_enabled) {return;}

            prev_direction = direction;

            if(debug && dtest%dtest_mod == 0)
            {
                std::cout << "Direction: " << direction << std::endl;
            }

            if(direction > 255) {direction = 255;}
            if(direction < -255) {direction = -255;}

            if(direction > 0)
            {
                turn_right(abs(direction));
            }
            else
            {
                turn_left(abs(direction));
            }
        }

        uint16_t previous_position = 0;
        uint16_t position = 0;

        unsigned int previous_update = 0;

        std::shared_ptr<DriveClass> _driver = nullptr;

        uint64_t time = 0;
        uint64_t prev_time = 0;
        uint64_t duration = 0;

        uint64_t distance = 0;
        uint64_t prev_distance_time = 0;
        uint64_t prev_distance = 0;
        int prev_distance_dist_center = 0;
        float prev_angle = 0;

        int prev_direction = 0;

        uint8_t dist_count = 0;

        uint64_t avg_distance = 0;

        const uint8_t debug = 1;
        int result_ready = -1;

        uint8_t startpos_count = 0;
        bool collected_startpos = false;

        const uint16_t ir_width = 65; //65mm, 65000um

        const float ir_modifier = ir_width/7.0; //0-7000 being the position value range, reduced to 7 because ir_width is mm

        const uint16_t ir_center = 3500;

        int dist_center = 0;
        int16_t prev_dist_center = 0;
        int16_t delta_dist_center = 0;
        int16_t delta_position = 0;
        uint32_t diff= 0;


        int16_t sum_delta_dist = 0;

        unsigned int stopcount = 0;

        int leftright = 0;

        int16_t prev_dist[10] = {0};
        int16_t prev_delta[5] = {0};
        int16_t part_tmp[5] = {0};
        bool _enabled = false;

        bool _wait_for_line = false;

        // We drive left wheel slower(power1), turning to the left
        void turn_left(uint8_t power)
        {
            uint8_t power_left = check_power(min_power,max_power,scale_power(min_power,max_power,power));
            uint8_t power_right = check_power(min_power,max_power,max_power);

            if(debug && dtest%dtest_mod == 0)
            {
                std::cout << "Turn Left: " << (int)power_left << std::endl;
            }

            prevLeftPower = power_left;
            prevRightPower = power_right;

            !_driver->drive(power_left,power_right);
        }
        //
        // We drive right wheel(power2) slower, turning to the right
        void turn_right(uint8_t power)
        {

            uint8_t power_left = check_power(min_power,max_power,max_power);
            uint8_t power_right = check_power(min_power,max_power,scale_power(min_power,max_power,power));

            if(debug && dtest%dtest_mod == 0)
            {
                std::cout << "Turn Right: " << (int)power_right << std::endl;
            }

            prevLeftPower = power_left;
            prevRightPower = power_right;

            !_driver->drive(power_left,power_right);
        }
        void setup_startposition(unsigned int position)
        {
            collected_startpos = true;
            prev_distance = _driver->getDistance();
            prev_time = time;
            prev_dist_center = position - ir_center;
            dist_center = position - ir_center;
            delta_dist_center = 0;
            /*for(int i=0;i<10;i++)
              {
              prev_delta[i] = delta_dist_center;
              //prev_dist[i] = dist_center;
              }*/
        }

        void drive_reverse()
        {
            _driver->driveDistance(110, 250, nullptr, true, false,true);
        }

    public:
        LineFollower(uint8_t max_power = 110, uint8_t min_power = 70)
        {
            this->max_power = max_power;
            this->min_power = min_power;
        }

        void update(unsigned int position)
        {
            if(!_enabled){return;}

            time = micros();
            duration = time - prev_time;
            prev_time = time;
            dtest++;

            std::cout << "Linefollower position: " << position << std::endl;
            std::cout << "stopcount : " << stopcount << std::endl;

            //If we dont find the line
            if(!_wait_for_line && check_if_stopcount_stop(check_line_found(position),stopcount,MAXIMUM_STOPCOUNT)){
                //_driver->stop([]{});
                _driver->abort();
                _driver->driveDistance(110, 250, nullptr, true, false,true);
                //
                //_driver->stop(stop_callback);
                //auto stop_callback = std::bind(&LineFollower::drive_reverse,this);
                _wait_for_line = true;
                return;
            }
            //We lost the line, but then we found it again
            else if(_wait_for_line && check_line_found(position))
            {
                _driver->abort();
                _driver->driveManual();
                _driver->set_distance_sensor_stop(false);
                stopcount = 0;
                _wait_for_line = false;
            }

            if(!collected_startpos){
                setup_startposition(position);
                _driver->drive(max_power,max_power);
                return;
            }

            calculate_power_from_position(position);

        }

        void calculate_power_from_position(int position)
        {
            distance = _driver->getDistance() - prev_distance;

            dist_center = position - ir_center;

            for(int i=4;i>0;i--)
              {
                  prev_delta[i-1] = prev_delta[i];
              }

            delta_dist_center = dist_center - prev_dist_center;

            prev_delta[4] = delta_dist_center;

            delta_dist_center = getaverage(prev_delta);

            if(debug && dtest%dtest_mod == 0){
                /*std::cout << "Position: " << position << std::endl;
                  std::cout << "Median dist center: " << dist_center << std::endl;
                  std::cout << "dist center: " << prev_dist[0] << std::endl;
                  std::cout << "Delta dist center: " << delta_dist_center << std::endl;
                  std::cout << "Distance : " << distance << std::endl;*/
            }


            out_pid_Input = (abs(dist_center)<1500?0:dist_center);

            outerPID->Compute();

            std::cout << "outer pid: " << out_pid_Output << std::endl;

            inn_pid_SetPoint = out_pid_Output;
            inn_pid_Input = ((abs(delta_dist_center) < 100) && (abs(dist_center)<1500))?0:delta_dist_center;

            if(innerPID->Compute())
            {
                if(debug && dtest%dtest_mod == 0)
                {
                    std::cout << "inner pid input: " << inn_pid_Input << std::endl;
                    std::cout << "inner pid: " << inn_pid_Output << std::endl;
                }
                do_turn(inn_pid_Output);
                //if(debug){Serial.println("PID output: " + String(pid_Output));}
            }

            //  1 time,         2 position,         3 dist_center,          4 avg dist center,    5 delta_dist_center,      6 median_delta_dist_center, 7 outer pid output, 8 inner pid output,         9 left power,  10 right power,                  11 distance                
            //csv << time << ";" << position << ";" << prev_dist[0] << ";" << dist_center << ";" << prev_delta[0] << ";" << delta_dist_center << ";" << out_pid_Output << ";" << inn_pid_Output << ";" << prevLeftPower << ";" << prevRightPower << ";"  << distance << std::endl;

            prev_dist_center = dist_center;
        }

        void setup(std::shared_ptr<DriveClass> driver)
        {
            _driver = driver;

            outerPID = std::unique_ptr<PID>(new PID(&out_pid_Input, &out_pid_Output, &out_pid_SetPoint,0.014,0,0,DIRECT,-50,50));
            out_pid_SetPoint = 0.0;
            outerPID->SetMode(AUTOMATIC);
            innerPID = std::unique_ptr<PID>(new PID(&inn_pid_Input, &inn_pid_Output, &inn_pid_SetPoint,4.0,0,0.0,DIRECT,-255,255));
            inn_pid_SetPoint = 0.0;
            innerPID->SetMode(AUTOMATIC);

            if(debug)
            {
                //csv.open("data.csv");
            }
        }
        bool enabled()
        {
            return _enabled;
        }

        void enable()
        {
            _driver->driveManual();
            _driver->set_distance_sensor_stop(false);
            _enabled = true;
        }

        void disable()
        {
            _enabled = false;
        }

};
