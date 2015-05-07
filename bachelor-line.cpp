#include "bachelor-line.h"

using namespace std;

int dtest = 0;
int dtest_mod = 1;
ofstream csv;

template <class DriveClass>
void LineFollower<DriveClass>::setup(shared_ptr<DriveClass> driver)
{
    _driver = driver;

    outerPID = unique_ptr<PID>(new PID(&out_pid_Input, &out_pid_Output, &out_pid_SetPoint,0.014,0,0,DIRECT,-50,50));
    out_pid_SetPoint = 0.0;
    outerPID->SetMode(AUTOMATIC);
    innerPID = unique_ptr<PID>(new PID(&inn_pid_Input, &inn_pid_Output, &inn_pid_SetPoint,3.0,0,0,DIRECT,-256,255));
    inn_pid_SetPoint = 0.0;
    innerPID->SetMode(AUTOMATIC);

    if(debug)
    {
        csv.open("data.csv");
    }
}

template <class DriveClass>
LineFollower<DriveClass>::LineFollower(uint8_t max_power, uint8_t min_power)
{
    this->max_power = max_power;
    this->min_power = min_power;
}

template <class DriveClass>
bool LineFollower<DriveClass>::enabled()
{
    return _enabled;
}

template <class DriveClass>
void LineFollower<DriveClass>::disable()
{
    _enabled = false;
}

template <class DriveClass>
void LineFollower<DriveClass>::enable()
{
    _driver->driveManual();
    _driver->set_distance_sensor_stop(false);
    _enabled = true;
    _driver->drive(max_power,max_power);
}

/*
 * Limits the power so we avoid driving to fast and try to avoid stoppping while turning
 */
static uint8_t check_power(const uint8_t min_power, const uint8_t max_power, uint8_t power)
{
    uint8_t power_out = power;

    if(power_out<90 && power_out >70){power_out = 70;}
    else if(power_out < min_power) { power_out = min_power; }
    else if(power_out > max_power) { power_out = max_power; }

    return power_out;
}

static uint8_t scale_power(const uint8_t min_power,const uint8_t max_power, uint8_t power)
{
    //For scaling the motor power with how much we want to turn
    float power_scaling = power/256.0;

    return max_power - round(power_scaling*(max_power-min_power));
}

//
// We drive right wheel(power2) slower, turning to the right
template <class DriveClass>
void LineFollower<DriveClass>::turn_right(uint8_t power)
{
    uint8_t power_left = check_power(min_power,max_power,max_power);
    uint8_t power_right = check_power(min_power,max_power,scale_power(min_power,max_power,power));

    if(debug && dtest%dtest_mod == 0)
    {
        LOG(DEBUG) << "Turn Right: " << power_right << endl;
    }

    prevLeftPower = power_left;
    prevRightPower = power_right;

    if(!_driver->drive(power_left,power_right))
    {
        disable();
    }
}

// We drive left wheel slower(power1), turning to the left
template <class DriveClass>
void LineFollower<DriveClass>::turn_left(uint8_t power)
{
    unsigned int power_left = check_power(min_power,max_power,scale_power(min_power,max_power,power));
    unsigned int power_right = check_power(min_power,max_power,max_power);

    if(debug && dtest%dtest_mod == 0)
    {
        LOG(DEBUG) << "Turn Left: " << power_left << endl;
    }

    prevLeftPower = power_left;
    prevRightPower = power_right;

    if(!_driver->drive(power_left,power_right))
    {
        disable();
    }
}

//Turn within the limits of max_power and min_power
//Direction from -256 to 255, negativ right, positiv left
template <class DriveClass>
void LineFollower<DriveClass>::do_turn(int direction)
{
    if(!_enabled) {return;}

    prev_direction = direction;

    if(debug && dtest%dtest_mod == 0)
    {
        LOG(DEBUG) << "Direction: " << direction << endl;
    }

    if(direction < 0)
    {
        turn_right(abs(direction));
    }
    else
    {
        turn_left(direction);
    }
}

template <class DriveClass>
void LineFollower<DriveClass>::setup_startposition(unsigned int position)
{
    collected_startpos = true;
    prev_distance = _driver->getDistance();
    prev_time = time;
    prev_dist_center = position - ir_center;
    dist_center = position - ir_center;
    delta_dist_center = 0;
    /*for(int i=0;i<20;i++)
    {
        prev_delta[i] = delta_dist_center;
        prev_dist[i] = dist_center;
    }*/
}

static bool check_line_not_found(unsigned int position, unsigned int &stopcount)
{
    if(position >= 7000 || position == 0)
    {
        if(stopcount>20)
        {
            return false;
        }

        stopcount++;
        return true;
    }
    else
    {
        stopcount = 0;
        return true;
    }
}

template <class DriveClass>
void LineFollower<DriveClass>::drive_reverse()
{
    _driver->driveDistance(110, 250, nullptr, true, false,true);
}

template <class DriveClass>
void LineFollower<DriveClass>::update(unsigned int position)
{
    time = micros();
    duration = time - prev_time;
    prev_time = time;
    dtest++;

    if(!collected_startpos){
        setup_startposition(position);
        _driver->drive(max_power,max_power);
        return;
    }

    if(!check_line_not_found(position,stopcount)){
        auto stop_callback = bind(&LineFollower::drive_reverse,this);
        _driver->stop(stop_callback);
        return;
    }

    distance = _driver->getDistance() - prev_distance;

    dist_center = position - ir_center;

    delta_dist_center = dist_center - prev_dist_center;

    if(debug && dtest%dtest_mod == 0){
        /*LOG(DEBUG) << "Position: " << position << endl;
          LOG(DEBUG) << "Median dist center: " << dist_center << endl;
          LOG(DEBUG) << "dist center: " << prev_dist[0] << endl;
          LOG(DEBUG) << "Delta dist center: " << delta_dist_center << endl;
          LOG(DEBUG) << "Distance : " << distance << endl;*/
    }

    out_pid_Input = dist_center;

    outerPID->Compute();

    inn_pid_SetPoint = out_pid_Output;
    inn_pid_Input = delta_dist_center;

    if(innerPID->Compute())
    {
        if(debug && dtest%dtest_mod == 0)
        {
            //LOG(DEBUG) << "inner pid: " << inn_pid_Output << endl;
        }
        do_turn(inn_pid_Output);
        //if(debug){Serial.println("PID output: " + String(pid_Output));}
    }

    //  1 time,         2 position,         3 dist_center,          4 avg dist center,    5 delta_dist_center,      6 median_delta_dist_center, 7 outer pid output, 8 inner pid output,         9 left power,  10 right power,                  11 distance                
    csv << time << ";" << position << ";" << prev_dist[0] << ";" << dist_center << ";" << prev_delta[0] << ";" << delta_dist_center << ";" << out_pid_Output << ";" << inn_pid_Output << ";" << prevLeftPower << ";" << prevRightPower << ";"  << distance << endl;

    prev_dist_center = dist_center;
}

static int getaverage(int16_t (&array)[20])
{
    int avg = 0;
    for(int i=0;i<20;i++)
    {
        avg += array[i];
    }
    return avg/20;
}

static int16_t GetMedian(int16_t *daArray, int size) {
    // Allocate an array of the same size and sort it.
    int16_t* dpSorted = new int16_t[4];
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

    /*if(dist_count < 5 && abs(dist_center - prev_dist_center) > 100)
      {
      dist_center = prev_dist_center; 
      dist_count++;
      }
      else
      {
      for(int i=4;i>0;i--)
      {
      prev_delta[i] = prev_delta[i-1];
      prev_dist[i] = prev_dist[i-1];
      }

      prev_dist[0] = dist_center;

      dist_center = GetMedian(prev_dist,5);
      dist_count = 0;
      }*/

    /*for(int i=19;i>0;i--)
    {
        prev_dist[i] = prev_dist[i-1];
    }

    prev_dist[0] = dist_center;*/

