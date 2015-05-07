#include "bachelor-line.h"

using namespace std;

int dtest = 0;
int dtest_mod = 1;
ofstream csv;

void LineFollower::setup(shared_ptr<Drive> driver)
{
    _driver = driver;

    outerPID = new PID(&out_pid_Input, &out_pid_Output, &out_pid_SetPoint,0.014,0,0,DIRECT,-50,50);
    out_pid_SetPoint = 0.0;
    outerPID->SetMode(AUTOMATIC);
    innerPID = new PID(&inn_pid_Input, &inn_pid_Output, &inn_pid_SetPoint,3.0,0,0,DIRECT,-256,255);
    inn_pid_SetPoint = 0.0;
    innerPID->SetMode(AUTOMATIC);

    if(debug)
    {
        csv.open("data.csv");
    }
}

LineFollower::~LineFollower()
{
    free(outerPID);
    free(innerPID);
}

bool LineFollower::enabled()
{
    return _enabled;
}

void LineFollower::disable()
{
    _enabled = false;
}

void LineFollower::enable()
{
    _driver->driveManual();
    _driver->set_distance_sensor_stop(false);
    _enabled = true;
    _driver->drive(maxPower,maxPower);
}

//Turn within the limits of maxpower and minpower
//Direction from -256 to 255, negativ right, positiv left
void LineFollower::do_turn(int direction)
{
    if(!_enabled) {return;}
    double nDirection = (double)(direction)/256.0;
    prev_direction = direction;
    //Serial.println("nDirection: " + String(nDirection));
    //
    if(debug && dtest%dtest_mod == 0)
    {
        LOG(DEBUG) << "Direction: " << direction << endl;
    }

    if(direction < 0)// We drive right wheel(power2) slower, turning to the right
    {
        //unsigned int power1 = minPower - nDirection*power_range;
        unsigned int power2 = maxPower + nDirection*power_range;
        //unsigned int power1 = maxPower - nDirection*power_range;

        //if(power1 <= 110 && power2 < 90 && power2 > 70){power2 = 70;}
        if(power2<90 && power2 >70){power2 = 70;}
        //else if(power2>80 && power2 > 70){power2 = 80;}

        if(debug && dtest%dtest_mod == 0)
        {
            LOG(DEBUG) << "Turn Right: " << power2 << endl;
        }
        //Serial.println("Turn right P1: " + String(maxPower) + " P2: " + String(power2)); 
        
        prevLeftPower = maxPower;
        prevRightPower = power2;
        if(!_driver->drive(maxPower,power2))
        {
            disable();
        }
    }
    else// We drive left wheel slower(power1), turning to the left
    {
        //    unsigned int power2 = minPower + nDirection*power_range;
        unsigned int power1 = maxPower - nDirection*power_range;
        //unsigned int power2 = maxPower + nDirection*power_range;
        if(power1<90 && power1 >70){power1 = 70;}
        //else if(power1>80 && power1 > 70){power1 = 80;}
        if(debug && dtest%dtest_mod == 0)
        {
            LOG(DEBUG) << "Turn Left: " << power1 << endl;
        }
        //Serial.println("Turn left P1: " + String(power1) + " P2: " + String(maxPower)); 
        
        prevLeftPower = power1;
        prevRightPower = maxPower;
        if(!_driver->drive(power1,maxPower))
        {
            disable();
        }
    }
}

bool LineFollower::update(unsigned int position)
{
    LOG(DEBUG) << "LineFollowing" << endl;
    //LOG(DEBUG) << "stopcount " << (int)stopcount << endl;
    if(!_enabled && stopcount > 20){return false;}
    else if(!_enabled){return true;}

    time = micros();

    duration = time - prev_time;

    /*if(duration > 3000)
    {
        LOG(DEBUG) << "LONG DURATION " << duration << " time " << time << " prevtime " << prev_time  << " nanos " << nanotime << " prev_nanos " << prev_nanos << " nano duration " << nanotime - prev_nanos << endl;
    }*/

    prev_time = time;

    dtest++;

    if(!collected_startpos){
        prev_distance = _driver->getDistance();
        prev_time = time;
        prev_distance_time = prev_time;
        previous_position = position;
        prev_dist_center = position - ir_center;
        collected_startpos = true;
        dist_center = position - ir_center;
        delta_dist_center = 0;
        for(int i=0;i<20;i++)
        {
            prev_delta[i] = delta_dist_center;
            prev_dist[i] = dist_center;
        }
        _driver->drive(maxPower,maxPower);
        return true;
    }

    if(position >= 7000 || position == 0)
    {
        /*if(stopcount>20){
            if(prev_direction < 0)
            {
                do_turn(-256);
            }
            else
            {
                do_turn(255);
            }
            return;
        }
        else if(stopcount > 200)
        {*/
        if(stopcount>20)
        {
            //LOG(DEBUG) << "Stopped" << endl;
            _driver->stop();
            previous_position = position;
            disable();
            return false;
        }
        stopcount++;
        return true;
    }

    stopcount = 0;
    
    if(dtest < 100)
    {
        return true;}

    /*if(position == previous_position && dtest > 10){
        LOG(DEBUG) << "Position = previous_position" << endl;
        return;
    }*/

    distance = _driver->getDistance() - prev_distance;

    if(distance == 0){
        //return;
    }

    dist_center = position - ir_center;

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

    for(int i=19;i>0;i--)
    {
        prev_dist[i] = prev_dist[i-1];
    }

    prev_dist[0] = dist_center;

    //dist_center = GetMedian(prev_dist,5);//getaverage(prev_dist);//GetMedian(prev_dist,5);*/

    //LOG(DEBUG) << "duration(ms) : " << duration/1000.0 << endl;
    
    //Calculate the average delta over the duration passed

    //prev_delta[0] = (dist_center - prev_dist_center);///((float)duration/1000.0);

    //delta_dist_center = (prev_delta[0] + prev_delta[1] + prev_delta[2] + prev_delta[3] + prev_delta[4])/5.0;
    /*if(dist_center - prev_dist_center !=  0)
    {
        for(int i=19;i>0;i--)
        {
            prev_delta[i] = prev_delta[i-1];
        }
        prev_delta[0] = dist_center - prev_dist_center;
        delta_dist_center = getaverage(prev_delta);//dist_center - prev_dist_center;//GetMedian(prev_delta,5);
        prev_distance = distance;
    }*/

    delta_dist_center = dist_center - prev_dist_center;

    /*if(abs(delta_dist_center) < 20){delta_dist_center = 0;}
    else
    {
        prev_dist_center = dist_center;
    }*/

    /*if(prevLeftPower < 90 && dist_center < 0)
    {
        delta_dist_center*=-1;
    }
    if(prevRightPower < 90 && dist_center > 0)
    {
        delta_dist_center*=-1;
    }*/

    //delta_dist_center = (prev_delta[0] + prev_delta[1] + prev_delta[2] + prev_delta[3])/4; //GetMedian(prev_delta);
    //delta_dist_center = GetMedian(prev_delta,5);

    if(debug && dtest%dtest_mod == 0){
        /*LOG(DEBUG) << "Position: " << position << endl;
        LOG(DEBUG) << "Median dist center: " << dist_center << endl;
        LOG(DEBUG) << "dist center: " << prev_dist[0] << endl;
        LOG(DEBUG) << "Delta dist center: " << delta_dist_center << endl;
        LOG(DEBUG) << "Distance : " << distance << endl;*/
    }

    if(true || (dist_center > 0 && delta_dist_center > 0) || (dist_center < 0 && delta_dist_center < 0))
    {
        out_pid_Input = dist_center;

        outerPID->Compute();
        //LOG(DEBUG) << "Outer pid: "<< out_pid_Output << endl;

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
    }

    //  1 time,         2 position,         3 dist_center,          4 avg dist center,    5 delta_dist_center,      6 median_delta_dist_center, 7 outer pid output, 8 inner pid output,         9 left power,  10 right power,                  11 distance                
    csv << time << ";" << position << ";" << prev_dist[0] << ";" << dist_center << ";" << prev_delta[0] << ";" << delta_dist_center << ";" << out_pid_Output << ";" << inn_pid_Output << ";" << prevLeftPower << ";" << prevRightPower << ";"  << distance << endl;

    prev_dist_center = dist_center;
    previous_position = position;
    return true;
}

int getaverage(int16_t (&array)[20])
{
    int avg = 0;
    for(int i=0;i<20;i++)
    {
        avg += array[i];
    }
    return avg/20;
}

int16_t GetMedian(int16_t *daArray, int size) {
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
