#include "bachelor-line.h"

using namespace std;

int dtest = 0;
int dtest_mod = 1;

void LineFollower::setup(shared_ptr<Drive> driver)
{
    _driver = driver;

    outerPID = new PID(&out_pid_Input, &out_pid_Output, &out_pid_SetPoint,0.015,0,0,DIRECT,-40,40);
    out_pid_SetPoint = 0.0;
    outerPID->SetMode(AUTOMATIC);
    innerPID = new PID(&inn_pid_Input, &inn_pid_Output, &inn_pid_SetPoint,4.5,0,0,DIRECT,-256,255);
    inn_pid_SetPoint = 0.0;
    innerPID->SetMode(AUTOMATIC);

    _driver->driveManual();
    _driver->set_distance_sensor_stop(false);
}

LineFollower::~LineFollower()
{
    free(outerPID);
    free(innerPID);
}

//Turn within the limits of maxpower and minpower
//Direction from -256 to 255, negativ right, positiv left
void LineFollower::do_turn(int direction)
{
    if(!enabled) {return;}
    double nDirection = (double)(direction)/256.0;
    //Serial.println("nDirection: " + String(nDirection));
    //
    if(debug && dtest%dtest_mod == 0)
    {
        LOG(DEBUG) << "Direction: " << direction << endl;
    }

    if(direction < 0)
    {
        //unsigned int power1 = minPower - nDirection*power_range;
        unsigned int power2 = maxPower + nDirection*power_range;
        if(power2<80 && power2 >70){power2 = 70;}
        else if(power2<90 && power2 >70){power2 = 90;}
        if(debug && dtest%dtest_mod == 0)
        {
            LOG(DEBUG) << "Turn Right: " << power2 << endl;
        }
        //Serial.println("Turn right P1: " + String(maxPower) + " P2: " + String(power2)); 
        
        if(!_driver->drive(maxPower,power2))
        {
            enabled = false;
        }
    }
    else
    {
        //    unsigned int power2 = minPower + nDirection*power_range;
        unsigned int power1 = maxPower - nDirection*power_range;
        if(power1<80 && power1 >70){power1 = 70;}
        else if(power1<90 && power1 >70){power1 = 90;}
        if(debug && dtest%dtest_mod == 0)
        {
            LOG(DEBUG) << "Turn Left: " << power1 << endl;
        }
        //Serial.println("Turn left P1: " + String(power1) + " P2: " + String(maxPower)); 
        
        if(!_driver->drive(power1,maxPower))
        {
            enabled = false;
        }
    }
}

void LineFollower::update(unsigned int position)
{
    if(!enabled){return;}

    duration = micros() - prev_time;
    prev_time = micros();

    dtest++;

    if(!collected_startpos){
        prev_distance = _driver->getDistance();
        prev_time = micros();
        prev_distance_time = prev_time;
        previous_position = position;
        prev_dist_center = position - ir_center; 
        collected_startpos = true; 
        dist_center = position - ir_center;
        delta_dist_center = dist_center - prev_dist_center;
        for(int i=0;i<4;i++)
        {
            prev_delta[i] = delta_dist_center;
            prev_dist[i] = dist_center;
        }
        return;
    }

    if(position >= 7000 || position == 0)
    {
        if(stopcount>20){
            LOG(DEBUG) << "Stopped" << endl;
            _driver->stop();
            previous_position = position;
            enabled = false;
            return;
        }
        stopcount++;
        return;
    }

    stopcount = 0;

    /*if(position == previous_position && dtest > 10){
        LOG(DEBUG) << "Position = previous_position" << endl;
        return;
    }*/

    distance = _driver->getDistance();

    if(false && distance < 1){
        LOG(DEBUG) << "Distance < 1" <<endl;
        previous_position = position;
        return;
    }

    dist_center = position - ir_center;

    /*for(int i=4;i>0;i--)
    {
        prev_delta[i] = prev_delta[i-1];
        prev_dist[i] = prev_dist[i-1];
    }

    prev_dist[0] = dist_center;*/

    //dist_center = GetMedian(prev_dist,5);
    if(dist_count < 5 && abs(dist_center - prev_dist_center) > 100)
    {
        dist_center = prev_dist_center; 
        dist_count++;
    }
    else
    {
        dist_count = 0;
    }

    LOG(DEBUG) << "duration(ms) : " << duration/1000.0 << endl;
    
    //Calculate the average delta over the duration passed
    //prev_delta[0] = (dist_center - prev_dist_center)/((float)duration/1000.0);

    LOG(DEBUG) << "Distance : " << distance << " prev distance: " << prev_distance << endl;
    LOG(DEBUG) << "Dist center: " << dist_center << " Prev dist center: " << prev_distance_dist_center << endl;
    //using angles
    float x = distance - prev_distance;
    float y = (dist_center-prev_distance_dist_center)*95000.0/7000.0;
    float angle = 1.0;
    if(x > 0 && y != 0)
    {
        prev_distance = distance;
        prev_distance_dist_center = dist_center;
        angle = atan2(y,x) * 180.0/PI;
        LOG(DEBUG) << "y: " << y << " x: " << x << endl;
        LOG(DEBUG) << "angle: " << angle << endl;
    }

    //delta_dist_center = (prev_delta[0] + prev_delta[1] + prev_delta[2] + prev_delta[3] + prev_delta[4])/5.0;

    /*if(prevLeftPower < 90 && dist_center < 0)
    {
        delta_dist_center*=-1;
    }
    if(prevRightPower < 90 && dist_center > 0)
    {
        delta_dist_center*=-1;
    }*/

    delta_position = position - previous_position;

    //delta_dist_center = (prev_delta[0] + prev_delta[1] + prev_delta[2] + prev_delta[3])/4; //GetMedian(prev_delta);
    //delta_dist_center = GetMedian(prev_delta,5);

    if(debug && dtest%dtest_mod == 0){

        LOG(DEBUG) << "Position: " << position << endl;
        LOG(DEBUG) << "Median dist center: " << dist_center << endl;
        LOG(DEBUG) << "Delta position: " << delta_position << endl;
        LOG(DEBUG) << "Distance : " << _driver->getDistance() << endl;
    }


    if(true || (dist_center > 0 && delta_dist_center > 0) || (dist_center < 0 && delta_dist_center < 0))
    {
        out_pid_Input = dist_center;

        outerPID->Compute();
        LOG(DEBUG) << "Outer pid: "<< out_pid_Output << endl;
        //do_turn(out_pid_Output);
        //

        inn_pid_SetPoint = out_pid_Output;
            
        if(x == 0 && y == 0)
        {
            inn_pid_Input = out_pid_Output;
        }
        else
        {
            inn_pid_Input = angle;
        }

        if(innerPID->Compute())
        {
            if(debug && dtest%dtest_mod == 0)
            {
                LOG(DEBUG) << "inner pid: " << inn_pid_Output << endl;
            }
            do_turn(inn_pid_Output);
            //if(debug){Serial.println("PID output: " + String(pid_Output));}
        }
    }

    prev_dist_center = dist_center;
    previous_position = position;
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
