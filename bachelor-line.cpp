#include "bachelor-line.h"

using namespace std;

void LineFollower::setup(shared_ptr<Drive> driver)
{
    _driver = driver;
    /*unsigned char sensor_pins[8] =  {22, 23, 24, 25, 26, 27, 28, 29};
    unsigned char EMITTER_PIN = PC7;     // emitter is controlled by digital pin 2
    qtrrc = new QTRSensors(sensor_pins,
            NUM_SENSORS, TIMEOUT, EMITTER_PIN, driver); */
    outerPID = new PID(&out_pid_Input, &out_pid_Output, &out_pid_SetPoint,1,0,0,DIRECT,-50,50);
    out_pid_SetPoint = 0.0;
    outerPID->SetMode(AUTOMATIC);
    innerPID = new PID(&inn_pid_Input, &inn_pid_Output, &inn_pid_SetPoint,1,0,0,DIRECT,-256,255);
    inn_pid_SetPoint = 0.0;
    innerPID->SetMode(AUTOMATIC);

    _driver->driveManual();
}

LineFollower::~LineFollower()
{
    free(outerPID);
    free(innerPID);
}

//Turn within the limits of maxpower and minpower50
//NO Direction 0-255, 0-127 towards right, 129-255 towards left, about.
//Direction -256-255 negativ right, positiv left
void LineFollower::do_turn(int direction)
{
    double nDirection = (double)(direction)/256.0;
    //Serial.println("nDirection: " + String(nDirection));
    LOG(DEBUG) << "Direction: " << direction << endl;

    if(direction < 0)
    {
        //unsigned int power1 = minPower - nDirection*power_range;
        unsigned int power2 = maxPower + nDirection*power_range;
        if(power2<90 && power2 >70){power2 = 70;}
        LOG(DEBUG) << "Turn Right: " << power2 << endl;
        //Serial.println("Turn right P1: " + String(maxPower) + " P2: " + String(power2)); 
        
        //if(!_driver->drive(maxPower,power2))
        {
            //enabled = false;
        }
    }
    else
    {
        //    unsigned int power2 = minPower + nDirection*power_range;
        unsigned int power1 = maxPower - nDirection*power_range;
        if(power1<90 && power1 >70){power1 = 70;}
        LOG(DEBUG) << "Turn Left: " << power1 << endl;
        //Serial.println("Turn left P1: " + String(power1) + " P2: " + String(maxPower)); 
        
        //if(!_driver->drive(maxPower,power1))
        {
            //enabled = false;
        }
    }
}

int dtest = 0;
void LineFollower::update(unsigned int position)
{
    if(!enabled){return;}
    dtest++;
    //read calibrated sensor values and obtain a measure of the line position from 0 to 5000
    // To get raw sensor values, call:
    //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
    /*if(result_ready<0)
    {
        //if(debug){Serial.println("result_ready: " + String(result_ready));}
        qtrrc->readLine(sensorValues, QTR_EMITTERS_ON,0, &position, &result_ready, preCalibratedMin, preCalibratedMax);
        return;
    }
    else if(result_ready == 0)
    {
        qtrrc->update();
        return;
    }

    result_ready = -1;//Next round we will restart readline process*/

    //unsigned int position = 1;
    //qtrrc->read(sensorValues);
    //if(debug){Serial.println("Position: " + String(position));}
    //
    //Serial.println(position);
    //
    //LOG(DEBUG) << "Position:"  << position;


    if(!collected_startpos){
        previous_position = position;
        prev_dist_center = position - ir_center; 
        collected_startpos = true; 
        dist_center = position - ir_center;
        delta_dist_center = dist_center - prev_dist_center;
        for(int i=0;i<4;i++)
        {
            prev_delta[i] = delta_dist_center;
        }
        return;
    }

    if(position >= 7000 || position == 0)
    {
        if(stopcount>20){
            LOG(DEBUG) << "Stopped" << endl;
            //_driver->stop();
            previous_position = position;
            return;
        }
        stopcount++;
        return;
    }

    stopcount = 0;

    if(position == previous_position){
        LOG(DEBUG) << "Position = previous_position" << endl;
        return;
    }

    uint32_t distance = _driver->getDistance();

    if(false && distance < 1){
        LOG(DEBUG) << "Distance < 1" <<endl;
        previous_position = position;
        return;
    }

    dist_center = position - ir_center;

    //out_pid_weight = 1.0;
    //out_pid_weight = abs(dist_center)/3500.0 * 0.7 + 0.3;
    //myPID->weight = abs(dist_center)/3500.0;
    //myPID->weight = abs(dist_center)/3500.0;

    delta_dist_center = dist_center - prev_dist_center;

    /*for(int i=3;i>0;i--)
    {
        prev_delta[i] = prev_delta[i-1];
    }

    prev_delta[0] = delta_dist_center;

    //delta_dist_center = (prev_delta[0] + prev_delta[1] + prev_delta[2] + prev_delta[3])/4; //GetMedian(prev_delta);
    delta_dist_center = GetMedian(prev_delta,4);*/

    LOG(DEBUG) << "Position: " << position << endl;
    LOG(DEBUG) << "Dist center: " << dist_center << endl;
    LOG(DEBUG) << "Dist center: " << round(dist_center*1e-3) << endl;
    LOG(DEBUG) << "Delta dist: " << delta_dist_center << endl;

    //if(abs(dist_center) < 1000 && abs(delta_dist_center) < 500){return;}
    //if(abs(dist_center) < 1000){return;}

    /*for (unsigned char i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();*/

    //double angle = atan2((position-previous_position),distance);
    //diff = position>=previous_position?position-previous_position:previous_position-position;

    //double angle = atan2((float)distance, (float)dist_center*ir_modifier);

    if(debug && dtest%1 == 0){

              //  Serial.println("previous_position");
              //Serial.println(previous_position);
              /*Serial.println("dist_center");
              Serial.println(dist_center);
              Serial.println("delta_dist_center");
              Serial.println(delta_dist_center);*/
    }

    out_pid_Input = dist_center;


    //  double gap =abs(pid_SetPoint-pid_Input);

    /*  if(gap<2500)
        {
        myPID.SetTunings(consKp,consKi,consKd);
        }
        else
        {
        myPID.SetTunings(aggKp,aggKi,aggKd);
        }*/

    outerPID->Compute();
    LOG(DEBUG) << "Outer pid: "<< out_pid_Output << endl;
    do_turn(out_pid_Output);

    /*inn_pid_SetPoint = out_pid_Output;
    inn_pid_Input = delta_dist_center;

    if(innerPID->Compute())
    {
        LOG(DEBUG) << "inner pid: " << inn_pid_Output << endl;
        do_turn(inn_pid_Output);
        //if(debug){Serial.println("PID output: " + String(pid_Output));}
    }*/

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
                double dTemp = dpSorted[j];
                dpSorted[j] = dpSorted[j+1];
                dpSorted[j+1] = dTemp;
            }
        }
    }

    // Middle or average of middle values in the sorted array.
    double dMedian = 0.0;
    if ((size % 2) == 0) {
        dMedian = (dpSorted[size/2] + dpSorted[(size/2) - 1])/2.0;
    } else {
        dMedian = dpSorted[size/2];
    }
    delete [] dpSorted;
    return dMedian;
}
