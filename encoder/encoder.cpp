#include "encoder.h"
using namespace std;

void Encoder::setup(unsigned char pinA, unsigned char pinB) {

    _pinA = pinA;
    _pinB = pinB;

    gpio_export(_pinA);
    gpio_set_dir(_pinA, INPUT_PIN);
    fd1 = gpio_start_read(_pinA);
    gpio_get_value(&encAoutprev, fd1);


    gpio_export(_pinB);
    gpio_set_dir(_pinB, INPUT_PIN);
    fd2 = gpio_start_read(_pinB);
    gpio_get_value(&encBoutprev, fd2);

    prevTime = nanos();
    //nano_time = nanos();
    start_time = prevTime;
}

uint64_t avg_dur  =0;
uint64_t enccount = 1;
uint64_t avg_count = 1;
void Encoder::update() {

    if(nanos() - prevTime < 100){return;}

    if(gpio_get_value(&encAout,fd1) < 0){return;}
    if(gpio_get_value(&encBout,fd2) < 0){return;}

    if((encAout == encAoutprev && encBout != encBoutprev && encBout == encAout)
            || (encAoutprev == encBoutprev && encAout != encAoutprev))
    {
        direction = 1;
        fDist++;
        updateSpeeds();
    }
    else if((encBout == encBoutprev && encAout != encAoutprev && encAout == encBout)
            || (encBoutprev == encAoutprev && encBout != encBoutprev))
    {
        direction = -1;
        bDist++;
        updateSpeeds();
        //Serial.println("Backward");
    }
    else
    {
        if(!(encAout == encAoutprev && encBout == encBoutprev))
        {
            //printf("duration: %" PRIu64 "\n", nanos() - prevTime);
            //LOG(DEBUG) << (int)_pinA << " WTF unknown encoder value, you're to slow?" <<endl;

        }
        //Serial.println("Not moving");
    }

    //if(encAout != encAoutprev){LOG(DEBUG) << (int)_pinA << " " << (int)encAout << "," << (int)encBout << endl;}
    //if(encBout != encBoutprev){LOG(DEBUG) << (int)_pinA << " " << (int)encAout << "," << (int)encBout << endl;}

    /*if(_pinA == 161)
    {
        printf("duration: %" PRIu64 "\n", nanos() - prevTime);
        printf("%d %d %d\n",_pinA, encAout,encBout);
    }*/

    encAoutprev = encAout;
    encBoutprev = encBout;

    /*
     *
     * uMeter / uSeconds = m/s
     *
     * */

    /*Serial.print(encAout);
      Serial.print(" ");
      Serial.print(encBout);
      Serial.println("");*/
    //Serial.print("12: " + String(d12) + " 13: " + String(d13) + "\n");

    /*    if(debug && counter%500==0)
          {  //Serial.print("Direction : " + String(dir) + " Val: " +String((res[0] <<2)|res[1]) + " A: " + String(encAout) + " B: " + String(encBout)+ "\n");
          Serial.print("Distance forward: " + String(fDist/64.0/18.75 * PI * 0.124) + " backward: " + String(bDist/64.0/18.75 * PI * 0.124) + "\n");
          }*/
}

void Encoder::updateSpeeds()
{
    time = nanos();
    //prevprevspeed = prevspeed;
    //prevspeed = speed;
    if(time == prevTime){return;}

    int sumspeed = 0;

    for(int i=9;i>0;i--)
    {
        speeds[i-1] = speeds[i];
        sumspeed+= speeds[i];
    }
    speeds[9] = encoder_tick_distance/(time-prevTime) * 1000 * 1000;
    sumspeed+= speeds[9];

    speed.store(sumspeed/10);
    //cout << "last speed: " << speeds[9] << endl;
    //cout << "avg speed: " << sumspeed/10 << endl;

    if(fDist>=bDist)
    {
        //val = fDist/64.0/18.75 * PI * WHEEL_SIZE;
        distance.store(fDist*encoder_tick_distance);
    }
    else
    {
        //val = -bDist/64.0/18.75 * PI * WHEEL_SIZE;
        distance.store(bDist*encoder_tick_distance);
    }
    prevTime = time;
}

//m/s
int Encoder::getSpeed()
{
    //lock_guard<mutex> lock(encodermutex);
    return speed.load();//middle_of_3<atomic<int>>(speed,prevspeed,prevprevspeed);
}

//Returns distance in micrometer
uint64_t Encoder::getDistance()
{
    //lock_guard<mutex> lock(encodermutex);
    return distance.load();
}

void Encoder::reset()
{
    //lock_guard<mutex> lock(encodermutex);
    prevTime = micros();
    fDist = 0;
    bDist = 0;
}

Encoder::~Encoder()
{
    gpio_stop_read(fd1);
    gpio_stop_read(fd2);
    gpio_unexport(_pinA);
    gpio_unexport(_pinB);
}

