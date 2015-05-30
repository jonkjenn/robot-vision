#include "encoder.h"
using namespace std;

uint64_t avg_prev  =0;
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

    prevTime = micros();
    //nano_time = nanos();
    start_time = prevTime;
    avg_prev = micros();
    previous_distance = 0;
    previous_time = start_time;
}

uint64_t avg_dur  =0;
uint64_t enccount = 1;
uint64_t avg_count = 1;
void Encoder::update() {

    /*avg_count++;
    avg_dur += micros()-avg_prev;

    if(avg_count % 10000 == 0){
        //cout << "average duration: " << (float)avg_dur/(float)avg_count << endl;
    }*/

    //cout << micros() << endl;
    //if(nanos() - prevTime < 10000){return;}

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
            LOG(DEBUG) << (int)_pinA << " WTF unknown encoder value, you're to slow?" <<endl;

        }
        //Serial.println("Not moving");
    }

    //cout << (int)_pinA << " " << (int)_pinB  << " - " << (int)encAout <<","<< (int)encBout << endl;

    //if(encAout != encAoutprev){LOG(DEBUG) << micros() << " " << (int)_pinA << " " << (int)encAout << "," << (int)encBout << endl;}
    //if(encBout != encBoutprev){LOG(DEBUG) << micros() << " " << (int)_pinA << " " << (int)encAout << "," << (int)encBout << endl;}

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
    avg_prev = micros();
}

void Encoder::updateSpeeds()
{
    time = micros();
    //time = nanos();
    //prevprevspeed = prevspeed;
    //prevspeed = speed;
    //if(time == prevTime){return;}
    //

    uint64_t dist;

    if(fDist>=bDist)
    {
        //val = fDist/64.0/18.75 * PI * WHEEL_SIZE;
        dist = fDist*encoder_tick_distance;
        distance.store(dist);
    }
    else
    {
        //val = -bDist/64.0/18.75 * PI * WHEEL_SIZE;
        dist = bDist*encoder_tick_distance;
        distance.store(dist);
    }

    if(time - previous_time > 10000)
    {
        int s = (int)(((dist-previous_distance)/((double)time-previous_time))*1000);
        //cout << "dist traveld: " << (int)(dist - previous_distance) << endl;
        //cout << "time travel: " << (int) (time-previous_time) << endl;
        //cout << "speed : " << s << endl;
        speed.store(s);
        previous_time = time;
        previous_distance = dist;
    }

    //int sumspeed = 0;

    /*for(int i=9;i>0;i--)
    {
        speeds[i-1] = speeds[i];
        sumspeed+= speeds[i];
    }
    speeds[9] = encoder_tick_distance/(time-prevTime) * 1000 * 1000;
    sumspeed+= speeds[9];*/

    //speed.store(sumspeed/10);
    //cout << "last speed: " << speeds[9] << endl;
    //cout << "avg speed: " << sumspeed/10 << endl;

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

