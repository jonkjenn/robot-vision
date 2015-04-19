#include "encoder.h"
using namespace std;

void Encoder::setup(unsigned char pinA, unsigned char pinB) {

    _pinA = pinA;
    _pinB = pinB;

    gpio_export(_pinA);
    gpio_set_dir(_pinA, INPUT_PIN);
    gpio_get_value(_pinA, &encAoutprev);

    fd1 = gpio_start_read(_pinA);

    gpio_export(_pinB);
    gpio_set_dir(_pinB, INPUT_PIN);
    gpio_get_value(_pinB, &encBoutprev);
    fd2 = gpio_start_read(_pinB);

    prevTime = micros();
}

void Encoder::update() {

    if(gpio_get_value(&encAout,fd1) < 0){return;}
    if(gpio_get_value(&encBout,fd2) < 0){return;}

//    lock_guard<mutex> lock(encodermutex);

    if((encAout == encAoutprev && encBout != encBoutprev && encBout == encAout)
            || (encAoutprev == encBoutprev && encAout != encAoutprev))
    {
        direction = 1;
        fDist++;
        updateSpeeds();
        //LOG(DEBUG) << (int)_pinA << " fDist: " << fDist;
               //LOG(DEBUG) << (int)_pinB << " Forward" <<endl;
    }
    else if((encBout == encBoutprev && encAout != encAoutprev && encAout == encBout)
            || (encBoutprev == encAoutprev && encBout != encBoutprev))
    {
        direction = -1;
        bDist++;
        updateSpeeds();
        //Serial.println("Backward");
               //LOG(DEBUG) << (int)_pinA << " Backward" << endl;
    }
    else
    {
        if(!(encAout == encAoutprev && encBout == encBoutprev))
        {
            LOG(DEBUG) << (int)_pinA << " WTF unknown encoder value, you're to slow?" <<endl;
        }
        //Serial.println("Not moving");
    }

    encAoutprev = encAout;
    encBoutprev = encBout;

    counter++;

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
    if(micros() - prevTime > 0){
        prevprevspeed = prevspeed;
        prevspeed = speed;
        speed = encoder_tick_distance/(micros()-prevTime);
        prevTime = micros();
    }
}

//m/s
float  Encoder::getSpeed()
{
    //lock_guard<mutex> lock(encodermutex);
    return middle_of_3(speed,prevspeed,prevprevspeed);
}

//Returns distance in micrometer
uint64_t Encoder::getDistance()
{
    //lock_guard<mutex> lock(encodermutex);
    uint64_t val = 0;
    if(fDist>=bDist)
    {
        //val = fDist/64.0/18.75 * PI * WHEEL_SIZE;
        val = fDist*encoder_tick_distance;
    }
    else
    {
        //val = -bDist/64.0/18.75 * PI * WHEEL_SIZE;
        val = bDist*encoder_tick_distance;
    }

    return val;
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

