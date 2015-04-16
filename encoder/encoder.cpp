#include "encoder.h"

void Encoder::setup(unsigned char pinA, unsigned char pinB) {

    _pinA = pinA;
    _pinB = pinB;

    gpio_export(_pinA);
    gpio_set_dir(_pinA, INPUT_PIN);
    gpio_get_value(_pinA, &encAoutprev);

    gpio_export(_pinB);
    gpio_set_dir(_pinB, INPUT_PIN);
    gpio_get_value(_pinB, &encBoutprev);

    prevTime = micros();
}

void Encoder::update() {

    if(gpio_get_value(_pinA, &encAout) < 0){return;}
    if(gpio_get_value(_pinB, &encBout) < 0){return;}

    if((encAout == encAoutprev && encBout != encBoutprev && encBout == encAout)
            || (encAoutprev == encBoutprev && encAout != encAoutprev))
    {
        direction = 1;
        fDist++;
//        LOG(DEBUG) << (int)_pinA << " Forward";
    }
    else if((encBout == encBoutprev && encAout != encAoutprev && encAout == encBout)
            || (encBoutprev == encAoutprev && encBout != encBoutprev))
    {
        direction = -1;
        bDist++;
        //Serial.println("Backward");
 //       LOG(DEBUG) << (int)_pinA << " Backward";
    }
    else
    {
        if(!(encAout == encAoutprev && encBout == encBoutprev))
        {
            LOG(DEBUG) << (int)_pinA << " WTF unknown encoder value, you're to slow?";
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
    prevprevspeed = prevspeed;
    prevspeed = speed;
    speed = encoder_tick_distance/(micros()-prevTime);
    prevTime = micros();

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


//m/s
float  Encoder::getSpeed()
     {
         return middle_of_3(speed,prevspeed,prevprevspeed);
     }

//Returns distance in micrometer
uint32_t Encoder::getDistance()
{
    uint32_t val = 0;
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
    prevTime = micros();
    fDist = 0;
    bDist = 0;
}

Encoder::~Encoder()
{
    gpio_unexport(_pinA);
    gpio_unexport(_pinB);
}

