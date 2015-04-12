#include "encoder.h"

void Encoder::setup(unsigned char pinA, unsigned char pinB) {
    /*
    _pinA = pinA;
    _pinB = pinB;

    prevTime = micros();
    //pinMode(_pinA,INPUT);
    //pinMode(_pinB,INPUT);
    DDRC &= B11110000;
    encAoutprev = (PINC & (1<< _pinA)) >> _pinA;
    encBoutprev = (PINC & (1<< _pinB)) >> _pinB;
    */
}

void Encoder::update() {

    encAout = (PINC & (1<< _pinA)) >> _pinA;
    encBout = (PINC & (1<< _pinB)) >> _pinB;

    //encAout = digitalRead(_pinA);
    //encBout = digitalRead(_pinB);

    if(encAoutprev >= 0 && encBoutprev >= 0){
        if((encAout == encAoutprev && encBout != encBoutprev && encBout == encAout)
                || (encAoutprev == encBoutprev && encAout != encAoutprev))
        {
            direction = 1;
            fDist++;
            //Serial.println("Forward");
        }
        else if((encBout == encBoutprev && encAout != encAoutprev && encAout == encBout)
                || (encBoutprev == encAoutprev && encBout != encBoutprev))
        {
            direction = -1;
            bDist++;
            //Serial.println("Backward");
        }
        else
        {
            if(!(encAout == encAoutprev && encBout == encBoutprev))
            {
                Serial.println("WTF");
            }
            //Serial.println("Not moving");
        }
    }
    else
    {
    }

    encAoutprev = encAout;
    encBoutprev = encBout;

    counter++;

     /*
      *
      * uMeter / uSeconds = m/s
      *
      * */
    speed = (float)getDistance()/(micros()-prevTime);
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
         return speed;
     }

//Returns distance in micrometer
uint32_t Encoder::getDistance()
{
    uint32_t val = 0;
    if(fDist>=bDist)
    {
        //val = fDist/64.0/18.75 * PI * WHEEL_SIZE;
        val = fDist*distance_modifier;
    }
    else
    {
        //val = -bDist/64.0/18.75 * PI * WHEEL_SIZE;
        val = fDist*distance_modifier;
    }

    return val;
}

void Encoder::reset()
{
    Serial.println("Reset");
    prevTime = micros();
    fDist = 0;
    bDist = 0;
}
