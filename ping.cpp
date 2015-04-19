#include "ping.hpp"

using namespace std;

Ping::Ping(unsigned char PIN)
{
    _PIN = PIN;
    gpio_export(_PIN);
}

void Ping::update()
{
    switch(step)
    {
        case 0:
            //LOG(DEBUG) << "Step 0";
            if(prevTime != 0 || ( micros() - prevTime > 200))
            {
                //LOG(DEBUG) << "Step 0 if";
                /*gpio_set_dir(_PIN, INPUT_PIN);
                gpio_get_value(_PIN, &input);*/
                gpio_set_dir(_PIN, OUTPUT_PIN);
                gpio_set_value(_PIN, HIGH);
                prevTime = micros();
                step++;
            }
            break;
        case 1:
            //LOG(DEBUG) << "Step 1";
            if(micros()-prevTime > pulse_out_delay)
            {
                //LOG(DEBUG) << "Step 1 if";
                gpio_set_dir(_PIN, OUTPUT_PIN);
                gpio_set_value(_PIN, LOW);
                gpio_set_dir(_PIN, INPUT_PIN);

                step++;
            }
            break;
        case 2:
            //LOG(DEBUG) << "Step 2";
            gpio_set_dir(_PIN, INPUT_PIN);
            gpio_get_value(_PIN, &input);
            if(input)
            {
                //LOG(DEBUG) << "Step 2 if";
                prevTime = micros();
                step++;
            }
            else
            {
                //LOG(DEBUG) << "BUG";
                step = 0;
            }
            break;
        case 3:
            //LOG(DEBUG) << "Step 3 if";
            unsigned int val;
            gpio_set_dir(_PIN, INPUT_PIN);
            gpio_get_value(_PIN, &val);
            if(val != input)
            {
                //LOG(DEBUG) << "Step 3 if";
                //lock_guard<mutex> lock(pingmutex);
                prevprevduration = prevduration;
                prevduration = duration;
                duration = micros() - prevTime;                
                step = 0;
                prevTime = micros();
            }
            break;
    }

}

float Ping::get_distance()
{
    //lock_guard<mutex> lock(pingmutex);
    return (middle_of_3<uint64_t>(prevprevduration, prevduration, duration)/2000000.0)*speed_of_sound_25c_us;//Divide by 2000000 becauses its round-trip-time
}
