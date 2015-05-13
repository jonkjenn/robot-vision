#include "ping.hpp"

using namespace std;

Ping::Ping(unsigned char PIN)
{
    _PIN = PIN;
    gpio_export(_PIN);
    ping_gpio = gpio_start_readwrite(_PIN);
}

Ping::~Ping()
{
    gpio_stop_readwrite(ping_gpio);
}

void Ping::update()
{
    switch(step)
    {
        case 0:
            //LOG(DEBUG) << "Step 0";
            if(prevTime != 0 || ( micros() - prevTime > 200))
            {
                //LOG(DEBUG) << "Step 0 if" << endl;
                /*gpio_set_dir(_PIN, INPUT_PIN);
                gpio_get_value(_PIN, &input);*/
                gpio_set_dir(_PIN, OUTPUT_PIN);
                gpio_set_value(HIGH,ping_gpio);
                prevTime = micros();
                delayMicroseconds(15);
                step++;
            }
            //LOG(DEBUG) << "Step 1";
            //if(micros()-prevTime > pulse_out_delay)
            {
                //LOG(DEBUG) << "Step 1 if" << endl;
                gpio_set_dir(_PIN, OUTPUT_PIN);
                gpio_set_value(LOW,ping_gpio);
                gpio_set_dir(_PIN, INPUT_PIN);

                prevTime = micros();
                step++;
            }
            break;
        case 2:
            //LOG(DEBUG) << "Step 2";
            gpio_get_value(&input, ping_gpio);
            //LOG(DEBUG) << "Step 2, input: "<< (int)input <<endl;
            if(input > 0)
            {
                //LOG(DEBUG) << "Step 2 if, input: "<< (int)input <<endl;
                //LOG(DEBUG) << "Time : " << micros()-prevTime << endl;
                prevTime = micros();
                step++;
            }
            else
            {
                //gpio_stop_read(ping_gpio);
                //LOG(DEBUG) << "BUG"<<endl;
                step = 0;
            }
            break;
        case 3:
            gpio_get_value(&val, ping_gpio);
            //LOG(DEBUG) << "Step 3 val: "<< (int)val << endl;
            if(val != input)
            {
                //gpio_stop_read(ping_gpio);

                //LOG(DEBUG) << "Step 3 if" <<endl;
                //lock_guard<mutex> lock(pingmutex);
                prevprevduration = prevduration;
                prevduration = duration;
                duration = micros() - prevTime;                
                if(prevprevduration == 0 && prevduration == 0)
                {
                    prevprevduration = duration;
                    prevduration = duration;
                }
                step = 0;
                prevTime = micros();
            }
            break;
    }

}

float Ping::get_distance()
{
    //lock_guard<mutex> lock(pingmutex);
    //return (middle_of_3<uint64_t>(prevprevduration, prevduration, duration)/2000000.0)*speed_of_sound_25c_us;//Divide by 2000000 becauses its round-trip-time
    return ((prevprevduration + prevduration + duration)/(3*2000000.0))*speed_of_sound_25c_us;//Divide by 2000000 becauses its round-trip-time
}
