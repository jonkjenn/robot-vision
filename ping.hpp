#ifndef BACHELOR_PING_GUARD
#define BACHELOR_PING_GUARD


#include "utility.hpp"
#include "SimpleGPIO.h"
#include <iostream>

class Ping{
    public:
        Ping(unsigned char PIN);
        ~Ping();
        void update();
        float get_distance();
    private:
        unsigned char _PIN = 0;
        unsigned char step = 0;
        uint64_t prevTime = 0;
        uint64_t prevprevduration = 0;
        uint64_t prevduration = 0;
        uint64_t duration = 0;
        const uint8_t pulse_out_delay = 1;
        const float speed_of_sound_25c_us = 346.3;
        uint8_t input = 0;
        int ping_gpio = 0;
        uint8_t val;

};
#endif
