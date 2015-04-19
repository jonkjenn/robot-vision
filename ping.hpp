#ifndef BACHELOR_PING_GUARD
#define BACHELOR_PING_GUARD


#include "utility.hpp"
#include "SimpleGPIO.h"
#include <mutex>

class Ping{
    public:
        Ping(unsigned char PIN);
        void update();
        float get_distance();
    private:
        unsigned char _PIN = 0;
        unsigned char step = 0;
        uint64_t prevTime = 0;
        uint64_t prevprevduration = 0;
        uint64_t prevduration = 0;
        uint64_t duration = 0;
        const uint8_t pulse_out_delay = 10;
        const float speed_of_sound_25c_us = 346.3;
        unsigned int input = 0;
        std::mutex pingmutex;
};
#endif
