
#ifndef BACHELOR_ENCODER
#define BACHELOR_ENCODER
#include "utility.hpp"
#include <cstdint>
#include "SimpleGPIO.h"
#include <iostream>
#include <atomic>
#include <cinttypes>


class Encoder{
    private:
        unsigned char _pinA, _pinB;
        int dir = 0;

        uint8_t encAout = 0;
        uint8_t encBout = 0;
        uint8_t encAoutprev = 0;
        uint8_t encBoutprev = 0;

        char direction = 0;

        uint32_t fDist = 0;
        uint32_t bDist = 0;

        std::atomic<uint64_t> distance;

        int counter = 0;

        void readData();

        const char debug = 1;

        //const float WHEEL_SIZE = 0.124;
        const float WHEEL_SIZE = 0.0982;
        const float encoder_tick_distance  = ((PI*WHEEL_SIZE)/(64.0*18.75))*1000.0*1000.0;//uMeter
        //const float encoder_tick_distance  = (PI*WHEEL_SIZE)/(64.0*18.75);//Meter

        uint64_t prevTime;
        uint64_t time;
        uint64_t nano_time; 
        uint64_t start_time;

        int speeds[10] = {0};

        std::atomic<int> speed;
        std::atomic<int> prevspeed;
        std::atomic<int> prevprevspeed;

        void updateSpeeds();

        int fd1 = 0;
        int fd2 = 0;

    public:
        ~Encoder();
        void setup(unsigned char pinA, unsigned char pinB);
        void update();
        uint64_t getDistance();
        int getSpeed();
        void reset();
};

#endif
