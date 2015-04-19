
#ifndef BACHELOR_ENCODER
#define BACHELOR_ENCODER
#include "utility.hpp"
#include <cstdint>
#include "SimpleGPIO.h"
#include <mutex>
#include <iostream>


class Encoder{
    private:
        unsigned char _pinA, _pinB;
        int dir = 0;

        unsigned int encAout = 0;
        unsigned int  encBout = 0;
        unsigned int encAoutprev = 0;
        unsigned int encBoutprev = 0;

        char direction = 0;

        uint32_t fDist = 0;
        uint32_t bDist = 0;

        int counter = 0;

        void readData();

        const char debug = 1;

        const float WHEEL_SIZE = 0.124;
        const float encoder_tick_distance  = ((PI*WHEEL_SIZE)/(64.0*18.75))*1000.0*1000.0;//uMeter
        //const float encoder_tick_distance  = (PI*WHEEL_SIZE)/(64.0*18.75);//Meter

        uint32_t prevTime;
        uint32_t time;

        float speed;
        float prevspeed;
        float prevprevspeed;

        void updateSpeeds();

        std::mutex encodermutex;

    public:
        ~Encoder();
        void setup(unsigned char pinA, unsigned char pinB);
        void update();
        uint64_t getDistance();
        float getSpeed();
        void reset();
};

#endif
