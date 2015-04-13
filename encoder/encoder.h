
#ifndef BACHELOR_ENCODER
#define BACHELOR_ENCODER
#include "easylogging++.h"
#include "utility.hpp"
#include <cstdint>
#include "SimpleGPIO.h"


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
        const float distance_modifier  = (PI*WHEEL_SIZE)/(64.0*18.75)*1000.0*1000.0;//uMeter

        uint32_t prevTime;
        uint32_t time;

        float speed;

        float updateSpeed();

    public:
        ~Encoder();
        void setup(unsigned char pinA, unsigned char pinB);
        void update();
        uint32_t getDistance();
        float getSpeed();
        void reset();
};

#endif
