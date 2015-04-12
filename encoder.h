#include "Arduino.h"

#ifndef BACHELOR_ENCODER
#define BACHELOR_ENCODER


class Encoder{
    private:
        unsigned char _pinA, _pinB;
        int dir = 0;

        uint8_t encAout = 0;
        uint8_t encBout = 0;
        uint8_t encAoutprev = -1;
        uint8_t encBoutprev = -1;
        uint8_t prevAout = 0;
        uint8_t prevBout = 0;

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
        void setup(unsigned char pinA, unsigned char pinB);
        void update();
        uint32_t getDistance();
        float getSpeed();
        void reset();
};

#endif
