#include  "Arduino.h"

#ifndef BACHELOR_GYRO
#define BACHELOR_GYRO

class Gyro{
    public:
        void start(float degrees);
        void stop();
        void update(float, uint32_t);
        bool active = false;
        float total_rotation = 0;
        float  goal_rotation = 0;

    private:
        uint32_t prevtime = 0;
        uint32_t duration = 0;
        const float DEG_RAD_RATIO = 0.01745;
        const float RAD_DEG_RATIO = 57.2957;
};

#endif
