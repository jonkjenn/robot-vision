#ifndef UTILITY_GUARD
#define UTILITY_GUARD

#include "time.h"
#include <cstdint>
#define LOG(DEBUG) cout
void delayMicroseconds(unsigned int ms);
uint64_t micros();
uint64_t nanos();
const float  PI=3.14159265358979f;
//void atomic_add_float(std::atomic<float> &atom, float bar);
void reset_micros();

enum Rotation_Direction{ LEFT,RIGHT};
template<typename T>
T middle_of_3(T a, T b, T c)
{
    T middle;

    if ((a <= b) && (a <= c))
    {
        middle = (b <= c) ? b : c;
    }
    else if ((b <= a) && (b <= c))
    {
        middle = (a <= c) ? a : c;
    }
    else
    {
        middle = (a <= b) ? a : b;
    }
    return middle;
}

struct PID_config{
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float setpoint = 0;
    float maximum = 0;
    float minimum = 0;
};
#endif
