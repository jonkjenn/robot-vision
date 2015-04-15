#include <utility.hpp>
#include <chrono>
#include <thread>

using namespace std;
using namespace std::chrono;

void delayMicroseconds(unsigned int ms)
{
    microseconds dura(ms);
    this_thread::sleep_for(dura);
}
unsigned long micros()
{
    auto dur =  high_resolution_clock::now().time_since_epoch();
    return duration_cast<microseconds>(dur).count();
}

float middle_of_3(float a, float b, float c)
{
    float middle;

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
