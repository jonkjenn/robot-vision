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
