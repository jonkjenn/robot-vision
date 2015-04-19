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


//Be careful using this, I belive this can loop indefinitely if multiple threads are WRITING to the variables
void atomic_add_float(atomic<float> &atom, float bar) {
      auto current = atom.load();
        while (!atom.compare_exchange_weak(current, current + bar))
                ;
}
