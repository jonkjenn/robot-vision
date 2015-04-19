#include <utility.hpp>
#include <chrono>
#include <thread>

using namespace std;
using namespace std::chrono;
unsigned long _micros = 0;

void delayMicroseconds(unsigned int ms)
{
    microseconds dura(ms);
    this_thread::sleep_for(dura);
}

void reset_micros()
{
    _micros = duration_cast<microseconds>(high_resolution_clock::now().time_since_epoch()).count();
}

unsigned long micros()
{
    return _micros;
}


//Be careful using this, I belive this can loop indefinitely if multiple threads are WRITING to the variables
void atomic_add_float(atomic<float> &atom, float bar) {
      auto current = atom.load();
        while (!atom.compare_exchange_weak(current, current + bar))
                ;
}
