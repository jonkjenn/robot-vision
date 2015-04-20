#include "utility.hpp"
#include <chrono>
#include <thread>
#include <iostream>
using namespace std;
using namespace std::chrono;
unsigned long _micros = 0;
struct timespec tv_t;

void delayMicroseconds(unsigned int ms)
{
    microseconds dura(ms);
    this_thread::sleep_for(dura);
}

void reset_micros()
{
    _micros = duration_cast<microseconds>(high_resolution_clock::now().time_since_epoch()).count();
}

uint64_t nanos()
{
    clock_gettime(CLOCK_MONOTONIC, &tv_t);
    return tv_t.tv_sec*1e12+tv_t.tv_nsec;
    //return _micros;
}

uint64_t micros()
{
    return nanos()/1000;
}


//Be careful using this, I belive this can loop indefinitely if multiple threads are WRITING to the variables
/*void atomic_add_float(atomic<float> &atom, float bar) {
      auto current = atom.load();
        while (!atom.compare_exchange_weak(current, current + bar))
                ;
}*/
