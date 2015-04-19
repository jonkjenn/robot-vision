#ifndef UTILITY_GUARD
#define UTILITY_GUARD
#include <atomic>
#define LOG(DEBUG) cout << endl; cout
void delayMicroseconds(unsigned int ms);
unsigned long micros();
const float  PI=3.14159265358979f;
void atomic_add_float(std::atomic<float> &atom, float bar);

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
#endif
