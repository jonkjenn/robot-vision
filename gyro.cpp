#include "gyro.h"

void Gyro::start(float degrees)
{
    active = true;
    goal_rotation = degrees * DEG_RAD_RATIO;
    total_rotation = 0;
}

void Gyro::stop()
{
    active = false;
}

void Gyro::update(float zgyro, uint32_t timer)
{
    if(prevtime == 0){prevtime = timer;return;}
    if(abs(zgyro)<= 0.01){return;}

    duration  = timer - prevtime;

    total_rotation += duration * 1e-6 *  zgyro;

    Serial.print("Total: ");
    Serial.println(total_rotation * RAD_DEG_RATIO);

    prevtime = timer;
}
