#ifndef GYROSCOPE_GUARD
#define GYROSCOPE_GUARD

#include "serial/serial.h"
#include <memory>
#include "mavlink.h"
#include "utility.hpp"
#include <cmath>
#include <mutex>
#include <iostream>

class gyroscope{
    public:
        ~gyroscope();
        gyroscope(const std::string& device, uint32_t speed);
        void start(float degrees);
        void update();

        float goal_rotation{0};

        float get_current_rotation();
        float get_total_rotation();
        float get_distance_rotation();

        std::mutex gyromutex;
    private:
        float current_rotation{0};
        float total_rotation{0};
        float distance_rotation{0};
        int packet_drops = 0;
        uint64_t prevtime = 0;
        uint32_t duration = 0;
        uint64_t gyro_dur;
        const float DEG_RAD_RATIO = 0.01745;
        const float RAD_DEG_RATIO = 57.2957;
        bool reset = false;
        std::vector<uint8_t> gyro_temp{};

        std::unique_ptr<serial::Serial> mSerial;
};
#endif
