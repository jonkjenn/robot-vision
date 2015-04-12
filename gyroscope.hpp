#include "serial/serial.h"
#include <memory>
#include "mavlink.h"
#include "utility.hpp"

class gyroscope{
    public:
        gyroscope(const std::string device&, uint32_t speed);
        void start(float degrees);
        void update();

        float total_rotation = 0;
        float goal_rotation = 0;

        float get_rotation();
    private:
        uint32_t prevtime = 0;
        uint32_t duration = 0;
        const float DEG_RAD_RATIO = 0.01745;
        const float RAD_DEG_RATIO = 57.2957;

        std::unique_ptr<serial::Serial> mSerial;
};
