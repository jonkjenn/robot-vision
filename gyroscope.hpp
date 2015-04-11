#include "serial/serial.h"
#include <memory>
#include "mavlink.h"

class gyroscope{
    public:
        void setup();
        void update();
        float get_rotation();
    private:
        float rotation = 0;
        std::unique_ptr<serial::Serial> mSerial;
};
