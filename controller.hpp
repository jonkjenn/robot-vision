#include "vision.hpp"
#include "easylogging++.h"
#include "utility.hpp"
#include "QTRSensors-JetsonTk1/QTRSensors.h"
#include <memory>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/utsname.h>
#include "arduinocomm.h"
#include "SimpleGPIO/SimpleGPIO.h"
#include "gyroscope.hpp"
#include "drive.h"
#include <functional>
#include <unistd.h>

#define MY_PRIORITY (49)
#define MAX_SAFE_STACK (8*1024)
#define NSEC_PER_SEC (1000000000)


class Controller{
    private:
        bool show_debug{false};
        void configure_pins();
        void configure_logger(const bool show_debug);
        void loop();
        std::unique_ptr<Vision> vision;
        std::shared_ptr<Arduinocomm> arduino;
        std::unique_ptr<gyroscope> gyro;
        void setup_vision(const bool show_video);
        void setup_vision(std::string &file, const bool show_video);
        void parsepacket();

        std::function<void()> callback;

        bool use_serial = true;
        bool waiting_ok = false;
        bool waiting_completed = false;
    public:
        void start();
        std::shared_ptr<Drive> driver;
        Controller(std::vector<std::string> &args, std::function<void()> callback);
};

