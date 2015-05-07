#ifndef BACHELOR_CONTROLLER
#define BACHELOR_CONTROLLER

#include "vision.hpp"
#include "utility.hpp"
#include <memory>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/utsname.h>
#include "arduinocomm.h"
#include "SimpleGPIO/SimpleGPIO.h"
#include "gyroscope.hpp"
#include "drive.h"
#include "bachelor-line.h"
#include <functional>
#include <unistd.h>

#define MY_PRIORITY (49)
#define MAX_SAFE_STACK (8*1024)
#define NSEC_PER_SEC (1000000000)


class Controller{
    private:
        bool show_debug{false};
        bool cam{true};
        void configure_pins();
        void configure_logger(const bool show_debug);
        void loop();
        std::unique_ptr<Vision> vision;
        Arduinocomm *arduino;
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
        std::shared_ptr<LineFollower<Drive>> line_follower;
        Controller(std::vector<std::string> &args, std::function<void()> callback);
        bool quit_robot = false;
};

#endif
