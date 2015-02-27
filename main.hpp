#include <vision.hpp>
#include "easylogging++.h"
#include <utility.hpp>
#include "QTRSensors.h"
#include <memory>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/utsname.h>

class Controller{
    private:
        bool show_debug{false};
        void configure_pins();
        void configure_logger(const bool show_debug);
        void loop();
        std::unique_ptr<Vision> vision;
        void setup_vision(const bool show_video);
        void setup_vision(std::string &file, const bool show_video);
    public:
        Controller(const bool show_debug, std::vector<std::string> &args);
};

