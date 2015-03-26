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

class Controller{
    private:
        bool show_debug{false};
        void configure_pins();
        void configure_logger(const bool show_debug);
        void loop();
        std::unique_ptr<Vision> vision;
        std::unique_ptr<Arduinocomm> arduino;
        void setup_vision(const bool show_video);
        void setup_vision(std::string &file, const bool show_video);
        void parsepacket();
    public:
        Controller(const bool show_debug, std::vector<std::string> &args);
};

