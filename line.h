#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <vision.h>

class Controller{
    private:
        Vision vision;
        bool show_debug{false};
        void configure_pins();
        void configure_logger(const bool show_debug);
        void handle_keys(bool &play, int &index);
        void loop();
    public:
        Controller(const bool show_video, const bool show_debug, const bool play);
};

