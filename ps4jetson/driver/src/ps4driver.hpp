#include "ps4eye.h"
#include <iostream>
#include <ctime>
#include <thread>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "unistd.h"
#include <chrono>
#include "utility.hpp"

class ps4driver{
    public:
        ps4driver();
        ~ps4driver();
        void update();
        cv::Mat getFrame();
    private:
        void setup();
};
