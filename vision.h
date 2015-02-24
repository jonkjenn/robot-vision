#ifndef ROBOT_VISION
#define ROBOT_VISION

#include "easylogging++.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/gpu/gpu.hpp>
#include "frameplayer.h"
#include <string>
#include <utility.h>
#include <thread>
#include <mutex>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>

class Vision{
    private:
        unsigned long previous_micros;
        std::mutex camera_mutex;
        cv::Mat frame;
        enum class Type{CAMERA, FILE};
        Type input_type;
        bool show_video, play, cuda;
        int frame_count;
        int index = 0;
        std::unique_ptr<cv::VideoCapture> cap;
        cv::Size size;
        Frameplayer fp;
        void hough(cv::Mat &frame);
        void hough_gpu(cv::gpu::GpuMat &gpu_frame, cv::Mat &frame);
        bool configure_videocapture(cv::VideoCapture &cap, int &fps, int &frame_count);
        bool configure_cuda();
        void draw_hough(cv::gpu::GpuMat &d_lines, cv::Mat &frame);
        void process_frame(cv::Mat &frame);
        void process_frame_cuda(cv::Mat &frame);
        void setup();
        void handle_keys();
        void capture_frames(cv::Mat &frame);

    public:
        double fps;
        Vision(std::vector<std::string> &args);
        void previous_frame();
        void update();
};

#endif
