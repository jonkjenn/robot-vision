#ifndef ROBOT_VISION
#define ROBOT_VISION

#include "easylogging++.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/gpu/gpu.hpp>
#include <frameplayer.h>
#include <string>
#include <utility.h>

class Vision{
    private:
        enum class Type{CAMERA, FILE};
        Type input_type;
        bool show_video, play, cuda;
        int frame_count, index;
        cv::VideoCapture cap;
        cv::Size size;
        Frameplayer fp;
        void hough(cv::Mat &frame, Frameplayer &fp);
        void hough_gpu(cv::gpu::GpuMat &gpu_frame, cv::Mat &frame, Frameplayer &fp);
        bool configure_videocapture(cv::VideoCapture &cap, int &fps, int &frame_count);
        bool configure_cuda();
        void draw_hough(cv::gpu::GpuMat &d_lines, cv::Mat &frame, Frameplayer &fp);
        void process_frame(cv::Mat &frame, Frameplayer &fp);
        void process_frame_cuda(cv::Mat &frame, Frameplayer &fp);
        void setup();

    public:
        double fps;
        Vision(int camera=0, bool show_video=false);
        Vision(std::string file, bool show_video);
        void previous_frame();
        void update();
};

#endif
