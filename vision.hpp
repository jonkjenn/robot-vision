#ifndef ROBOT_VISION
#define ROBOT_VISION

#include "easylogging++.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/gpu/gpu.hpp>
#include "frameplayer.hpp"
#include <string>
#include <utility.hpp>
#include <thread>
#include <mutex>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>

class Vision{
    private:
        unsigned long previous_micros = 0;
        std::mutex camera_mutex;
        cv::Mat frame;
        enum class Type{CAMERA, FILE};
        Type input_type;
        bool show_video, play, cuda, no_wait;
        int frame_count;
        int index = 0;
        std::unique_ptr<cv::VideoCapture> cap;
        cv::Size size;
        Frameplayer fp;
        void hough(cv::Mat &frame);
        void hough_gpu(cv::gpu::GpuMat &gpu_frame, cv::Mat &frame);
        bool configure_videocapture(cv::VideoCapture &cap, int &fps, int &frame_count);
        void configure_cuda();
        void draw_hough(cv::gpu::GpuMat &d_lines, cv::Mat &frame);
        void process_frame(cv::Mat &frame);
        void process_frame_cuda(cv::Mat &frame);
        void setup();
        void handle_keys();
        void capture_frames(cv::Mat &frame);
        void capture_frames_file(cv::Mat &frame);

        cv::Ptr<cv::gpu::FilterEngine_GPU> blur_filter;

        cv::gpu::GpuMat gpu_grayscale;
        cv::gpu::GpuMat gpu_border;
        cv::gpu::GpuMat gpu_blur;
        cv::gpu::GpuMat gpu_canny;
        cv::gpu::GpuMat gpu_subframe;
        cv::gpu::GpuMat gpu_frame;

        cv::gpu::GpuMat gpu_hough;
        cv::gpu::GpuMat d_lines;
        cv::gpu::HoughLinesBuf d_buf;

        cv::Rect sub_rect;

        const float hough_angles = (float)(CV_PI/360.0f);

    public:
        double fps;
        Vision(std::vector<std::string> &args);
        void previous_frame();
        void update();
};

#endif
