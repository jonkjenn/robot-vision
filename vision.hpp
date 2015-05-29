#ifndef ROBOT_VISION
#define ROBOT_VISION


#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/gpu/gpu.hpp>
#include "frameplayer.hpp"
#include <string>
#include "utility.hpp"
#include <thread>
#include <mutex>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include "ps4driver.hpp"
#include <atomic>

class Vision{
    private:
        bool save_video = false;
        uint64_t previous_frame_saved = 0;
        cv::VideoWriter writer;
        std::thread capture_thread;
        std::atomic<bool> quit{false};
        unsigned long previous_micros = 0;
        std::mutex camera_mutex;
        cv::Mat frame;
        enum class Type{PS4, CAMERA, FILE};
        Type input_type;
        bool show_video = false, play = false, cuda = false, no_wait = false, camera = false, ps4 = false;
        int camera_id = 0;
        int frame_count;
        int frame_counter = 0;

        int index = 0;
        std::unique_ptr<cv::VideoCapture> cap;
        cv::Size size;
        Frameplayer fp;
        void hough(cv::Mat &frame);
        //void hough_gpu(cv::gpu::GpuMat &gpu_frame, cv::Mat &frame);
        bool configure_videocapture(cv::VideoCapture &cap, int &fps, int &frame_count);
        void configure_cuda();
        //void draw_hough(cv::gpu::GpuMat &d_lines, cv::Mat &frame);
        void process_frame(cv::Mat &frame);
        void process_frame_cuda(cv::Mat &frame);
        void setup();
        void handle_keys();
        void capture_frames(cv::Mat &frame);
        void capture_ps4(cv::Mat &frame);
        void capture_frames_file(cv::Mat &frame);

        std::unique_ptr<ps4driver> ps4cam;


        cv::Rect sub_rect;

        const float hough_angles = (float)(CV_PI/360.0f);

    public:
        double fps;
        Vision(std::vector<std::string> &args);
        void previous_frame();
        cv::Mat update();
        void stop();
};

#endif
