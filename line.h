#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

void hough(cv::Mat &frame, Frameplayer &fp);
void hough_gpu(cv::gpu::GpuMat &gpu_frame, cv::Mat &frame, Frameplayer &fp);
void configure_pins();
bool configure_cuda();
bool configure_videocapture(cv::VideoCapture &cap, int &fps, int &frame_count);
void configure_logger(const bool show_debug);
void setup(const bool show_video, const bool play, const bool show_debug);
void main_loop(cv::VideoCapture &cap, const cv::Size &size, Frameplayer &fp, const int frame_count, const bool cuda, bool play);
void draw_hough(cv::gpu::GpuMat &d_lines, cv::Mat &frame, Frameplayer &fp);
void handle_keys(cv::VideoCapture &cap, bool &play, int &index);
void process_frame(cv::Mat &frame, Frameplayer &fp);
void process_frame_cuda(cv::Mat &frame, Frameplayer &fp);
