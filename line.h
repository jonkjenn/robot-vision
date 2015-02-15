#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

void hough(cv::Mat &frame);
void hough_gpu(cv::gpu::GpuMat &gpu_frame, cv::Mat &frame);
