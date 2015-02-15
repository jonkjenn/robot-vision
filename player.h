#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

//Header guard
#ifndef PLAYER_H
#define PLAYER_H

namespace player{
void create_windows(int count, cv::Size size);
void show_frame(const cv::Mat &frame);
void show_frame(const cv::gpu::GpuMat &frame);
void loop();
}

extern bool show_video;

//End of header guard
#endif
