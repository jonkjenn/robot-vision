//Header guard
#ifndef PLAYER_H
#define PLAYER_H

#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <string>
#include <iostream>
#include "utility.hpp"
#include <vector>


class Frameplayer{
    private:
        int counter{0};
        bool show_video{false};
        bool is_enabled{false};
        std::vector<std::string> window_names;
        std::vector<cv::Size> window_sizes;
    public:
        void create_windows(const int count, const cv::Size &size);
        void show_frame(const cv::Mat &frame);
        void show_frame(const cv::gpu::GpuMat &frame);
        void loop();
        Frameplayer(bool show_video=false);
        Frameplayer(const bool show_video, const int count, const cv::Size &size);
        bool enabled();
        void enable();
        void disable();
};

//End of header guard
#endif
