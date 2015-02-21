#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

//Header guard
#ifndef PLAYER_H
#define PLAYER_H

class Frameplayer{
    private:
        int counter{0};
        bool show_video{false};
        bool is_enabled{false};
    public:
        void create_windows(const int count, const cv::Size &size);
        void show_frame(const cv::Mat &frame);
        void show_frame(const cv::gpu::GpuMat &frame);
        void loop();
        Frameplayer(bool show_video);
        Frameplayer(const bool show_video, const int count, const cv::Size &size);
        bool enabled();
};

//End of header guard
#endif
