#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

class Find_lines{
    public:
        void find(cv::Mat);
        Find_lines();
    private:
        void hough_gpu();
        void draw_hough();

        cv::Mat _frame;
        cv::Ptr<cv::gpu::FilterEngine_GPU> blur_filter;

        cv::gpu::GpuMat gpu_grayscale;
        //cv::gpu::GpuMat gpu_border;
        cv::gpu::GpuMat gpu_blur;
        cv::gpu::GpuMat gpu_canny;
        cv::gpu::GpuMat gpu_subframe;
        cv::gpu::GpuMat gpu_frame;

        cv::gpu::GpuMat gpu_hough;
        cv::gpu::GpuMat d_lines;
        cv::gpu::HoughLinesBuf d_buf;

        cv::Rect sub_rect;

        const float hough_angles = (float)(CV_PI/360.0f);
};
