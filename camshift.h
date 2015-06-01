#include <opencv2/opencv.hpp>
#include "utility.hpp"

class camshift{
    public:
        int update_camshift(cv::Mat frame);
        void setup_camshift(bool show_video,bool save_video);
    private:
        cv::VideoWriter writer;
        cv::VideoWriter writer2;
        unsigned int did_not_find_target();
        bool camshift_init = false;
        cv::Mat hist,hsv,mask,roi,hsv_roi,back_project,roi_hist_h,roi_hist_s,roi_hist_v, roi_hist;
        int cc,cr,cch,cw = 0;
        cv::Rect rect;
        cv::Rect s_rect;

        int hbins = 50;
        int sbins = 60;

        int reset_count = 0;

        uint64_t previous_frame_saved = 0;

        static const int num_channels = 2;
        int histSize [num_channels] = {hbins,sbins};
        /*float hranges[2] = {22,40};
        float sranges[2] = {150,256};*/
        float hranges[2] = {0,180};
        float sranges[2] = {0,256};
        const float* ranges[num_channels] = {hranges,sranges};
        int channels[num_channels] = {0,1};

        bool do_show_video = false, do_save_video = false;

        unsigned int previous_position = 0;

        unsigned int calculate_position(cv::Point2f, cv::Mat frame);
};
