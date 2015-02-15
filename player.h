#include <opencv2/opencv.hpp>

//Header guard
#ifndef PLAYER_H
#define PLAYER_H

namespace player{
void create_windows(int count, cv::Size size);
void show_frame(const cv::Mat &frame);
void loop();
}

//End of header guard
#endif
