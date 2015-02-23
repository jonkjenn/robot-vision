#include <frameplayer.h>
#include <string>
#include <opencv2/gpu/gpu.hpp>

using namespace cv;
using namespace std;

Frameplayer::Frameplayer(bool show_video)
{
    is_enabled = show_video;
}

Frameplayer::Frameplayer(const bool show_video, const int count, const Size &size)
    :is_enabled{show_video}
{
    create_windows(count,size);
}

void Frameplayer::create_windows(const int count, const Size &size)
{
    if(!is_enabled){return;}
    int columns = (1600)/size.width;
    int row = 0;
    for(int i=0;i<count;i++)
    {
        if(i>0 && i%columns==0){row++;}
        namedWindow("nw" + to_string(i), WINDOW_AUTOSIZE);
        moveWindow("nw" + to_string(i), (i%columns)*size.width, row*(size.height+20)+20);
    }
}

void Frameplayer::show_frame(const Mat &frame)
{
    if(!is_enabled){return;}
    imshow("nw" + to_string(counter), frame);
    counter++;
}

void Frameplayer::show_frame(const cv::gpu::GpuMat &frame)
{
    if(!is_enabled){return;}
    cv::Mat cpu_frame;
    frame.download(cpu_frame);
    imshow("nw" + to_string(counter), cpu_frame);
    counter++;
}

void Frameplayer::loop()
{
    counter = 0;
}
bool Frameplayer::enabled()
{
    return is_enabled;
}

void Frameplayer::enable()
{
    is_enabled = true;
}

void Frameplayer::disable()
{
    is_enabled = false;
}
