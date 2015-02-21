#include <player.h>
#include <line.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/gpu/gpu.hpp>
#include "QTRSensors-JetsonTk1/QTRSensors.h"
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

using namespace cv;
using namespace cv::gpu;
using namespace std;

bool show_debug = false;

void configure_pins()
{
    vector<unsigned char> pins = {160, 161, 162, 163};
    QTRSensorsRC q(pins, 164);
    LOG(INFO) << "Calibrating";
    q.calibrate();
    LOG(INFO) << "Calibrated done";
}

int main(int argc, char** argv)
{
    bool show_video = false;
    bool play = false;

    vector<string> args(argv, argv+argc);
    for(string s:args)
    {
        if(s.compare("--video") == 0)
        {
            show_video = true;
            continue;
        }

        if(s.compare("--debug") == 0)
        {
            show_debug = true;
            continue;
        }

        if(s.compare("--play") == 0)
        {
            play = true;
            continue;
        }
    }

    setup(show_video,play, show_debug);

    return 0;
}

bool configure_cuda()
{
    bool cuda = gpu::getCudaEnabledDeviceCount()>0;
    LOG(INFO) <<"Setting CUDA device";
    if(cuda){gpu::setDevice(0);}
    LOG(DEBUG) << "Cuda? " << cuda;
    return cuda;
}

bool configure_videocapture(VideoCapture &cap, int &fps, int &frame_count)
{
    LOG(DEBUG) << "Loaded video";

    if(!cap.isOpened())
    {
        LOG(ERROR) << "Could not open video/camera";
        return false;
    }

    LOG(INFO) << "FPS: " << cap.get(CV_CAP_PROP_FPS);

    frame_count = cap.get(CV_CAP_PROP_FRAME_COUNT);

    LOG(DEBUG) << "Frame count " << frame_count;

    if(frame_count <0){frame_count = 1000000;}

    return true;
 }

void configure_logger(const bool show_debug)
{
    el::Configurations c;
    c.setToDefault();
    c.parseFromText("*GLOBAL:\n ENABLED = false"); //Disable logging for the default logger setting

    if(show_debug){
        el::Configurations conf("log.conf");
        el::Loggers::reconfigureAllLoggers(conf);
    }else{ 
        el::Loggers::reconfigureAllLoggers(c); //Configure the silent default logger
    }

    LOG(INFO) << "Configured logger";
}

void setup(const bool show_video, const bool play, const bool show_debug)
{
    configure_logger(show_debug);

    configure_pins();

    bool cuda = configure_cuda();

    VideoWriter outputVideo;

    bool write = false;

    //VideoCapture cap("../out.mp4");
    
    int fps,frame_count;
    VideoCapture cap(0);
    configure_videocapture(cap,fps,frame_count);

    if(write){
        int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
    }


    /*if(write){
        outputVideo.open("../out.mp4",CV_FOURCC('m','4','s','2'),30,s,true);
    }*/

    Size size = Size(320,240);
    Frameplayer fp{show_video, 9, size};

    main_loop(cap,size, fp, frame_count, cuda, play);
}

void main_loop(VideoCapture &cap, const Size &size, Frameplayer &fp, const int frame_count, const bool cuda, bool play)
{
    Mat frame;
    for(int i=0;i<frame_count;i++)
    {
        auto loop_time = micros();
        LOG(INFO) << "Loading frame";
        do{
            cap >> frame;
        }while(frame.empty());
        LOG(INFO) << "Loaded frame";

        resize(frame,frame, size);
        LOG(INFO) << "Resized frame";

        if(cuda)
        {
            process_frame_cuda(frame, fp);
        }
        else
        {
            process_frame(frame, fp);
        }

        fp.loop();
        handle_keys(cap, play, i);

        auto dur = micros()-loop_time;
        LOG(DEBUG) << "Loop duration: " << dur << " fps: " << (float)1000000/dur;
        printf("fps: %f\n", (float)1000000/dur);
    }
}

void process_frame_cuda(Mat &frame, Frameplayer &fp)
{
    LOG(DEBUG) << "Processing frame on GPU";
    auto loop_time = micros();

    gpu::GpuMat gpu_frame;
    gpu_frame.upload(frame);//Uploads the frame to the GPU
    LOG(DEBUG) << "Cuda frame uploaded ";

    fp.show_frame(gpu_frame);

    hough_gpu(gpu_frame, frame, fp);
}

void process_frame(Mat &frame, Frameplayer &fp)
{
    LOG(DEBUG) << "Processing frame on CPU";

    LOG(DEBUG) << "Doing hough++: ";

    hough(frame, fp);

    LOG(DEBUG) << "Hough++ complete";

}

void handle_keys(VideoCapture &cap, bool &play, int &index)
{
    double fps = cap.get(CV_CAP_PROP_FPS);
    auto key = 0;
    if(play && fps>0)
    {
        LOG(DEBUG) << "Waitkey with FPS";
        key = waitKey(1000/fps) & 255;
    }
    else if(play)
    {
        LOG(DEBUG) << "waitKey(1)";
        waitKey(1);
    }
    else
    {
        LOG(INFO) << "Waiting for key";
        key = waitKey(0) & 255;
    }

    LOG(DEBUG) <<"Key:"<< key;
    switch(key)
    {
        case 97://a - play previous frame
            index-=2;
            if(index<=0){index=0;}
            cap.set(CV_CAP_PROP_POS_FRAMES,index);
            break;
        case 32://space - pause/play
            play = !play;
            return;
        default:
            return;
    }
}

void hough(Mat &frame, Frameplayer &fp)
{
    int subframe_y = frame.rows-40;

    fp.show_frame(frame);

    cvtColor(frame,frame,CV_BGR2GRAY);
    fp.show_frame(frame);

    blur(frame, frame, Size(4,4));
    fp.show_frame(frame);

    Mat subframe(frame,Rect(0,frame.rows-40,frame.cols,40));
    Canny(subframe, subframe, 50,100, 3);
    fp.show_frame(frame);

    vector<Vec4i> lines;
    HoughLinesP(subframe, lines, 1, CV_PI/720, 10,10,10);

    if(fp.enabled()){
        cvtColor(frame, frame, CV_GRAY2BGR);
        for(size_t j=0;j<lines.size();j++)
        {
            line(frame, Point(lines[j][0], subframe_y + lines[j][1]), Point(lines[j][2], subframe_y + lines[j][3]), Scalar(0,0,255), 3, 8);
        }
        fp.show_frame(frame);
    }
}


void hough_gpu(gpu::GpuMat &gpu_frame, Mat &frame, Frameplayer &fp)
{
    int subframe_y = frame.rows-40;

    LOG(DEBUG) << "Grayscale start";
    
    gpu::GpuMat gpu_frame2(gpu_frame.rows, gpu_frame.cols, gpu_frame.type());
    gpu::cvtColor(gpu_frame,gpu_frame2,CV_BGR2GRAY);
    LOG(DEBUG) << "Grayscale stop";

    fp.show_frame(gpu_frame2);
    
    LOG(DEBUG) << "Blur start";

    //Create border for using image with blur, dont know why need 2 pixels instead of 1
    gpu::GpuMat gpu_frame3(gpu_frame2.rows +4 , gpu_frame2.cols + 4, gpu_frame2.type());
    gpu::copyMakeBorder(gpu_frame2, gpu_frame3, 2, 2 , 2, 2, BORDER_REPLICATE);

    gpu::blur(gpu_frame3, gpu_frame, Size(3,3));

    gpu::GpuMat roi(gpu_frame, Rect(2, 2, gpu_frame.cols-4, gpu_frame.rows-4));

    LOG(DEBUG) << "Blur stop";

    fp.show_frame(roi);

    //gpu::GpuMat subframe(gpu_frame,Rect(0,frame.rows-40,frame.cols,40));
    //gpu::GpuMat subframe2(subframe.rows, subframe.cols, subframe.type());
    //gpu::Canny(subframe, subframe2, 50,100, 3);

    LOG(DEBUG) << "Canny start";
    gpu::Canny(roi, gpu_frame2, 50,100, 3);
    LOG(DEBUG) << "Canny stop";

    fp.show_frame(gpu_frame2);

    gpu::GpuMat d_lines;
    gpu::HoughLinesBuf d_buf;

    LOG(DEBUG) << "Hough start";
    gpu::HoughLinesP(gpu_frame2, d_lines,d_buf, 1.0f, (float)(CV_PI/360.0f),10,5);
    LOG(DEBUG) << "Hough stop";

    if(fp.enabled()){
        draw_hough(d_lines, frame, fp);
    }
}

void draw_hough(GpuMat &d_lines, Mat &frame, Frameplayer &fp)
{
    vector<Vec4i> lines;
    vector<Vec4i> lines_gpu;

    lines_gpu.resize(d_lines.cols);
    Mat h_lines(1, d_lines.cols, CV_32SC4, &lines_gpu[0]);
    d_lines.download(h_lines);

    cvtColor(frame, frame, CV_GRAY2BGR);

    Vec4i l;
    for(auto j = 0;j<lines_gpu.size();j++)
    {
        l = lines_gpu[j];
        line(frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
    }
    fp.show_frame(frame);
}
