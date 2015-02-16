#include <stdio.h>
#include <player.h>
#include <line.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/gpu/gpu.hpp>

using namespace cv;
using namespace cv::gpu;
using namespace std;

bool show_video = false;
bool show_debug = false;
bool cuda = false;

int main(int argc, char** argv)
{
    vector<string> args(argv, argv+argc);

    for(size_t i=0;i<args.size();i++)
    {
        if(args.at(i).compare("--video") == 0)
        {
            show_video = true;
            continue;
        }

        if(args.at(i).compare("--debug") == 0)
        {
            show_debug = true;
            continue;
        }
    }

    cuda = gpu::getCudaEnabledDeviceCount()>0;
    if(cuda){gpu::setDevice(0);}
    if(show_debug){cout << "Cuda? " << cuda << "\n";}

    if(show_debug)
    {
        cout << "Show video? " << show_video << "\n";
        cout << "Show debug? " << show_debug << "\n";
    }

    VideoWriter outputVideo;
    Size s = Size(320,240);

    bool write = false;

    double t = (double)getTickCount();
    VideoCapture cap("../out.mp4");
    t = ((double)getTickCount() - t)/getTickFrequency();

    if(show_debug)
    {
        cout << "Loaded video : " << t << "s\n";
    }

    if(!cap.isOpened())
    {
        printf("Could not open");
        return -1;
    }

    if(show_debug)
    {
        printf("FPS: %f\n", cap.get(CV_CAP_PROP_FPS));
    }

    gpu::GpuMat gpu_frame;
    gpu::GpuMat gpu_frame2;

    double fps = cap.get(CV_CAP_PROP_FPS);

    int frame_count = cap.get(CV_CAP_PROP_FRAME_COUNT);

    if(write){

        int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
    }

    if(show_video)
    {
        player::create_windows(9,s);
    }

    Mat frame;

    if(write){
        outputVideo.open("../out.mp4",CV_FOURCC('m','4','s','2'),30,s,true);
    }

    bool play = (show_debug?false:true);

    for(int i=0;i<frame_count;i++)
    {
        t = (double)getTickCount();
        cap >> frame;
        t = ((double)getTickCount() - t)/getTickFrequency();

        if(show_debug)
        {
            cout << "Loaded frame : " << t << "s\n";
        }

        if(cuda)
        {
            t = (double)getTickCount();
            gpu_frame.upload(frame);
            t = ((double)getTickCount() - t)/getTickFrequency();

            if(show_debug)
            {
                cout << "Cuda frame uploaded : " << t << "s\n";
            }
        }

        t = (double)getTickCount();
        if(cuda){
            //gpu::GpuMat gpu_frame2(s,gpu_frame.type());
            gpu::resize(gpu_frame,gpu_frame2, s);
            gpu_frame2.download(frame);
            player::show_frame(frame);
        }
        else
        {
            resize(frame,frame, s);
        }
        t = ((double)getTickCount() - t)/getTickFrequency();

        if(show_debug)
        {
            cout << "Resized frame : " << t << "s\n";
        }

        t = (double)getTickCount();
        if(cuda)
        {
            hough_gpu(gpu_frame2, frame);
        }
        else
        {
            hough(frame);
        }

        t = ((double)getTickCount() - t)/getTickFrequency();

        if(show_debug)
        {
            cout << "Houghfilter ++ : " << t << "s\n";
        }

        if(write){
            outputVideo << frame;
        }

        if(show_video)
        {
            player::loop();
        }

        int key;

        if(!show_video || play)
        {
            key = waitKey(1000/fps) & 255;
        }
        else
        {
            if(show_debug){cout << "Waiting for key\n";}
            key = waitKey(0) & 255;
        }

        if(show_video)
        {
            if(show_debug)
            {
                printf("Key: %d\n", key);
            }
            switch(key)
            {
                case 97://a - play previous frame
                    i-=2;
                    if(i<=0){i=0;}
                    cap.set(CV_CAP_PROP_POS_FRAMES,i);
                    break;
                case 32://space - pause/play
                    play = !play;
                    break;
                case 113://q - quit
                    return 0;
            }
        }
    }

    return 0;
}

vector<Vec4i> lines;
vector<Vec4i> lines_gpu;
gpu::GpuMat d_lines;
gpu::HoughLinesBuf d_buf;

void hough(Mat &frame)
{
    int subframe_y = frame.rows-40;

    player::show_frame(frame);

    cvtColor(frame,frame,CV_BGR2GRAY);
    if(show_video){player::show_frame(frame);}

    blur(frame, frame, Size(4,4));
    player::show_frame(frame);

    Mat subframe(frame,Rect(0,frame.rows-40,frame.cols,40));
    Canny(subframe, subframe, 50,100, 3);
    player::show_frame(frame);

    HoughLinesP(subframe, lines, 1, CV_PI/720, 10,10,10);
    if(show_video){
        cvtColor(frame, frame, CV_GRAY2BGR);
        for(size_t j=0;j<lines.size();j++)
        {
            line(frame, Point(lines[j][0], subframe_y + lines[j][1]), Point(lines[j][2], subframe_y + lines[j][3]), Scalar(0,0,255), 3, 8);
        }
        player::show_frame(frame);
    }
}


void hough_gpu(gpu::GpuMat &gpu_frame, Mat &frame)
{
    int subframe_y = frame.rows-40;

    if(show_debug){cout << "type: " << gpu_frame.type() << "\n";}
    
    gpu::GpuMat gpu_frame2(gpu_frame.rows, gpu_frame.cols, gpu_frame.type());
    gpu::cvtColor(gpu_frame,gpu_frame2,CV_BGR2GRAY);
    if(show_video)
    {
        gpu_frame2.download(frame);
        player::show_frame(frame);
    }
    
    if(show_debug){cout << "type: " << gpu_frame2.type() << "\n";}

    //Create border for using image with blur, dont know why need 2 pixels instead of 1
    gpu::GpuMat gpu_frame3(gpu_frame2.rows +4 , gpu_frame2.cols + 4, gpu_frame2.type());
    gpu::copyMakeBorder(gpu_frame2, gpu_frame3, 2, 2 , 2, 2, BORDER_REPLICATE);

    gpu::blur(gpu_frame3, gpu_frame, Size(3,3));

    gpu::GpuMat roi(gpu_frame, Rect(2, 2, gpu_frame.cols-4, gpu_frame.rows-4));

    if(show_debug)
    {
        cout << "gpu_frame2.size: " << gpu_frame2.size() << " gpu_frame.size: " << gpu_frame.size() << "\n";
        cout << "gpu_frame3.size: " << gpu_frame3.size() << " roi.size: " << roi.size() << "\n";
    }

    if(show_video)
    {
        roi.download(frame);
        player::show_frame(frame);
    }

    //gpu::GpuMat subframe(gpu_frame,Rect(0,frame.rows-40,frame.cols,40));
    //gpu::GpuMat subframe2(subframe.rows, subframe.cols, subframe.type());
    //gpu::Canny(subframe, subframe2, 50,100, 3);

    gpu::Canny(roi, gpu_frame2, 50,100, 3);

    if(show_video)
    {
        gpu_frame2.download(frame);
        player::show_frame(frame);
    }

    gpu::HoughLinesP(gpu_frame2, d_lines,d_buf, 1.0f, (float)(CV_PI/360.0f),10,5);
    if(show_video){
        lines_gpu.resize(d_lines.cols);
        Mat h_lines(1, d_lines.cols, CV_32SC4, &lines_gpu[0]);
        d_lines.download(h_lines);

        cvtColor(frame, frame, CV_GRAY2BGR);
        for(size_t j = 0;j<lines_gpu.size();j++)
        {
            Vec4i l = lines_gpu[j];
            line(frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
        }
        player::show_frame(frame);
    }
}
