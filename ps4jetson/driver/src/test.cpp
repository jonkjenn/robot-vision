#include "ps4eye.h"
#include <iostream>
#include <ctime>
#include <thread>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "unistd.h"
#include <chrono>
#include <signal.h>
#include <mutex>

using namespace std;
using namespace std::chrono;
using namespace cv;

bool update();
void setup();
void updateThread();

mutex update_mutex;

// mesure cam fps
thread mThread;
uint32_t mCamFrameCount;
float mCamFps;
uint32_t mCamFpsLastSampleFrame;
double mCamFpsLastSampleTime;
int mDepthThreshold1;
int mDepthThreshold2;
uint8_t *frame_rgb_left;
uint8_t *frame_rgb_right;
unsigned long previous_micros = 0;

bool mShouldQuit = false;

using namespace ps4eye;
PS4EYECam::PS4EYERef eye;

unsigned long micros()
{
    auto dur =  high_resolution_clock::now().time_since_epoch();
    return duration_cast<microseconds>(dur).count();
}

void my_function(int sig){ // can be called asynchronously
    eye->shutdown();
    exit(0);
}

int main(int argc, char *argv[])
{
   signal (SIGINT,my_function);
   setup();
}


void setup()
{

    mShouldQuit = false;

    // list out the devices
    std::vector<PS4EYECam::PS4EYERef> devices( PS4EYECam::getDevices() );
    // console() << "found " << devices.size() << " cameras" << std::endl;

    /*mTimer = Timer(true);
    mCamFrameCount = 0;
    mCamFps = 0;
    mCamFpsLastSampleFrame = 0;
    mCamFpsLastSampleTime = 0;*/


    float gh = 15;
    float slw = 340 - 20;
   // setFrameRate(300.0f);
    if(devices.size())
    {
        namedWindow("window", WINDOW_AUTOSIZE);
        namedWindow("window2", WINDOW_AUTOSIZE);
        moveWindow("window", 670, 0);
        //namedWindow("window2", WINDOW_AUTOSIZE);
        //namedWindow("window3", WINDOW_AUTOSIZE);

        eye =  devices.at(0);
        //check or load firmware to PlayStation Camera and exit
        eye->firmware_path="firmware.bin";
        eye->firmware_upload();
        //eye->start_sensors_streaming();
        //mode 0: 60,30,15,8 fps 1280x800
        //mode 1: 120,60,30,15,8 fps 640x400
        //mode 2: 240,120,60,30 fps 320x192
        debug("Before init\n");
        if(!(eye->init(1, 60)))
        {
            cout << "init failed" << std::endl;
            //don't use when you are debugging is fine.
            //eye->stop();
           // exit(0);

        }
        
        //   console() << "init eye result " << res << std::endl;
        eye->start();
        //test values use it in depth mode
        mDepthThreshold1=35000;
        mDepthThreshold2=36000;

        //frame_rgb_left = new uint8_t[eye->getWidth()*eye->getHeight()*3];
        //frame_rgb_right = new uint8_t[eye->getWidth()*eye->getHeight()*3];

        //mFrameLeft = Surface(frame_rgb_left, eye->getWidth(), eye->getHeight(), eye->getWidth()*3,SurfaceChannelOrder::RGB);
        //memset(frame_rgb_left, 0, eye->getWidth()*eye->getHeight()*3);

        //mFrameRight = Surface(frame_rgb_right, eye->getWidth(), eye->getHeight(), eye->getWidth()*3,SurfaceChannelOrder::RGB);
        //memset(frame_rgb_right, 0, eye->getWidth()*eye->getHeight()*3);
        // create and launch the thread
        mThread = thread( updateThread  );

        bool isUpdate = false;

        while(true)
        {
            isUpdate = update();
            waitKey(1);
            if(isUpdate)
            {
                auto dur = micros()-previous_micros;
                cout << "Loop duration: " << dur << " fps: " << (float)1000000/dur;
                printf("fps: %f\n", (float)1000000/dur);
                previous_micros = micros();
            }
        }

    }
    else
    {
        cout << "There is not a PlayStation Camera device on this System" << std::endl;
        exit(0);
    }

}

void updateThread()
{
    while( true )
    {
        lock_guard<mutex> lock(update_mutex);
        bool res = ps4eye::PS4EYECam::updateDevices();
        if(!res) break;
    }
}

bool update()
{
    eyeframe *frame;
    bool isNewFrame = false;

    if(eye)
    {
        {
            lock_guard<mutex> lock(update_mutex);
            isNewFrame = eye->isNewFrame();
        }

        if(isNewFrame)
        {
            lock_guard<mutex> lock(update_mutex);
            eye->check_ff71();
            frame=eye->getLastVideoFramePointer();
            
            debug("Rightframe %d\n", eye->rightflag);
            if(eye->rightflag)
            {
                //convert_depth_to_RGBA(eye->getLastDepthFramePointer(),frame_bgr,eye->getWidth(),eye->getHeight(),mDepthThreshold1,mDepthThreshold2);
               // convert_YUYV_to_RGB24(eye->getLastDepthFramePointer(),frame_bgra,eye->getWidth(), eye->getHeight());
                

          //      convert_opencv_to_RGBA((uint8 *)eye->getLastDepthFramePointer(), frame_bgra, eye->getWidth(), eye->getHeight());

            //    convert_opencv_to_RGB((uint8 *)eye->getLastVideoRightFramePointer(), frame_bgr, eye->getWidth(), eye->getHeight());

                //convert_opencv_to_RGB(frame->videoRightFrame, frame_rgb_right, eye->getWidth(), eye->getHeight());
                
                
                Mat f(eye->getHeight(), eye->getWidth(),CV_8UC2, frame->videoRightFrame);
                Mat g(eye->getHeight(), eye->getWidth(),CV_8UC2, frame->videoLeftFrame);

                imshow("window",f);
                imshow("window2",g);

               //mTextureRight = gl::Texture( mFrameRight );

               // depthtexture = gl::Texture( mFrame );



            }
            else
            {
        //        convert_opencv_to_RGBA((uint8 *)eye->getLastVideoFramePointer(), frame_bgra, eye->getWidth(), eye->getHeight());
            //    convert_opencv_to_RGB((uint8 *)eye->getLastVideoLeftFramePointer(), frame_bgr, eye->getWidth(), eye->getHeight());
                //convert_opencv_to_RGB(frame->videoLeftFrame, frame_rgb_left, eye->getWidth(), eye->getHeight());
                


               // mTexture = gl::Texture( mFrame );
                //mTextureLeft = gl::Texture( mFrameLeft );
                debug("h: %d, w: %d\n", eye->getHeight(), eye->getWidth());

                //Mat f(eye->getHeight(), eye->getWidth(),CV_8UC2, frame->videoLeftFrame);
//                Mat o(eye->getHeight(), eye->getWidth(),CV_8UC3);

                Mat yuv(eye->getHeight(), eye->getWidth(),CV_8UC2, frame->videoRightFrame);
                Mat yuvl(eye->getHeight(), eye->getWidth(),CV_8UC2, frame->videoLeftFrame);
                //
 //               cvtColor(f,o,CV_YUV2BGR_YUYV);
 //
                Mat rgb(eye->getHeight(),eye->getWidth(),CV_8UC3);
                Mat rgbl(eye->getHeight(),eye->getWidth(),CV_8UC3);

                //Mat rgb2(eye->getHeight(),eye->getWidth(),CV_8UC3);
                //Mat rgb3(eye->getHeight(),eye->getWidth(),CV_8UC3);

                //cv::cvtColor(yuv, rgb, CV_YUV2BGR_YVYU);
                //cv::cvtColor(yuv, rgb, CV_YUV2BGR_YUY2);
                //cv::cvtColor(yuv, rgb, CV_BayerGR2RGB);
                //
                cv::cvtColor(yuv, rgb, 117);
                cv::cvtColor(yuvl, rgbl, 117);
                
                //yuyvToRgb(frame->videoLeftFrame,frame_rgb_left,eye->getWidth(),eye->getHeight());

                //Mat f(eye->getHeight(), eye->getWidth(),CV_8UC3,frame_rgb_left);

                imshow("window",rgb);
                imshow("window2",rgbl);
            }
        }
        //mCamFrameCount += isNewFrame ? 1 : 0;
        //double now = mTimer.getSeconds();
        /*if( now > mCamFpsLastSampleTime + 1 ) {
            uint32_t framesPassed = mCamFrameCount - mCamFpsLastSampleFrame;
            mCamFps = (float)(framesPassed / (now - mCamFpsLastSampleTime));

            mCamFpsLastSampleTime = now;
            mCamFpsLastSampleFrame = mCamFrameCount;
        }*/

        //gui->update();
        //eyeFpsLab->update_fps(mCamFps);
        // eyeFpsLab->update_fps(getAverageFps());
    }
    return isNewFrame;
}
