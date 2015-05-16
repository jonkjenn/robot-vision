#include "ps4driver.hpp"
#include <csignal>

using namespace std;
using namespace std::chrono;
using namespace cv;

uint32_t mCamFrameCount;
float mCamFps;
uint32_t mCamFpsLastSampleFrame;
double mCamFpsLastSampleTime;
int mDepthThreshold1;
int mDepthThreshold2;
uint8_t *frame_rgb_left;
uint8_t *frame_rgb_right;
unsigned long previous_micros = 0;

Mat rgbr,rgbl;

using namespace ps4eye;
PS4EYECam::PS4EYERef eye;

ps4driver::ps4driver()
{
    setup();
}

void ps4driver::setup()
{
    // list out the devices
    std::vector<PS4EYECam::PS4EYERef> devices( PS4EYECam::getDevices() );

    if(devices.size())
    {
        eye =  devices.at(0);
        //
        //check or load firmware to PlayStation Camera and exit
        eye->firmware_path="firmware.bin";
        eye->firmware_upload();
        //
        //mode 0: 60,30,15,8 fps 1280x800
        //mode 1: 120,60,30,15,8 fps 640x400
        //mode 2: 240,120,60,30 fps 320x192
        if(!(eye->init(2, 240)))
        {
            LOG(DEBUG) << "PS4Camera init failed" << endl;
            //don't use when you are debugging is fine.
            //eye->stop();
           // exit(0);

        }
        
        //   console() << "init eye result " << res << std::endl;

        rgbr.create(eye->getHeight(),eye->getWidth(),CV_8UC3);
        rgbl.create(eye->getHeight(),eye->getWidth(),CV_8UC3);

        eye->start();
        //test values use it in depth mode
        //mDepthThreshold1=35000;
        //mDepthThreshold2=36000;
    }
    else
    {
        LOG(DEBUG) << "There is not a PlayStation Camera device on this System";
    }

}

void ps4driver::update()
{
    //cout << "update devices" << endl;
    bool res = ps4eye::PS4EYECam::updateDevices();
    //cout << "update devices done" << endl;
}

void ps4driver::stop()
{
    eye->shutdown();
}

Mat ps4driver::getFrame()
{
    //cout << "getframe" << endl;

    eyeframe *frame;
    bool isNewFrame = false;

    isNewFrame = eye->isNewFrame();

    if(!isNewFrame){return Mat{};}

    eye->check_ff71();
    frame=eye->getLastVideoFramePointer();
    
    Mat yuv(eye->getHeight(), eye->getWidth(),CV_8UC2, frame->videoRightFrame);
    //Mat yuvl(eye->getHeight(), eye->getWidth(),CV_8UC2, frame->videoLeftFrame);

    cv::cvtColor(yuv, rgbr, 117);
    //cv::cvtColor(yuvl, rgbl, 117);

    return rgbr;
}

ps4driver::~ps4driver(void)
{
}
