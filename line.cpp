#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv)
{
    VideoWriter outputVideo;
    Size s = Size(640,480);

    bool write = false;
    VideoCapture cap("../out.mp4");
    //VideoCapture cap("../out.mp4");
    if(!cap.isOpened())
    {
        printf("Could not open");
        return -1;
    }

    cap.set(CV_CAP_PROP_FPS,29.8);

    printf("FPS: %f\n", cap.get(CV_CAP_PROP_FPS));

    int frame_count = cap.get(CV_CAP_PROP_FRAME_COUNT);

    if(write){

        int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
    }

    namedWindow("Test");
    moveWindow("Test",0,0);
    namedWindow("Test2");
    moveWindow("Test2",640,0);
    namedWindow("Test3");
    moveWindow("Test3",0,480);

    //imshow("Test", image);
    Mat frame;

    if(write){
        outputVideo.open("../out.mp4",CV_FOURCC('m','4','s','2'),30,s,true);
    }

    for(int i=0;i<frame_count;i++)
    {
        cap >> frame;
        //resize(frame,frame,Size(640,480));
        imshow("Test",frame);
        imshow("Test2",frame);
        imshow("Test3",frame);
        //printf("i: %d\n", i);
        if(write){
            outputVideo << frame;
        }
        waitKey(1);

    }

    return 0;
}
