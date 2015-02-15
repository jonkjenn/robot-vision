#include <stdio.h>
#include <player.h>
#include <line.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    VideoWriter outputVideo;
    Size s = Size(320,240);

    bool write = false;
    VideoCapture cap("../out.mp4");

    if(!cap.isOpened())
    {
        printf("Could not open");
        return -1;
    }

    printf("FPS: %f\n", cap.get(CV_CAP_PROP_FPS));

    double fps = cap.get(CV_CAP_PROP_FPS);

    int frame_count = cap.get(CV_CAP_PROP_FRAME_COUNT);

    if(write){

        int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
    }

    player::create_windows(6,s);

    Mat frame;

    if(write){
        outputVideo.open("../out.mp4",CV_FOURCC('m','4','s','2'),30,s,true);
    }


    bool play = false;

    for(int i=0;i<frame_count;i++)
    {
        cap >> frame;

        resize(frame,frame, s);

        hough(frame);

        if(write){
            outputVideo << frame;
        }
        player::loop();
        int key;

        if(play)
        {
            key = waitKey(1000/fps);
        }
        else
        {
            key = waitKey(0);
        }

        printf("Key: %d\n", key);
        switch(key)
        {
            case 97:
                i-=2;
                if(i<=0){i=0;}
                cap.set(CV_CAP_PROP_POS_FRAMES,i);
                break;
            case 32:
                play = !play;
        }
        printf("Count %d\n", i);
    }

    return 0;
}

vector<Vec4i> lines;

void hough(Mat &frame)
{
        Mat subframe(frame,Rect(0,240-40,320,40));

        cout << "sub";

        player::show_frame(frame);
        cvtColor(frame,frame,CV_BGR2GRAY);
        player::show_frame(frame);
        blur(frame, frame, Size(4,4));
        player::show_frame(frame);
        Canny(subframe, subframe, 50,200, 3);
        player::show_frame(subframe);
        HoughLinesP(subframe, lines, 1, CV_PI/720, 10,10,3);
        cvtColor(subframe, subframe, CV_GRAY2BGR);
        for(size_t j=0;j<lines.size();j++)
        {
            line(subframe, Point(lines[j][0], lines[j][1]), Point(lines[j][2], lines[j][3]), Scalar(0,0,255), 3, 8);
        }
        player::show_frame(subframe);
}
