#include <stdio.h>
#include <player.h>
#include <line.h>

using namespace cv;
using namespace std;

bool show_output = false;

int main(int argc, char** argv)
{

    vector<string> args(argv, argv+argc);

    for(size_t i=0;i<args.size();i++)
    {
        if(args.at(i).compare("--output") == 0)
        {
            show_output = true;
        }
    }

    cout << "Show output? " << show_output << "\n";

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

    if(show_output)
    {
        player::create_windows(6,s);
    }

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

        if(show_output)
        {
            player::loop();
        }

        int key;

        if(!show_output || play)
        {
            key = waitKey(1000/fps);
        }
        else
        {
            key = waitKey(0);
        }

        if(show_output)
        {
            printf("Key: %d\n", key);
            switch(key)
            {
                case 97://a - play previous frame
                    i-=2;
                    if(i<=0){i=0;}
                    cap.set(CV_CAP_PROP_POS_FRAMES,i);
                    break;
                case 32://space - pause/play
                    play = !play;
                case 113://q - quit
                    return 0;
            }
        }
    }

    return 0;
}

vector<Vec4i> lines;

void hough(Mat &frame)
{
    int subframe_y = 240-40;

    if(show_output){player::show_frame(frame);}

    cvtColor(frame,frame,CV_BGR2GRAY);
    if(show_output){player::show_frame(frame);}

    blur(frame, frame, Size(4,4));
    if(show_output){player::show_frame(frame);}

    Mat subframe(frame,Rect(0,240-40,320,40));
    Canny(subframe, subframe, 50,100, 3);
    if(show_output){player::show_frame(frame);}

    HoughLinesP(subframe, lines, 1, CV_PI/720, 10,10,10);
    if(show_output){
        cvtColor(frame, frame, CV_GRAY2BGR);
        for(size_t j=0;j<lines.size();j++)
        {
            line(frame, Point(lines[j][0], subframe_y + lines[j][1]), Point(lines[j][2], subframe_y + lines[j][3]), Scalar(0,0,255), 3, 8);
        }
        player::show_frame(frame);
    }
}
