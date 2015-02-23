#include <line.h>
#include "QTRSensors-JetsonTk1/QTRSensors.h"
#include "easylogging++.h"
#include "opencv2/core/core.hpp"
#include <utility.h>
#include "vision.h"

INITIALIZE_EASYLOGGINGPP

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    bool show_video = false;
    bool play = false;
    bool show_debug = false;

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

    Controller controller{show_video,show_debug,play};

    return 0;
}

Controller::Controller(const bool show_video, const bool show_debug, const bool play)
{
    configure_logger(show_debug);

    configure_pins();

    bool write = false;

    //VideoCapture cap("../out.mp4");

    vision = Vision{0, show_video};
    
    loop();
}

void Controller::configure_logger(const bool show_debug)
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

void Controller::configure_pins()
{
    vector<unsigned char> pins = {160, 161, 162, 163};
    QTRSensorsRC q(pins, 164);
    LOG(INFO) << "Calibrating";
    q.calibrate();
    LOG(INFO) << "Calibrated done";
}


void Controller::handle_keys(bool &play)
{
    auto key = 0;
    if(play && vision.fps>0)
    {
        LOG(DEBUG) << "Waitkey with FPS";
        key = waitKey(1000/vision.fps) & 255;
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
            vision.previous_frame();
            break;
        case 32://space - pause/play
            vision.toggle_play();
            play = !play;
            return;
        default:
            return;
    }
}

void Controller::loop()
{
    while(true)
    {
        vision.update();
    }
}

