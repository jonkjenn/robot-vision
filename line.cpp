#include <line.h>

INITIALIZE_EASYLOGGINGPP

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    bool show_video = false;
    bool play = false;
    bool show_debug = false;

    string *file = nullptr;

    vector<string> args(argv, argv+argc);
    for(auto i=0;i<args.size();i++)
    {
        if(args[i].compare("--video") == 0)
        {
            show_video = true;
            continue;
        }

        if(args[i].compare("--debug") == 0)
        {
            show_debug = true;
            continue;
        }

        if(args[i].compare("--play") == 0)
        {
            play = true;
            continue;
        }

        if(args[i].compare("--file") == 0 && args.size()>i+1)
        {
            file = &args[++i];
        }
    }

    Controller controller{show_video,show_debug,file};

    return 0;
}

Controller::Controller(const bool show_video, const bool show_debug, string *file)
{
    configure_logger(show_debug);

    configure_pins();

    //VideoCapture cap("../out.mp4");

    if(file == nullptr)
    {
        setup_vision(show_video);
    }
    else
    {
        setup_vision(*file, show_video);
    }

    loop();
}

void Controller::setup_vision(const bool show_video){
    vision = unique_ptr<Vision>(new Vision{0, show_video});
}

void Controller::setup_vision(string &file, const bool show_video)
{
    vision = unique_ptr<Vision>(new Vision{file, show_video});
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


void Controller::loop()
{
    
    while(true)
    {
        vision->update();
    }
}

