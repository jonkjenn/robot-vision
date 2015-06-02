#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"
#include "controller.hpp"
#include "drive.h"
#include <csignal>
#include "opencv2/core/core.hpp"
#include "camshift.h"
#include "find_lines.hpp"
#include "utility.hpp"

using namespace std;
using namespace cv;

void loop(Mat frame);
void drive_complete();
void main_stop();

static const char *usage = 
    "Usage: main [ options ]\n"
    "For several of the operations the program has to be run as root.\n"
    "   --stop                          Stop the motors\n"
    "   --r ANGLE                       Rotate the vehicle\n"
    "                                    ANGLE which angle to rotate\n" 
    "                                       ANGLE > 0 rotate right\n"
    "                                       ANGLE < 0 rotate left\n"
    "   --d DISTANCE SPEED            Drive straight forward or backward\n"
    "                                    SPEED Set desired motor speed\n"
    "                                       0 = full speed backwards\n"
    "                                       90 = stop\n"
    "                                       180 = full speed forwards\n"
    "                                    DISTANCE how far to drive in millimeter\n"
    "                                       DISTANCE < 0: drive backwards\n"
    "   --do_not_drive                  Lets you use commands without actually driving\n"
    "                                    the motors\n"
    "   --follow_steps                  Follow the step-wise program in main.cpp\n"
    "   --step STEP                     Which step to start at in --follow_steps\n"
    "   --follow_ball                   Follow colored ball\n"
    "   --follow_line                   Follow line\n"
    "   --video                         Display video\n"
    "   --nocam                         Do not use a camera\n"
    "   --noserial                      Do not use serial\n"
    "   --camera CAMERA                 Use camera number CAMERA\n"
    "   --ps4                           Use ps4 camera\n"
    "   --savevideo                     Writes video to out.avi\n"
    "                                    and one uncompressed image to out.png\n"
    "\n"
    "Examples:\n"
    "   Drive forward for 1000 mm at a moderate speed\n"
    "       sudo bin/main --d --distance 1000 --speed 110 --nocam \n"
    "   Drive backwards for 2000 mm at a moderate speed\n"
    "       sudo bin/main --d --distance 1000 --speed 110 --nocam \n"
    "   Rotate 180 degrees to the right\n"
    "       bin/main --r --angle 180 --nocam\n"
    "   Rotate 90 degrees to the left\n"
    "       bin/main --r --angle -90 --nocam\n";

int step = -1;
bool stop = false;
float angle = 0;
uint8_t speed = 110;
int args_dist = 500;
Rotation_Direction dir = LEFT;

bool do_camshift = false;

camshift cshift;
//Find_lines flines;

unique_ptr<Controller> c = nullptr;
shared_ptr<Drive> driver = nullptr;
shared_ptr<LineFollower<Drive>> lineFollower = nullptr;

map<string,PID_config> pids;

enum do_what {FOLLOW_STEPS,DRIVE_DISTANCE,ROTATE,NOTHING,TRACK_BALL,FOLLOW_LINE};
do_what what = NOTHING;

uint64_t start_time = 0;

void load_config()
{
    using boost::property_tree::ptree;
    ptree pt;


    read_json("config.json",pt);

    for(auto& p:pt.get_child("pids"))
    {
        auto& pid = p.second;
        //cout << "pids: "<<  pid.second.get<float>("kp") << endl;
        //cout << "KP " << pid.second.get<float>("kp") << endl;
        //
        PID_config pc;
        pc.kp  = pid.get<float>("kp");
        pc.kd = pid.get<float>("kd");
        pc.ki = pid.get<float>("ki");
        pc.setpoint = pid.get<float>("setpoint");
        pc.minimum = pid.get<float>("minimum");
        pc.maximum = pid.get<float>("maximum");
        pids.insert(pair<string,PID_config>(pid.get("name","unknown"),pc));
    }
}

void loop(Mat frame)
{
    if(micros() - start_time < 100000){return;}
    if(stop)
    {
        cout << "stopping" << endl;
        if(what != NOTHING)
        {
            driver->stop();
        }
        c->quit_robot = true;
        return;
    }

    //flines.find(frame);


    if(what == NOTHING){
        return;
    }
    else if(what == TRACK_BALL)
    {
        int pos =  cshift.update_camshift(frame);
        frame.release();
        //cout << "pos: " << pos << endl;
        if(pos >= 0)
        {
            lineFollower->update(pos);
        }
    }
    else if(what == FOLLOW_STEPS)
    {
        //Example for how you can chain several different driving operations
        switch(step)
        {
            case 0:
                cout << "case 0" << endl;
               
                driver->set_distance_sensor_stop(false);//Disable stopping when ultrasound sensor triggers
                //driver->driveDistance(speed,args_dist,[]{drive_complete();});//Drive forward
                //lineFollower->enable();//Enable linefollowing

                step++;
                break;
            case 2:
                LOG(DEBUG) << "Case 2";
                //driver->rotate(110,45,RIGHT,[]{drive_complete();});//Rotate 45 degrees
                step++;
                break;
            case 4:
                c->quit_robot = true;
                LOG(DEBUG) << "Stopping" <<endl;
                break;
        }
    }
    else if(what == FOLLOW_LINE)
    {
        if(c->line_position >= 0)
        {
            lineFollower->update(c->line_position);
        }
    }

}

//Triggers from CTRL C
void my_handler(int s){
    stop = true;
}

void print_menu()
{
    cout << usage << endl;
}

int main(int argc, char** argv)
{
    //To capture CTRL + C
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    bool enable_drive = true;
    bool show_video = false;
    bool save_video = false;

    //Checks from commandline arguments
    vector<string> args(argv, argv+argc);
    string file;
    if(args.size() == 1)
    {
        print_menu();
        return 0;
    }

    for(size_t i=0;i<args.size();i++)
    {
        if(args[i].compare("--stop") == 0)
        {
            stop = true;
            break;
        }
        else if(args[i].compare("--r") == 0)
        {
            what = ROTATE;
            angle = atof(args[++i].c_str());
            dir = (angle>0?RIGHT:LEFT);
        }
        else if(args[i].compare("--d") == 0)
        {
            what = DRIVE_DISTANCE;
            args_dist = atoi(args[++i].c_str());
            speed = atoi(args[++i].c_str());
        }
        else if(args[i].compare("--donothing") == 0)
        {
            what  = NOTHING;
        }
        else if(args[i].compare("--do_not_drive") == 0)
        {
            enable_drive = false;
        }
        else if(args[i].compare("--step") == 0)
        {
            step = atoi(args[++i].c_str());
        }
        else if(args[i].compare("--follow_ball") == 0)
        {
            what = TRACK_BALL;
        }
        else if(args[i].compare("--follow_line") == 0)
        {
            what = FOLLOW_LINE;
        }
        else if(args[i].compare("--video") == 0)
        {
            show_video = true;
        }
        else if(args[i].compare("--savevideo") == 0)
        {
            save_video = true;
        }
        else if(args[i].compare("--follow_steps") == 0)
        {
            what = FOLLOW_STEPS;
        }
    }

    load_config();

    //We pass the loop function to the Controller
    //because this will make the controller execute
    //the loop function continously
    c = unique_ptr<Controller>(new Controller(args, pids, [&](Mat frame){loop(frame);}));
    driver = c->driver;
    driver->enable_drive = enable_drive;
    lineFollower = c->line_follower;

    start_time = micros();

    //Drive distance from command line
    if(what == DRIVE_DISTANCE)
    {
        //Negative distance, drive backwards
        bool reverse = false;
        if(args_dist < 0)
        {
            reverse = true;
        }
        driver->set_distance_sensor_stop(false);//Disable ultrasound
        driver->driveDistance(speed,abs(args_dist),[]{main_stop();},reverse,true);
    }
    //Rotate from command line
    else if(what == ROTATE)
    {
        driver->set_distance_sensor_stop(false);//Skrur av ultralyd sensor stop
        if(angle>0)//Turn right
        {
            driver->rotate(110,angle,RIGHT,[]{main_stop();});
        }
        else//Turn left
        {
            driver->rotate(110,angle,LEFT,[]{main_stop();});
        }
    }
    else if(what == TRACK_BALL)
    {
        cshift.setup_camshift(show_video,save_video);
        lineFollower->enable();
    }
    else if(what == FOLLOW_LINE)
    {
        lineFollower->enable();
    }
    else if(what == FOLLOW_STEPS)
    {
        step = 0;
    }

    c->start();

    return 0;
}

void main_stop()
{
    c->quit_robot = true;
}

void drive_complete()
{
    LOG(DEBUG) << "Drive is completed";
    step++;
}
