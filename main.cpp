#include "controller.hpp"
#include "drive.h"
#include <csignal>

using namespace std;

void loop();
void drive_complete();
void main_stop();

int step = 0;
bool stop = false;
float angle = 0;
uint8_t speed = 110;
int args_dist = 500;
Rotation_Direction dir = LEFT;

unique_ptr<Controller> c = nullptr;
shared_ptr<Drive> driver = nullptr;
shared_ptr<LineFollower<Drive>> lineFollower = nullptr;

enum do_what {FOLLOW_STEPS,DRIVE_DISTANCE,ROTATE,NOTHING};
do_what what = FOLLOW_STEPS;

uint64_t start_time = 0;

void loop()
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

    if(what == NOTHING){return;}

    //Example for how you can chain several different driving operations
    switch(step)
    {
        case 0:
            cout << "case 0" << endl;
           
            driver->set_distance_sensor_stop(false);//Disable stopping when ultrasound sensor triggers
            //driver->driveDistance(speed,args_dist,[]{drive_complete();});//Drive forward
            lineFollower->enable();//Enable linefollowing

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


//Triggers from CTRL C
void my_handler(int s){
    stop = true;
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

    //Checks from commandline arguments
    vector<string> args(argv, argv+argc);
    string file;
    for(size_t i=0;i<args.size();i++)
    {
        if(args[i].compare("--stop") == 0)
        {
            stop = true;
            break;
        }
        else if(args[i].compare("--angle") == 0)
        {
            angle = atof(args[++i].c_str());
            dir = (angle>0?RIGHT:LEFT);
        }
        else if(args[i].compare("--speed") == 0)
        {
            speed = atoi(args[++i].c_str());
        }
        else if(args[i].compare("--distance") == 0)
        {
            args_dist = atoi(args[++i].c_str());
        }
        else if(args[i].compare("--r") == 0)
        {
            what = ROTATE;
        }
        else if(args[i].compare("--d") == 0)
        {
            what = DRIVE_DISTANCE;
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
    }

    //Disable execution of the command list
    if(what != FOLLOW_STEPS)
    {
        cout << "Step -1 " << endl;
        step = -1;
    }

    //We pass the loop function to the Controller
    //because this will make the controller execute
    //the loop function continously
    c = unique_ptr<Controller>(new Controller(args, []{loop();}));
    driver = c->driver;
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
        driver->enable_drive = enable_drive;
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

