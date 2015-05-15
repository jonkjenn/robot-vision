#include "controller.hpp"
#include "drive.h"
#include <csignal>

using namespace std;

void loop();
void drive_complete();
void main_stop();

int step = -1;
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
        if(what != NOTHING)
        {
            driver->stop();
        }
        c->quit_robot = true;
        return;
    }

    if(what == NOTHING){return;}

    switch(step)
    {
        case 0:
            cout << "case 0" << endl;
           
            driver->set_distance_sensor_stop(false);//Skrur av ultralyd sensor stop
            //driver->set_distance_sensor_stop(true);//Skrur på ultralyd sensor stop
            driver->driveDistance(speed,400,[]{drive_complete();});//Kjører fremover
            //driver->driveDistance(speed,args_dist,[]{drive_complete();},true);//Kjører bakover
            if(angle < 0)
            {
                cout << "angle < 0" << endl;
                //driver->rotate(110,angle,LEFT,[]{drive_complete();});
            }
            else if(angle > 0)
            {
                cout << "angle > 0" << endl;
                //driver->rotate(110,angle,RIGHT,[]{drive_complete();});
            }
            else
            {
                //driver->rotate(110,0,LEFT,[]{drive_complete();});
            }
            //lineFollower->enable();//Linjefølging


            step++;
            break;
        case 2:
            LOG(DEBUG) << "Case 2";
            //driver->driveDistance(speed,args_dist,[]{drive_complete();});
            //driver->driveDistance(110,500,[]{drive_complete();});
            //driver->driveDistance(speed, args_dist,[]{drive_complete();});

            driver->rotate(110,90,LEFT,[]{drive_complete();});
            step++;
            break;
        case 4:
            LOG(DEBUG) << "Case 2";
            //driver->driveDistance(speed, args_dist,[]{drive_complete();});
            //driver->rotate(110,-90,LEFT,[]{drive_complete();});

            driver->driveDistance(speed,1350,[]{drive_complete();},true);//Kjører fremover
            step++;
            break;
        case 6:
            LOG(DEBUG) << "Case 2";
            //driver->driveDistance(speed, args_dist,[]{drive_complete();});
            driver->rotate(110,90,RIGHT,[]{drive_complete();});

            step++;
            break;
        case 8:
            LOG(DEBUG) << "Case 2";
            //driver->driveDistance(speed, args_dist,[]{drive_complete();});

            driver->driveDistance(speed,600,[]{drive_complete();},true);//Kjører fremover
            step++;
            break;
        case 10:
            LOG(DEBUG) << "Case 2";
            //driver->driveDistance(speed, args_dist,[]{drive_complete();});
            driver->driveDistance(speed,200,[]{drive_complete();});//Kjører fremover
            //driver->rotate(110,90,LEFT,[]{drive_complete();});

            step++;
            break;
        case 12:
            LOG(DEBUG) << "Case 2";
            //driver->driveDistance(speed, args_dist,[]{drive_complete();});

            driver->rotate(110,90,RIGHT,[]{drive_complete();});

            //driver->driveDistance(speed,args_dist,[]{drive_complete();});//Kjører fremover
            step++;
            break;
        case 14:
            LOG(DEBUG) << "Case 2";
            //driver->driveDistance(speed, args_dist,[]{drive_complete();});
            //driver->rotate(110,90,LEFT,[]{drive_complete();});
            driver->driveDistance(140,2000,[]{drive_complete();},true);//Kjører fremover

            step++;
            break;
        case 16:
            c->quit_robot = true;
            LOG(DEBUG) << "Stopping" <<endl;
            break;
    }
}


void my_handler(int s){
    stop = true;
}

int main(int argc, char** argv)
{
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

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
    }

    if(what == FOLLOW_STEPS)
    {
        step = 0;
    }

    c = unique_ptr<Controller>(new Controller(args, []{loop();}));
    driver = c->driver;
    lineFollower = c->line_follower;

    start_time = micros();

    if(what == DRIVE_DISTANCE)
    {
        bool reverse = false;
        if(args_dist < 0)
        {
            reverse = true;
        }
        driver->driveDistance(speed,abs(args_dist),[]{main_stop();},reverse,true);
    }
    else if(what == ROTATE)
    {
        driver->rotate(110,angle,LEFT,[]{main_stop();});
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

