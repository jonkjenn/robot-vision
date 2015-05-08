#include "controller.hpp"
#include "drive.h"
#include <csignal>

using namespace std;

void loop();
void drive_complete();

unsigned int step = 0;
bool stop = false;

float angle = 0;
uint8_t speed = 110;
unsigned int args_dist = 500;
Rotation_Direction dir = LEFT;

unique_ptr<Controller> c = nullptr;
shared_ptr<Drive> driver = nullptr;
shared_ptr<LineFollower<Drive>> lineFollower = nullptr;

uint64_t start_time = 0;

void loop()
{
    if(micros() - start_time < 100000){return;}
    if(stop)
    {
        driver->stop();
        c->quit_robot = true;
        return;
    }

    switch(step)
    {
        case 0:
            cout << "case 0" << endl;
           
            driver->set_distance_sensor_stop(false);//Skrur av ultralyd sensor stop
            //driver->set_distance_sensor_stop(true);//Skrur påOC ultralyd sensor stop
            //driver->driveDistance(speed,args_dist,[]{drive_complete();});//Kjører fremover
            //driver->driveDistance(speed,args_dist,[]{drive_complete();},true);//Kjører bakover
            if(angle < 0)
            {
                cout << "angle < 0" << endl;
                driver->rotate(110,angle,LEFT,[]{drive_complete();});
            }
            else if(angle > 0)
            {
                cout << "angle > 0" << endl;
                driver->rotate(110,angle,RIGHT,[]{drive_complete();});
            }
            else
            {
                driver->rotate(110,0,LEFT,[]{drive_complete();});
            }
            //lineFollower->enable();//Linjefølging


            step++;
            break;
        case 2:
            LOG(DEBUG) << "Case 2";
            //driver->driveDistance(speed,args_dist,[]{drive_complete();});
            //driver->driveDistance(110,500,[]{drive_complete();});
            //driver->driveDistance(speed, args_dist,[]{drive_complete();});

            driver->rotate(110,0,LEFT,[]{drive_complete();});
            step++;
            break;
        case 4:
            LOG(DEBUG) << "Case 2";
            //driver->driveDistance(speed, args_dist,[]{drive_complete();});
            //driver->rotate(110,-90,LEFT,[]{drive_complete();});

            driver->rotate(110,0,LEFT,[]{drive_complete();});
            step++;
            break;
        case 6:
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
    c = unique_ptr<Controller>(new Controller(args, []{loop();}));
    driver = c->driver;
    lineFollower = c->line_follower;

    string file;
    for(size_t i=0;i<args.size();i++)
    {
        if(args[i].compare("--stop") == 0)
        {
            stop = true;
            break;
        }

        if(args[i].compare("--angle") == 0)
        {
            angle = atof(args[++i].c_str());
            dir = (angle>0?RIGHT:LEFT);
        }

        if(args[i].compare("--speed") == 0)
        {
            speed = atoi(args[++i].c_str());
        }

        if(args[i].compare("--distance") == 0)
        {
            args_dist = atoi(args[++i].c_str());
        }
    }

    start_time = micros();

    c->start();

    return 0;
}

void drive_complete()
{
    LOG(DEBUG) << "Drive is completed";
    step++;
}

