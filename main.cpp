#include "controller.hpp"
#include "drive.h"

using namespace std;

void loop();
void drive_complete();

unsigned int step = 0;
bool stop = false;

float angle = 0;
uint8_t speed = 110;
unsigned int args_dist = 1000;
Rotation_Direction dir = LEFT;

unique_ptr<Controller> c;
shared_ptr<Drive> driver;

uint64_t start_time = 0;

int main(int argc, char** argv)
{
    vector<string> args(argv, argv+argc);
    c = unique_ptr<Controller>(new Controller(args, []{loop();}));
    driver = c->driver;

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

void loop()
{
    if(micros() - start_time < 100000){return;}
    if(stop)
    {
        driver->stop();
        exit(0);
    }

    switch(step)
    {
        case 0:
            cout << "case 0" << endl;
            //driver->driveDistance(speed, args_dist,[]{drive_complete();});
            driver->rotate(110,180,LEFT,[]{drive_complete();});
            step++;
            step = 3;
            break;
        case 2:
            LOG(DEBUG) << "Case 2";
            //driver->rotate(110,180,LEFT,[]{drive_complete();});
            step++;
            break;
        case 4:
            c->quit_robot = true;
            LOG(DEBUG) << "Stopping" <<endl;
            break;
    }
}

void drive_complete()
{
    LOG(DEBUG) << "Drive is completed";
    step++;
}

