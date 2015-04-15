#include "easylogging++.h"
#include "controller.hpp"
#include "drive.h"
#define ELPP_CUSTOM_COUT std::cerr

using namespace std;

void loop();
void drive_complete();

unsigned int step = 0;
bool stop = false;

unique_ptr<Controller> c;
shared_ptr<Drive> driver;

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
    }

    c->start();

    return 0;
}

void loop()
{
    if(stop)
    {
        driver->stop();
        exit(0);
    }

    switch(step)
    {
        case 0:
            LOG(DEBUG) << "Sending drive";
            //driver->driveDuration(110,3000, []{drive_complete();});
            driver->rotateRight(110,90,[]{drive_complete();});
            step++;
            break;
        case 2:
            //driver->rotateLeft(110,90,[]{drive_complete();});
            step++;
            break;
    }
}

void drive_complete()
{
    LOG(DEBUG) << "Drive is completed";
    step++;
}

