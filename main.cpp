#include "main.hpp"

INITIALIZE_EASYLOGGINGPP

using namespace std;
using namespace cv;

#define MY_PRIORITY (49)
#define MAX_SAFE_STACK (8*1024)
#define NSEC_PER_SEC (1000000000)

void stack_prefault(void){
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
    return;
}

bool preempt_check()
{
    struct utsname u;
    char *crit1, crit2 = 0;
    FILE *fd;

    uname(&u);
    crit1 = strcasestr (u.version, "PREEMPT RT");

    if ((fd = fopen("/sys/kernel/realtime","r")) != NULL) {
        int flag;
        crit2 = ((fscanf(fd, "%d", &flag) == 1) && (flag == 1));
        fclose(fd);
    }

    return crit1 && crit2;
}

int main(int argc, char** argv)
{
    bool show_debug = false;

    vector<string> args(argv, argv+argc);
    for(size_t i=0;i<args.size();i++)
    {
        if(args[i].compare("--debug") == 0)
        {
            show_debug = true;
            continue;
        }
    }

    Controller controller{show_debug,args};

    return 0;
}

Controller::Controller(const bool show_debug, vector<string> &args)
{
    configure_logger(show_debug);

    LOG(DEBUG) << "Show debug " << show_debug;

    configure_pins();

    //VideoCapture cap("../out.mp4");

    vision = std::unique_ptr<Vision>(new Vision{args});

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


void Controller::loop()
{
    bool preempt = preempt_check();

    if(preempt)
    {
        struct sched_param param;
        param.sched_priority = MY_PRIORITY;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1){
            perror("sched_setscheduler failed");
            exit(-1);
        }

        stack_prefault();

        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1){
            perror("mlockall failed");
            exit(-2);
        }

        struct timespec t;
        int interval = 50000;
        clock_gettime(CLOCK_MONOTONIC, &t);
        t.tv_sec++;

        while(true)
        {
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
            vision->update();

            t.tv_nsec += interval;
            while(t.tv_nsec >= NSEC_PER_SEC){
                t.tv_nsec -= NSEC_PER_SEC;
                t.tv_sec++;
            }
        }
    }
    else
    {
        while(true)
        {
            vision->update();
            delayMicroseconds(50);
        }
    }
}

