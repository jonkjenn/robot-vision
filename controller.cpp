#include "controller.hpp"
INITIALIZE_EASYLOGGINGPP

using namespace std;

unsigned int serial_delay = 0;

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
    crit1 = strcasestr (u.version, "PREEMPT");

    if ((fd = fopen("/sys/kernel/realtime","r")) != NULL) {
        int flag;
        crit2 = ((fscanf(fd, "%d", &flag) == 1) && (flag == 1));
        fclose(fd);
    }

    return crit1 || crit2;
}

int prevval = 0;
void readPin(int pin)
{
    unsigned int val = 0;
    gpio_export(pin);
    gpio_set_dir(pin, INPUT_PIN);
    gpio_get_value(pin, &val);
    gpio_unexport(pin);
    if(val != prevval){
        LOG(DEBUG) << "READPIN: " << val;
        prevval = val;
    }
}

Controller::Controller(vector<string> &args, function<void()> callback)
{
    this->callback = callback;
    bool show_debug = false;
    int pin = -1;

    for(size_t i=0;i<args.size();i++)
    {
        if(args[i].compare("--debug") == 0)
        {
            show_debug = true;
            continue;
        }

        if(args[i].compare("--readpin") == 0)
        {
            pin = stoi(args[++i]);
        }

        if(args[i].compare("--noserial") == 0)
        {
            use_serial = false;
        }

        if(args[i].compare("--nocam") == 0)
        {
            cam = false;
        }
    }

    configure_logger(show_debug);

    LOG(DEBUG) << "Show debug " << show_debug;

    //configure_pins();

    //VideoCapture cap("../out.mp4");

    vision = std::unique_ptr<Vision>(new Vision{args});

    if(use_serial)
    {
        arduino = std::shared_ptr<Arduinocomm>(new Arduinocomm("/dev/ttyTHS0",115200));
    }

    driver = std::shared_ptr<Drive>(new Drive(161,160,163,164,arduino));
}

void Controller::start()
{
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

void Controller::parsepacket()
{
    LOG(DEBUG) << "Parsing packet: " << (int)arduino->packet_buffer[0] << " size:" << (int)arduino->packet_size;
    uint8_t s = arduino->packet_size;
    if(s>0)
    {
        switch(arduino->packet_buffer[0])
        {
            case Arduinocomm::OK:
                LOG(DEBUG) << "Got OK";
                if(waiting_ok)
                {
                    waiting_ok = false;
                }
                else
                {
                    LOG(DEBUG) << "OK when not waiting for OK";
                }
            case Arduinocomm::DRIVE_COMPLETED:
                LOG(DEBUG) << "Got drive completed";
            case Arduinocomm::DEBUG:
                LOG(DEBUG) << "From Arduino : " << (int)arduino->packet_buffer[1];
            break;
        }
    }
    arduino->packet_ready = false;
}

void Controller::loop()
{
    bool preempt = preempt_check();

    LOG(DEBUG) << "Preempt? " << preempt;
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

            //LOG(DEBUG) << "driver";
            driver->update();

            //LOG(DEBUG) << "vision";
            if(cam)
            {
                vision->update();
            }

            //LOG(DEBUG) << "arduino";
            arduino->update();
            if(arduino->packet_ready)
            {
                parsepacket();
            }

            //LOG(DEBUG) << "callback";
            callback();

            //LOG(DEBUG) << "complete";

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
            if(use_serial && serial_delay > 1000)
            {
                arduino->update();
                if(arduino->packet_ready)
                {
                    parsepacket();
                }
                //delayMicroseconds(1000000);
                //LOG(DEBUG) << "Arduino available " << arduino->available();
                //
                //vision->update();
            }
            else
            {
                serial_delay++;
            }
            vision->update();
            //delayMicroseconds(50);
        }
    }

}
