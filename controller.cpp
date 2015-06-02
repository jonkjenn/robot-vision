#include "controller.hpp"

using namespace std;
using namespace cv;

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
    //gpio_get_value(pin, &val);
    gpio_unexport(pin);
    if(val != prevval){
        LOG(DEBUG) << "READPIN: " << val;
        prevval = val;
    }
}

Controller::Controller(vector<string> &args, map<string,PID_config> &pids, function<void(Mat)> callback)
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

    if(cam)
    {
        vision = std::unique_ptr<Vision>(new Vision{args});
    }

    if(use_serial)
    {
        arduino = new Arduinocomm("/dev/ttyTHS0",115200);
        driver = std::make_shared<Drive>(161,160,163,164,arduino, pids);
        line_follower = std::make_shared<LineFollower<Drive>>(115,65);
        line_follower->setup(driver);

        delayMicroseconds(1e2);//Let serial connections start up
        driver->drive(90,90);
        driver->drive(90,90);
        driver->drive(90,90);
    }
}

void Controller::start()
{
    loop();
}

void Controller::configure_logger(const bool show_debug)
{
    /*
    el::Configurations c;
    c.setToDefault();
    c.parseFromText("*GLOBAL:\n ENABLED = false"); //Disable logging for the default logger setting

    if(show_debug){
        el::Configurations conf("log.conf");
        el::Loggers::reconfigureAllLoggers(conf);
    }else{ 
        el::Loggers::reconfigureAllLoggers(c); //Configure the silent default logger
    }

    LOG(INFO) << "Configured logger";*/
}

void Controller::parsepacket()
{
    //LOG(DEBUG) << "Parsing packet: " << (int)arduino->packet_buffer[0] << " size:" << (int)arduino->packet_size << endl;
    uint8_t s = arduino->packet_size;
    if(s>0)
    {
        switch(arduino->packet_buffer[0])
        {
            case Arduinocomm::OK:
                //LOG(DEGBU) << "Got OK" << endl;
                //driver->confirm_stop();
                /*if(waiting_ok)
                {
                    waiting_ok = false;
                }
                else
                {
                    LOG(DEBUG) << "OK when not waiting for OK" << endl;
                }*/
                break;
            case Arduinocomm::LINE_POSITION:
                {
                    //LOG(DEBUG) << "Packet: " << (int)arduino->packet_buffer[0] << " "<< (int)arduino->packet_buffer[1] << " " << (int)arduino->packet_buffer[2] << endl;
                    line_position = arduino->read_uint16(1);
                    //LOG(DEBUG) << "Position: " << pos << endl;
                    //LOG(DEBUG) << "Duration:"  << nanos() - prevpos << endl;
                    //LOG(DEBUG) << "Line packet: enabled: " << line_follower->enabled() << endl;
                    //if(line_follower->enabled())
                    {
                        //line_follower->update(pos);
                    }
                }
                break;
            case Arduinocomm::DRIVE_COMPLETED:
                break;
            case Arduinocomm::DEBUG:
                LOG(DEBUG) << "From Arduino : " << (int)arduino->packet_buffer[1] << endl;
                break;
            case Arduinocomm::CONFIRM_STOP:
                LOG(DEBUG) << "Got confirm stop " << endl;
                driver->confirm_stop();
                break;
            break;
        }
    }
    arduino->packet_ready = false;
}

void Controller::loop()
{
    //bool preempt = preempt_check();

    if(true)
    {
        /*struct sched_param param;
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
        t.tv_sec++;*/

        uint64_t pt = micros();

        while(true)
        {
            ////cout << "cont loop " << endl;
            //reset_micros();
            if(quit_robot){
                cout << "main loop stopping" << endl;
                if(cam)
                {
                    vision->stop();
                }
                driver->stop_driver();
                cout << "main loop stop" << endl;
                return;
            }
            //clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

            pt = nanos();
            if(use_serial)
            {
                driver->update();
                //cout << "Driver: " << endl;
            }

            //LOG(DEBUG) << "vision" << endl;
            Mat frame;
            if(cam)
            {
                frame = vision->update();
            }

            pt = nanos();

            if(use_serial)
            {
                arduino->update();
                //cout << "arduino update: " << nanos() - pt << endl;
                if(arduino->packet_ready)
                {
                    parsepacket();
                }
            }

            if(callback){callback(frame);}
        }
    }
}
