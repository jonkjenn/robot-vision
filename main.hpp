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

void main_stop()
{
    c->quit_robot = true;
}

void drive_complete()
{
    LOG(DEBUG) << "Drive is completed";
    step++;
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

    bool enable_drive = true;

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

    if(what != FOLLOW_STEPS)
    {
        cout << "Step -1 " << endl;
        step = -1;
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
        driver->enable_drive = enable_drive;
        driver->set_distance_sensor_stop(false);//Skrur av ultralyd sensor stop
        driver->driveDistance(speed,abs(args_dist),[]{main_stop();},reverse,true);
    }
    else if(what == ROTATE)
    {
        driver->set_distance_sensor_stop(false);//Skrur av ultralyd sensor stop
        if(angle>0)
        {
            driver->rotate(110,angle,RIGHT,[]{main_stop();});
        }
        else
        {
            driver->rotate(110,angle,LEFT,[]{main_stop();});
        }
    }

    c->start();

    return 0;
}
