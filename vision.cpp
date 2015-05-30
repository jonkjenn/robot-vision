#include "vision.hpp"

using namespace std;
using namespace cv;
using namespace cv::gpu;

Vision::Vision(vector<string> &args)
{

    string file;
    for(size_t i=0;i<args.size();i++)
    {
        if(args[i].compare("--camera") == 0)
        {
            camera = true;
            camera_id = stoi(args[++i]);//should be more checks here
            continue;
        }

        if(args[i].compare("--video") == 0)
        {
            show_video = true;
            continue;
        }

        if(args[i].compare("--play") == 0)
        {
            play = true;
            continue;
        }

        if(args[i].compare("--cuda") == 0)
        {
            cuda = true;
            continue;
        }

        if(args[i].compare("--ps4") == 0)
        {
            ps4 = true;
            continue;
        }

        if(args[i].compare("--file") == 0 && args.size()>i+1)
        {
            camera = false;
            file = args[++i];//should be more checks here
            continue;
        }

        if(args[i].compare("--nowait") == 0)
        {
            no_wait = true;
            continue;
        }

        if(args[i].compare("--savevideo") == 0)
        {
            save_video = true;
            continue;
        }
    }

    LOG(DEBUG) << "Cuda: " << cuda << endl;

    if(camera)
    {
        LOG(DEBUG) << "Loading camera " << camera << endl;
        input_type = Type::CAMERA;
        cap = unique_ptr<VideoCapture>(new VideoCapture(camera_id));
        cap->set(CV_CAP_PROP_FRAME_WIDTH,898);
        cap->set(CV_CAP_PROP_FRAME_HEIGHT,200);
        LOG(DEBUG) << "Setting fps" << endl;
        cap->set(CV_CAP_PROP_FPS,240);
        frame_count = -1;
        play = true;
    }
    else if(ps4)
    {
        LOG(DEBUG) << "Loading PS4 camera" << endl;
        input_type = Type::PS4;
        ps4cam = unique_ptr<ps4driver>(new ps4driver());
        frame_count = -1;
        play = true;
    }
    else
    {
        LOG(DEBUG) << "Loading file" << endl;
        input_type = Type::FILE;
        cap = unique_ptr<VideoCapture>(new VideoCapture(file));
        frame_count = cap->get(CV_CAP_PROP_FRAME_COUNT);
        LOG(DEBUG) << "frame_count " << frame_count;
    }

    setup();
}

void Vision::setup()
{
    if(!ps4 && !cap->isOpened())
    {
        LOG(ERROR) << "Could not open video/camera" << endl;
        return;
    }

    if(!ps4)
    {
        fps = cap->get(CV_CAP_PROP_FPS);
    }

    LOG(DEBUG) << "Fps:" << fps << endl;

    LOG(DEBUG) << "Cuda: " << cuda << endl;

    if(cuda)
    {
        //configure_cuda();
    }
    //VideoWriter outputVideo;

    /*if(write){
        int ex = static_cast<int>(cap->get(CV_CAP_PROP_FOURCC));
    }*/

    /*if(write){
        outputVideo.open("../out.mp4",CV_FOURCC('m','4','s','2'),30,s,true);
    }*/


    if(false && save_video)
    {
        size = Size(177,144);
        sub_rect = Rect(0,size.height-41,size.width,40);
        Size sz = Size(320,192);
        //writer.open("out.avi",CV_FOURCC('M','J','P','G') ,200,sz);
        //writer.open("out.avi",CV_FOURCC('H','2','6','4'),200,sz);
        writer.open("out.avi",CV_FOURCC('M','P','4','V'),15,sz);
    }

    //fp = Frameplayer{show_video, 9, size};

    LOG(DEBUG) << "Frameplayer started" << endl;

    if(input_type == Type::CAMERA)
    {
        LOG(DEBUG) << "Starting camera thread" << endl;
        capture_thread = thread(&Vision::capture_frames,this, ref(frame));
    }
    else if(input_type == Type::PS4)
    {
        LOG(DEBUG) << "Starting PS4 thread" << endl;
        capture_thread = thread(&Vision::capture_ps4,this, ref(frame));
    }
    else if(input_type == Type::FILE)
    {
        LOG(DEBUG) << "Starting file thread" << endl;
        capture_thread = thread(&Vision::capture_frames_file,this, ref(frame));
    }
    previous_micros = micros();
}

/*void Vision::configure_cuda()
{
    cuda = gpu::getCudaEnabledDeviceCount()>0;
    LOG(INFO) <<"Setting CUDA device" << endl;
    if(!cuda){return;}
    gpu::setDevice(0);
    LOG(DEBUG) << "Cuda? " << cuda;

}*/

void Vision::capture_frames_file(Mat &frame)
{
    Mat buffer;
    while(true)
    {
        do{
            delayMicroseconds(10);
        }while(!play || !frame.empty());
        *cap.get() >> buffer;
        if(buffer.empty()){continue;}
        lock_guard<mutex> lock(camera_mutex); 
        frame = buffer;
        //resize(frame,frame, size);
        buffer.release();
    }
}

void Vision::capture_frames(Mat &frame)
{
    Mat buffer;
    while(true)
    {
        do{
            *cap.get() >> buffer;
        }while(buffer.empty());
        lock_guard<mutex> lock(camera_mutex); 

        frame = buffer;
        //resize(frame,frame, size);
        buffer.release();
    }
}

void Vision::capture_ps4(Mat &frame)
{
    Mat buffer;
    while(true)
    {
        do{
            //cout << "preloop" << endl;
            if(_quit.load()){cout << "vision thread _quit" << endl;return;}
            //cout << "loop" << endl;
            ps4cam->update();
            buffer = ps4cam->getFrame();
        }while(buffer.empty());
        lock_guard<mutex> lock(camera_mutex); 

        frame = buffer;
        //resize(frame,frame, size);
        buffer.release();
    }
}

Mat Vision::update()
{
    //cout << "vision update" << endl;
    Mat buffer;
    if(input_type == Type::FILE && index >= frame_count){play = false; handle_keys();return buffer;}

    if(input_type == Type::CAMERA || input_type == Type::PS4)
    {
        {
            lock_guard<mutex> lock(camera_mutex);
            buffer = frame;
            frame.release();
        }
        if(buffer.empty()){return buffer;}
        frame_counter++;

    }else if(input_type == Type::FILE)
    {
        {
            lock_guard<mutex> lock(camera_mutex);
            if(frame.empty()){LOG(DEBUG) << "Buffer empty";}
            buffer = frame;
            frame.release();
        }
        //cap.get() >> buffer;
        if(buffer.empty()){return buffer;}
        frame_counter++;
    }


    //LOG(DEBUG) << buffer.size() << endl;
    //LOG(INFO) << "Loaded frame: " << (micros() - loading_time) << " microseconds";
    //LOG(INFO) << "Resized frame";


    //fp.loop();

    if(!no_wait && (show_video || input_type == Type::FILE))
    {
        if(input_type == Type::CAMERA && index%5==0)
        {
            handle_keys();
        }
        else
        {
            handle_keys();
        }
    }

    auto dur = micros()-previous_micros;
    LOG(DEBUG) << "Loop duration: " << dur << " fps: " << (float)1000000/(float)dur << endl;
    //printf("fps: %f\n", (float)1000000/dur);
    if(false && save_video && micros()-previous_frame_saved > 66000){writer << buffer;previous_frame_saved = micros(); imwrite("out.png",buffer); }
    index++;
    previous_micros = micros();
    return buffer;
}

void Vision::previous_frame()
{
    if(input_type == Type::FILE && index > 1){
        LOG(DEBUG) << " index " << index;
        cap->set(CV_CAP_PROP_POS_FRAMES,index-=2);
    }
}

void Vision::process_frame_cuda(Mat &frame)
{
    LOG(DEBUG) << "Processing frame on GPU";

    //gpu_frame.upload(frame);//Uploads the frame to the GPU
    LOG(DEBUG) << "Cuda frame uploaded ";

    //fp.show_frame(gpu_frame);

    //hough_gpu(gpu_frame, frame);
}

void Vision::process_frame(Mat &frame )
{
    LOG(DEBUG) << "Processing frame on CPU";

    LOG(DEBUG) << "Doing hough++: ";

    hough(frame);

    LOG(DEBUG) << "Hough++ complete";

}

void Vision::hough(Mat &frame)
{
    int subframe_y = frame.rows-40;

    fp.show_frame(frame);

    LOG(DEBUG) << "Grayscale start";
    cvtColor(frame,frame,CV_BGR2GRAY);
    LOG(DEBUG) << "Grayscale stop";
    fp.show_frame(frame);

    LOG(DEBUG) << "Blur start";
    blur(frame, frame, Size(4,4));
    LOG(DEBUG) << "Blur stop";
    fp.show_frame(frame);

    LOG(DEBUG) << "Canny start";
    //Mat subframe(frame,Rect(0,frame.rows-40,frame.cols,40));
    Mat subframe(frame);
    Canny(subframe, subframe, 50,100, 3);
    LOG(DEBUG) << "Canny stop";
    fp.show_frame(frame);

    LOG(DEBUG) << "HOUGH start";
    vector<Vec4i> lines;
    HoughLinesP(subframe, lines, 1, CV_PI/360, 10,10,10);
    LOG(DEBUG) << "HOUGH stop";

    if(fp.enabled()){
        cvtColor(frame, frame, CV_GRAY2BGR);
        for(size_t j=0;j<lines.size();j++)
        {
            line(frame, Point(lines[j][0], subframe_y + lines[j][1]), Point(lines[j][2], subframe_y + lines[j][3]), Scalar(0,0,255), 3, 8);
        }
        fp.show_frame(frame);
    }
}



void Vision::handle_keys()
{
    auto dur_ms = (micros() - previous_micros)/1000;
    LOG(DEBUG) << "Play: " << play;
    auto key = 0;
    if(play && fps>0 && !ps4)
    {
        LOG(DEBUG) << "Waitkey with FPS" << endl;
        unsigned int wait_dur = 1000/fps-dur_ms;
        if(wait_dur<=0){wait_dur = 1;}
        LOG(DEBUG) << "Waiting for " << wait_dur << " ms" << endl;
        key = waitKey(wait_dur) & 255;
    }
    else if(play)
    {
        LOG(DEBUG) << "waitKey(1)";
        waitKey(1);
    }
    else
    {
        LOG(INFO) << "Waiting for key" << endl;
        key = waitKey(0) & 255;
    }

    LOG(DEBUG) <<"Key:"<< key << endl;
    switch(key)
    {
        case 113://q
            exit(0);
            break;
        case 97://a - play previous frame
            previous_frame();
            break;
        case 32://space - pause/play
            play = !play;
            return;
        default:
            return;
    }
}

void Vision::stop()
{
    _quit.store(true);
    if(capture_thread.joinable())
    {
        capture_thread.join();
    }
    if(ps4cam)
    {
        cout << "stopping ps4" << endl;
        ps4cam->stop();
    }

}
