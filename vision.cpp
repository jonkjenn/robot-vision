#include <vision.hpp>

using namespace std;
using namespace cv;
using namespace cv::gpu;

Vision::Vision(vector<string> &args)
{
    bool camera = true;
    string file;
    for(auto i=0;i<args.size();i++)
    {
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

        if(args[i].compare("--file") == 0 && args.size()>i+1)
        {
            camera = false;
            file = args[++i];
            continue;
        }

        if(args[i].compare("--nowait") == 0)
        {
            no_wait = true;
            continue;
        }
    }

    LOG(DEBUG) << "Loading camera " << camera;
    if(camera)
    {
        input_type = Type::CAMERA;
        cap = unique_ptr<VideoCapture>(new VideoCapture(0));
        frame_count = -1;
        play = true;
    }
    else
    {
        input_type = Type::FILE;
        cap = unique_ptr<VideoCapture>(new VideoCapture(file));
        frame_count = cap->get(CV_CAP_PROP_FRAME_COUNT);
        LOG(DEBUG) << "frame_count " << frame_count;
    }
    setup();
}

void Vision::setup()
{
    if(!cap->isOpened())
    {
        LOG(ERROR) << "Could not open video/camera";
        return;
    }

    fps = cap->get(CV_CAP_PROP_FPS);

    LOG(DEBUG) << "Fps:" << fps;

    if(cuda)
    {
        configure_cuda();
    }
    //VideoWriter outputVideo;

    /*if(write){
        int ex = static_cast<int>(cap->get(CV_CAP_PROP_FOURCC));
    }*/

    /*if(write){
        outputVideo.open("../out.mp4",CV_FOURCC('m','4','s','2'),30,s,true);
    }*/

    size = Size(320,240);
    sub_rect = Rect(0,size.height-41,size.width,40);
    fp = Frameplayer{show_video, 9, size};

    if(input_type == Type::CAMERA)
    {
        thread capture_thread(&Vision::capture_frames,this, ref(frame));
        capture_thread.detach();
    }
    if(input_type == Type::FILE)
    {
        thread capture_thread(&Vision::capture_frames_file,this, ref(frame));
        capture_thread.detach();
    }
    previous_micros = micros();
}

void Vision::configure_cuda()
{
    cuda = gpu::getCudaEnabledDeviceCount()>0;
    LOG(INFO) <<"Setting CUDA device";
    if(!cuda){return;}
    gpu::setDevice(0);
    LOG(DEBUG) << "Cuda? " << cuda;

    blur_filter = gpu::createGaussianFilter_GPU(CV_8U,Size(3,3),-1);
}

void Vision::capture_frames_file(Mat &frame)
{
    Mat buffer;
    while(true)
    {
        do{
            delayMicroseconds(10);
        }while(!frame.empty());
        *cap.get() >> buffer;
        lock_guard<mutex> lock(camera_mutex); 
        frame = buffer;
        resize(frame,frame, size);
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
        resize(frame,frame, size);
        buffer.release();
    }
}

void Vision::update()
{
    if(input_type == Type::FILE && index >= frame_count){handle_keys();return;}
    Mat buffer;
    LOG(INFO) << "Loading frame";

    if(input_type == Type::CAMERA)
    {
        {
            lock_guard<mutex> lock(camera_mutex);
            buffer = frame;
            frame.release();
        }
        if(buffer.empty()){return;}

    }else if(input_type == Type::FILE)
    {
        {
            lock_guard<mutex> lock(camera_mutex);
            if(frame.empty()){LOG(DEBUG) << "Buffer empty"; return;}
            buffer = frame;
            frame.release();
        }
        /*cap.get() >> buffer;
        if(buffer.empty()){return;}*/
    }

    LOG(INFO) << "Loaded frame";
    //LOG(INFO) << "Resized frame";

    if(cuda)
    {
        process_frame_cuda(buffer);
    }
    else
    {
        process_frame(buffer);
    }

    fp.loop();

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
    LOG(DEBUG) << "Loop duration: " << dur << " fps: " << (float)1000000/dur;
    printf("fps: %f\n", (float)1000000/dur);
    index++;
    previous_micros = micros();
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

    gpu_frame.upload(frame);//Uploads the frame to the GPU
    LOG(DEBUG) << "Cuda frame uploaded ";

    fp.show_frame(gpu_frame);

    hough_gpu(gpu_frame, frame);
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
    Mat subframe(frame,Rect(0,frame.rows-40,frame.cols,40));
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


void Vision::hough_gpu(gpu::GpuMat &gpu_frame, Mat &frame)
{
    //int subframe_y = frame.rows-40;

    LOG(DEBUG) << "Grayscale start";
    
    /*if(gpu_grayscale.empty())
    {
        gpu_grayscale = gpu::GpuMat(gpu_frame.rows, gpu_frame.cols, gpu_frame.type());
    }*/
    gpu::cvtColor(gpu_frame,gpu_grayscale,CV_BGR2GRAY,1);
    LOG(DEBUG) << "Grayscale stop";

    fp.show_frame(gpu_grayscale);
    
    LOG(DEBUG) << "Blur start";

    //Create border for using image with blur, dont know why need 2 pixels instead of 1
    /*if(gpu_border.empty()){
        gpu_border = gpu::GpuMat gpu::GpuMat(gpu_grayscale.rows +4 , gpu_grayscale.cols + 4, gpu_grayscale.type());
    }

    gpu::copyMakeBorder(gpu_grayscale, gpu_frame3, 2, 2 , 2, 2, BORDER_REPLICATE);*/

    /*if(gpu_blurbuffer.empty())
    {
        gpu_blurbuffer = gpu::GpuMat(gpu_frame.rows, gpu_frame.cols, gpu_frame.type());
    }*/

/*    if(gpu_blur.empty())
    {
        gpu_blur = gpu::GpuMat(gpu_frame.rows, gpu_frame.cols, gpu_frame.type());
    }

    gpu::blur(gpu_grayscale, gpu_blur, Size(3,3));

    gpu::GpuMat roi(gpu_frame, Rect(2, 2, gpu_frame.cols-4, gpu_frame.rows-4));
    */

    LOG(DEBUG) << "Subframe start";
    gpu_subframe = gpu::GpuMat(gpu_grayscale,sub_rect);

    LOG(DEBUG) << "Filter start";

    blur_filter->apply(gpu_subframe, gpu_blur, Rect(0,0,gpu_subframe.cols, gpu_subframe.rows));

    LOG(DEBUG) << "Blur stop";

    fp.show_frame(gpu_blur);

    //gpu::GpuMat subframe(gpu_frame,Rect(0,frame.rows-40,frame.cols,40));
    //gpu::GpuMat subframe2(subframe.rows, subframe.cols, subframe.type());
    //gpu::Canny(subframe, subframe2, 50,100, 3);



    LOG(DEBUG) << "Canny start";
    gpu::Canny(gpu_blur, gpu_canny, 50.0f,100.0f);
    LOG(DEBUG) << "Canny stop, rows " << gpu_canny.rows << "cols " << gpu_canny.cols;

    fp.show_frame(gpu_canny);

    LOG(DEBUG) << "Hough start";
    gpu::HoughLinesP(gpu_canny, d_lines,d_buf, 1.0f,hough_angles,10,10);
    LOG(DEBUG) << "Hough stop";

    if(fp.enabled()){
        draw_hough(d_lines, frame);
    }
}

void Vision::draw_hough(GpuMat &d_lines, Mat &frame)
{
    vector<Vec4i> lines;
    vector<Vec4i> lines_gpu;

    lines_gpu.resize(d_lines.cols);
    Mat h_lines(1, d_lines.cols, CV_32SC4, &lines_gpu[0]);
    d_lines.download(h_lines);

    //cvtColor(frame, frame, CV_GRAY2BGR);

    //Vec4i l;
    //for(auto j = 0;j<lines_gpu.size();j++)
    for(Vec4i l:lines_gpu)
    {
        //l = lines_gpu[j];
        line(frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
    }
    fp.show_frame(frame);
}

void Vision::handle_keys()
{
    auto dur_ms = (micros() - previous_micros)/1000;
    LOG(DEBUG) << "Play: " << play;
    auto key = 0;
    if(play && fps>0)
    {
        LOG(DEBUG) << "Waitkey with FPS";
        unsigned int wait_dur = 1000/fps-dur_ms;
        if(wait_dur<=0){wait_dur = 1;}
        LOG(DEBUG) << "Waiting for " << wait_dur << " ms";
        key = waitKey(wait_dur) & 255;
    }
    else if(play)
    {
        LOG(DEBUG) << "waitKey(1)";
        waitKey(1);
    }
    else
    {
        LOG(INFO) << "Waiting for key";
        key = waitKey(0) & 255;
    }

    LOG(DEBUG) <<"Key:"<< key;
    switch(key)
    {
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
