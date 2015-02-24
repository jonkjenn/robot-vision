#include <vision.h>

using namespace std;
using namespace cv;
using namespace cv::gpu;

Vision::Vision()
{
}

Vision::Vision(int camera, bool show_video)
{
    LOG(DEBUG) << "Loading camera " << camera;
    input_type = Type::CAMERA;
    cap = unique_ptr<VideoCapture>(new VideoCapture(camera));
    Vision::show_video = show_video;
    frame_count = -1;
    play = true;
    setup();
}

Vision::Vision(const string &file, bool show_video)
{
    input_type = Type::FILE;
    cap = unique_ptr<VideoCapture>(new VideoCapture(file));
    Vision::show_video = show_video;
    frame_count = cap->get(CV_CAP_PROP_FRAME_COUNT);
    play = false;
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

    configure_cuda();
    //VideoWriter outputVideo;

    /*if(write){
        int ex = static_cast<int>(cap->get(CV_CAP_PROP_FOURCC));
    }*/


    /*if(write){
        outputVideo.open("../out.mp4",CV_FOURCC('m','4','s','2'),30,s,true);
    }*/

    size = Size(320,240);
    fp = Frameplayer{show_video, 9, size};

    if(input_type == Type::CAMERA)
    {
        thread capture_thread(&Vision::capture_frames,this, ref(frame));
        capture_thread.detach();
    }
}

bool Vision::configure_cuda()
{
    cuda = gpu::getCudaEnabledDeviceCount()>0;
    LOG(INFO) <<"Setting CUDA device";
    if(cuda){gpu::setDevice(0);}
    LOG(DEBUG) << "Cuda? " << cuda;
    return cuda;
}

void Vision::capture_frames(Mat &frame)
{
    Mat buffer;
    while(true)
    {
        buffer.release();
        LOG(DEBUG) << "Frame from camera";
        do{
            *cap.get() >> buffer;
        }while(buffer.empty());
        LOG(DEBUG) << "Complete frame from camera";
        lock_guard<mutex> lock(camera_mutex); 
        frame = buffer;
    }
}

void Vision::update()
{
    if(input_type == Type::FILE && index >= frame_count){handle_keys();return;}
    Mat buffer;
    auto loop_time = micros();
    LOG(INFO) << "Loading frame";

    if(input_type == Type::CAMERA)
    {
        do{
            lock_guard<mutex> lock(camera_mutex);
            buffer = frame;
        }while(buffer.empty());
    }else if(input_type == Type::FILE)
    {
        do{
            *cap.get() >> buffer;
        }while(buffer.empty());
    }

    LOG(INFO) << "Loaded frame";

    resize(buffer,buffer, size);
    LOG(INFO) << "Resized frame";

    if(cuda)
    {
        process_frame_cuda(buffer);
    }
    else
    {
        process_frame(buffer);
    }

    fp.loop();

    handle_keys();

    auto dur = micros()-loop_time;
    LOG(DEBUG) << "Loop duration: " << dur << " fps: " << (float)1000000/dur;
    printf("fps: %f\n", (float)1000000/dur);

    index++;
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

    gpu::GpuMat gpu_frame;
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

    cvtColor(frame,frame,CV_BGR2GRAY);
    fp.show_frame(frame);

    blur(frame, frame, Size(4,4));
    fp.show_frame(frame);

    Mat subframe(frame,Rect(0,frame.rows-40,frame.cols,40));
    Canny(subframe, subframe, 50,100, 3);
    fp.show_frame(frame);

    vector<Vec4i> lines;
    HoughLinesP(subframe, lines, 1, CV_PI/720, 10,10,10);

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
    
    gpu::GpuMat gpu_frame2(gpu_frame.rows, gpu_frame.cols, gpu_frame.type());
    gpu::cvtColor(gpu_frame,gpu_frame2,CV_BGR2GRAY);
    LOG(DEBUG) << "Grayscale stop";

    fp.show_frame(gpu_frame2);
    
    LOG(DEBUG) << "Blur start";

    //Create border for using image with blur, dont know why need 2 pixels instead of 1
    gpu::GpuMat gpu_frame3(gpu_frame2.rows +4 , gpu_frame2.cols + 4, gpu_frame2.type());
    gpu::copyMakeBorder(gpu_frame2, gpu_frame3, 2, 2 , 2, 2, BORDER_REPLICATE);

    gpu::blur(gpu_frame3, gpu_frame, Size(3,3));

    gpu::GpuMat roi(gpu_frame, Rect(2, 2, gpu_frame.cols-4, gpu_frame.rows-4));

    LOG(DEBUG) << "Blur stop";

    fp.show_frame(roi);

    //gpu::GpuMat subframe(gpu_frame,Rect(0,frame.rows-40,frame.cols,40));
    //gpu::GpuMat subframe2(subframe.rows, subframe.cols, subframe.type());
    //gpu::Canny(subframe, subframe2, 50,100, 3);

    LOG(DEBUG) << "Canny start";
    gpu::Canny(roi, gpu_frame2, 50,100, 3);
    LOG(DEBUG) << "Canny stop";

    fp.show_frame(gpu_frame2);

    gpu::GpuMat d_lines;
    gpu::HoughLinesBuf d_buf;

    LOG(DEBUG) << "Hough start";
    gpu::HoughLinesP(gpu_frame2, d_lines,d_buf, 1.0f, (float)(CV_PI/360.0f),10,5);
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
    LOG(DEBUG) << "Play: " << play;
    auto key = 0;
    if(play && fps>0)
    {
        LOG(DEBUG) << "Waitkey with FPS";
        key = waitKey(1000/fps) & 255;
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
