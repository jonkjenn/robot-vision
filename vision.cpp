#include "vision.hpp"

using namespace std;
using namespace cv;
using namespace cv::gpu;

void update_camshift(Mat frame);
void setup_camshift(bool show_video);
bool camshift_init = false;

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
        configure_cuda();
    }
    //VideoWriter outputVideo;

    /*if(write){
        int ex = static_cast<int>(cap->get(CV_CAP_PROP_FOURCC));
    }*/

    /*if(write){
        outputVideo.open("../out.mp4",CV_FOURCC('m','4','s','2'),30,s,true);
    }*/

    size = Size(177,144);
    sub_rect = Rect(0,size.height-41,size.width,40);

    if(save_video)
    {
        Size sz = Size(320,192);
        //writer.open("out.avi",CV_FOURCC('M','J','P','G') ,200,sz);
        //writer.open("out.avi",CV_FOURCC('H','2','6','4'),200,sz);
        writer.open("out.avi",CV_FOURCC('M','P','4','V'),200,sz);
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
        setup_camshift(show_video);
    }
    else if(input_type == Type::FILE)
    {
        LOG(DEBUG) << "Starting file thread" << endl;
        capture_thread = thread(&Vision::capture_frames_file,this, ref(frame));
    }
    previous_micros = micros();
}

void Vision::configure_cuda()
{
    cuda = gpu::getCudaEnabledDeviceCount()>0;
    LOG(INFO) <<"Setting CUDA device" << endl;
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
            if(quit.load()){cout << "vision thread quit" << endl;return;}
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

void Vision::update()
{
    //cout << "vision update" << endl;
    if(input_type == Type::FILE && index >= frame_count){play = false; handle_keys();return;}
    Mat buffer;
    auto loading_time = micros();

    if(input_type == Type::CAMERA || input_type == Type::PS4)
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


    LOG(DEBUG) << buffer.size() << endl;
    //LOG(INFO) << "Loaded frame: " << (micros() - loading_time) << " microseconds";
    //LOG(INFO) << "Resized frame";

    if(cuda)
    {
        //process_frame_cuda(buffer);
    }
    else
    {
        //process_frame(buffer);
    }

    if(save_video){writer << buffer;}

    update_camshift(buffer);

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
    LOG(DEBUG) << "Loop duration: " << dur << " fps: " << (float)1000000/(float)dur;
    //printf("fps: %f\n", (float)1000000/dur);
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
    LOG(DEBUG) << "Vision stop true " << endl;
    quit.store(true);
    capture_thread.join();
    ps4cam->stop();
    cout << "capture thread joined" << endl;
}


Mat frame,hsv,mask,roi,hsv_roi,back_project,roi_hist_h,roi_hist_s,roi_hist_v, roi_hist;
int cc,cr,cch,cw = 0;
Rect rect;
const int channels[]{0};
const int histSize[]{180};
const float hrange[]{30,70};
const float srange[]{127,175};
const float vrange[]{127,200};
const float* ranges[]{hrange};
const float* sranges[]{srange};
const float* vranges[]{vrange};
bool do_show_video = false;

void setup_camshift(bool show_video)
{
    do_show_video = show_video;
    //imwrite("bildet.png",frame);
    //
    Mat frame = imread("bildet.png");

    imshow("original_hist", frame);

    camshift_init = true;
    cvtColor(frame,hsv,COLOR_BGR2HSV);

    rect = Rect{0,0,frame.cols,frame.rows};
    roi = hsv(rect);
    roi = hsv;
    Mat h_chan(roi.rows,roi.cols,CV_8UC1);
    Mat s_chan(roi.rows,roi.cols,CV_8UC1);
    Mat v_chan(roi.rows,roi.cols,CV_8UC1);

    cc = 0,cr=0,cw=frame.cols,cch=frame.rows;

    Mat out[] = {h_chan,s_chan,v_chan};
    int from_to[] = {0,0,1,1,2,2};
    //mixChannels( &roi, 1, out, 3, from_to, 3);
    split(roi,out);

    if(do_show_video){
        /*imshow("h_chan", h_chan);
    imshow("s_chan", s_chan);
    imshow("v_chan", v_chan);*/
        imshow("h_chan", h_chan);
        moveWindow("h_chan", frame.cols, 0);
        imshow("s_chan", s_chan);
        moveWindow("s_chan", frame.cols + h_chan.cols, 0);
        imshow("v_chan", v_chan);
        moveWindow("v_chan", frame.cols + 2*s_chan.cols, 0);
    }

    calcHist(&h_chan,1,channels,Mat(),roi_hist_h,1,histSize,ranges);

    const int svhistSize[]{180};
    const float* sranges[]{srange};
    calcHist(&s_chan,1,channels,Mat(),roi_hist_s,1,svhistSize,sranges);
    calcHist(&v_chan,1,channels,Mat(),roi_hist_v,1,svhistSize,vranges);

    normalize(roi_hist_h,roi_hist_h,0,255,cv::NORM_MINMAX);
    normalize(roi_hist_s,roi_hist_s,0,255,cv::NORM_MINMAX);
    normalize(roi_hist_v,roi_hist_v,0,255,cv::NORM_MINMAX);

    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/256 );

    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

    /// Draw for each channel
    for( int i = 1; i < 256; i++ )
    {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(roi_hist_h.at<float>(i-1)) ) ,
                Point( bin_w*(i), hist_h - cvRound(roi_hist_h.at<float>(i)) ),
                Scalar( 255, 0, 0), 2, 8, 0  );
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(roi_hist_s.at<float>(i-1)) ) ,
                Point( bin_w*(i), hist_h - cvRound(roi_hist_s.at<float>(i)) ),
                Scalar( 0, 255, 0), 2, 8, 0  );
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(roi_hist_v.at<float>(i-1)) ) ,
                Point( bin_w*(i), hist_h - cvRound(roi_hist_v.at<float>(i)) ),
                Scalar( 0, 0, 255), 2, 8, 0  );
    }

    if(do_show_video){
        imshow("hist", histImage);
        moveWindow("hist",frame.cols + 3*s_chan.cols, 0);
    }

    roi_hist = Mat{roi_hist_h.size(), CV_32FC3};
    Mat hists[] = {roi_hist_h, roi_hist_s,roi_hist_v};
    int ft[] = {0,0,1,0,2,0};
    mixChannels(hists,3,&roi_hist,1,ft,3);

}

void update_camshift(Mat frame)
{
        //inRange(hsv_roi,Mat{1,3,CV_8U,low},Mat{1,3,CV_8U,high},mask);

        TermCriteria crit = TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,20,1);

        RotatedRect camshift_rect;

        cvtColor(frame,hsv,COLOR_BGR2HSV);
        //cout << "calcback" << endl;
        int ch[] = {0,1,2};
        calcBackProject(&hsv,1,ch,roi_hist,back_project,ranges,1.0);

        if(do_show_video){
            imshow("win3",back_project);
            moveWindow("win3",0,300);
            //fp.show_frame(back_project);
        }

        try{
            camshift_rect = CamShift(back_project,rect,crit);
        }catch(Exception)
        {
            rect = Rect{cc,cr,cw,cch};
        }

        Point2f vertices[4];
        camshift_rect.points(vertices);
        for(int i=0;i<4;i++)
        {
            line(frame, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
        }

        if(do_show_video){
            imshow("win",frame);
            moveWindow("win",back_project.cols,300);
            //fp.show_frame(frame);
        }
        //imshow("win2",roi);
        //waitKey(1);
}
