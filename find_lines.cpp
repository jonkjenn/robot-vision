#include "find_lines.hpp"

using namespace std;
using namespace cv;
using namespace cv::gpu;

Find_lines::Find_lines()
{
    blur_filter = gpu::createGaussianFilter_GPU(CV_8U,Size(3,3),-1);
}

void Find_lines::find(Mat frame)
{
    if(frame.empty()){return;}
    //sub_rect = Rect(0,size.height-41,size.width,40);
    gpu_frame.upload(frame);
    _frame = frame;
    hough_gpu();
}

void Find_lines::hough_gpu()
{
    gpu::cvtColor(gpu_frame,gpu_grayscale,CV_BGR2GRAY,0);
   // cout << "Subframe start";
    //gpu_subframe = gpu::GpuMat(gpu_grayscale,sub_rect);

    //cout << "Filter start";

    blur_filter->apply(gpu_grayscale, gpu_blur, Rect(0,0,gpu_grayscale.cols, gpu_grayscale.rows));

    cout << "Canny start" << endl;
    gpu::Canny(gpu_blur, gpu_canny, 50.0f,100.0f);
    cout << "Canny stop, rows " << gpu_canny.rows << "cols " << gpu_canny.cols << endl;

    //fp.show_frame(gpu_canny);

    /*cout << "Hough start" << endl;
    gpu::HoughLinesP(gpu_canny, d_lines,d_buf, 1.0f,hough_angles,10,10);
    cout << "Hough stop" << endl;*/

    imshow("frame",_frame);
    gpu_canny.download(_frame);
    imshow("canny", _frame);
    moveWindow("canny", _frame.cols + 10,0);

    //draw_hough();
}

void Find_lines::draw_hough()
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
        line(_frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
    }
    imshow("hough", _frame);
}
