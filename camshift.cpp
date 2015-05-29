#include "camshift.h"

using namespace cv;
using namespace std;

void camshift::setup_camshift(bool show_video)
{
    do_show_video = show_video;
    cout << "Show video " << do_show_video << endl;
    Mat frame = imread("bildet.png");
    int cols = frame.cols;
    cc = 0,cr=0,cw=frame.cols,cch=frame.rows;
    if(do_show_video)
    {
        imshow("org_frame", frame);
        s_rect = Rect{0,0,frame.cols,frame.rows};
    }
    //rect = Rect{125,50,frame.cols-205,frame.rows-80};
    //frame = frame(rect);

    if(do_show_video)
    {
        imshow("org_rio", frame);
        moveWindow("org_rio",cols,0);
    }

    cvtColor(frame,hsv,COLOR_BGR2HSV);


    int from_to[] = {0,0,1,1};

    //Mat targets[] = {hue,sat};

    /*Mat hue(hsv.size(),hsv.depth());
    Mat sat(hsv.size(),hsv.depth());
    mixChannels(&hsv,3,targets,2,from_to,2);*/

    Mat mask;

    inRange(hsv,Scalar(22,89,150),Scalar(50,255,255),mask);

    if(do_show_video)
    {
        imshow("mask", mask);
        moveWindow("mask",0,300);
    }

    cout << "channels: " << hsv.channels() << endl;

    calcHist(&hsv,1,channels,mask,hist,num_channels,histSize,ranges);

    normalize(hist,hist,0,255,cv::NORM_MINMAX);

    cout << "hist = " << endl << hist << endl;
}

int camshift::update_camshift(Mat frame)
{
    if(frame.empty()){return -1;}
    cvtColor(frame,hsv,COLOR_BGR2HSV);

    //int channels[] = {0,1};
    //const float* ranges[] = {hranges,sranges};

    //int from_to[] = {0,0};
    //Mat hue(hsv.size(),hsv.depth());
    //mixChannels(&hsv,3,&hue,1,from_to,1);

    Mat bp(hsv.size(),hsv.depth());
    calcBackProject(&hsv,1,channels,hist,bp,ranges,10.0);

    erode(bp,bp,Mat(),Point{-1,-1},1);
    dilate(bp,bp,Mat(),Point{-1,-1},5);

    if(do_show_video)
    {
        imshow("back_project", bp);
        moveWindow("back_project", frame.cols*2,0);
    }

    Mat mask;

    inRange(hsv,Scalar(22,89,150),Scalar(50,255,255),mask);

    erode(mask,mask,Mat(),Point{-1,-1},1);
    dilate(mask,mask,Mat(),Point{-1,-1},5);

    vector<vector<Point>> contours;
    findContours(mask,contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);;

    if(contours.size() == 0){return -1;}

    int largest_index = 0;
    float largest_area = 0;
    for(int i=0;i<contours.size();i++)
    {
        float a = contourArea(contours[i]);
        if(a>largest_area){largest_area = a; largest_index = i;}
    }

    Mat only_mask = Mat::zeros(mask.size(),mask.depth());
    drawContours(only_mask,contours,largest_index,Scalar(255),CV_FILLED);

    Moments m = moments(contours[largest_index],false);
    Point2f centroid;
    centroid.x = m.m10 / m.m00;
    centroid.y = m.m01 / m.m00;

    if(do_show_video)
    {
        imshow("mask2", only_mask);
        moveWindow("mask2",340,300);
    }

    TermCriteria crit = TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,40,1);

    RotatedRect camshift_rect;

    try{
        camshift_rect = CamShift(bp,s_rect,crit);
        //cout << "s_rect: " << s_rect << endl;
        s_rect = camshift_rect.boundingRect();
        //cout << "s_rected: " << s_rect << endl;
    }catch(Exception)
    {
       s_rect = Rect{0,0,frame.cols,frame.rows};
    }

    if(s_rect.width * s_rect.height < 4){s_rect = Rect{0,0,frame.cols,frame.rows};}

    Mat output;
    frame.copyTo(output);

    Point2f vertices[4];
    camshift_rect.points(vertices);
    for(int i=0;i<4;i++)
    {
        line(output, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
    }

    rectangle(output,camshift_rect.boundingRect(),Scalar(0,0,255));

    if(do_show_video){
        imshow("win",output);
        moveWindow("win",frame.cols*2 + bp.cols,0);
        //fp.show_frame(frame);
    }
    
    cout << "s_rect size: " << s_rect.size() << endl;

    return calculate_position(centroid, frame);
}


unsigned int camshift::calculate_position(Point2f centroid, Mat frame)
{
   unsigned int my_x = frame.cols/2; 
   unsigned int my_y = frame.rows-1; 

   cout << "calculating" << endl;

   return ((-atan2(centroid.y-my_y, centroid.x - my_x))/PI)*7000.0;
}
