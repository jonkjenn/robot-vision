#include "camshift.h"

using namespace cv;
using namespace std;

void camshift::setup_camshift(bool show_video, bool save_video)
{
    do_show_video = show_video;
    do_save_video = save_video;
    cout << "Show video " << do_show_video << endl;
    Mat frame = imread("bildet.png");
    int cols = frame.cols;
    cc = 0,cr=0,cw=frame.cols,cch=frame.rows;
    if(do_show_video)
    {
        imshow("org_frame", frame);
        //s_rect = Rect{0,0,frame.cols,frame.rows};
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

    inRange(hsv,Scalar(35,89,150),Scalar(50,255,255),mask);

    if(do_show_video)
    {
        imshow("mask", mask);
        moveWindow("mask",0,300);
    }

    //cout << "channels: " << hsv.channels() << endl;

    calcHist(&hsv,1,channels,mask,hist,num_channels,histSize,ranges);

    normalize(hist,hist,0,255,cv::NORM_MINMAX);

    //cout << "hist = " << endl << hist << endl;
}

unsigned int camshift::did_not_find_target()
{
    reset_count++;
    cout << "Did not find target" << endl;
    if(previous_position < 3500)
    {
        return 0;
    }
    else
    {
        return 7000;
    }
}

int camshift::update_camshift(Mat frame)
{

    if(frame.empty()){return -1;}

    Rect roi = s_rect;

    if(reset_count > 100)
    {
        cout << "Reset the search window " << endl;
        s_rect = Rect{0,0,frame.cols,frame.rows};
        reset_count = 0;
    }

    if(s_rect.area() == 0)
    {
        s_rect = Rect{0,0,frame.cols,frame.rows};
    }

    if(roi.tl().x < 0){
        roi.x = 0;
    }
    if(roi.tl().y < 0)
    {
        roi.y = 0;
    }

    if(roi.br().x > frame.cols)
    {
        roi.x -= roi.br().x - frame.cols;
    }

    if(roi.br().x > frame.rows)
    {
        roi.y -= roi.br().y - frame.rows;
    }


    cout << "rect: " << s_rect.size() << endl;
    cout << "tl: " << s_rect.tl() << endl;
    cout << "br: " << s_rect.br() << endl;
    cout << "frame: " << frame.size() << endl;

    Mat hsv;
    cvtColor(frame,hsv,COLOR_BGR2HSV);
    //blur(hsv,hsv,Size(3,3));

    //int channels[] = {0,1};
    //const float* ranges[] = {hranges,sranges};

    //int from_to[] = {0,0};
    //Mat hue(hsv.size(),hsv.depth());
    //mixChannels(&hsv,3,&hue,1,from_to,1);

    /*Mat bp(hsv.size(),hsv.depth());
    calcBackProject(&hsv,1,channels,hist,bp,ranges,1.0);

    erode(bp,bp,Mat(),Point{-1,-1},1);
    dilate(bp,bp,Mat(),Point{-1,-1},5);

    if(do_show_video)
    {
        imshow("back_project", bp);
        moveWindow("back_project", frame.cols*2,0);
    }*/

    Mat mask;

    inRange(hsv,Scalar(35,85,100),Scalar(50,255,255),mask);

    if(do_show_video)
    {
        imshow("mask1", mask);
        moveWindow("mask1", frame.cols*4,0);
    }

    //dilate(mask,mask,Mat(),Point{-1,-1},5);

    //Mat can;
    //Canny(mask,can,0,10,3);

    /*if(do_show_video)
    {
        imshow("canny",can);
        moveWindow("canny",frame.cols*4,frame.rows);
    }*/

    erode(mask,mask,Mat(),Point{-1,-1},1);
    dilate(mask,mask,Mat(),Point{-1,-1},5);

    vector<vector<Point>> contours;
    findContours(mask,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);;

    if(contours.size() == 0){
        return did_not_find_target();
    }

    int best_index = -1;
    float largest_area = 0;
    Point2f centroid;
    for(int i=0;i<contours.size();i++)
    {
        float a = contourArea(contours[i]);
        if(a>largest_area){
            RotatedRect area = minAreaRect(contours[i]);

            Rect isec = (area.boundingRect() & s_rect);
            if(isec.area() < area.boundingRect().area()){
                continue;
            }

            int b_w = area.size.width;
            int b_h = area.size.height;
            float ratio = abs(b_w-b_h)/(float)(b_w+b_h);

            cout << "b_w: " << b_w << " b_h: " << b_h << " ratio: " << ratio;
            if(b_w * b_h > 4 && ratio < 0.1)
            {
                Moments m = moments(contours[i],false);
                centroid.x = m.m10 / m.m00;
                centroid.y = m.m01 / m.m00;

                //600 at 40
                //6200 at 144
                //
                /*unsigned int my_x = frame.cols/2; 
                unsigned int my_y = frame.rows-1; 
                float dist = sqrt(pow(my_x - centroid.x,2) + pow(my_y - centroid.y,2));

                float max_area = dist/100.0 * 7000;

                cout << "dist: " << dist << endl;
                cout << "max area: " << max_area << endl;
                cout << "area " << a << " x,y " << centroid.x << "," << centroid.y <<  endl;*/

                largest_area = a; 
                best_index = i;

            }
        }
    }

    cout << "best index " << best_index << endl;

    if(best_index < 0){
        return did_not_find_target();
    }

    //cout << "Area: " << largest_area << endl;

    Mat only_mask = Mat::zeros(mask.size(),mask.depth());
    drawContours(only_mask,contours,best_index,Scalar(255),CV_FILLED);


    if(do_show_video)
    {
        imshow("mask2", only_mask);
        moveWindow("mask2",340,300);
    }

    TermCriteria crit = TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,40,1);

    RotatedRect camshift_rect;

    Rect prev_srect = s_rect;

    try{
        //Rect window = Rect{0,0,only_mask.cols,only_mask.rows};
        camshift_rect = CamShift(only_mask,s_rect,crit);
        //cout << "s_rect: " << s_rect << endl;
        s_rect = camshift_rect.boundingRect();
        //cout << "s_rected: " << s_rect << endl;
    }catch(Exception)
    {
        reset_count++;
        cout << "exception " << endl;
        return -1;
    }

    cout << "s_rect size: " << s_rect.size() << endl;
    cout << "s_rect tl: " << s_rect.tl() << endl;
    if(s_rect.width * s_rect.height < 4){
        cout << "too small " << endl;
        s_rect = prev_srect;
        reset_count++;
        return -1;
    }

    s_rect.x -= 10;
    s_rect.y -= 10;
    s_rect.height += 20;
    s_rect.width += 20;

    /*Mat output;
    frame.copyTo(output);*/

    /*Point2f vertices[4];
    camshift_rect.points(vertices);
    for(int i=0;i<4;i++)
    {
        line(output, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
    }*/

    rectangle(frame,s_rect,Scalar(0,0,255));

    if(do_show_video){
        imshow("win",frame);
        moveWindow("win",frame.cols*2 + frame.cols,0);
        //fp.show_frame(frame);
    }

    if(do_save_video)
    {
        if(!writer.isOpened())
        {
            cout << "WRITER FRAME SIZE: " << frame.size() << endl;
            Size sz = frame.size();
            //writer.open("out.avi",CV_FOURCC('M','J','P','G') ,200,sz);
            //writer.open("out.avi",CV_FOURCC('H','2','6','4'),200,sz);
            writer.open("ball.avi",CV_FOURCC('M','P','4','V'),15,sz);
        }

        if(micros()-previous_frame_saved > 33333){cout << "writing frame" << endl;writer << frame;previous_frame_saved = micros();} //imwrite("ball.png",frame); }
    }

    return calculate_position(centroid, frame);
}


unsigned int camshift::calculate_position(Point2f centroid, Mat frame)
{
   unsigned int my_x = frame.cols/2; 
   unsigned int my_y = frame.rows-1; 

   cout << "calculating" << endl;

   return 7000-((-atan2(centroid.y-my_y, centroid.x - my_x))/PI)*7000.0;
}
